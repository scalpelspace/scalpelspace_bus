/*******************************************************************************
 * @file scalpel_bus.cpp
 * @brief ScalpelBus: unified host controller for ScalpelSpace CAN devices.
 *******************************************************************************
 */

#include "scalpel_bus.h"

/** Static instance pointer for allocator trampolines. ************************/

ScalpelBus *ScalpelBus::s_activeInstance = NULL;

/** Construction. *************************************************************/

ScalpelBus::ScalpelBus(const uint8_t csPin, SPIClass &spi,
                       const ScalpelBusOscillator oscillator)
    : _transport(new Mcp2518fdTransport(csPin, spi, oscillator)),
      _ownsTransport(true), _deviceListHead(NULL),
      _unhandledFrameCallback(NULL), _status(SCALPEL_BUS_IDLE),
      _strategy(can_id_strategy_fifo), _nodeCount(0),
      _allocationComplete(false), _discoveryOpen(false) {}

ScalpelBus::ScalpelBus(ScalpelCanTransport &transport)
    : _transport(&transport), _ownsTransport(false), _deviceListHead(NULL),
      _unhandledFrameCallback(NULL), _status(SCALPEL_BUS_IDLE),
      _strategy(can_id_strategy_fifo), _nodeCount(0),
      _allocationComplete(false), _discoveryOpen(false) {}

ScalpelBus::~ScalpelBus() {
  if (s_activeInstance == this) {
    s_activeInstance = NULL;
  }
  if (_ownsTransport) {
    delete _transport;
  }
}

/** Device registration (called from ScalpelBusDevice constructor). ***********/

void ScalpelBus::registerDevice(ScalpelBusDevice *device) {
  // Append at tail: positional binding follows construction order.
  device->_nextDevice = NULL;
  if (_deviceListHead == NULL) {
    _deviceListHead = device;
    return;
  }
  ScalpelBusDevice *tail = _deviceListHead;
  while (tail->_nextDevice != NULL) {
    tail = tail->_nextDevice;
  }
  tail->_nextDevice = device;
}

/** Begin / allocation. *******************************************************/

bool ScalpelBus::begin(const uint32_t bitrateBps,
                       const uint16_t discoveryWindowMs,
                       const uint16_t ackTimeoutMs) {
  s_activeInstance = this;
  if (!_transport->begin(bitrateBps)) {
    _status = SCALPEL_BUS_ERR_TRANSPORT;
    return false;
  }
  return runAllocation(discoveryWindowMs, ackTimeoutMs);
}

bool ScalpelBus::rediscover(const uint16_t discoveryWindowMs,
                            const uint16_t ackTimeoutMs) {
  if (_status == SCALPEL_BUS_IDLE || _status == SCALPEL_BUS_ERR_TRANSPORT) {
    return false; // Transport not up; call begin() first.
  }
  s_activeInstance = this;
  return runAllocation(discoveryWindowMs, ackTimeoutMs);
}

void ScalpelBus::seedAllocationSession() {
  // The vendored allocator's session counter is a private static that
  // restarts at 0 on every MCU reset, so after a reset the first DISCOVER
  // would always carry session ID 1. Frames left in flight by a run that
  // the reset interrupted would then carry the *same* session ID and be
  // accepted by the new run (ghost advertise entries, false ACKs). There
  // is no upstream API to seed the counter, but each start() +
  // state_machine() cycle increments it, so advance it a time-jittered
  // number of steps against a sink TX (no bus traffic) once per boot.
  // Replace with an upstream session-seed API in can_driver when
  // available.
  static bool seeded = false;
  if (seeded) {
    return;
  }
  seeded = true;

  allocator_config_t sink;
  sink.can_tx_func = &ScalpelBus::allocatorTxSink;
  sink.allocator_assigned_func = NULL; // Never reached during a seed step.
  sink.strategy = NULL;

  const uint8_t steps = (uint8_t)(micros() ^ (millis() << 4));
  for (uint8_t i = 0; i < steps; i++) {
    can_id_allocator_start(sink);
    can_id_allocator_state_machine(); // Increments the session counter.
  }
}

bool ScalpelBus::runAllocation(const uint16_t discoveryWindowMs,
                               const uint16_t ackTimeoutMs) {
  seedAllocationSession();
  _nodeCount = 0;
  _allocationComplete = false;

  allocator_config_t config;
  config.can_tx_func = &ScalpelBus::allocatorTxTrampoline;
  config.allocator_assigned_func = &ScalpelBus::allocatorAssignedTrampoline;
  config.strategy = _strategy;

  can_id_allocator_start(config);
  can_id_allocator_state_machine(); // DISCOVER broadcast goes out here.

  // Discovery window: collect ADVERTISE messages.
  _discoveryOpen = true;
  const uint32_t discoveryStart = millis();
  while ((millis() - discoveryStart) < discoveryWindowMs) {
    pumpRx();
  }
  _discoveryOpen = false;
  can_id_allocator_end_discovery();
  // Mirror the allocator's strategy run on the shadow node table so ACK
  // tracking below can match node IDs before the completion callback.
  computeNodeIdsFromStrategy();
  can_id_allocator_state_machine(); // ASSIGN messages go out here.

  // Await assignment ACKs.
  const uint32_t ackStart = millis();
  while (!_allocationComplete && ((millis() - ackStart) < ackTimeoutMs)) {
    pumpRx();
    can_id_allocator_state_machine();
  }

  bindDevicesPositional();
  _status = _allocationComplete ? SCALPEL_BUS_OK : SCALPEL_BUS_ERR_ACK_TIMEOUT;
  return _allocationComplete;
}

void ScalpelBus::computeNodeIdsFromStrategy() {
  // Strategies are pure functions of the discovered UID set (plus the UID
  // table configured via useUidTableAllocation()), so running the same
  // strategy on the shadow copy reproduces the allocator's assignment.
  uint16_t uids0[CAN_ID_MAX_NODES] = {0};
  uint16_t uids1[CAN_ID_MAX_NODES] = {0};
  uint16_t uids2[CAN_ID_MAX_NODES] = {0};
  can_node_id_t nodeIds[CAN_ID_MAX_NODES] = {0};
  for (uint8_t i = 0; i < _nodeCount; i++) {
    uids0[i] = _nodes[i].uid[0];
    uids1[i] = _nodes[i].uid[1];
    uids2[i] = _nodes[i].uid[2];
  }
  _strategy(uids0, uids1, uids2, _nodeCount, nodeIds);
  for (uint8_t i = 0; i < _nodeCount; i++) {
    _nodes[i].nodeId = nodeIds[i];
  }
}

void ScalpelBus::bindDevicesPositional() {
  uint8_t index = 0;
  for (ScalpelBusDevice *device = _deviceListHead; device != NULL;
       device = device->_nextDevice) {
    // Skip nodes left unassigned by the strategy (node ID space exhausted).
    while (index < _nodeCount &&
           _nodes[index].nodeId == CAN_ID_NODE_ID_UNASSIGNED) {
      index++;
    }
    if (index < _nodeCount) {
      device->attach(_nodes[index].nodeId);
      index++;
    } else {
      device->detach();
    }
  }
}

/** RX pump and dispatch. *****************************************************/

void ScalpelBus::poll() { pumpRx(); }

void ScalpelBus::pumpRx() {
  _transport->service();
  ScalpelCanFrame frame;
  while (_transport->receive(frame)) {
    dispatchFrame(frame);
  }
}

void ScalpelBus::dispatchFrame(const ScalpelCanFrame &frame) {
  can_message_id_t messageId = 0;
  can_node_id_t nodeId = 0;
  if (!can_id_unpack(frame.id, &messageId, &nodeId)) {
    return;
  }

  if (messageId >= (can_message_id_t)CAN_MSG_ENUM_DISCOVER) {
    handleAllocationFrame(messageId, frame);
    return;
  }

  const uint16_t baseId = (uint16_t)(frame.id & ~(uint16_t)0x1Fu);
  bool handled = false;
  for (ScalpelBusDevice *device = _deviceListHead; device != NULL;
       device = device->_nextDevice) {
    if (device->attached() && device->nodeId() == nodeId) {
      device->handleFrame(baseId, frame.data, frame.dlc);
      handled = true;
    }
  }
  if (!handled && _unhandledFrameCallback != NULL) {
    _unhandledFrameCallback(frame);
  }
}

void ScalpelBus::handleAllocationFrame(const can_message_id_t messageId,
                                       const ScalpelCanFrame &frame) {
  can_header_t header;
  header.standard_id = frame.id;
  header.extended_id = 0;
  header.ide = 0;
  header.dlc = frame.dlc;
  header.rtr = 0;

  switch (messageId) {
  case (can_message_id_t)CAN_MSG_ENUM_ADVERTISE: {
    can_rx_can_id_allocator_advertise(&header, frame.data);
    // Shadow record of the advertised UID so nodes can be inspected even
    // if allocation later stalls; overwritten by the authoritative
    // completion callback on success. Mirrors the allocator's accept
    // window (advertises after discovery closes are ignored there too).
    if (_discoveryOpen && frame.dlc == 8 && _nodeCount < CAN_ID_MAX_NODES) {
      ScalpelBusNode &node = _nodes[_nodeCount];
      node.uid[0] = (uint16_t)(frame.data[0] | ((uint16_t)frame.data[1] << 8));
      node.uid[1] = (uint16_t)(frame.data[2] | ((uint16_t)frame.data[3] << 8));
      node.uid[2] = (uint16_t)(frame.data[4] | ((uint16_t)frame.data[5] << 8));
      // Node ID is provisional until computeNodeIdsFromStrategy() runs
      // when the discovery window closes.
      node.nodeId = CAN_ID_NODE_ID_UNASSIGNED;
      node.acked = false;
      _nodeCount++;
    }
    break;
  }
  case (can_message_id_t)CAN_MSG_ENUM_ACK: {
    can_rx_can_id_allocator_ack(&header, frame.data);
    const can_node_id_t ackedNodeId = (can_node_id_t)(frame.id & 0x1Fu);
    for (uint8_t i = 0; i < _nodeCount; i++) {
      if (_nodes[i].nodeId == ackedNodeId) {
        _nodes[i].acked = true;
        break;
      }
    }
    break;
  }
  default:
    // DISCOVER/ASSIGN on the wire come from this (or a conflicting)
    // allocator; nothing to do on the host side.
    break;
  }
  can_id_allocator_state_machine();
}

/** Allocation completion. ****************************************************/

void ScalpelBus::onAllocationAssigned(const uint16_t *uids0,
                                      const uint16_t *uids1,
                                      const uint16_t *uids2,
                                      const uint8_t nodeCount) {
  // Authoritative result: UID arrays are indexed by discovery order.
  _nodeCount = (nodeCount <= CAN_ID_MAX_NODES) ? nodeCount : CAN_ID_MAX_NODES;
  for (uint8_t i = 0; i < _nodeCount; i++) {
    _nodes[i].uid[0] = uids0[i];
    _nodes[i].uid[1] = uids1[i];
    _nodes[i].uid[2] = uids2[i];
    _nodes[i].acked = true;
  }
  computeNodeIdsFromStrategy();
  _allocationComplete = true;
}

/** Node info / version probe. ************************************************/

bool ScalpelBus::nodeInfo(const uint8_t index, ScalpelBusNode &out) const {
  if (index >= _nodeCount) {
    return false;
  }
  out = _nodes[index];
  return true;
}

bool ScalpelBus::probeVersion(const uint8_t nodeId, ScalpelVersionInfo &out,
                              const uint16_t timeoutMs) {
  can_id_t requestId = 0;
  can_id_t responseId = 0;
  if (!can_id_pack(kMsgIdVersionGet, nodeId, &requestId) ||
      !can_id_pack(kMsgIdVersionResponse, nodeId, &responseId)) {
    return false;
  }
  if (!sendFrame(requestId, NULL, 0)) {
    return false;
  }

  const uint32_t start = millis();
  ScalpelCanFrame frame;
  while ((millis() - start) < timeoutMs) {
    _transport->service();
    while (_transport->receive(frame)) {
      if (frame.id == responseId && frame.dlc >= 4) {
        out.major = frame.data[0];
        out.minor = frame.data[1];
        out.patch = frame.data[2];
        out.identifier = (char)frame.data[3];
        out.lastUpdateMs = millis();
        return true;
      }
      dispatchFrame(frame); // Keep other traffic flowing while we wait.
    }
  }
  return false;
}

/** TX. ***********************************************************************/

bool ScalpelBus::sendFrame(const uint16_t id, const uint8_t *data,
                           const uint8_t dlc) {
  ScalpelCanFrame frame;
  frame.id = id;
  frame.dlc = (dlc <= 8u) ? dlc : 8u;
  for (uint8_t i = 0; i < 8; i++) {
    frame.data[i] = (data != NULL && i < frame.dlc) ? data[i] : 0u;
  }
  return _transport->send(frame);
}

/** Allocator trampolines. ****************************************************/

bool ScalpelBus::allocatorTxSink(const can_message_t *message,
                                 const uint8_t data[8]) {
  // Swallows the DISCOVER emitted by each session seed step.
  (void)message;
  (void)data;
  return true;
}

bool ScalpelBus::allocatorTxTrampoline(const can_message_t *message,
                                       const uint8_t data[8]) {
  if (s_activeInstance == NULL || message == NULL) {
    return false;
  }
  return s_activeInstance->sendFrame((uint16_t)message->message_id, data,
                                     message->dlc);
}

void ScalpelBus::allocatorAssignedTrampoline(
    uint16_t uids0[CAN_ID_MAX_NODES], uint16_t uids1[CAN_ID_MAX_NODES],
    uint16_t uids2[CAN_ID_MAX_NODES], can_node_id_t nodeIds[CAN_ID_MAX_NODES],
    can_node_id_t nodeCount) {
  // nodeIds is indexed by node ID with an ambiguous 0 default, so node IDs
  // are instead recomputed from the configured strategy (deterministic on
  // the same UID set) inside onAllocationAssigned().
  (void)nodeIds;
  if (s_activeInstance != NULL) {
    s_activeInstance->onAllocationAssigned(uids0, uids1, uids2, nodeCount);
  }
}
