/*******************************************************************************
 * @file scalpel_bus.h
 * @brief ScalpelBus: unified host controller for ScalpelSpace CAN devices.
 *******************************************************************************
 * Owns the CAN transport (MCP2518FD SPI-CAN breakout by default) and acts as
 * the node ID allocator for the ScalpelSpace CAN protocol. Device handles
 * (McStepper, McBrushed, MomentumCan) register against a bus instance and are
 * bound to allocated node IDs by ScalpelBus::begin().
 *
 * ALLOCATOR ROLE ONLY: the host is the bus master and must be the only
 * allocator on the bus. begin() always assigns node IDs; there is no
 * passive join, and the allocatee (device) role is not provided. Do not
 * connect to a bus that already has another master.
 *
 * Only one ScalpelBus instance may exist per sketch: the underlying
 * can_driver allocator state machine is a C module with static state.
 *******************************************************************************
 */

#ifndef SCALPELSPACE_BUS__SCALPEL_BUS_H
#define SCALPELSPACE_BUS__SCALPEL_BUS_H

#include <Arduino.h>

extern "C" {
#include "../can_driver/can_driver.h"
#include "../can_driver/can_id.h"
#include "../can_driver/can_id_allocator.h"
}

#include "../hal/mcp2518fd/mcp2518fd_transport.h"
#include "../hal/scalpel_can_transport.h"
#include "scalpel_bus_device.h"
#include "scalpel_bus_types.h"

/** @brief Result of the last ScalpelBus::begin() / rediscover() call. */
enum ScalpelBusStatus {
  SCALPEL_BUS_IDLE = 0,            // begin() not called yet.
  SCALPEL_BUS_OK = 1,              // Allocation completed (0+ nodes).
  SCALPEL_BUS_ERR_TRANSPORT = 2,   // CAN controller init failed.
  SCALPEL_BUS_ERR_ACK_TIMEOUT = 3, // Node(s) failed to ACK assignment.
};

/** @brief One discovered node on the bus. */
struct ScalpelBusNode {
  uint16_t uid[3]; // 48-bit UID hash, segments [0..15], [16..31], [32..47].
  uint8_t nodeId;  // Assigned node ID (0 if the bus ran out of IDs).
  bool acked;      // True once the node ACKed its assignment.
};

/**
 * @brief One UID -> node ID mapping for useUidTableAllocation().
 *
 * Fields: uid_0/uid_1/uid_2 are the 48-bit UID hash segments (bits [0..15],
 * [16..31], [32..47]; run the uid_scan example to read them) and node_id is
 * the fixed node ID to assign (1..30).
 */
typedef can_id_uid_table_entry_t ScalpelBusNodeIdEntry;

class ScalpelBus {
public:
  /**
   * @brief Bus on the ScalpelSpace SPI-CAN breakout (MCP2518FD).
   *
   * @param csPin SPI chip-select pin wired to the breakout.
   * @param spi SPI bus instance (defaults to SPI).
   * @param oscillator Crystal fitted on the breakout (defaults to 40 MHz).
   */
  explicit ScalpelBus(uint8_t csPin, SPIClass &spi = SPI,
                      ScalpelBusOscillator oscillator = SCALPEL_BUS_OSC_40MHZ);

  /** @brief Bus on a user-supplied transport (custom CAN controller). */
  explicit ScalpelBus(ScalpelCanTransport &transport);

  ~ScalpelBus();

  /**
   * @brief Initialize the CAN controller and run node ID allocation.
   *
   * Acts as the bus master: always broadcasts discovery and assigns node
   * IDs to every unassigned device. Never call this on a bus that already
   * has another allocator.
   *
   * Broadcasts DISCOVER, collects node advertisements for
   * @p discoveryWindowMs, assigns node IDs using the selected strategy
   * (FIFO by default; see useUidTableAllocation() for fixed UID -> node ID
   * mappings), then waits up to @p ackTimeoutMs for every node to ACK.
   * Registered device handles are then bound positionally in discovery
   * order: the first constructed handle gets the first discovered node,
   * and so on. Positional binding is only deterministic when all devices
   * are the same product or the bus has a single device; on mixed buses
   * use useUidTableAllocation() and bind explicitly with
   * ScalpelBusDevice::attach().
   *
   * Nodes must be powered and listening before discovery opens; call
   * rediscover() to re-run allocation if devices boot late.
   *
   * @return True when allocation completed (including an empty bus; check
   *         nodeCount()). False on transport error or missing ACKs.
   */
  bool begin(uint32_t bitrateBps = 500000UL, uint16_t discoveryWindowMs = 500,
             uint16_t ackTimeoutMs = 250);

  /** @brief Re-run node ID allocation (fresh session) and re-bind devices. */
  bool rediscover(uint16_t discoveryWindowMs = 500,
                  uint16_t ackTimeoutMs = 250);

  /** Node ID assignment strategy (select before begin()/rediscover()). ******/

  /**
   * @brief Assign node IDs 1..N in discovery (arrival) order. Default.
   *
   * Simple, but not deterministic across power cycles when several nodes
   * boot together; fine for single-device buses.
   */
  void useFifoAllocation() { _strategy = can_id_strategy_fifo; }

  /**
   * @brief Assign node IDs 1..N in ascending 48-bit UID order.
   *
   * Deterministic for a fixed set of hardware without maintaining a table:
   * the same devices always end up with the same node IDs.
   */
  void useUidAscendingAllocation() {
    _strategy = can_id_strategy_uid_ascending;
  }

  /**
   * @brief Assign a fixed node ID to each known UID (recommended when the
   * sketch addresses devices by node ID).
   *
   * Devices found in @p entries receive their mapped node ID; unknown
   * devices still get a free node ID above the highest mapped one. The
   * array is used by reference and must stay alive while the bus is in use.
   *
   * @param entries UID -> node ID mappings (node IDs 1..30, unique).
   * @param count Number of entries.
   */
  void useUidTableAllocation(const ScalpelBusNodeIdEntry *entries,
                             uint8_t count) {
    can_id_strategy_uid_table_set(entries, count);
    _strategy = can_id_strategy_uid_table;
  }

  /**
   * @brief Service the transport and dispatch received frames.
   *
   * Call frequently from loop(); device telemetry caches update here.
   */
  void poll();

  ScalpelBusStatus status() const { return _status; }

  /** @brief Number of nodes discovered by the last allocation run. */
  uint8_t nodeCount() const { return _nodeCount; }

  /** @brief Info for the node at discovery index [0, nodeCount()). */
  bool nodeInfo(uint8_t index, ScalpelBusNode &out) const;

  /**
   * @brief Query a node's firmware version (blocking, up to @p timeoutMs).
   *
   * version_get/version_response share the same message IDs across all
   * ScalpelSpace CAN products, so this works on any node; the returned
   * identifier character tells products apart.
   */
  bool probeVersion(uint8_t nodeId, ScalpelVersionInfo &out,
                    uint16_t timeoutMs = 100);

  /** @brief Send a raw frame (advanced use). */
  bool sendFrame(uint16_t id, const uint8_t *data, uint8_t dlc);

  /** @brief Callback for frames not handled by any device or the allocator. */
  void onUnhandledFrame(void (*callback)(const ScalpelCanFrame &frame)) {
    _unhandledFrameCallback = callback;
  }

  /** @brief Access the underlying transport (advanced use). */
  ScalpelCanTransport &transport() { return *_transport; }

private:
  friend class ScalpelBusDevice;

  // version_get / version_response message IDs, shared by all current
  // ScalpelSpace CAN products.
  static const can_message_id_t kMsgIdVersionGet = 30;
  static const can_message_id_t kMsgIdVersionResponse = 31;

  void registerDevice(ScalpelBusDevice *device);
  void seedAllocationSession();
  bool runAllocation(uint16_t discoveryWindowMs, uint16_t ackTimeoutMs);
  void computeNodeIdsFromStrategy();
  void bindDevicesPositional();
  void pumpRx();
  void dispatchFrame(const ScalpelCanFrame &frame);
  void handleAllocationFrame(can_message_id_t messageId,
                             const ScalpelCanFrame &frame);
  void onAllocationAssigned(const uint16_t *uids0, const uint16_t *uids1,
                            const uint16_t *uids2, uint8_t nodeCount);

  // Trampolines into the single active instance; the can_driver allocator
  // takes plain function pointers without a context argument.
  static ScalpelBus *s_activeInstance;
  static bool allocatorTxSink(const can_message_t *message,
                              const uint8_t data[8]);
  static bool allocatorTxTrampoline(const can_message_t *message,
                                    const uint8_t data[8]);
  static void allocatorAssignedTrampoline(
      uint16_t uids0[CAN_ID_MAX_NODES], uint16_t uids1[CAN_ID_MAX_NODES],
      uint16_t uids2[CAN_ID_MAX_NODES], can_node_id_t nodeIds[CAN_ID_MAX_NODES],
      can_node_id_t nodeCount);

  ScalpelCanTransport *_transport;
  bool _ownsTransport;
  ScalpelBusDevice *_deviceListHead;
  void (*_unhandledFrameCallback)(const ScalpelCanFrame &frame);

  ScalpelBusStatus _status;
  node_id_assignment_strategy_t _strategy;
  ScalpelBusNode _nodes[CAN_ID_MAX_NODES];
  uint8_t _nodeCount;
  volatile bool _allocationComplete;
  bool _discoveryOpen; // True only while the discovery window is open.
};

#endif
