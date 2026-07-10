/*******************************************************************************
 * @file scalpel_bus_device.cpp
 * @brief Base class for ScalpelSpace CAN device handles bound to a ScalpelBus.
 *******************************************************************************
 */

#include "scalpel_bus_device.h"

#include "scalpel_bus.h"

ScalpelBusDevice::ScalpelBusDevice(ScalpelBus &bus)
    : _bus(bus), _nodeId(CAN_ID_NODE_ID_UNASSIGNED), _nextDevice(NULL) {
  bus.registerDevice(this);
}

bool ScalpelBusDevice::sendMessage(const can_message_t &message,
                                   const uint8_t *data) {
  if (!attached()) {
    return false;
  }
  const can_message_id_t messageId =
      (can_message_id_t)((message.message_id >> 5) & 0x3Fu);
  can_id_t id = 0;
  if (!can_id_pack(messageId, _nodeId, &id)) {
    return false;
  }
  return _bus.sendFrame(id, data, message.dlc);
}

void ScalpelBusDevice::packSignal(const can_message_t &message,
                                  const uint8_t signalIndex, uint8_t *data,
                                  const double physicalValue) {
  const can_signal_t *signal = &message.signals[signalIndex];
  pack_signal_raw32(signal, data, physical_to_raw(physicalValue, signal));
}

double ScalpelBusDevice::decodeSignal(const can_message_t &message,
                                      const uint8_t signalIndex,
                                      const uint8_t *data) {
  return decode_signal(&message.signals[signalIndex], data);
}
