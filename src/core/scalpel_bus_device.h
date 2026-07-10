/*******************************************************************************
 * @file scalpel_bus_device.h
 * @brief Base class for ScalpelSpace CAN device handles bound to a ScalpelBus.
 *******************************************************************************
 */

#ifndef SCALPELSPACE_BUS__SCALPEL_BUS_DEVICE_H
#define SCALPELSPACE_BUS__SCALPEL_BUS_DEVICE_H

#include <stdint.h>

extern "C" {
#include "../can_driver/can_driver.h"
#include "../can_driver/can_id.h"
}

class ScalpelBus;

/**
 * @brief Base class for device handles (McStepper, McBrushed, MomentumCan).
 *
 * Constructing a device handle registers it with the bus. During
 * ScalpelBus::begin() handles are bound to allocated node IDs in
 * construction order (see ScalpelBus for binding rules); call attach() to
 * bind explicitly instead.
 */
class ScalpelBusDevice {
public:
  explicit ScalpelBusDevice(ScalpelBus &bus);

  /** @brief True once the handle is bound to an allocated node ID. */
  bool attached() const { return _nodeId != CAN_ID_NODE_ID_UNASSIGNED; }

  /** @brief Bound node ID, or 0 (unassigned) when not attached. */
  uint8_t nodeId() const { return _nodeId; }

  /**
   * @brief Bind this handle to a specific node ID.
   *
   * Overrides automatic positional binding; useful on mixed-device buses
   * where construction order cannot be relied on.
   */
  void attach(uint8_t nodeId) { _nodeId = nodeId; }

  /** @brief Unbind the handle; frames are ignored until re-attached. */
  void detach() { _nodeId = CAN_ID_NODE_ID_UNASSIGNED; }

protected:
  friend class ScalpelBus;

  /**
   * @brief Handle a frame addressed from/to this device's node ID.
   *
   * @param baseId CAN ID with the node ID field cleared (message_id << 5).
   * @param data Frame payload (8 bytes, zero padded).
   * @param dlc Received data length.
   */
  virtual void handleFrame(uint16_t baseId, const uint8_t *data,
                           uint8_t dlc) = 0;

  /**
   * @brief Transmit a DBC message to this device's node.
   *
   * The on-wire CAN ID is the message's base ID with this device's node ID
   * packed into the low 5 bits. Fails when the handle is not attached.
   */
  bool sendMessage(const can_message_t &message, const uint8_t *data);

  /**
   * @brief Clamp, scale and pack one signal of @p message into @p data.
   */
  static void packSignal(const can_message_t &message, uint8_t signalIndex,
                         uint8_t *data, double physicalValue);

  /**
   * @brief Decode one signal of @p message from @p data.
   */
  static double decodeSignal(const can_message_t &message, uint8_t signalIndex,
                             const uint8_t *data);

  ScalpelBus &_bus;
  uint8_t _nodeId;
  ScalpelBusDevice *_nextDevice; // Intrusive list owned by ScalpelBus.
};

#endif
