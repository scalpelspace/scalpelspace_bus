/*******************************************************************************
 * @file scalpel_can_transport.h
 * @brief Abstract CAN transport interface for ScalpelBus.
 *******************************************************************************
 */

#ifndef SCALPELSPACE_BUS__SCALPEL_CAN_TRANSPORT_H
#define SCALPELSPACE_BUS__SCALPEL_CAN_TRANSPORT_H

#include "scalpel_can_frame.h"

/**
 * @brief Abstract classic CAN transport.
 *
 * ScalpelBus talks to the wire exclusively through this interface. The
 * default implementation is Mcp2518fdTransport (SPI-CAN breakout); supply a
 * custom implementation to run the ScalpelSpace protocol over any other
 * classic CAN controller.
 */
class ScalpelCanTransport {
public:
  virtual ~ScalpelCanTransport() {}

  /**
   * @brief Initialize the controller for standard 11-bit classic CAN.
   *
   * @param bitrateBps Nominal bitrate in bits per second (e.g. 500000).
   *
   * @return True on success.
   */
  virtual bool begin(uint32_t bitrateBps) = 0;

  /**
   * @brief Queue a frame for transmission. Non-blocking.
   *
   * @return True if the frame was accepted for transmission.
   */
  virtual bool send(const ScalpelCanFrame &frame) = 0;

  /**
   * @brief Fetch one received frame, if any. Non-blocking.
   *
   * @return True if a frame was written to @p frame.
   */
  virtual bool receive(ScalpelCanFrame &frame) = 0;

  /**
   * @brief Service the controller (polled drivers). Called from
   * ScalpelBus::poll(); default is a no-op for interrupt-driven transports.
   */
  virtual void service() {}

  /**
   * @brief Driver-specific error code from the last begin() call
   * (0 = success). Diagnostic aid; meaning depends on the transport.
   */
  virtual uint32_t lastErrorCode() const { return 0; }
};

#endif
