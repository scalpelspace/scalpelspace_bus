/*******************************************************************************
 * @file mcp2518fd_transport.h
 * @brief MCP2518FD (SPI-CAN breakout) transport backed by the ACAN2517 driver.
 *******************************************************************************
 */

#ifndef SCALPELSPACE_BUS__MCP2518FD_TRANSPORT_H
#define SCALPELSPACE_BUS__MCP2518FD_TRANSPORT_H

#include <ACAN2517.h>
#include <SPI.h>

#include "../scalpel_can_transport.h"

/**
 * @brief Crystal/oscillator fitted on the MCP2518FD board.
 *
 * The ScalpelSpace SPI-CAN breakout uses a 40 MHz crystal (the default).
 */
enum ScalpelBusOscillator {
  SCALPEL_BUS_OSC_4MHZ = 0,
  SCALPEL_BUS_OSC_20MHZ = 1,
  SCALPEL_BUS_OSC_40MHZ = 2,
};

/**
 * @brief Polled MCP2518FD transport.
 *
 * Runs the controller in classic CAN (2.0B) mode without an interrupt line:
 * ScalpelBus::poll() services the controller over SPI. This keeps wiring to
 * power, SPI and CS only, matching the ScalpelSpace SPI-CAN breakout.
 */
class Mcp2518fdTransport : public ScalpelCanTransport {
public:
  /**
   * @param csPin SPI chip-select pin wired to the breakout.
   * @param spi SPI bus instance (defaults to SPI).
   * @param oscillator Crystal fitted on the board (defaults to 40 MHz).
   */
  explicit Mcp2518fdTransport(
      uint8_t csPin, SPIClass &spi = SPI,
      ScalpelBusOscillator oscillator = SCALPEL_BUS_OSC_40MHZ);

  bool begin(uint32_t bitrateBps) override;
  bool send(const ScalpelCanFrame &frame) override;
  bool receive(ScalpelCanFrame &frame) override;
  void service() override;

  /**
   * @brief ACAN2517 error code from the last begin() call (0 = success).
   * See ACAN2517.h kXxx constants for bit meanings.
   */
  uint32_t lastErrorCode() const override { return _beginErrorCode; }

private:
  ACAN2517 _can;
  SPIClass &_spi;
  ScalpelBusOscillator _oscillator;
  uint32_t _beginErrorCode;
};

#endif
