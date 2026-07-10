/*******************************************************************************
 * @file mcp2518fd_transport.cpp
 * @brief MCP2518FD (SPI-CAN breakout) transport backed by the ACAN2517 driver.
 *******************************************************************************
 */

#include "mcp2518fd_transport.h"

static ACAN2517Settings::Oscillator
to_acan_oscillator(const ScalpelBusOscillator oscillator) {
  switch (oscillator) {
  case SCALPEL_BUS_OSC_4MHZ:
    return ACAN2517Settings::OSC_4MHz;
  case SCALPEL_BUS_OSC_20MHZ:
    return ACAN2517Settings::OSC_20MHz;
  case SCALPEL_BUS_OSC_40MHZ:
  default:
    return ACAN2517Settings::OSC_40MHz;
  }
}

Mcp2518fdTransport::Mcp2518fdTransport(const uint8_t csPin, SPIClass &spi,
                                       const ScalpelBusOscillator oscillator)
    : _can(csPin, spi, 255), // 255 = no INT pin, polled operation.
      _spi(spi), _oscillator(oscillator), _beginErrorCode(0) {}

bool Mcp2518fdTransport::begin(const uint32_t bitrateBps) {
  // ACAN2517 does not initialize the SPI peripheral itself (its demos call
  // SPI.begin() from the sketch); without this the controller readback
  // check fails and begin() reports an error.
  _spi.begin();

  ACAN2517Settings settings(to_acan_oscillator(_oscillator), bitrateBps);
  settings.mRequestedMode = ACAN2517Settings::Normal20B;
#if defined(__AVR__)
  // Shrink the heap-allocated driver FIFOs (default 32 RX + 16 TX
  // CANMessages is ~768 bytes) to fit AVR-class SRAM.
#if RAMEND <= 0x8FF
  // 2 KB SRAM parts (Uno, classic Nano): leave room for the stack.
  settings.mDriverReceiveFIFOSize = 4;
  settings.mDriverTransmitFIFOSize = 2;
#else
  settings.mDriverReceiveFIFOSize = 8;
  settings.mDriverTransmitFIFOSize = 4;
#endif
#endif
  _beginErrorCode = _can.begin(settings, NULL); // NULL ISR = polled.
  return _beginErrorCode == 0;
}

bool Mcp2518fdTransport::send(const ScalpelCanFrame &frame) {
  CANMessage message;
  message.id = frame.id;
  message.ext = false;
  message.rtr = false;
  message.len = frame.dlc;
  for (uint8_t i = 0; i < frame.dlc && i < 8; i++) {
    message.data[i] = frame.data[i];
  }
  return _can.tryToSend(message);
}

bool Mcp2518fdTransport::receive(ScalpelCanFrame &frame) {
  CANMessage message;
  while (_can.receive(message)) {
    if (message.ext || message.rtr) {
      continue; // Protocol uses standard-ID data frames only.
    }
    frame.id = (uint16_t)(message.id & 0x7FFu);
    frame.dlc = message.len;
    for (uint8_t i = 0; i < 8; i++) {
      frame.data[i] = (i < message.len) ? message.data[i] : 0u;
    }
    return true;
  }
  return false;
}

void Mcp2518fdTransport::service() { _can.poll(); }
