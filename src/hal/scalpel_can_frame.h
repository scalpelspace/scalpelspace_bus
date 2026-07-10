/*******************************************************************************
 * @file scalpel_can_frame.h
 * @brief Minimal classic CAN frame type used across the library.
 *******************************************************************************
 */

#ifndef SCALPELSPACE_BUS__SCALPEL_CAN_FRAME_H
#define SCALPELSPACE_BUS__SCALPEL_CAN_FRAME_H

#include <stdint.h>

/**
 * @brief Classic CAN data frame with a standard 11-bit identifier.
 *
 * The ScalpelSpace CAN protocol uses standard 11-bit IDs and classic (non-FD)
 * data frames only, so extended IDs and remote frames are not represented.
 */
struct ScalpelCanFrame {
  uint16_t id;     // Standard 11-bit CAN ID [0, 0x7FF].
  uint8_t dlc;     // Data length [0, 8].
  uint8_t data[8]; // Frame payload.
};

#endif
