/*******************************************************************************
 * @file scalpel_bus_types.h
 * @brief Shared value types used by ScalpelBus and device handles.
 *******************************************************************************
 * Cached telemetry structs carry a lastUpdateMs field: millis() timestamp of
 * the most recent update, or 0 when no frame has been received yet.
 *******************************************************************************
 */

#ifndef SCALPELSPACE_BUS__SCALPEL_BUS_TYPES_H
#define SCALPELSPACE_BUS__SCALPEL_BUS_TYPES_H

#include <stdint.h>

/** @brief Firmware version reported by a node. */
struct ScalpelVersionInfo {
  uint8_t major;
  uint8_t minor;
  uint8_t patch;
  char identifier; // Product-specific ASCII identifier.
  uint32_t lastUpdateMs;
};

/** @brief Date/time as exchanged by the datetime messages. */
struct ScalpelDateTime {
  uint8_t year;    // 0..99.
  uint8_t month;   // 1..12.
  uint8_t date;    // 1..31.
  uint8_t weekday; // 1..7.
  uint8_t hours;
  uint8_t minutes;
  uint8_t seconds;
  uint32_t lastUpdateMs;
};

#endif
