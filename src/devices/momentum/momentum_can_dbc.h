/*******************************************************************************
 * VENDORED FILE - DO NOT EDIT.
 * Source: https://github.com/scalpelspace/momentum_driver
 * Version: 5415d3b (ref: v0.4.2)
 * Synced by CI tooling.
 *******************************************************************************
 */

/*******************************************************************************
 * @file momentum_can_dbc.h
 * @brief Auto-generated CAN message definitions from DBC file.
 *******************************************************************************
 */

#ifndef MOMENTUM_CAN_DBC_H
#define MOMENTUM_CAN_DBC_H

/** Includes. *****************************************************************/

#include "can_driver/can_driver.h"

/** CPP guard open. ***********************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/** DBC message index enum. **************************************************/

typedef enum {
  MOMENTUM_CAN_DBC_IDX_STATE = 0,
  MOMENTUM_CAN_DBC_IDX_BAROMETRIC = 1,
  MOMENTUM_CAN_DBC_IDX_GNSS1 = 2,
  MOMENTUM_CAN_DBC_IDX_GNSS2 = 3,
  MOMENTUM_CAN_DBC_IDX_GNSS3 = 4,
  MOMENTUM_CAN_DBC_IDX_QUATERNION = 5,
  MOMENTUM_CAN_DBC_IDX_GYROSCOPE = 6,
  MOMENTUM_CAN_DBC_IDX_MAGNETOMETER = 7,
  MOMENTUM_CAN_DBC_IDX_ACCELEROMETER = 8,
  MOMENTUM_CAN_DBC_IDX_LINEAR_ACCELEROMETER = 9,
  MOMENTUM_CAN_DBC_IDX_GRAVITY_ACCELEROMETER = 10,
  MOMENTUM_CAN_DBC_IDX_DATETIME_GET = 11,
  MOMENTUM_CAN_DBC_IDX_DATETIME_RESPONSE = 12,
  MOMENTUM_CAN_DBC_IDX_GNSS_UTC_GET = 13,
  MOMENTUM_CAN_DBC_IDX_GNSS_UTC_RESPONSE = 14,
  MOMENTUM_CAN_DBC_IDX_RGB_LED_SET = 15,
  MOMENTUM_CAN_DBC_IDX_VERSION_GET = 16,
  MOMENTUM_CAN_DBC_IDX_VERSION_RESPONSE = 17,

  MOMENTUM_CAN_DBC_IDX_COUNT // Total message count.
} momentum_can_dbc_index_t;

/** Public variables. *********************************************************/

extern const can_message_t momentum_dbc_messages[];

/** CPP guard close. **********************************************************/

#ifdef __cplusplus
}
#endif

#endif // MOMENTUM_CAN_DBC_H
