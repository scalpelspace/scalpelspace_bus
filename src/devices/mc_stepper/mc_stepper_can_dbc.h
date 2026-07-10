/*******************************************************************************
 * VENDORED FILE - DO NOT EDIT.
 * Source: https://github.com/scalpelspace/mc_stepper_driver
 * Version: c953fbb (ref: v0.2.2)
 * Synced by CI tooling.
 *******************************************************************************
 */

/*******************************************************************************
 * @file mc_stepper_can_dbc.h
 * @brief Auto-generated CAN message definitions from DBC file.
 *******************************************************************************
 */

#ifndef MC_STEPPER_CAN_DBC_H
#define MC_STEPPER_CAN_DBC_H

/** Includes. *****************************************************************/

#include "can_driver/can_driver.h"

/** CPP guard open. ***********************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/** DBC message index enum. **************************************************/

typedef enum {
  MC_STEPPER_CAN_DBC_IDX_STATE = 0,
  MC_STEPPER_CAN_DBC_IDX_STALLGUARD_EVENT = 1,
  MC_STEPPER_CAN_DBC_IDX_COMMAND_STEPPER = 2,
  MC_STEPPER_CAN_DBC_IDX_COMMAND_STEPPER_ZERO = 3,
  MC_STEPPER_CAN_DBC_IDX_SENSOR = 4,
  MC_STEPPER_CAN_DBC_IDX_CONTROLS_DIAGNOSTIC = 5,
  MC_STEPPER_CAN_DBC_IDX_MOTION_CONFIG_SET = 6,
  MC_STEPPER_CAN_DBC_IDX_MOTION_CONFIG_GET = 7,
  MC_STEPPER_CAN_DBC_IDX_MOTION_CONFIG_RESPONSE = 8,
  MC_STEPPER_CAN_DBC_IDX_CURRENT_CONFIG_SET = 9,
  MC_STEPPER_CAN_DBC_IDX_CURRENT_CONFIG_GET = 10,
  MC_STEPPER_CAN_DBC_IDX_CURRENT_CONFIG_RESPONSE = 11,
  MC_STEPPER_CAN_DBC_IDX_CONTROLS_CONFIG_SET = 12,
  MC_STEPPER_CAN_DBC_IDX_CONTROLS_CONFIG_GET = 13,
  MC_STEPPER_CAN_DBC_IDX_CONTROLS_CONFIG_RESPONSE = 14,
  MC_STEPPER_CAN_DBC_IDX_STALLGUARD_SET = 15,
  MC_STEPPER_CAN_DBC_IDX_STALLGUARD_GET = 16,
  MC_STEPPER_CAN_DBC_IDX_STALLGUARD_RESPONSE = 17,
  MC_STEPPER_CAN_DBC_IDX_DATETIME_SET = 18,
  MC_STEPPER_CAN_DBC_IDX_DATETIME_GET = 19,
  MC_STEPPER_CAN_DBC_IDX_DATETIME_RESPONSE = 20,
  MC_STEPPER_CAN_DBC_IDX_RGB_LED_SET = 21,
  MC_STEPPER_CAN_DBC_IDX_VERSION_GET = 22,
  MC_STEPPER_CAN_DBC_IDX_VERSION_RESPONSE = 23,

  MC_STEPPER_CAN_DBC_IDX_COUNT // Total message count.
} mc_stepper_can_dbc_index_t;

/** Public variables. *********************************************************/

extern const can_message_t mc_stepper_dbc_messages[];

/** CPP guard close. **********************************************************/

#ifdef __cplusplus
}
#endif

#endif // MC_STEPPER_CAN_DBC_H
