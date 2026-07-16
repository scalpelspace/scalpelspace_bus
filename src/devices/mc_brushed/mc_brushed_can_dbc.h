/*******************************************************************************
 * VENDORED FILE - DO NOT EDIT.
 * Source: https://github.com/scalpelspace/mc_brushed_driver
 * Version: 6df2fa6 (ref: v0.1.1)
 * Synced by CI tooling.
 *******************************************************************************
 */

/*******************************************************************************
 * @file mc_brushed_can_dbc.h
 * @brief Auto-generated CAN message definitions from DBC file.
 *******************************************************************************
 */

#ifndef MC_BRUSHED_CAN_DBC_H
#define MC_BRUSHED_CAN_DBC_H

/** Includes. *****************************************************************/

#include "can_driver/can_driver.h"

/** CPP guard open. ***********************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/** DBC message index enum. **************************************************/

typedef enum {
  MC_BRUSHED_CAN_DBC_IDX_STATE = 0,
  MC_BRUSHED_CAN_DBC_IDX_COMMAND_BRUSHED = 1,
  MC_BRUSHED_CAN_DBC_IDX_COMMAND_BRUSHED_ZERO = 2,
  MC_BRUSHED_CAN_DBC_IDX_SENSOR = 3,
  MC_BRUSHED_CAN_DBC_IDX_CONTROLS_DIAGNOSTIC = 4,
  MC_BRUSHED_CAN_DBC_IDX_CONTROLS_CONFIG_SET = 5,
  MC_BRUSHED_CAN_DBC_IDX_CONTROLS_CONFIG_GET = 6,
  MC_BRUSHED_CAN_DBC_IDX_CONTROLS_CONFIG_RESPONSE = 7,
  MC_BRUSHED_CAN_DBC_IDX_ENCODER_CONFIG_SET = 8,
  MC_BRUSHED_CAN_DBC_IDX_ENCODER_CONFIG_GET = 9,
  MC_BRUSHED_CAN_DBC_IDX_ENCODER_CONFIG_RESPONSE = 10,
  MC_BRUSHED_CAN_DBC_IDX_DATETIME_SET = 11,
  MC_BRUSHED_CAN_DBC_IDX_DATETIME_GET = 12,
  MC_BRUSHED_CAN_DBC_IDX_DATETIME_GET_RESPONSE = 13,
  MC_BRUSHED_CAN_DBC_IDX_RGB_LED_SET = 14,
  MC_BRUSHED_CAN_DBC_IDX_VERSION_GET = 15,
  MC_BRUSHED_CAN_DBC_IDX_VERSION_GET_RESPONSE = 16,

  MC_BRUSHED_CAN_DBC_IDX_COUNT // Total message count.
} mc_brushed_can_dbc_index_t;

/** Public variables. *********************************************************/

extern const can_message_t mc_brushed_dbc_messages[];

/** CPP guard close. **********************************************************/

#ifdef __cplusplus
}
#endif

#endif // MC_BRUSHED_CAN_DBC_H
