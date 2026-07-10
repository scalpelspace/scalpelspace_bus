/*******************************************************************************
 * VENDORED FILE - DO NOT EDIT.
 * Source: https://github.com/scalpelspace/can_driver
 * Version: 972bdec (ref: v0.5.0)
 * Synced by CI tooling.
 *******************************************************************************
 */

/*******************************************************************************
 * @file can_id_allocation_dbc.h
 * @brief Auto-generated CAN message definitions from DBC file.
 *******************************************************************************
 */

#ifndef CAN_ID_ALLOCATION_DBC_H
#define CAN_ID_ALLOCATION_DBC_H

/** Includes. *****************************************************************/

#include "can_driver/can_driver.h"

/** CPP guard open. ***********************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/** DBC message index enum. **************************************************/

typedef enum {
  CAN_ID_ALLOCATION_DBC_IDX_NODE_ID_DISCOVER = 0,
  CAN_ID_ALLOCATION_DBC_IDX_NODE_ID_ADVERTISE = 1,
  CAN_ID_ALLOCATION_DBC_IDX_NODE_ID_ASSIGN = 2,
  CAN_ID_ALLOCATION_DBC_IDX_NODE_ID_ACK_00 = 3,

  CAN_ID_ALLOCATION_DBC_IDX_COUNT // Total message count.
} can_id_allocation_dbc_index_t;

/** Public variables. *********************************************************/

extern const can_message_t allocation_dbc[];

/** CPP guard close. **********************************************************/

#ifdef __cplusplus
}
#endif

#endif // CAN_ID_ALLOCATION_DBC_H
