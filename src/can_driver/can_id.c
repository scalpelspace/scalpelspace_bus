/*******************************************************************************
 * VENDORED FILE - DO NOT EDIT.
 * Source: https://github.com/scalpelspace/can_driver
 * Version: 972bdec (ref: v0.5.0)
 * Synced by CI tooling.
 *******************************************************************************
 */

/*******************************************************************************
 * @file can_id.c
 * @brief Custom CAN ID standard for ScalpelSpace specific node devices.
 *******************************************************************************
 * Custom 11-bit CAN ID packing for:
 *   [ 6 bits message_id ][ 5 bits node_id ]
 *
 * Bit layout (MSB -> LSB):
 *    bits 10..5  : message_id  (0..63)   (6 bits)
 *    bits  4..0  : node_id     (0..31)   (5 bits)
 *
 * Result is always an 11-bit standard CAN identifier (0..0x7FF).
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "can_id.h"

/** Definitions. **************************************************************/

// Bit field masks.
#define NODE_ID_MASK ((uint16_t)NODE_MAX)
#define MESSAGE_ID_MASK ((uint16_t)MESSAGE_ID_MAX)

// Shifts.
#define NODE_ID_SHIFT (0u)
#define MESSAGE_ID_SHIFT (NODE_BITS) // 5.

/** Private types. ************************************************************/

enum {
  MESSAGE_ID_BITS = 6u,
  NODE_BITS = 5u,

  NODE_MAX = (1u << NODE_BITS) - 1u,             // 31.
  MESSAGE_ID_MAX = (1u << MESSAGE_ID_BITS) - 1u, // 63.

  STD_MAX = 0x7FFu
};

/** Private functions. ********************************************************/

static bool can_id_fields_valid(const can_message_id_t message_id,
                                const can_node_id_t node_id) {
  if ((uint16_t)message_id > MESSAGE_ID_MAX)
    return false;
  if (node_id > NODE_MAX)
    return false;
  return true;
}

/** Public functions. *********************************************************/

bool can_id_pack(const can_message_id_t message_id, const can_node_id_t node_id,
                 can_id_t *out_can_id) {
  if (out_can_id == NULL)
    return false;
  if (!can_id_fields_valid(message_id, node_id))
    return false;

  can_id_t id = 0u;
  id |= ((can_id_t)(message_id & MESSAGE_ID_MASK) << MESSAGE_ID_SHIFT);
  id |= ((can_id_t)(node_id & NODE_ID_MASK) << NODE_ID_SHIFT);

  // Safety: ensure 11-bit
  id &= STD_MAX;

  *out_can_id = id;
  return true;
}

bool can_id_unpack(const can_id_t can_id, can_message_id_t *out_message_id,
                   can_node_id_t *out_node_id) {
  if (can_id > STD_MAX)
    return false;
  if (out_message_id) {
    *out_message_id =
        (can_message_id_t)((can_id >> MESSAGE_ID_SHIFT) & MESSAGE_ID_MASK);
  }
  if (out_node_id) {
    *out_node_id = (can_node_id_t)((can_id >> NODE_ID_SHIFT) & NODE_ID_MASK);
  }
  return true;
}

bool can_id_is_unassigned(const can_id_t can_id) {
  can_node_id_t node = 0u;
  if (!can_id_unpack(can_id, NULL, &node))
    return false;
  return (node == (can_node_id_t)CAN_ID_NODE_ID_UNASSIGNED);
}

bool can_id_is_broadcast(const can_id_t can_id) {
  can_node_id_t node = 0u;
  if (!can_id_unpack(can_id, NULL, &node))
    return false;
  return (node == (can_node_id_t)CAN_ID_NODE_ID_BROADCAST);
}
