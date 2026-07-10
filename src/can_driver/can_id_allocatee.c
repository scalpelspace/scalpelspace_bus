/*******************************************************************************
 * VENDORED FILE - DO NOT EDIT.
 * Source: https://github.com/scalpelspace/can_driver
 * Version: 72e4d5d (ref: v0.4.0)
 * Synced by CI tooling.
 *******************************************************************************
 */

/*******************************************************************************
 * @file can_id_allocatee.c
 * @brief CAN ID standard (allocatee) for ScalpelSpace specific nodes.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "can_id_allocatee.h"
#include "can_id.h"
#include "can_id_allocation_dbc.h"
#include <stdint.h>
#include <string.h>

/** Private types. ************************************************************/

typedef enum {
  ALLOCATEE_IDLE = 0, // Allocatee not started via can_id_allocatee_start().
  ALLOCATEE_AWAIT_DISCOVERY = 1,   // Pending discovery command from allocator.
  ALLOCATEE_DISCOVERY_STARTED = 2, // Discovery inited by allocator.
  ALLOCATEE_ADVERTISE = 3,         // Transmitting self advertisement.
  ALLOCATEE_AWAIT_ASSIGNMENT = 4,  // Awaiting Node ID assignment.
  ALLOCATEE_ASSIGNED = 5,          // Node ID assigned.
  ALLOCATEE_ACK = 6,               // Transmitting Node ID acknowledgement.
} allocatee_state_t;

/** Private variables. ********************************************************/

// CAN ID allocatee configuration.
static allocatee_config_t config = {NULL, NULL};

// State machine variable for allocatee.
static allocatee_state_t allocatee_state = ALLOCATEE_IDLE;

static uint8_t session_id = 0; // Expected session ID.
static uint8_t node_id = 0;    // Assigned node ID.

/** Public functions. *********************************************************/

bool can_id_allocatee_start(const allocatee_config_t allocatee) {
  config = allocatee; // Update configuration.
  node_id = 0;
  session_id = 0;
  allocatee_state = ALLOCATEE_AWAIT_DISCOVERY;
  return true;
}

bool can_rx_can_id_allocatee_discovery(const can_header_t *header,
                                       const uint8_t *data) {
  const can_message_t msg =
      allocation_dbc[CAN_ID_ALLOCATION_DBC_IDX_NODE_ID_DISCOVER];

  if (!header || !data)
    return false;
  if (allocatee_state != ALLOCATEE_AWAIT_DISCOVERY)
    return false;

  // Ensure CAN header consistency.
  if (header->standard_id != msg.message_id || header->dlc != msg.dlc)
    return false;

  // Decode field.
  const uint8_t rx_session_id = (uint8_t)decode_signal(&msg.signals[0], data);

  session_id = rx_session_id;                    // Store session ID.
  allocatee_state = ALLOCATEE_DISCOVERY_STARTED; // Transition state.
  return true;
}

void can_rx_can_id_allocatee_assignment(const can_header_t *header,
                                        const uint8_t *data) {
  const can_message_t msg =
      allocation_dbc[CAN_ID_ALLOCATION_DBC_IDX_NODE_ID_ASSIGN];

  if (!header || !data)
    return;
  if (allocatee_state != ALLOCATEE_AWAIT_ASSIGNMENT)
    return;

  // Ensure CAN header consistency.
  if (header->standard_id != msg.message_id || header->dlc != msg.dlc)
    return;

  // Decode fields.
  const uint16_t rx_uid_0 = (uint16_t)decode_signal(&msg.signals[0], data);
  const uint16_t rx_uid_1 = (uint16_t)decode_signal(&msg.signals[1], data);
  const uint16_t rx_uid_2 = (uint16_t)decode_signal(&msg.signals[2], data);
  const uint8_t rx_session_id = (uint8_t)decode_signal(&msg.signals[3], data);
  const uint16_t rx_node_id = (uint16_t)decode_signal(&msg.signals[4], data);

  // UID must match.
  uint16_t uid_0 = 0;
  uint16_t uid_1 = 0;
  uint16_t uid_2 = 0;
  config.get_uid_hash48_func(&uid_0, &uid_1, &uid_2);
  if (uid_0 != rx_uid_0 || uid_1 != rx_uid_1 || uid_2 != rx_uid_2)
    return;

  // Session must match.
  if (session_id != rx_session_id)
    return;

  node_id = rx_node_id;                 // Assign new Node ID.
  allocatee_state = ALLOCATEE_ASSIGNED; // Transition state.
}

void can_id_allocatee_state_machine(void) {
  uint8_t tx_data[8] = {0};
  uint16_t uid_0 = 0;
  uint16_t uid_1 = 0;
  uint16_t uid_2 = 0;
  can_message_t msg = {0};

  switch (allocatee_state) {
  case ALLOCATEE_IDLE:
    // State transition via can_id_allocatee_start().
    break;

  case ALLOCATEE_AWAIT_DISCOVERY:
    // State transition via can_rx_can_id_allocatee_discovery().
    break;

  case ALLOCATEE_DISCOVERY_STARTED:
    // TODO: Software timed limit/watchdog of some kind?
    allocatee_state = ALLOCATEE_ADVERTISE;
    break;

  case ALLOCATEE_ADVERTISE:
    // Calculate UID.
    config.get_uid_hash48_func(&uid_0, &uid_1, &uid_2);

    // Pack signals and send.
    msg = allocation_dbc[CAN_ID_ALLOCATION_DBC_IDX_NODE_ID_ADVERTISE];
    pack_signal_raw32(&msg.signals[0], tx_data, uid_0);
    pack_signal_raw32(&msg.signals[1], tx_data, uid_1);
    pack_signal_raw32(&msg.signals[2], tx_data, uid_2);
    pack_signal_raw32(&msg.signals[3], tx_data, session_id);
    pack_signal_raw32(&msg.signals[4], tx_data, 0u);
    config.can_tx_func(&msg, tx_data);

    // State transition.
    allocatee_state = ALLOCATEE_AWAIT_ASSIGNMENT;
    break;

  case ALLOCATEE_AWAIT_ASSIGNMENT:
    // State transition via can_rx_can_id_allocatee_assignment().
    break;

  case ALLOCATEE_ASSIGNED:
    // TODO: Add a software timed limit/watchdog of some kind?
    allocatee_state = ALLOCATEE_ACK;
    break;

  case ALLOCATEE_ACK:
    // Calculate UID.
    config.get_uid_hash48_func(&uid_0, &uid_1, &uid_2);

    // ACK CAN ID encodes the assigned node_id.
    can_id_t ack_id;
    can_id_pack(CAN_MSG_ENUM_ACK, node_id, &ack_id);

    // Pack signals and send.
    // Use ACK_00 as a signal-definition template, patch the new message ID.
    msg = allocation_dbc[CAN_ID_ALLOCATION_DBC_IDX_NODE_ID_ACK_00];
    msg.message_id = (uint32_t)ack_id;
    pack_signal_raw32(&msg.signals[0], tx_data, uid_0);
    pack_signal_raw32(&msg.signals[1], tx_data, uid_1);
    pack_signal_raw32(&msg.signals[2], tx_data, uid_2);
    pack_signal_raw32(&msg.signals[3], tx_data, session_id);
    pack_signal_raw32(&msg.signals[4], tx_data, node_id);
    pack_signal_raw32(&msg.signals[5], tx_data, 0u);
    config.can_tx_func(&msg, tx_data);

    // State transition.
    allocatee_state = ALLOCATEE_IDLE;

    // Call assignment complete callback.
    config.allocatee_assigned_func(node_id);
    break;

  default:
    break;
  }
}
