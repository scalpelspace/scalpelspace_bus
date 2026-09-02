/*******************************************************************************
 * VENDORED FILE - DO NOT EDIT.
 * Source: https://github.com/scalpelspace/can_driver
 * Version: 2287e60 (ref: v0.6.0)
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
static allocatee_config_t config = {0};

// State machine variable for allocatee.
static allocatee_state_t allocatee_state = ALLOCATEE_IDLE;

static uint8_t session_id = 0; // Expected session ID.
static uint8_t node_id = 0;    // Currently held node ID (0 = unassigned).

/** Public functions. *********************************************************/

bool can_id_allocatee_start(const allocatee_config_t allocatee) {
  if (allocatee.can_tx_func == NULL || allocatee.get_uid_hash48_func == NULL)
    return false; // Transmit and UID functions are required for the protocol.

  // Reserved alloc_mode bits must be transmittable as 0.
  if (allocatee.alloc_mode != CAN_ALLOC_MODE_REASSIGNABLE &&
      allocatee.alloc_mode != CAN_ALLOC_MODE_NOT_REASSIGNABLE)
    return false;

  // Reject reserved (31 = broadcast) and out of range starting Node IDs.
  if (allocatee.node_id > CAN_ID_MAX_NODES)
    return false;

  // A node that refuses reassignment must be told which Node ID it holds,
  // otherwise it has no ID to advertise and none to keep.
  if (allocatee.alloc_mode == CAN_ALLOC_MODE_NOT_REASSIGNABLE &&
      allocatee.node_id == CAN_ID_NODE_ID_UNASSIGNED)
    return false;

  config = allocatee;          // Update configuration.
  node_id = allocatee.node_id; // Hardcoded or previously held Node ID.
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

  // A fixed node keeps its Node ID even if a misbehaving allocator sends an
  // ASSIGN for it (defensive, it never enters ALLOCATEE_AWAIT_ASSIGNMENT).
  if (config.alloc_mode == CAN_ALLOC_MODE_NOT_REASSIGNABLE)
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

  // Reject reserved Node IDs (0 = unassigned, 31 = broadcast) and any value
  // outside the assignable range (1..30).
  if (rx_node_id == CAN_ID_NODE_ID_UNASSIGNED ||
      rx_node_id >= CAN_ID_NODE_ID_BROADCAST)
    return;

  node_id = (can_node_id_t)rx_node_id;  // Assign new Node ID.
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

    // ADVERTISE CAN ID encodes the Node ID currently held (0 = unassigned),
    // so the allocator sees which IDs are already taken on the bus.
    can_id_t advertise_id = 0u;
    if (!can_id_pack(CAN_MSG_ENUM_ADVERTISE, node_id, &advertise_id)) {
      // Unreachable: node_id is range-validated on start and on assignment.
      // Reset rather than transmit a malformed ADVERTISE.
      allocatee_state = ALLOCATEE_IDLE;
      break;
    }

    // Pack signals and send.
    // Use ADVERTISE as a signal-definition template, patch the message ID.
    msg = allocation_dbc[CAN_ID_ALLOCATION_DBC_IDX_NODE_ID_ADVERTISE];
    msg.message_id = (uint32_t)advertise_id;
    pack_signal_raw32(&msg.signals[0], tx_data, uid_0);
    pack_signal_raw32(&msg.signals[1], tx_data, uid_1);
    pack_signal_raw32(&msg.signals[2], tx_data, uid_2);
    pack_signal_raw32(&msg.signals[3], tx_data, session_id);
    // alloc_mode owns all of byte 7. Mask to the defined bit so the reserved
    // bits always transmit as 0, a stray 1 would decode as a future flag.
    pack_signal_raw32(&msg.signals[4], tx_data,
                      (uint32_t)config.alloc_mode & CAN_ALLOC_MODE_MASK);
    config.can_tx_func(&msg, tx_data);

    // State transition. A node that refuses reassignment receives no ASSIGN,
    // its part in the session ends with the advertisement.
    allocatee_state = (config.alloc_mode == CAN_ALLOC_MODE_NOT_REASSIGNABLE)
                          ? ALLOCATEE_AWAIT_DISCOVERY
                          : ALLOCATEE_AWAIT_ASSIGNMENT;
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
    can_id_t ack_id = 0u;
    if (!can_id_pack(CAN_MSG_ENUM_ACK, node_id, &ack_id)) {
      // Unreachable: node_id was range-validated on assignment. Reset rather
      // than transmit a malformed ACK.
      allocatee_state = ALLOCATEE_IDLE;
      break;
    }

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

    // State transition. Return to awaiting discovery (rather than idle) so the
    // node keeps answering DISCOVER now that it holds a Node ID.
    allocatee_state = ALLOCATEE_AWAIT_DISCOVERY;

    // Call assignment complete callback (optional).
    if (config.allocatee_assigned_func) {
      config.allocatee_assigned_func(node_id);
    }
    break;

  default:
    break;
  }
}
