/*******************************************************************************
 * VENDORED FILE - DO NOT EDIT.
 * Source: https://github.com/scalpelspace/can_driver
 * Version: 972bdec (ref: v0.5.0)
 * Synced by CI tooling.
 *******************************************************************************
 */

/*******************************************************************************
 * @file can_id_allocator.c
 * @brief CAN ID standard (allocator) for ScalpelSpace specific nodes.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "can_id_allocator.h"
#include "can_id.h"
#include "can_id_allocation_dbc.h"
#include <stdint.h>
#include <string.h>

/** Private types. ************************************************************/

typedef enum {
  ALLOCATOR_IDLE = 0,     // Allocator not started via can_id_allocator_start().
  ALLOCATOR_DISCOVER = 1, // Transmitting discovery command to allocatee(s).
  ALLOCATOR_AWAIT_ADVERTISE = 2, // Awaiting allocatee advertisements.
  ALLOCATOR_ASSIGN = 3,          // Assigning discovered allocatee(s).
  ALLOCATOR_AWAIT_ACK = 4,       // Awaiting Node ID acknowledgements.
} allocator_state_t;

/** Private variables. ********************************************************/

// CAN ID allocator configuration.
static allocator_config_t config = {NULL};

// State machine variable for allocator.
static allocator_state_t allocator_state = ALLOCATOR_IDLE;
static uint8_t session_id = 0; // Expected session ID.

// Counters for node management.
static uint8_t discovered_nodes = 0;       // Nodes found in discovery.
static uint8_t soft_assigned_nodes = 0;    // Nodes attempted assignment.
static uint8_t assignment_acked_nodes = 0; // Nodes that ACKed assignment.

// Arrays indexed by received order of UIDs (discovered_nodes):
// Discovered UID hash48s (0..15).
static uint16_t discovered_uids_0[CAN_ID_MAX_NODES] = {0};
// Discovered UID hash48s (16..31).
static uint16_t discovered_uids_1[CAN_ID_MAX_NODES] = {0};
// Discovered UID hash48s (32..47).
static uint16_t discovered_uids_2[CAN_ID_MAX_NODES] = {0};
// Assigning Node ID value (awaiting ACK).
static can_node_id_t assigning_node_ids[CAN_ID_MAX_NODES] = {0};

// ACKed Node IDs, index-aligned with the discovered UID arrays.
// 0 (CAN_ID_NODE_ID_UNASSIGNED) = node has not ACKed.
static can_node_id_t acked_node_ids[CAN_ID_MAX_NODES] = {0};

// UID table strategy state.
static const can_id_uid_table_entry_t *uid_table_entries = NULL;
static uint8_t uid_table_count = 0;

/** Private functions. ********************************************************/

/**
 * @brief Search for matching UIDs (allocator ACK verification).
 *
 * @param uid_0 UID hash48 (0..15) to check for match.
 * @param uid_1 UID hash48 (16..31) to check for match.
 * @param uid_2 UID hash48 (32..47) to check for match.
 */
static int16_t search_received_uids(const uint16_t uid_0, const uint16_t uid_1,
                                    const uint16_t uid_2) {
  for (uint8_t i = 0; i < discovered_nodes; i++) {
    if ((discovered_uids_0[i] == uid_0) && (discovered_uids_1[i] == uid_1) &&
        (discovered_uids_2[i] == uid_2)) {
      return i;
    }
  }
  return -1;
}

/**
 * @brief Invoke the configured assignment strategy and store results.
 *
 * Falls back to @ref can_id_strategy_fifo when no strategy is configured.
 */
static void node_id_strategy(void) {
  const node_id_assignment_strategy_t strategy =
      config.strategy ? config.strategy : can_id_strategy_fifo;

  strategy(discovered_uids_0, discovered_uids_1, discovered_uids_2,
           discovered_nodes, assigning_node_ids);
}

/** Public functions. *********************************************************/

bool can_id_allocator_start(const allocator_config_t allocator) {
  if (allocator.can_tx_func == NULL)
    return false; // Transmit function is required to run the protocol.

  config = allocator; // Update configuration.
  allocator_state = ALLOCATOR_DISCOVER;
  discovered_nodes = 0;
  soft_assigned_nodes = 0;
  assignment_acked_nodes = 0;
  memset(acked_node_ids, 0, sizeof(acked_node_ids));
  return true;
}

bool can_id_allocator_end_discovery(void) {
  if (allocator_state == ALLOCATOR_AWAIT_ADVERTISE) {
    allocator_state = ALLOCATOR_ASSIGN;
    return true;
  }
  return false;
}

void can_rx_can_id_allocator_advertise(const can_header_t *header,
                                       const uint8_t *data) {
  const can_message_t msg =
      allocation_dbc[CAN_ID_ALLOCATION_DBC_IDX_NODE_ID_ADVERTISE];

  if (!header || !data)
    return;
  if (allocator_state != ALLOCATOR_AWAIT_ADVERTISE)
    return;

  // Ensure maximum Node count is not reached.
  if (discovered_nodes >= CAN_ID_MAX_NODES)
    return;

  // Ensure CAN header consistency.
  if (header->standard_id != msg.message_id || header->dlc != msg.dlc)
    return;

  // Decode fields.
  const uint16_t rx_uid_0 = (uint16_t)decode_signal(&msg.signals[0], data);
  const uint16_t rx_uid_1 = (uint16_t)decode_signal(&msg.signals[1], data);
  const uint16_t rx_uid_2 = (uint16_t)decode_signal(&msg.signals[2], data);
  const uint8_t rx_session_id = (uint8_t)decode_signal(&msg.signals[3], data);

  // Session must match.
  if (session_id != rx_session_id)
    return;

  // Ignore duplicate ADVERTISE (e.g. retransmission) from a known UID.
  if (search_received_uids(rx_uid_0, rx_uid_1, rx_uid_2) >= 0)
    return;

  // Store UID.
  discovered_uids_0[discovered_nodes] = rx_uid_0;
  discovered_uids_1[discovered_nodes] = rx_uid_1;
  discovered_uids_2[discovered_nodes] = rx_uid_2;

  // Increment index.
  discovered_nodes += 1;
}

void can_rx_can_id_allocator_ack(const can_header_t *header,
                                 const uint8_t *data) {
  const can_message_t msg = allocation_dbc
      [CAN_ID_ALLOCATION_DBC_IDX_NODE_ID_ACK_00]; // Reference only message.

  if (!header || !data)
    return;
  if (allocator_state != ALLOCATOR_AWAIT_ACK)
    return;

  // Validate message_id and node_id from CAN ID.
  can_message_id_t rx_msg_id = 0;
  can_node_id_t rx_node_id_in_can_id = 0;
  if (!can_id_unpack(header->standard_id, &rx_msg_id, &rx_node_id_in_can_id))
    return;
  if (rx_msg_id != (can_message_id_t)CAN_MSG_ENUM_ACK)
    return;
  if (rx_node_id_in_can_id == CAN_ID_NODE_ID_UNASSIGNED ||
      rx_node_id_in_can_id == CAN_ID_NODE_ID_BROADCAST)
    return;

  // DLC must match.
  if (header->dlc != msg.dlc)
    return;

  // Decode fields (using ACK message[0] signals as reference for decoding).
  const uint16_t rx_uid_0 = (uint16_t)decode_signal(&msg.signals[0], data);
  const uint16_t rx_uid_1 = (uint16_t)decode_signal(&msg.signals[1], data);
  const uint16_t rx_uid_2 = (uint16_t)decode_signal(&msg.signals[2], data);
  const uint8_t rx_session_id = (uint8_t)decode_signal(&msg.signals[3], data);
  const can_node_id_t rx_node_id_in_data =
      (can_node_id_t)decode_signal(&msg.signals[4], data);

  // Session must match.
  if (session_id != rx_session_id)
    return;

  // Node ID in data must match node ID in CAN ID.
  if (rx_node_id_in_data != rx_node_id_in_can_id)
    return;

  // Find UID and validate match.
  const int16_t uid_index = search_received_uids(rx_uid_0, rx_uid_1, rx_uid_2);
  if (uid_index < 0)
    return;

  // Ignore duplicate ACK (e.g. retransmission) from an already ACKed node.
  if (acked_node_ids[uid_index] != CAN_ID_NODE_ID_UNASSIGNED)
    return;

  // Successful Node ID assignment confirmed.
  acked_node_ids[uid_index] = rx_node_id_in_data;
  assignment_acked_nodes += 1;
}

void can_id_strategy_fifo(const uint16_t uids_0[CAN_ID_MAX_NODES],
                          const uint16_t uids_1[CAN_ID_MAX_NODES],
                          const uint16_t uids_2[CAN_ID_MAX_NODES],
                          const uint8_t node_count,
                          can_node_id_t node_ids_out[CAN_ID_MAX_NODES]) {
  // Suppress unused-parameter warnings; UIDs are not needed for FIFO ordering.
  (void)uids_0;
  (void)uids_1;
  (void)uids_2;

  // Assign Node IDs 1..N in the order nodes were discovered.
  for (uint8_t i = 0; i < node_count; i++) {
    node_ids_out[i] = (can_node_id_t)(i + 1u);
  }
}

void can_id_strategy_uid_ascending(
    const uint16_t uids_0[CAN_ID_MAX_NODES],
    const uint16_t uids_1[CAN_ID_MAX_NODES],
    const uint16_t uids_2[CAN_ID_MAX_NODES], const uint8_t node_count,
    can_node_id_t node_ids_out[CAN_ID_MAX_NODES]) {

  // Build a sortable index array [0..node_count-1].
  uint8_t order[CAN_ID_MAX_NODES];
  for (uint8_t i = 0; i < node_count; i++) {
    order[i] = i;
  }

  // Insertion sort on the 48-bit UID value (MSB segment first: uid_2 > uid_1 >
  // uid_0). Insertion sort is used to keep code size low and avoid dynamic
  // allocation; node_count is at most CAN_ID_MAX_NODES (30) so O(N^2) is fine.
  for (uint8_t i = 1; i < node_count; i++) {
    const uint8_t key = order[i];
    int8_t j = (int8_t)(i - 1);

    // Compare 48-bit UID of order[j] against key.
    while (j >= 0) {
      const uint8_t idx = order[(uint8_t)j];

      // Evaluate uid_2 (bits 32..47) first - most significant segment.
      if (uids_2[idx] > uids_2[key]) {
        order[(uint8_t)(j + 1)] = order[(uint8_t)j];
        j--;
        continue;
      }
      if (uids_2[idx] < uids_2[key])
        break;

      // uid_2 equal - compare uid_1 (bits 16..31).
      if (uids_1[idx] > uids_1[key]) {
        order[(uint8_t)(j + 1)] = order[(uint8_t)j];
        j--;
        continue;
      }
      if (uids_1[idx] < uids_1[key])
        break;

      // uid_1 equal - compare uid_0 (bits 0..15), least significant segment.
      if (uids_0[idx] > uids_0[key]) {
        order[(uint8_t)(j + 1)] = order[(uint8_t)j];
        j--;
        continue;
      }
      break; // uid_0[idx] <= uid_0[key]: insertion point found.
    }

    order[(uint8_t)(j + 1)] = key;
  }

  // Assign Node IDs 1..N in ascending UID order.
  // order[0] is the discovery index of the node with the smallest UID.
  for (uint8_t rank = 0; rank < node_count; rank++) {
    node_ids_out[order[rank]] = (can_node_id_t)(rank + 1u);
  }
}

void can_id_strategy_uid_table(const uint16_t uids_0[CAN_ID_MAX_NODES],
                               const uint16_t uids_1[CAN_ID_MAX_NODES],
                               const uint16_t uids_2[CAN_ID_MAX_NODES],
                               const uint8_t node_count,
                               can_node_id_t node_ids_out[CAN_ID_MAX_NODES]) {

  // Bitmap of Node IDs already claimed (bits 1..30).
  uint32_t claimed = 0u;

  // Pass 1: assign table-mapped nodes and mark their Node IDs as claimed.
  for (uint8_t i = 0; i < node_count; i++) {
    node_ids_out[i] = 0u; // Mark as unresolved.

    if (!uid_table_entries)
      continue;

    for (uint8_t t = 0; t < uid_table_count; t++) {
      if (uid_table_entries[t].uid_0 == uids_0[i] &&
          uid_table_entries[t].uid_1 == uids_1[i] &&
          uid_table_entries[t].uid_2 == uids_2[i]) {
        const can_node_id_t mapped = uid_table_entries[t].node_id;

        // Only accept valid, unclaimed Node IDs.
        if (mapped >= 1u && mapped <= 30u && !(claimed & (1u << mapped))) {
          node_ids_out[i] = mapped;
          claimed |= (1u << mapped);
        }
        break;
      }
    }
  }

  // Pass 2: nodes not resolved by the table get the next free Node ID.
  can_node_id_t next_free = 1u;
  for (uint8_t i = 0; i < node_count; i++) {
    if (node_ids_out[i] != 0u)
      continue; // Already assigned.

    // Advance next_free past any claimed slots.
    while (next_free <= 30u && (claimed & (1u << next_free))) {
      next_free++;
    }

    if (next_free <= 30u) {
      node_ids_out[i] = next_free;
      claimed |= (1u << next_free);
      next_free++;
    }
    // If next_free > 30 the bus is full; node_ids_out[i] stays 0 (unassigned).
  }
}

void can_id_strategy_uid_table_set(const can_id_uid_table_entry_t *entries,
                                   const uint8_t count) {
  uid_table_entries = entries;
  uid_table_count = count;
}

void can_id_allocator_state_machine(void) {
  uint8_t tx_data[8] = {0};
  can_message_t msg = {0};

  switch (allocator_state) {
  case ALLOCATOR_IDLE:
    // State transition via can_id_allocator_start().
    break;

  case ALLOCATOR_DISCOVER:
    // Increment session ID for new allocation session (discovery) initiated.
    session_id += 1;

    // Pack signals and send.
    msg = allocation_dbc[CAN_ID_ALLOCATION_DBC_IDX_NODE_ID_DISCOVER];
    pack_signal_raw32(&msg.signals[0], tx_data, session_id);
    config.can_tx_func(&msg, tx_data);

    allocator_state = ALLOCATOR_AWAIT_ADVERTISE; // State transition.
    break;

  case ALLOCATOR_AWAIT_ADVERTISE:
    // TODO: Add a software timed limit/watchdog of some kind?

    // Transition state if max nodes is found.
    if (discovered_nodes >= CAN_ID_MAX_NODES) {
      allocator_state = ALLOCATOR_ASSIGN;
    }

    // Timed state transition via can_id_allocator_end_discovery().
    break;

  case ALLOCATOR_ASSIGN:
    // Run Node ID assignment strategy, assignments set in assigning_node_ids.
    node_id_strategy();

    // Transmit the assignment with each related UID.
    for (uint8_t i = 0; i < discovered_nodes; i++) {
      // Skip nodes the strategy could not resolve (no free Node ID).
      if (assigning_node_ids[i] == CAN_ID_NODE_ID_UNASSIGNED)
        continue;

      // Pack signals and send.
      msg = allocation_dbc[CAN_ID_ALLOCATION_DBC_IDX_NODE_ID_ASSIGN];
      pack_signal_raw32(&msg.signals[0], tx_data, discovered_uids_0[i]);
      pack_signal_raw32(&msg.signals[1], tx_data, discovered_uids_1[i]);
      pack_signal_raw32(&msg.signals[2], tx_data, discovered_uids_2[i]);
      pack_signal_raw32(&msg.signals[3], tx_data, session_id);
      pack_signal_raw32(&msg.signals[4], tx_data, assigning_node_ids[i]);
      pack_signal_raw32(&msg.signals[5], tx_data, 0u);
      config.can_tx_func(&msg, tx_data);

      memset(tx_data, 0, sizeof(tx_data)); // Clear TX data buffer.

      soft_assigned_nodes += 1; // Increment assigned, but not yet ACKed count.
    }

    allocator_state = ALLOCATOR_AWAIT_ACK; // State transition.
    break;

  case ALLOCATOR_AWAIT_ACK:
    // TODO: Add a software timed limit/watchdog of some kind?

    // Transition state once all previously assigned nodes ACK.
    if (soft_assigned_nodes == assignment_acked_nodes) {

      // State transition.
      allocator_state = ALLOCATOR_IDLE;

      // Call assignment complete callback (optional).
      if (config.allocator_assigned_func) {
        config.allocator_assigned_func(discovered_uids_0, discovered_uids_1,
                                       discovered_uids_2, acked_node_ids,
                                       discovered_nodes);
      }
    }
    break;

  default:
    break;
  }
}
