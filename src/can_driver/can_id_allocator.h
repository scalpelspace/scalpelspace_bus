/*******************************************************************************
 * VENDORED FILE - DO NOT EDIT.
 * Source: https://github.com/scalpelspace/can_driver
 * Version: 72e4d5d (ref: v0.4.0)
 * Synced by CI tooling.
 *******************************************************************************
 */

/*******************************************************************************
 * @file can_id_allocator.h
 * @brief CAN ID standard (allocator) for ScalpelSpace specific nodes.
 *******************************************************************************
 */

#ifndef CAN_DRIVER__CAN_ID_ALLOCATOR_H
#define CAN_DRIVER__CAN_ID_ALLOCATOR_H

/** Includes. *****************************************************************/

#include "can_driver.h"
#include "can_id.h"
#include "can_id_allocatee.h" // Included for can_tx_func_t type definition.
#include <stdbool.h>
#include <stdint.h>

/** Public types. *************************************************************/

/**
 * @brief Define function pointer type for post allocatee node ID assignment.
 *
 * @param uids_0
 * @param uids_1
 * @param uids_2
 * @param node_ids
 * @param node_count
 */
typedef void (*allocator_assigned_func_t)(
    uint16_t uids_0[CAN_ID_MAX_NODES], uint16_t uids_1[CAN_ID_MAX_NODES],
    uint16_t uids_2[CAN_ID_MAX_NODES], can_node_id_t node_ids[CAN_ID_MAX_NODES],
    can_node_id_t node_count);

/**
 * @brief Node ID assignment strategy function pointer.
 *
 * Called once the discovery phase ends. Implementations receive the full list
 * of discovered UID hashes and must populate node_ids_out[] with the Node ID
 * to assign to each node (index-aligned with the uid arrays).
 *
 * Rules for implementations:
 *   - node_ids_out[i] must be in [1 .. 30] (0 = unassigned, 31 = broadcast).
 *   - Each assigned Node ID must be unique across all indices.
 *   - node_count is guaranteed to be <= CAN_ID_MAX_NODES.
 *
 * @param uids_0 Array of UID bits  0..15, length = node_count.
 * @param uids_1 Array of UID bits 16..31, length = node_count.
 * @param uids_2 Array of UID bits 32..47, length = node_count.
 * @param node_count Number of discovered nodes.
 * @param node_ids_out Output array to fill with assigned Node IDs,
 *                     length = node_count.
 */
typedef void (*node_id_assignment_strategy_t)(
    const uint16_t uids_0[CAN_ID_MAX_NODES],
    const uint16_t uids_1[CAN_ID_MAX_NODES],
    const uint16_t uids_2[CAN_ID_MAX_NODES], uint8_t node_count,
    can_node_id_t node_ids_out[CAN_ID_MAX_NODES]);

typedef struct allocator_config {
  can_tx_func_t can_tx_func; // CAN message transmit function pointer.
  allocator_assigned_func_t
      allocator_assigned_func;            // Allocation success callback.
  node_id_assignment_strategy_t strategy; // Node ID assignment strategy.
} allocator_config_t;

/** Public functions. *********************************************************/

/**
 * @brief CAN RX callback function for allocator advertise message processing.
 *
 * @param header
 * @param data
 */
void can_rx_can_id_allocator_advertise(const can_header_t *header,
                                       const uint8_t *data);

/**
 * @brief CAN RX callback function for allocator ack message processing.
 *
 * @param header
 * @param data
 */
void can_rx_can_id_allocator_ack(const can_header_t *header,
                                 const uint8_t *data);

/**
 * @brief Begin (or restart) the CAN ID allocator state machine.
 *
 * Safe to call from any state. If the allocator is currently mid-session
 * (e.g. stalled in ALLOCATOR_AWAIT_ACK), calling this resets all state and
 * begins a fresh discovery session with an incremented session_id, causing
 * in-flight messages from the previous session to be discarded.
 *
 * @param allocator
 *
 * @return Always true.
 */
bool can_id_allocator_start(allocator_config_t allocator);

/**
 * @brief Manually end CAN ID allocator discovery state.
 *
 * @return Success status.
 * @retval true -> Allocator ended discovery successfully.
 * @retval false -> Error.
 */
bool can_id_allocator_end_discovery(void);

/**
 * @brief FIFO strategy: assign Node IDs 1..N in discovery (arrival) order.
 *
 * The first node to advertise gets Node ID 1, the second gets Node ID 2, etc.
 * This is the default when allocator_config_t::strategy is NULL.
 */
void can_id_strategy_fifo(const uint16_t uids_0[CAN_ID_MAX_NODES],
                          const uint16_t uids_1[CAN_ID_MAX_NODES],
                          const uint16_t uids_2[CAN_ID_MAX_NODES],
                          uint8_t node_count,
                          can_node_id_t node_ids_out[CAN_ID_MAX_NODES]);

/**
 * @brief UID-ascending strategy: sort nodes by their 48-bit UID value and
 * assign Node IDs 1..N in that ascending order.
 *
 * Produces a deterministic assignment regardless of advertisement timing,
 * which is useful when the same physical hardware must always receive the
 * same Node ID across allocation sessions.
 */
void can_id_strategy_uid_ascending(
    const uint16_t uids_0[CAN_ID_MAX_NODES],
    const uint16_t uids_1[CAN_ID_MAX_NODES],
    const uint16_t uids_2[CAN_ID_MAX_NODES], uint8_t node_count,
    can_node_id_t node_ids_out[CAN_ID_MAX_NODES]);

/**
 * @brief UID table strategy: assign a fixed Node ID to each known UID.
 *
 * Nodes whose UID appears in the table receive the mapped Node ID. Nodes
 * not present in the table are assigned Node IDs sequentially starting
 * from the first free slot above the highest table-mapped ID. This
 * ensures unknown nodes still receive a valid (if arbitrary) assignment
 * rather than being skipped entirely.
 *
 * Must be configured via @ref can_id_strategy_uid_table_set before use.
 */
void can_id_strategy_uid_table(const uint16_t uids_0[CAN_ID_MAX_NODES],
                               const uint16_t uids_1[CAN_ID_MAX_NODES],
                               const uint16_t uids_2[CAN_ID_MAX_NODES],
                               uint8_t node_count,
                               can_node_id_t node_ids_out[CAN_ID_MAX_NODES]);

/**
 * @brief Entry in the UID -> Node ID lookup table used by
 * @ref can_id_strategy_uid_table.
 */
typedef struct {
  uint16_t uid_0;        // UID bits  0..15.
  uint16_t uid_1;        // UID bits 16..31.
  uint16_t uid_2;        // UID bits 32..47.
  can_node_id_t node_id; // Desired Node ID (must be in [1 .. 30]).
} can_id_uid_table_entry_t;

/**
 * @brief Configure the lookup table used by @ref can_id_strategy_uid_table.
 *
 * Call this once (e.g. at system init) before starting the allocator.
 * The table is copied by reference; the caller must keep the array alive
 * for the duration of the allocation session.
 *
 * @param entries Pointer to an array of UID->Node ID mappings.
 * @param count Number of entries in the array.
 */
void can_id_strategy_uid_table_set(const can_id_uid_table_entry_t *entries,
                                   uint8_t count);

/**
 * @brief Run CAN ID allocator state machine.
 */
void can_id_allocator_state_machine(void);

#endif
