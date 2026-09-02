/*******************************************************************************
 * VENDORED FILE - DO NOT EDIT.
 * Source: https://github.com/scalpelspace/can_driver
 * Version: 2287e60 (ref: v0.6.0)
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
#include "can_id_allocatee.h"
#include <stdbool.h>
#include <stdint.h>

/** Public types. *************************************************************/

/**
 * @brief Define function pointer type for post allocatee node ID assignment.
 *
 * @param uids_0 Array of UID bits  0..15, indexed by discovery order.
 * @param uids_1 Array of UID bits 16..31, indexed by discovery order.
 * @param uids_2 Array of UID bits 32..47, indexed by discovery order.
 * @param node_ids ACKed Node IDs, index-aligned with the uid arrays.
 *                 0 (CAN_ID_NODE_ID_UNASSIGNED) = node did not ACK.
 *                 Nodes that advertised CAN_ALLOC_MODE_NOT_REASSIGNABLE are
 *                 never assigned and never ACK, they report the Node ID they
 *                 advertised (0 if they advertised while unassigned).
 * @param node_count Number of discovered nodes (valid length of the arrays).
 */
typedef void (*allocator_assigned_func_t)(
    uint16_t uids_0[CAN_ID_MAX_NODES], uint16_t uids_1[CAN_ID_MAX_NODES],
    uint16_t uids_2[CAN_ID_MAX_NODES], can_node_id_t node_ids[CAN_ID_MAX_NODES],
    can_node_id_t node_count);

/**
 * @brief Discovery results handed to a Node ID assignment strategy.
 *
 * All arrays are index-aligned and hold node_count valid entries.
 */
typedef struct {
  const uint16_t *uids_0; // UID bits  0..15, one entry per discovered node.
  const uint16_t *uids_1; // UID bits 16..31, one entry per discovered node.
  const uint16_t *uids_2; // UID bits 32..47, one entry per discovered node.
  const bool *reserved;   // True where the node advertised
                          // CAN_ALLOC_MODE_NOT_REASSIGNABLE.
  uint8_t node_count;     // Discovered node count (<= CAN_ID_MAX_NODES).
  uint32_t reserved_mask; // Node IDs held by reserved nodes, bit N set means
                          // Node ID N is in use and must not be assigned.
} node_id_assignment_ctx_t;

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
 *   - A Node ID whose bit is set in ctx->reserved_mask is already held by a
 *     node that refuses reassignment and must not be handed out. The allocator
 *     drops any assignment that violates this.
 *   - Entries where ctx->reserved[i] is true keep the Node ID they advertised.
 *     No ASSIGN is transmitted for them, leave node_ids_out[i] at 0.
 *   - ctx->node_count is guaranteed to be <= CAN_ID_MAX_NODES.
 *
 * @param ctx Discovery results for the session.
 * @param node_ids_out Output array to fill with assigned Node IDs,
 *                     length = ctx->node_count.
 */
typedef void (*node_id_assignment_strategy_t)(
    const node_id_assignment_ctx_t *ctx,
    can_node_id_t node_ids_out[CAN_ID_MAX_NODES]);

typedef struct allocator_config {
  can_tx_func_t can_tx_func; // CAN message transmit function pointer. Required.
  allocator_assigned_func_t
      allocator_assigned_func; // Allocation success callback. Optional (NULL to
                               // skip).
  node_id_assignment_strategy_t strategy; // Node ID assignment strategy.
                                          // Optional (NULL for FIFO).
} allocator_config_t;

/** Public functions. *********************************************************/

/**
 * @brief CAN RX callback function for allocator advertise message processing.
 *
 * Accepts the ADVERTISE CAN ID range, 0x720 (unassigned) through 0x73E,
 * broadcast (0x73F) is rejected. The Node ID carried in the CAN ID is the one
 * the advertising node currently holds. A node advertising
 * CAN_ALLOC_MODE_NOT_REASSIGNABLE has that Node ID marked reserved and taken
 * out of the pool offered to the assignment strategy.
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
 * @return Success status.
 * @retval true -> Allocator started.
 * @retval false -> Invalid configuration (can_tx_func is NULL).
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
 * @brief FIFO strategy: assign the lowest free Node IDs in discovery (arrival)
 * order.
 *
 * The first node to advertise gets Node ID 1, the second gets Node ID 2, etc.
 * Node IDs held by reserved nodes are skipped, as are the reserved nodes
 * themselves. This is the default when allocator_config_t::strategy is NULL.
 */
void can_id_strategy_fifo(const node_id_assignment_ctx_t *ctx,
                          can_node_id_t node_ids_out[CAN_ID_MAX_NODES]);

/**
 * @brief UID-ascending strategy: sort nodes by their 48-bit UID value and
 * assign the lowest free Node IDs in that ascending order.
 *
 * Produces a deterministic assignment regardless of advertisement timing,
 * which is useful when the same physical hardware must always receive the
 * same Node ID across allocation sessions.
 */
void can_id_strategy_uid_ascending(
    const node_id_assignment_ctx_t *ctx,
    can_node_id_t node_ids_out[CAN_ID_MAX_NODES]);

/**
 * @brief UID table strategy: assign a fixed Node ID to each known UID.
 *
 * Nodes whose UID appears in the table receive the mapped Node ID. Nodes
 * not present in the table are assigned the lowest free (unclaimed) Node
 * IDs sequentially. This ensures unknown nodes still receive a valid (if
 * arbitrary) assignment rather than being skipped entirely, unless no free
 * Node IDs remain (unresolved nodes are left at 0 and skipped).
 *
 * Node IDs held by reserved nodes count as claimed, so a table entry mapping
 * to one of them is dropped and that node falls back to the lowest free ID.
 *
 * Must be configured via @ref can_id_strategy_uid_table_set before use.
 */
void can_id_strategy_uid_table(const node_id_assignment_ctx_t *ctx,
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
