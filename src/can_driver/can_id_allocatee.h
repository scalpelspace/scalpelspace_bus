/*******************************************************************************
 * VENDORED FILE - DO NOT EDIT.
 * Source: https://github.com/scalpelspace/can_driver
 * Version: 972bdec (ref: v0.5.0)
 * Synced by CI tooling.
 *******************************************************************************
 */

/*******************************************************************************
 * @file can_id_allocatee.h
 * @brief CAN ID standard (allocatee) for ScalpelSpace specific nodes.
 *******************************************************************************
 */

#ifndef CAN_DRIVER__CAN_ID_ALLOCATEE_H
#define CAN_DRIVER__CAN_ID_ALLOCATEE_H

/** Includes. *****************************************************************/

#include "can_driver.h"
#include "can_id.h"
#include <stdbool.h>
#include <stdint.h>

/** Public types. *************************************************************/

/**
 * @brief Define function pointer type for sending CAN bus messages.
 *
 * @param msg
 * @param data
 *
 * @return Success status.
 * @retval true -> Transmit successful.
 * @retval false -> Transmit unsuccessful.
 */
typedef bool (*can_tx_func_t)(const can_message_t *msg, const uint8_t data[8]);

/**
 * @brief Determine 3x 16-bit (48-bit split) UID hash.
 *
 * @param uid0 UID hash48, 16-bit segment 1 of 3 (bits 0..15 of 48-bit hash).
 * @param uid1 UID hash48, 16-bit segment 2 of 3 (bits 16..31 of 48-bit hash).
 * @param uid2 UID hash48, 16-bit segment 3 of 3 (bits 32..47 of 48-bit hash).
 */
typedef void (*get_uid_hash48_func_t)(uint16_t *uid0, uint16_t *uid1,
                                      uint16_t *uid2);

/**
 * @brief Define function pointer type for post allocatee node ID assignment.
 *
 * @param node_id Assigned node ID.
 */
typedef void (*allocatee_assigned_func_t)(can_node_id_t node_id);

typedef struct allocatee_config {
  can_tx_func_t can_tx_func; // CAN message transmit function pointer. Required.
  get_uid_hash48_func_t get_uid_hash48_func; // Get UID hash48 function pointer.
                                             // Required.
  allocatee_assigned_func_t
      allocatee_assigned_func; // Allocatee success callback. Optional (NULL
                               // to skip).
} allocatee_config_t;

/** Public functions. *********************************************************/

/**
 * @brief CAN RX callback function for allocatee discovery message processing.
 *
 * @param header
 * @param data
 *
 * @return Whether a valid discovery message was accepted and session started.
 * @retval true -> Discovery accepted, application should arm its deadline
 *                 timer from this point.
 * @retval false -> Message ignored (wrong state, bad header, or null args).
 */
bool can_rx_can_id_allocatee_discovery(const can_header_t *header,
                                       const uint8_t *data);

/**
 * @brief CAN RX callback function for allocatee assignment message processing.
 *
 * @param header
 * @param data
 */
void can_rx_can_id_allocatee_assignment(const can_header_t *header,
                                        const uint8_t *data);

/**
 * @brief Begin (or restart) the CAN ID allocatee state machine.
 *
 * Safe to call from any state. If the allocatee is currently mid-session
 * (e.g. stalled in ALLOCATEE_AWAIT_ASSIGNMENT), calling this resets all state
 * and waits for the next DISCOVER broadcast. The session_id and node_id are
 * cleared so stale in-flight messages are discarded.
 *
 * @param allocatee
 *
 * @return Success status.
 * @retval true -> Allocatee started.
 * @retval false -> Invalid configuration (can_tx_func or get_uid_hash48_func
 *                  is NULL).
 */
bool can_id_allocatee_start(allocatee_config_t allocatee);

/**
 * @brief Run CAN ID allocatee state machine.
 */
void can_id_allocatee_state_machine(void);

#endif
