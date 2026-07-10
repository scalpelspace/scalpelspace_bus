/*******************************************************************************
 * VENDORED FILE - DO NOT EDIT.
 * Source: https://github.com/scalpelspace/can_driver
 * Version: 72e4d5d (ref: v0.4.0)
 * Synced by CI tooling.
 *******************************************************************************
 */

/*******************************************************************************
 * @file can_driver.h
 * @brief Low level simplified CAN bus communication drivers.
 *******************************************************************************
 */

#ifndef __CAN_DRIVER_H
#define __CAN_DRIVER_H

/** Includes. *****************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/** CPP guard open. ***********************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/** Public types. *************************************************************/

typedef struct {
  uint32_t standard_id; // Standard is [0, 0x7FF].
  uint32_t extended_id; // Extended is [0, 0x1FFFFFFF].
  uint8_t ide;          // 0 = standard ID frame, 1 = extended ID frame.
  uint8_t dlc;          // Frame length, classic is [0, 8], CAN FD is [0, 64].
  uint8_t rtr;          // 0 = data frame, 1 = remote frame (request for data).
} can_header_t;

/**
 * @brief Define function pointer types for processing CAN bus messages.
 *
 * Two distinct handlers are defined:
 *   1. can_rx_handler_t: For decoding received CAN messages.
 *   2. can_tx_handler_t: For encoding data into CAN messages for transmission.
 */
typedef void (*can_rx_handler_t)(can_header_t *header, uint8_t *data);
typedef void (*can_tx_handler_t)(uint8_t *data_out);

/**
 * @brief Enumeration for CAN signal byte order.
 */
typedef enum {
  CAN_LITTLE_ENDIAN = 0, // Little Endian byte order.
  CAN_BIG_ENDIAN = 1     // Big Endian byte order.
} can_byte_order_t;

/**
 * @brief Enumeration for CAN signal multiplexing roles.
 *
 * Simple multiplexing only (single selector per message, no nested or
 * extended multiplexing).
 */
typedef enum {
  CAN_MUX_NONE = 0,     // Always-present (non-multiplexed) signal.
  CAN_MUX_SELECTOR = 1, // Multiplexor switch signal (DBC "M").
  CAN_MUX_DEPENDENT = 2 // Multiplexed signal (DBC "mN"), valid only when the
                        // selector's raw value equals mux_value.
} can_mux_role_t;

/**
 * @brief Struct defining a CAN message signal.
 *
 * This struct describes an individual signal within a CAN message.
 * It includes fields for bit-position, length, scaling, and validation.
 * An optional name field aids in debugging and logging.
 */
typedef struct {
  const char *name;            // Optional signal identifier (for debugging).
  uint8_t start_bit;           // Start bit-position (0-63 for 8-byte CAN).
  uint8_t bit_length;          // Length of the signal in bits.
  can_byte_order_t byte_order; // Byte order: little or big endian.
  bool is_signed;              // Mark for signed or unsigned type.
  float scale;                 // Scaling factor.
  float offset;                // Offset to apply after scaling.
  float min_value;             // Minimum physical value (optional validation).
  float max_value;             // Maximum physical value (optional validation).
  can_mux_role_t mux_role;     // Multiplexing role (CAN_MUX_NONE if unused).
  uint32_t mux_value; // Selector raw value activating this signal. Only
                      // meaningful when mux_role == CAN_MUX_DEPENDENT.
} can_signal_t;

/**
 * @brief Struct defining a CAN message configuration.
 *
 * This struct holds the static configuration for a CAN message, including its
 * ID, optional name, data length, and the associated handler functions. Signals
 * for the message are referenced via a pointer to a statically allocated,
 * exactly-sized const array (typically emitted by the DBC code generator).
 */
typedef struct {
  const char *name;    // Optional message name (for debugging).
  uint32_t message_id; // CAN message ID.
  uint32_t id_mask; // ID mask for filtering (supports ranges or specific IDs).
  uint8_t dlc;      // Data Length Code.
  can_rx_handler_t rx_handler; // Function pointer for receiving (decoding).
  can_tx_handler_t tx_handler; // Function pointer for transmitting (encoding).
  const can_signal_t *signals; // Statically allocated signal array.
  uint8_t signal_count;        // Number of valid signals in the array.
} can_message_t;

/** Public functions. *********************************************************/

/**
 * @brief Normalize physical values to uint32_t raw values.
 *
 * @param physical_value Physical double type value.
 * @param signal Reference signal to ensure clamping and normalization.
 *
 * @return Normalized raw uint32_t data.
 */
uint32_t physical_to_raw(double physical_value, const can_signal_t *signal);

/**
 * @brief Packs an uint32_t value into the CAN message data buffer.
 *
 * @param signal Pointer to the signal definition.
 * @param data Pointer to the CAN data array.
 * @param raw_value The physical value to encode.
 */
void pack_signal_raw32(const can_signal_t *signal, uint8_t *data,
                       uint32_t raw_value);

/**
 * @brief Extract a signal value from a CAN message payload.
 *
 * This function extracts the raw signal value from the provided data array
 * using the specified bit-position, length, and byte order. It then applies
 * scaling and offset to convert the raw value into a physical value.
 *
 * @param signal Pointer to the CAN signal configuration.
 * @param data Pointer to the raw CAN message payload.
 *
 * @return The decoded physical signal value.
 */
double decode_signal(const can_signal_t *signal, const uint8_t *data);

/**
 * @brief Extract the raw (unscaled, unsigned) multiplexor value of a message.
 *
 * Searches the message's signals for the CAN_MUX_SELECTOR signal and extracts
 * its raw value from the payload. Scale/offset are intentionally not applied,
 * mux_value comparisons are defined on raw selector values.
 *
 * @param message Pointer to the CAN message configuration.
 * @param data Pointer to the raw CAN message payload.
 * @param out_mux_value Output raw multiplexor value.
 *
 * @return True on success, False if message has no selector or NULL args.
 */
bool can_message_get_mux_value(const can_message_t *message,
                               const uint8_t *data, uint32_t *out_mux_value);

/**
 * @brief Check if a signal is active (present) for a given multiplexor value.
 *
 * Non-multiplexed signals and the selector itself are always active.
 *
 * @param signal Pointer to the CAN signal configuration.
 * @param mux_value Current raw multiplexor value of the message.
 *
 * @return True if the signal should be decoded/encoded, else False.
 */
bool can_signal_is_active(const can_signal_t *signal, uint32_t mux_value);

/** CPP guard close. **********************************************************/

#ifdef __cplusplus
}
#endif

#endif
