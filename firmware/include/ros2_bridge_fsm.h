/*
 * ros2_bridge_fsm.h
 * Deterministic AUTOSAR E2E Profile 1 Streaming State Machine &
 * Ring-Buffered UART DMA Transport Abstraction for Bare-Metal MCUs
 *
 * Target: STM32 (F4/G4/H7 HAL/LL), ESP32-S3, RP2040
 * Standard: AUTOSAR E2E Profile 1 | ISO 26262 Error Detection (d_H >= 4)
 * Author: Engineer Abdullah Bin Zafar
 * License: Apache-2.0
 */

#ifndef ROS2_BRIDGE_FSM_H
#define ROS2_BRIDGE_FSM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define E2E_SYNC_BYTE_1     0xAAU
#define E2E_SYNC_BYTE_2     0x55U
#define E2E_MAX_PAYLOAD     64U
#define E2E_FRAME_OVERHEAD  6U
#define E2E_MAX_FRAME_SIZE  (E2E_FRAME_OVERHEAD + E2E_MAX_PAYLOAD)
#define E2E_COBS_MAX_FRAME  (E2E_MAX_FRAME_SIZE + 2U)
#define E2E_RING_CAPACITY   512U

/**
 * @brief Sequence counter verification outcomes per AUTOSAR E2E Profile 1.
 */
typedef enum {
    E2E_STATUS_OK = 0,          /**< CRC-8 valid and delta_S == 0 */
    E2E_STATUS_INITIAL,         /**< First valid packet received after reset */
    E2E_STATUS_REPEATED,        /**< Duplicate alive counter (delta_S == 15) */
    E2E_STATUS_WRONG_SEQUENCE,  /**< Packet loss detected within max delta window */
    E2E_STATUS_CRC_ERROR,       /**< SAE J1850 CRC-8 mismatch */
    E2E_STATUS_FRAME_ERROR,     /**< Length bounds or COBS framing violation */
    E2E_STATUS_DESYNC           /**< Sequence gap exceeded max_delta_counter */
} e2e_rx_status_t;

/**
 * @brief 7-state deterministic stream parser states.
 */
typedef enum {
    FSM_WAIT_SYNC_1 = 0,
    FSM_WAIT_SYNC_2,
    FSM_READ_MSG_ID,
    FSM_READ_SEQ_CTR,
    FSM_READ_LENGTH,
    FSM_READ_PAYLOAD,
    FSM_READ_CRC8
} bridge_fsm_state_t;

/**
 * @brief Decoded E2E Telemetry / Actuator Frame.
 */
typedef struct {
    uint8_t         msg_id;
    uint8_t         seq_ctr;
    uint8_t         len;
    uint8_t         payload[E2E_MAX_PAYLOAD];
    uint8_t         crc8;
    e2e_rx_status_t status;
    uint8_t         lost_frames;
} bridge_decoded_frame_t;

/**
 * @brief Diagnostic counters for hardware-in-the-loop health monitoring.
 */
typedef struct {
    uint64_t frames_valid;
    uint64_t frames_crc_err;
    uint64_t frames_seq_lost;
    uint64_t frames_repeated;
    uint64_t resync_events;
    uint64_t ring_overruns;
} bridge_diag_counters_t;

/**
 * @brief Lock-free static single-producer / single-consumer circular DMA ring buffer.
 */
typedef struct {
    uint8_t           data[E2E_RING_CAPACITY];
    volatile uint16_t head;
    volatile uint16_t tail;
    uint32_t          overrun_bytes;
} bridge_ringbuf_t;

/**
 * @brief Complete streaming parser state context (zero heap allocation).
 */
typedef struct {
    bridge_fsm_state_t     fsm_state;
    uint8_t                cur_msg_id;
    uint8_t                cur_seq_ctr;
    uint8_t                cur_len;
    uint8_t                payload_idx;
    uint8_t                rx_payload[E2E_MAX_PAYLOAD];
    uint8_t                running_crc;
    bool                   seq_initialized;
    uint8_t                last_seq_ctr;
    uint8_t                max_delta_counter;
    bool                   use_data_id;
    uint16_t               data_id;
    uint8_t                cobs_buf[E2E_COBS_MAX_FRAME];
    size_t                 cobs_idx;
    bool                   cobs_overflow;
    bridge_diag_counters_t diag;
    bridge_ringbuf_t       rx_ring;
} bridge_parser_ctx_t;

void bridge_parser_init(bridge_parser_ctx_t *ctx, uint8_t max_delta_counter);
void bridge_parser_init_e2e(bridge_parser_ctx_t *ctx, uint8_t max_delta_counter, uint16_t data_id);
void bridge_parser_set_data_id(bridge_parser_ctx_t *ctx, uint16_t data_id);
void bridge_parser_reset(bridge_parser_ctx_t *ctx);

/**
 * @brief Push bytes from STM32 UART RXNE / DMA HT/TC ISR into ring buffer.
 */
size_t bridge_ring_push_isr(bridge_ringbuf_t *ring, const uint8_t *bytes, size_t count);

/**
 * @brief Pop single byte in main control loop.
 */
bool bridge_ring_pop(bridge_ringbuf_t *ring, uint8_t *out_byte);

/**
 * @brief Feed a single incoming byte through the 7-state E2E FSM.
 * @return true when a complete packet (valid or CRC-rejected) terminates.
 */
bool bridge_parser_feed_byte(bridge_parser_ctx_t *ctx,
                             uint8_t byte,
                             bridge_decoded_frame_t *out_frame);

/**
 * @brief Drain RX DMA ring buffer and process all available frames.
 * @return Number of decoded frames dispatched.
 */
size_t bridge_parser_poll_ring(bridge_parser_ctx_t *ctx,
                               bridge_decoded_frame_t *out_frames,
                               size_t max_frames);

/**
 * @brief Encode a payload with Consistent Overhead Byte Stuffing (COBS).
 * Guarantees zero 0x00 delimiter occurrences inside encoded stream.
 */
size_t bridge_cobs_encode(const uint8_t *input, size_t len, uint8_t *output, size_t out_cap);

/**
 * @brief Decode a COBS-encoded frame back into raw bytes.
 * @return Decoded length, or 0 on malformed framing.
 */
size_t bridge_cobs_decode(const uint8_t *input, size_t len, uint8_t *output, size_t out_cap);

/**
 * @brief Serialize an E2E frame, COBS-encode it, and append trailing 0x00 delimiter.
 * @return Total encoded wire length (including trailing 0x00), or 0 on error.
 */
size_t bridge_cobs_serialize_e2e(uint8_t id,
                                 uint8_t seq,
                                 const uint8_t *data,
                                 uint8_t len,
                                 uint8_t *out_buf,
                                 size_t out_cap);

/**
 * @brief Serialize a Data-ID-authenticated E2E frame with COBS framing + 0x00 delimiter.
 * @return Total encoded wire length (including trailing 0x00), or 0 on error.
 */
size_t bridge_cobs_serialize_e2e_with_data_id(uint16_t data_id,
                                              uint8_t id,
                                              uint8_t seq,
                                              const uint8_t *data,
                                              uint8_t len,
                                              uint8_t *out_buf,
                                              size_t out_cap);

/**
 * @brief Feed a single byte from a 0x00-delimited COBS UART stream.
 * Accumulates encoded frame until 0x00 delimiter, calls bridge_cobs_decode(),
 * and dispatches decoded bytes into bridge_parser_feed_byte().
 */
bool bridge_cobs_feed_byte(bridge_parser_ctx_t *ctx,
                           uint8_t byte,
                           bridge_decoded_frame_t *out_frame);

/**
 * @brief Drain RX DMA ring buffer using COBS-framed E2E decoding.
 */
size_t bridge_cobs_poll_ring(bridge_parser_ctx_t *ctx,
                             bridge_decoded_frame_t *out_frames,
                             size_t max_frames);

#ifdef __cplusplus
}
#endif

#endif /* ROS2_BRIDGE_FSM_H */
