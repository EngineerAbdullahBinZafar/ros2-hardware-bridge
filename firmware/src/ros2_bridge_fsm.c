/*
 * ros2_bridge_fsm.c
 * Deterministic 7-State AUTOSAR E2E Profile 1 Stream Parser & COBS Codec
 *
 * Zero dynamic memory allocation. Bounded O(1) per-byte execution (< 35 us/frame on M4).
 * Author: Engineer Abdullah Bin Zafar
 * License: Apache-2.0
 */

#include "ros2_bridge_fsm.h"
#include "../ros2_bridge.h"
#include <string.h>

void bridge_parser_init(bridge_parser_ctx_t *ctx, uint8_t max_delta_counter)
{
    if (ctx == NULL) {
        return;
    }
    memset(ctx, 0, sizeof(*ctx));
    ctx->fsm_state = FSM_WAIT_SYNC_1;
    if (max_delta_counter == 0U) {
        ctx->max_delta_counter = 2U;
    } else if (max_delta_counter > 14U) {
        ctx->max_delta_counter = 14U;
    } else {
        ctx->max_delta_counter = max_delta_counter;
    }
    ctx->running_crc = 0xFFU;
}

void bridge_parser_init_e2e(bridge_parser_ctx_t *ctx, uint8_t max_delta_counter, uint16_t data_id)
{
    bridge_parser_init(ctx, max_delta_counter);
    bridge_parser_set_data_id(ctx, data_id);
}

void bridge_parser_set_data_id(bridge_parser_ctx_t *ctx, uint16_t data_id)
{
    if (ctx == NULL) {
        return;
    }
    ctx->use_data_id = true;
    ctx->data_id = data_id;
}

void bridge_parser_reset(bridge_parser_ctx_t *ctx)
{
    if (ctx == NULL) {
        return;
    }
    ctx->fsm_state = FSM_WAIT_SYNC_1;
    ctx->payload_idx = 0U;
    ctx->cur_len = 0U;
    ctx->running_crc = 0xFFU;
}

static e2e_rx_status_t evaluate_sequence(bridge_parser_ctx_t *ctx,
                                         uint8_t received_seq,
                                         uint8_t *out_lost)
{
    *out_lost = 0U;
    uint8_t seq = received_seq & 0x0FU;

    if (!ctx->seq_initialized) {
        ctx->seq_initialized = true;
        ctx->last_seq_ctr = seq;
        ctx->diag.frames_valid++;
        return E2E_STATUS_INITIAL;
    }

    uint8_t expected = (uint8_t)((ctx->last_seq_ctr + 1U) & 0x0FU);
    uint8_t delta = (uint8_t)(((unsigned int)seq - (unsigned int)expected) & 0x0FU);

    if (delta == 0U) {
        ctx->last_seq_ctr = seq;
        ctx->diag.frames_valid++;
        return E2E_STATUS_OK;
    }
    if (delta == 15U) {
        ctx->diag.frames_repeated++;
        return E2E_STATUS_REPEATED;
    }

    ctx->last_seq_ctr = seq;
    *out_lost = delta;
    ctx->diag.frames_seq_lost += delta;

    if (delta > ctx->max_delta_counter) {
        ctx->diag.resync_events++;
        return E2E_STATUS_DESYNC;
    }
    return E2E_STATUS_WRONG_SEQUENCE;
}

bool bridge_parser_feed_byte(bridge_parser_ctx_t *ctx,
                             uint8_t byte,
                             bridge_decoded_frame_t *out_frame)
{
    if (ctx == NULL || out_frame == NULL) {
        return false;
    }

    switch (ctx->fsm_state) {
        case FSM_WAIT_SYNC_1:
            if (byte == E2E_SYNC_BYTE_1) {
                ctx->fsm_state = FSM_WAIT_SYNC_2;
            }
            return false;

        case FSM_WAIT_SYNC_2:
            if (byte == E2E_SYNC_BYTE_2) {
                ctx->running_crc = 0xFFU;
                if (ctx->use_data_id) {
                    uint8_t id_lo = (uint8_t)(ctx->data_id & 0xFFU);
                    uint8_t id_hi = (uint8_t)((ctx->data_id >> 8) & 0xFFU);
                    ctx->running_crc = CRC8_SAE_J1850_TABLE[ctx->running_crc ^ id_lo];
                    ctx->running_crc = CRC8_SAE_J1850_TABLE[ctx->running_crc ^ id_hi];
                }
                ctx->fsm_state = FSM_READ_MSG_ID;
            } else if (byte != E2E_SYNC_BYTE_1) {
                ctx->diag.resync_events++;
                ctx->fsm_state = FSM_WAIT_SYNC_1;
            }
            return false;

        case FSM_READ_MSG_ID:
            ctx->cur_msg_id = byte;
            ctx->running_crc = CRC8_SAE_J1850_TABLE[ctx->running_crc ^ byte];
            ctx->fsm_state = FSM_READ_SEQ_CTR;
            return false;

        case FSM_READ_SEQ_CTR:
            ctx->cur_seq_ctr = byte & 0x0FU;
            ctx->running_crc = CRC8_SAE_J1850_TABLE[ctx->running_crc ^ byte];
            ctx->fsm_state = FSM_READ_LENGTH;
            return false;

        case FSM_READ_LENGTH:
            if (byte > E2E_MAX_PAYLOAD) {
                ctx->diag.frames_crc_err++;
                ctx->diag.resync_events++;
                memset(out_frame, 0, sizeof(*out_frame));
                out_frame->status = E2E_STATUS_FRAME_ERROR;
                bridge_parser_reset(ctx);
                return true;
            }
            ctx->cur_len = byte;
            ctx->payload_idx = 0U;
            ctx->running_crc = CRC8_SAE_J1850_TABLE[ctx->running_crc ^ byte];
            ctx->fsm_state = (byte == 0U) ? FSM_READ_CRC8 : FSM_READ_PAYLOAD;
            return false;

        case FSM_READ_PAYLOAD:
            ctx->rx_payload[ctx->payload_idx++] = byte;
            ctx->running_crc = CRC8_SAE_J1850_TABLE[ctx->running_crc ^ byte];
            if (ctx->payload_idx >= ctx->cur_len) {
                ctx->fsm_state = FSM_READ_CRC8;
            }
            return false;

        case FSM_READ_CRC8: {
            uint8_t calc_crc = (uint8_t)(ctx->running_crc ^ 0xFFU);
            out_frame->msg_id = ctx->cur_msg_id;
            out_frame->seq_ctr = ctx->cur_seq_ctr;
            out_frame->len = ctx->cur_len;
            out_frame->crc8 = byte;
            out_frame->lost_frames = 0U;

            if (ctx->cur_len > 0U) {
                memcpy(out_frame->payload, ctx->rx_payload, ctx->cur_len);
            }

            if (calc_crc == byte) {
                out_frame->status = evaluate_sequence(ctx, ctx->cur_seq_ctr, &out_frame->lost_frames);
            } else {
                ctx->diag.frames_crc_err++;
                out_frame->status = E2E_STATUS_CRC_ERROR;
            }

            bridge_parser_reset(ctx);
            return true;
        }

        default:
            bridge_parser_reset(ctx);
            return false;
    }
}

size_t bridge_parser_poll_ring(bridge_parser_ctx_t *ctx,
                               bridge_decoded_frame_t *out_frames,
                               size_t max_frames)
{
    if (ctx == NULL || out_frames == NULL || max_frames == 0U) {
        return 0U;
    }

    size_t decoded_count = 0U;
    uint8_t b = 0U;

    while (decoded_count < max_frames && bridge_ring_pop(&ctx->rx_ring, &b)) {
        if (bridge_parser_feed_byte(ctx, b, &out_frames[decoded_count])) {
            decoded_count++;
        }
    }
    return decoded_count;
}

size_t bridge_cobs_encode(const uint8_t *input, size_t len, uint8_t *output, size_t out_cap)
{
    if (input == NULL || output == NULL || out_cap < len + 2U) {
        return 0U;
    }

    size_t read_idx = 0U;
    size_t write_idx = 1U;
    size_t code_idx = 0U;
    uint8_t code = 1U;

    while (read_idx < len) {
        if (write_idx >= out_cap) {
            return 0U;
        }
        if (input[read_idx] == 0U) {
            output[code_idx] = code;
            code = 1U;
            code_idx = write_idx++;
            read_idx++;
        } else {
            output[write_idx++] = input[read_idx++];
            code++;
            if (code == 0xFFU) {
                output[code_idx] = code;
                code = 1U;
                code_idx = write_idx++;
            }
        }
    }

    if (code_idx >= out_cap) {
        return 0U;
    }
    output[code_idx] = code;
    return write_idx;
}

size_t bridge_cobs_decode(const uint8_t *input, size_t len, uint8_t *output, size_t out_cap)
{
    if (input == NULL || output == NULL || len == 0U) {
        return 0U;
    }

    size_t read_idx = 0U;
    size_t write_idx = 0U;

    while (read_idx < len) {
        uint8_t code = input[read_idx];
        if (code == 0U || read_idx + code > len + 1U) {
            return 0U;
        }
        read_idx++;

        for (uint8_t i = 1U; i < code; i++) {
            if (read_idx >= len || write_idx >= out_cap) {
                return 0U;
            }
            output[write_idx++] = input[read_idx++];
        }

        if (code < 0xFFU && read_idx < len) {
            if (write_idx >= out_cap) {
                return 0U;
            }
            output[write_idx++] = 0U;
        }
    }
    return write_idx;
}

size_t bridge_cobs_serialize_e2e(uint8_t id,
                                 uint8_t seq,
                                 const uint8_t *data,
                                 uint8_t len,
                                 uint8_t *out_buf,
                                 size_t out_cap)
{
    if (out_buf == NULL) {
        return 0U;
    }
    uint8_t raw[E2E_MAX_FRAME_SIZE];
    int raw_len = bridge_serialize_e2e(id, seq, data, len, raw);
    if (raw_len <= 0) {
        return 0U;
    }
    size_t enc_len = bridge_cobs_encode(raw, (size_t)raw_len, out_buf, out_cap);
    if (enc_len == 0U || enc_len >= out_cap) {
        return 0U;
    }
    out_buf[enc_len] = 0x00U;
    return enc_len + 1U;
}

size_t bridge_cobs_serialize_e2e_with_data_id(uint16_t data_id,
                                              uint8_t id,
                                              uint8_t seq,
                                              const uint8_t *data,
                                              uint8_t len,
                                              uint8_t *out_buf,
                                              size_t out_cap)
{
    if (out_buf == NULL) {
        return 0U;
    }
    uint8_t raw[E2E_MAX_FRAME_SIZE];
    int raw_len = bridge_serialize_e2e_with_data_id(data_id, id, seq, data, len, raw);
    if (raw_len <= 0) {
        return 0U;
    }
    size_t enc_len = bridge_cobs_encode(raw, (size_t)raw_len, out_buf, out_cap);
    if (enc_len == 0U || enc_len >= out_cap) {
        return 0U;
    }
    out_buf[enc_len] = 0x00U;
    return enc_len + 1U;
}

bool bridge_cobs_feed_byte(bridge_parser_ctx_t *ctx,
                           uint8_t byte,
                           bridge_decoded_frame_t *out_frame)
{
    if (ctx == NULL || out_frame == NULL) {
        return false;
    }

    if (byte != 0x00U) {
        if (!ctx->cobs_overflow) {
            if (ctx->cobs_idx < sizeof(ctx->cobs_buf)) {
                ctx->cobs_buf[ctx->cobs_idx++] = byte;
            } else {
                ctx->cobs_overflow = true;
            }
        }
        return false;
    }

    if (ctx->cobs_overflow) {
        ctx->cobs_idx = 0U;
        ctx->cobs_overflow = false;
        ctx->diag.frames_crc_err++;
        ctx->diag.resync_events++;
        memset(out_frame, 0, sizeof(*out_frame));
        out_frame->status = E2E_STATUS_FRAME_ERROR;
        return true;
    }

    if (ctx->cobs_idx == 0U) {
        return false;
    }

    uint8_t decoded[E2E_MAX_FRAME_SIZE];
    size_t dec_len = bridge_cobs_decode(ctx->cobs_buf, ctx->cobs_idx, decoded, sizeof(decoded));
    ctx->cobs_idx = 0U;

    if (dec_len < E2E_FRAME_OVERHEAD) {
        ctx->diag.frames_crc_err++;
        ctx->diag.resync_events++;
        memset(out_frame, 0, sizeof(*out_frame));
        out_frame->status = E2E_STATUS_FRAME_ERROR;
        return true;
    }

    bridge_parser_reset(ctx);
    bool produced = false;
    for (size_t i = 0U; i < dec_len; i++) {
        if (bridge_parser_feed_byte(ctx, decoded[i], out_frame)) {
            produced = true;
            break;
        }
    }
    if (!produced) {
        bridge_parser_reset(ctx);
        ctx->diag.frames_crc_err++;
        ctx->diag.resync_events++;
        memset(out_frame, 0, sizeof(*out_frame));
        out_frame->status = E2E_STATUS_FRAME_ERROR;
    }
    return true;
}

size_t bridge_cobs_poll_ring(bridge_parser_ctx_t *ctx,
                             bridge_decoded_frame_t *out_frames,
                             size_t max_frames)
{
    if (ctx == NULL || out_frames == NULL || max_frames == 0U) {
        return 0U;
    }

    size_t decoded_count = 0U;
    uint8_t b = 0U;

    while (decoded_count < max_frames && bridge_ring_pop(&ctx->rx_ring, &b)) {
        if (bridge_cobs_feed_byte(ctx, b, &out_frames[decoded_count])) {
            decoded_count++;
        }
    }
    return decoded_count;
}
