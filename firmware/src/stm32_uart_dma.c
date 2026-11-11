/*
 * stm32_uart_dma.c
 * Lock-Free Single-Producer Single-Consumer (SPSC) Ring Buffer Driver
 * Designed for STM32 HAL/LL UART RXNE / DMA Half-Transfer & Transfer-Complete ISRs
 *
 * Author: Engineer Abdullah Bin Zafar
 * License: Apache-2.0
 */

#include "ros2_bridge_fsm.h"

size_t bridge_ring_push_isr(bridge_ringbuf_t *ring, const uint8_t *bytes, size_t count)
{
    if (ring == NULL || bytes == NULL || count == 0U) {
        return 0U;
    }

    size_t written = 0U;
    uint16_t head = ring->head;
    uint16_t tail = ring->tail;

    for (size_t i = 0U; i < count; i++) {
        uint16_t next_head = (uint16_t)((head + 1U) % E2E_RING_CAPACITY);
        if (next_head == tail) {
            ring->overrun_bytes += (uint32_t)(count - i);
            break;
        }
        ring->data[head] = bytes[i];
        head = next_head;
        written++;
    }

    ring->head = head;
    return written;
}

bool bridge_ring_pop(bridge_ringbuf_t *ring, uint8_t *out_byte)
{
    if (ring == NULL || out_byte == NULL) {
        return false;
    }

    uint16_t tail = ring->tail;
    if (tail == ring->head) {
        return false;
    }

    *out_byte = ring->data[tail];
    ring->tail = (uint16_t)((tail + 1U) % E2E_RING_CAPACITY);
    return true;
}
