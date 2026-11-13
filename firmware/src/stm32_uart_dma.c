/*
 * stm32_uart_dma.c
 * Lock-Free Single-Producer Single-Consumer (SPSC) Ring Buffer Driver
 * Designed for STM32 HAL/LL UART RXNE / DMA Half-Transfer & Transfer-Complete ISRs
 *
 * Enforces C11 memory-barrier / release-acquire semantics between producer (ISR)
 * and consumer (main loop / thread).
 *
 * Author: Engineer Abdullah Bin Zafar
 * License: Apache-2.0
 */

#include "ros2_bridge_fsm.h"
#include <stdatomic.h>

size_t bridge_ring_push_isr(bridge_ringbuf_t *ring, const uint8_t *bytes, size_t count)
{
    if (ring == NULL || bytes == NULL || count == 0U) {
        return 0U;
    }

#if defined(__GNUC__) || defined(__clang__)
    uint16_t head = __atomic_load_n(&ring->head, __ATOMIC_RELAXED);
    uint16_t tail = __atomic_load_n(&ring->tail, __ATOMIC_ACQUIRE);
#else
    uint16_t head = ring->head;
    uint16_t tail = ring->tail;
    atomic_thread_fence(memory_order_acquire);
#endif

    if (head >= E2E_RING_CAPACITY || tail >= E2E_RING_CAPACITY) {
        return 0U;
    }

    size_t written = 0U;
    for (size_t i = 0U; i < count; i++) {
        uint16_t next_head = (uint16_t)((head + 1U) % E2E_RING_CAPACITY);
        if (next_head == tail) {
            uint32_t lost = (uint32_t)(count - i);
#if defined(__GNUC__) || defined(__clang__)
            __atomic_fetch_add(&ring->overrun_bytes, lost, __ATOMIC_RELAXED);
#else
            ring->overrun_bytes += lost;
#endif
            break;
        }
        ring->data[head] = bytes[i];
        head = next_head;
        written++;
    }

#if defined(__GNUC__) || defined(__clang__)
    __atomic_store_n(&ring->head, head, __ATOMIC_RELEASE);
#else
    atomic_thread_fence(memory_order_release);
    ring->head = head;
#endif
    return written;
}

bool bridge_ring_pop(bridge_ringbuf_t *ring, uint8_t *out_byte)
{
    if (ring == NULL || out_byte == NULL) {
        return false;
    }

#if defined(__GNUC__) || defined(__clang__)
    uint16_t tail = __atomic_load_n(&ring->tail, __ATOMIC_RELAXED);
    uint16_t head = __atomic_load_n(&ring->head, __ATOMIC_ACQUIRE);
#else
    uint16_t tail = ring->tail;
    uint16_t head = ring->head;
    atomic_thread_fence(memory_order_acquire);
#endif

    if (head >= E2E_RING_CAPACITY || tail >= E2E_RING_CAPACITY) {
        return false;
    }
    if (tail == head) {
        return false;
    }

    *out_byte = ring->data[tail];
    uint16_t next_tail = (uint16_t)((tail + 1U) % E2E_RING_CAPACITY);

#if defined(__GNUC__) || defined(__clang__)
    __atomic_store_n(&ring->tail, next_tail, __ATOMIC_RELEASE);
#else
    atomic_thread_fence(memory_order_release);
    ring->tail = next_tail;
#endif
    return true;
}
