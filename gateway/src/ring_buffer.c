/*
 * ring_buffer.c - DMA 环形缓冲实现
 *
 * 设计要点：
 *   1) 容量强制为 2 的幂, 取模退化为按位与, ISR 里也是常数时间；
 *   2) head/tail 为单调递增的 32 位计数器, used = head - tail,
 *      无符号减法自动处理回绕, 不需要额外标志位区分空/满；
 *   3) 写的路径永远不覆盖未读数据：宁可丢弃新数据并计数 overrun,
 *      也不破坏已经收了一半的帧（帧完整性优先于吞吐）。
 */
#include "ring_buffer.h"

#include <string.h>

int gw_rb_init(gw_ringbuf_t *rb, uint8_t *storage, uint32_t size)
{
    if ((rb == NULL) || (storage == NULL) || (size < 2u)) {
        return GW_ERR_PARAM;
    }
    if ((size & (size - 1u)) != 0u) {
        return GW_ERR_PARAM;   /* 必须是 2 的幂 */
    }

    memset(rb, 0, sizeof(*rb));
    rb->storage = storage;
    rb->size    = size;
    rb->mask    = size - 1u;
    return GW_OK;
}

void gw_rb_reset(gw_ringbuf_t *rb)
{
    if (rb == NULL) {
        return;
    }
    rb->head = 0u;
    rb->tail = 0u;
    rb->bytes_written      = 0u;
    rb->bytes_read         = 0u;
    rb->overrun_count      = 0u;
    rb->max_used           = 0u;
    rb->dma_commit_count   = 0u;
}

uint32_t gw_rb_used(const gw_ringbuf_t *rb)
{
    if (rb == NULL) {
        return 0u;
    }
    return (uint32_t)(rb->head - rb->tail);
}

uint32_t gw_rb_free(const gw_ringbuf_t *rb)
{
    if (rb == NULL) {
        return 0u;
    }
    /* 预留 1 字节用于区分"刚好写满"与"计数器相等", 简化 DMA 指针运算 */
    return rb->size - 1u - gw_rb_used(rb);
}

uint32_t gw_rb_capacity(const gw_ringbuf_t *rb)
{
    return (rb != NULL) ? rb->size : 0u;
}

static void rb_update_watermark(gw_ringbuf_t *rb)
{
    uint32_t used = gw_rb_used(rb);
    if (used > rb->max_used) {
        rb->max_used = used;
    }
}

uint32_t gw_rb_write(gw_ringbuf_t *rb, const uint8_t *data, uint32_t len)
{
    uint32_t space;
    uint32_t first;
    uint32_t head;

    if ((rb == NULL) || (data == NULL) || (len == 0u)) {
        return 0u;
    }

    space = gw_rb_free(rb);
    if (len > space) {
        rb->overrun_count += (len - space);
        len = space;
    }
    if (len == 0u) {
        return 0u;
    }

    head  = rb->head;
    first = rb->size - (head & rb->mask);
    if (first > len) {
        first = len;
    }

    memcpy(&rb->storage[head & rb->mask], data, first);
    if (len > first) {
        memcpy(&rb->storage[0], data + first, len - first);
    }

    rb->head = head + len;
    rb->bytes_written += len;
    rb_update_watermark(rb);
    return len;
}

uint32_t gw_rb_read(gw_ringbuf_t *rb, uint8_t *out, uint32_t len)
{
    uint32_t n;

    n = gw_rb_peek(rb, out, len);
    if (n > 0u) {
        (void)gw_rb_skip(rb, n);
    }
    return n;
}

uint32_t gw_rb_peek(gw_ringbuf_t *rb, uint8_t *out, uint32_t len)
{
    uint32_t avail;
    uint32_t first;
    uint32_t tail;

    if ((rb == NULL) || (out == NULL) || (len == 0u)) {
        return 0u;
    }

    avail = gw_rb_used(rb);
    if (len > avail) {
        len = avail;
    }
    if (len == 0u) {
        return 0u;
    }

    tail  = rb->tail;
    first = rb->size - (tail & rb->mask);
    if (first > len) {
        first = len;
    }

    memcpy(out, &rb->storage[tail & rb->mask], first);
    if (len > first) {
        memcpy(out + first, &rb->storage[0], len - first);
    }
    return len;
}

uint32_t gw_rb_skip(gw_ringbuf_t *rb, uint32_t len)
{
    uint32_t avail;

    if ((rb == NULL) || (len == 0u)) {
        return 0u;
    }
    avail = gw_rb_used(rb);
    if (len > avail) {
        len = avail;
    }
    rb->tail = rb->tail + len;
    rb->bytes_read += len;
    return len;
}

int gw_rb_contiguous(gw_ringbuf_t *rb, const uint8_t **seg1, uint32_t *len1,
                     const uint8_t **seg2, uint32_t *len2)
{
    uint32_t avail;
    uint32_t tail;
    uint32_t first;

    if ((rb == NULL) || (seg1 == NULL) || (len1 == NULL) ||
        (seg2 == NULL) || (len2 == NULL)) {
        return GW_ERR_PARAM;
    }

    avail = gw_rb_used(rb);
    tail  = rb->tail & rb->mask;
    first = rb->size - tail;
    if (first > avail) {
        first = avail;
    }

    *seg1 = &rb->storage[tail];
    *len1 = first;
    if (avail > first) {
        *seg2 = &rb->storage[0];
        *len2 = avail - first;
    } else {
        *seg2 = NULL;
        *len2 = 0u;
    }
    return GW_OK;
}

uint8_t *gw_rb_dma_write_ptr(gw_ringbuf_t *rb, uint32_t *max_len)
{
    uint32_t space;
    uint32_t cont;

    if ((rb == NULL) || (max_len == NULL)) {
        return NULL;
    }

    space = gw_rb_free(rb);
    if (space == 0u) {
        *max_len = 0u;
        rb->overrun_count++;      /* 无空间可用：DMA 周期数据被丢弃 */
        return NULL;
    }

    cont = rb->size - (rb->head & rb->mask);
    *max_len = (cont < space) ? cont : space;
    return &rb->storage[rb->head & rb->mask];
}

void gw_rb_dma_commit(gw_ringbuf_t *rb, uint32_t len)
{
    uint32_t space;

    if ((rb == NULL) || (len == 0u)) {
        return;
    }
    space = gw_rb_free(rb);
    if (len > space) {
        rb->overrun_count += (len - space);
        len = space;
    }
    rb->head += len;
    rb->bytes_written += len;
    rb->dma_commit_count++;
    rb_update_watermark(rb);
}
