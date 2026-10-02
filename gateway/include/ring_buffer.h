/*
 * ring_buffer.h - DMA 环形缓冲
 *
 * 场景：RS485/CAN 高频报文下, 中断/DMA 每来一个字节都必须立刻入队,
 * 不能被上层解析线程阻塞。因此：
 *   - 生产侧（DMA/ISR）只做 gw_rb_dma_write_ptr + gw_rb_dma_commit,
 *     不做任何内存拷贝, 也不加锁（单生产者）；
 *   - 消费侧（解析线程）用 gw_rb_peek/gw_rb_skip 两段式消费, 允许
 *     "先看后取", 便于帧同步状态机在缓冲上直接判断而不搬数据。
 *
 * 用"单调递增的 head/tail 计数器 + 掩码取模"而不是 head==tail 判空,
 * 这样天然区分"空"与"满", 且 32 位计数器在 2^32 字节后才会回绕,
 * 按 115200bps 计算需要 10 小时以上, 回绕判断用无符号差值天然正确。
 */
#ifndef GW_RING_BUFFER_H
#define GW_RING_BUFFER_H

#include "gw_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t *storage;          /* 调用者提供的静态存储             */
    uint32_t size;             /* 容量, 必须为 2 的幂              */
    uint32_t mask;             /* size - 1                         */

    volatile uint32_t head;    /* 生产者位置（累计计数, 不回绕清零）*/
    volatile uint32_t tail;    /* 消费者位置（累计计数）            */

    /* 统计（诊断线程周期性上报） */
    uint32_t bytes_written;    /* 累计写入字节                     */
    uint32_t bytes_read;       /* 累计读出字节                     */
    uint32_t overrun_count;    /* 因满而丢弃的字节数（关键可靠性指标）*/
    uint32_t max_used;         /* 使用量高水位                     */
    uint32_t dma_commit_count; /* DMA 提交次数（帧/半满中断次数）   */
} gw_ringbuf_t;

/* 初始化：storage 由调用者提供（静态数组或 DMA 专用 RAM 段） */
int      gw_rb_init(gw_ringbuf_t *rb, uint8_t *storage, uint32_t size);
void     gw_rb_reset(gw_ringbuf_t *rb);

uint32_t gw_rb_used(const gw_ringbuf_t *rb);
uint32_t gw_rb_free(const gw_ringbuf_t *rb);
uint32_t gw_rb_capacity(const gw_ringbuf_t *rb);

/* 生产者（拷贝语义）：返回实际写入字节数, 满时丢弃多余数据并计数 */
uint32_t gw_rb_write(gw_ringbuf_t *rb, const uint8_t *data, uint32_t len);

/* 消费者 */
uint32_t gw_rb_read(gw_ringbuf_t *rb, uint8_t *out, uint32_t len);   /* 读并消费 */
uint32_t gw_rb_peek(gw_ringbuf_t *rb, uint8_t *out, uint32_t len);   /* 只读不消费 */
uint32_t gw_rb_skip(gw_ringbuf_t *rb, uint32_t len);                 /* 丢弃 len 字节 */

/* 零拷贝分段读：返回可连续读的两段指针与长度（第二段可为 0） */
int gw_rb_contiguous(gw_ringbuf_t *rb, const uint8_t **seg1, uint32_t *len1,
                     const uint8_t **seg2, uint32_t *len2);

/* ---------------- DMA 直写路径（无拷贝） ---------------- */
/*
 * 返回本次可连续写入的起始地址与最大长度, 供 DMA 直接落数据。
 * 注意：返回 >0 后必须调用 gw_rb_dma_commit 才会对消费者可见。
 */
uint8_t *gw_rb_dma_write_ptr(gw_ringbuf_t *rb, uint32_t *max_len);
/* 提交 len 字节（DMA 传输完成中断 / 半满中断里调用） */
void     gw_rb_dma_commit(gw_ringbuf_t *rb, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* GW_RING_BUFFER_H */
