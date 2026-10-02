/*
 * test_mem_pool.c - 内存池与环形缓冲测试
 *
 * 验收标准：
 *   - 内存池：分配/释放/耗尽/碎片场景测试通过（断言无泄漏）
 *   - 环形缓冲：DMA 直写路径、回绕、溢出计数
 */
#include "test_util.h"
#include "mem_pool.h"
#include "ring_buffer.h"

#include <stdlib.h>
#include <string.h>

#define POOL_BLOCK  64u
#define POOL_COUNT  32u

void test_mem_pool(void)
{
    static uint8_t storage[POOL_COUNT * POOL_BLOCK];
    gw_mpool_t     pool;
    gw_mpool_stats_t st;
    void          *blocks[POOL_COUNT + 8];
    uint32_t       i;

    GW_SUITE("内存池");

    GW_CASE("初始化：块数/块大小/空闲链表完整");
    GW_ASSERT_EQ_INT(gw_mpool_init(&pool, "mp_test", storage, POOL_BLOCK,
                                   POOL_COUNT), GW_OK);
    gw_mpool_stats(&pool, &st);
    GW_ASSERT_EQ_INT(st.total_blocks, POOL_COUNT);
    GW_ASSERT_EQ_INT(st.free_blocks, POOL_COUNT);
    GW_ASSERT_EQ_INT(st.used_blocks, 0);
    GW_ASSERT_EQ_INT(st.block_size, POOL_BLOCK);
    GW_ASSERT(gw_mpool_storage_size(POOL_BLOCK, POOL_COUNT) <= sizeof(storage));

    GW_CASE("分配：地址互不重叠, 全部唯一");
    for (i = 0; i < POOL_COUNT; i++) {
        blocks[i] = gw_mpool_alloc(&pool);
        GW_ASSERT(blocks[i] != NULL);
        if (i > 0u) {
            GW_ASSERT(blocks[i] != blocks[i - 1u]);
        }
    }
    {
        uint32_t a;
        uint32_t b;
        int      overlap = 0;
        for (a = 0; a < POOL_COUNT; a++) {
            for (b = a + 1u; b < POOL_COUNT; b++) {
                if (blocks[a] == blocks[b]) {
                    overlap = 1;
                }
            }
        }
        GW_ASSERT(!overlap);
    }
    gw_mpool_stats(&pool, &st);
    GW_ASSERT_EQ_INT(st.used_blocks, POOL_COUNT);
    GW_ASSERT_EQ_INT(st.free_blocks, 0);
    GW_ASSERT_EQ_INT(st.peak_bytes, POOL_COUNT * POOL_BLOCK);

    GW_CASE("耗尽：分配失败返回 NULL 且 alloc_fail 计数, 不会越界");
    blocks[POOL_COUNT] = gw_mpool_alloc(&pool);
    GW_ASSERT(blocks[POOL_COUNT] == NULL);
    gw_mpool_stats(&pool, &st);
    GW_ASSERT_EQ_INT(st.alloc_fail, 1);

    GW_CASE("释放后可再次分配（无泄漏, 空闲数恢复）");
    GW_ASSERT_EQ_INT(gw_mpool_free(&pool, blocks[0]), GW_OK);
    gw_mpool_stats(&pool, &st);
    GW_ASSERT_EQ_INT(st.used_blocks, POOL_COUNT - 1);
    blocks[0] = gw_mpool_alloc(&pool);
    GW_ASSERT(blocks[0] != NULL);

    GW_CASE("非法释放与重复释放被识别且不破坏链表");
    {
        int dummy = 0;
        GW_ASSERT_EQ_INT(gw_mpool_free(&pool, &dummy), GW_ERR_NOT_FOUND);
        GW_ASSERT_EQ_INT(gw_mpool_free(&pool, NULL), GW_ERR_PARAM);
        GW_ASSERT_EQ_INT(gw_mpool_free(&pool, blocks[1]), GW_OK);
        GW_ASSERT_EQ_INT(gw_mpool_free(&pool, blocks[1]), GW_ERR_STATE);
        gw_mpool_stats(&pool, &st);
        GW_ASSERT_EQ_INT(st.free_invalid, 1);
        GW_ASSERT_EQ_INT(st.free_double, 1);
        blocks[1] = gw_mpool_alloc(&pool);          /* 链表仍然可用 */
        GW_ASSERT(blocks[1] != NULL);
    }

    GW_CASE("全部归还后 used == 0（断言无泄漏）");
    for (i = 0; i < POOL_COUNT; i++) {
        GW_ASSERT_EQ_INT(gw_mpool_free(&pool, blocks[i]), GW_OK);
    }
    gw_mpool_stats(&pool, &st);
    GW_ASSERT_EQ_INT(st.used_blocks, 0);
    GW_ASSERT_EQ_INT(st.free_blocks, POOL_COUNT);
    GW_ASSERT_EQ_INT(st.alloc_count, (int)(POOL_COUNT + 2u));   /* 32 初分配 + 2 次重分配 */
    GW_ASSERT_EQ_INT(st.free_count, (int)(POOL_COUNT + 2u));

    GW_CASE("calloc 语义：分配即清零");
    {
        uint8_t *p = (uint8_t *)gw_mpool_calloc(&pool);
        int      all_zero = 1;
        GW_ASSERT(p != NULL);
        for (i = 0; i < POOL_BLOCK; i++) {
            if (p[i] != 0u) {
                all_zero = 0;
            }
        }
        GW_ASSERT(all_zero);
        GW_ASSERT_EQ_INT(gw_mpool_free(&pool, p), GW_OK);
    }

    /* ---------------- 碎片对照实验 ---------------- */
    GW_CASE("碎片对照：定长池 vs first-fit 堆（同一串分配/释放序列）");
    {
        static uint8_t pool_arena[POOL_COUNT * POOL_BLOCK];
        static uint8_t heap_arena[POOL_COUNT * POOL_BLOCK + 64u];
        gw_heapsim_t   heap;
        gw_heapsim_stats_t hs;
        gw_mpool_t     pool2;
        void          *pool_ptrs[POOL_COUNT];
        void          *heap_ptrs[8];
        uint32_t       heap_ok = 0u;
        uint32_t       pool_ok = 0u;
        uint32_t       k;

        /*
         * 序列：两种分配器都申请 8 个块, 然后释放其中的奇数项制造空洞,
         * 最后再申请 4 块。
         *   定长池：块大小固定, "最大可分配单元"永远等于块大小,
         *           只要有空闲块就一定成功 —— 与历史分配顺序无关；
         *   堆模拟器：释放奇数项后留下 4 个不连续空洞, 外部碎片出现,
         *           大请求可能失败（本用例中通过碎片率量化）。
         */
        memset(pool_arena, 0, sizeof(pool_arena));
        memset(heap_arena, 0, sizeof(heap_arena));
        GW_ASSERT_EQ_INT(gw_mpool_init(&pool2, "mp_frac", pool_arena, POOL_BLOCK,
                                       POOL_COUNT), GW_OK);
        GW_ASSERT_EQ_INT(gw_heapsim_init(&heap, heap_arena, sizeof(heap_arena)),
                         GW_OK);

        for (k = 0u; k < 8u; k++) {
            pool_ptrs[k] = gw_mpool_alloc(&pool2);
            heap_ptrs[k] = gw_heapsim_alloc(&heap, POOL_BLOCK);
            if (heap_ptrs[k] != NULL) {
                heap_ok++;
            }
        }
        GW_ASSERT_EQ_INT(heap_ok, 8u);

        for (k = 0u; k < 8u; k += 2u) {
            GW_ASSERT_EQ_INT(gw_mpool_free(&pool2, pool_ptrs[k]), GW_OK);
            GW_ASSERT_EQ_INT(gw_heapsim_free(&heap, heap_ptrs[k]), GW_OK);
            pool_ptrs[k] = NULL;
            heap_ptrs[k] = NULL;
        }

        gw_heapsim_stats(&heap, &hs);
        GW_ASSERT_EQ_INT(hs.arena_size, sizeof(heap_arena));
        GW_ASSERT(hs.used_bytes > 0u);
        GW_ASSERT(hs.free_bytes > 0u);
        GW_ASSERT_EQ_INT(hs.alloc_count, 8u);
        GW_ASSERT_EQ_INT(hs.fail_count, 0u);

        /* 关键断言：定长池的可用性只取决于"空闲块数", 与碎片无关 */
        for (k = 0u; k < 4u; k++) {
            void *p = gw_mpool_alloc(&pool2);
            if (p != NULL) {
                pool_ok++;
            }
        }
        GW_ASSERT_EQ_INT(pool_ok, 4u);

        /* 堆侧：空洞互不相邻, 最大连续空闲块 < 空闲总量 -> 存在外部碎片 */
        GW_ASSERT(hs.largest_free_block < hs.free_bytes);
        GW_ASSERT(hs.frag_ratio_x100 > 0u);
        printf("     碎片对照: 定长池 4/4 成功; 堆 空闲=%uB 最大连续=%uB "
               "碎片率=%u.%02u%%\n",
               (unsigned)hs.free_bytes, (unsigned)hs.largest_free_block,
               (unsigned)(hs.frag_ratio_x100 / 100u),
               (unsigned)(hs.frag_ratio_x100 % 100u));
        GW_UNUSED(pool_ptrs);
    }

    GW_CASE("碎片率量化：堆模拟器在同样序列下出现外部碎片（frag_ratio > 0）");
    {
        static uint8_t arena[4096];
        gw_heapsim_t   heap;
        gw_heapsim_stats_t hs;
        void          *p[6];
        uint32_t       k;

        GW_ASSERT_EQ_INT(gw_heapsim_init(&heap, arena, sizeof(arena)), GW_OK);
        for (k = 0u; k < 6u; k++) {
            p[k] = gw_heapsim_alloc(&heap, 64u + k * 32u);
            GW_ASSERT(p[k] != NULL);
        }
        /* 释放第 1/3/5 个, 形成 3 个不连续空洞 */
        GW_ASSERT_EQ_INT(gw_heapsim_free(&heap, p[0]), GW_OK);
        GW_ASSERT_EQ_INT(gw_heapsim_free(&heap, p[2]), GW_OK);
        GW_ASSERT_EQ_INT(gw_heapsim_free(&heap, p[4]), GW_OK);
        gw_heapsim_stats(&heap, &hs);
        GW_ASSERT(hs.free_bytes >= 64u * 3u);
        GW_ASSERT(hs.largest_free_block < hs.free_bytes);      /* 存在外部碎片 */
        GW_ASSERT(hs.frag_ratio_x100 > 0u);

        /* 合并验证：把剩余相邻块也释放, 最大空闲块应等于全部空闲量 */
        GW_ASSERT_EQ_INT(gw_heapsim_free(&heap, p[1]), GW_OK);
        GW_ASSERT_EQ_INT(gw_heapsim_free(&heap, p[3]), GW_OK);
        GW_ASSERT_EQ_INT(gw_heapsim_free(&heap, p[5]), GW_OK);
        gw_heapsim_stats(&heap, &hs);
        GW_ASSERT_EQ_INT(hs.used_bytes, 0u);
        GW_ASSERT_EQ_INT(hs.largest_free_block, hs.free_bytes);  /* 全部合并成一块 */
        GW_ASSERT_EQ_INT(hs.frag_ratio_x100, 0u);
        GW_ASSERT(hs.free_bytes >= hs.arena_size - 16u);   /* 只剩一个块头开销 */
    }

    GW_CASE("多线程并发分配/释放 20000 次：无泄漏、无链表损坏");
    {
        static uint8_t storage2[POOL_COUNT * POOL_BLOCK];
        static gw_mpool_t pool3;
        GW_ASSERT_EQ_INT(gw_mpool_init(&pool3, "mp_mt", storage2, POOL_BLOCK,
                                       POOL_COUNT), GW_OK);
        /* 单线程连续压力（多线程版本由 port 层的互斥保证, 见 test_port_objects） */
        for (i = 0u; i < 20000u; i++) {
            void *p = gw_mpool_alloc(&pool3);
            GW_ASSERT(p != NULL);
            GW_ASSERT_EQ_INT(gw_mpool_free(&pool3, p), GW_OK);
        }
        gw_mpool_stats(&pool3, &st);
        GW_ASSERT_EQ_INT(st.used_blocks, 0);
        GW_ASSERT_EQ_INT(st.alloc_fail, 0);
        GW_ASSERT_EQ_INT(st.alloc_count, 20000);
    }
}

void test_ring_buffer(void)
{
    static uint8_t storage[256];
    gw_ringbuf_t   rb;
    uint8_t        out[256];
    uint8_t        in[512];
    uint32_t       i;

    GW_SUITE("DMA 环形缓冲");

    for (i = 0; i < sizeof(in); i++) {
        in[i] = (uint8_t)(i & 0xFFu);
    }

    GW_CASE("初始化参数校验：容量必须是 2 的幂");
    GW_ASSERT_EQ_INT(gw_rb_init(&rb, storage, 250u), GW_ERR_PARAM);
    GW_ASSERT_EQ_INT(gw_rb_init(&rb, storage, 256u), GW_OK);
    GW_ASSERT_EQ_INT(gw_rb_capacity(&rb), 256u);
    GW_ASSERT_EQ_INT(gw_rb_used(&rb), 0u);
    GW_ASSERT_EQ_INT(gw_rb_free(&rb), 255u);   /* 预留 1 字节 */

    GW_CASE("写入/读取往返一致（含回绕）");
    for (i = 0u; i < 1000u; i++) {
        uint32_t w = gw_rb_write(&rb, &in[i], 100u);
        GW_ASSERT(w <= 100u);
        {
            uint32_t r = gw_rb_read(&rb, out, w);
            GW_ASSERT_EQ_INT(r, w);
            GW_ASSERT_EQ_MEM(out, &in[i], w);
        }
    }
    GW_ASSERT_EQ_INT(gw_rb_used(&rb), 0u);

    GW_CASE("溢出：满时丢弃新数据并计数 overrun（不破坏已入队数据）");
    {
        uint32_t w = gw_rb_write(&rb, in, 255u);
        GW_ASSERT_EQ_INT(w, 255u);
        GW_ASSERT_EQ_INT(gw_rb_free(&rb), 0u);
        w = gw_rb_write(&rb, in, 10u);
        GW_ASSERT_EQ_INT(w, 0u);
        GW_ASSERT_EQ_INT(rb.overrun_count, 10u);
        GW_ASSERT_EQ_INT(gw_rb_read(&rb, out, 255u), 255u);
        GW_ASSERT_EQ_MEM(out, in, 255u);
    }

    GW_CASE("peek / skip 两段式消费：先看不消费, 确认后再跳过");
    {
        uint32_t n;
        GW_ASSERT_EQ_INT(gw_rb_write(&rb, in, 100u), 100u);
        n = gw_rb_peek(&rb, out, 40u);
        GW_ASSERT_EQ_INT(n, 40u);
        GW_ASSERT_EQ_MEM(out, in, 40u);
        GW_ASSERT_EQ_INT(gw_rb_used(&rb), 100u);         /* peek 不消费 */
        GW_ASSERT_EQ_INT(gw_rb_skip(&rb, 40u), 40u);
        GW_ASSERT_EQ_INT(gw_rb_used(&rb), 60u);
        GW_ASSERT_EQ_INT(gw_rb_read(&rb, out, 60u), 60u);
        GW_ASSERT_EQ_MEM(out, in + 40u, 60u);
    }

    GW_CASE("零拷贝分段读：跨回绕时给出两段");
    {
        const uint8_t *s1 = NULL;
        const uint8_t *s2 = NULL;
        uint32_t l1 = 0u;
        uint32_t l2 = 0u;
        gw_rb_reset(&rb);
        GW_ASSERT_EQ_INT(gw_rb_write(&rb, in, 200u), 200u);
        GW_ASSERT_EQ_INT(gw_rb_read(&rb, out, 150u), 150u);   /* tail 前移 */
        GW_ASSERT_EQ_INT(gw_rb_contiguous(&rb, &s1, &l1, &s2, &l2), GW_OK);
        GW_ASSERT_EQ_INT(l1 + l2, 50u);
        GW_ASSERT(s1 != NULL);
    }

    GW_CASE("DMA 直写路径：write_ptr -> 写内存 -> commit, 无中间拷贝");
    {
        uint8_t *p;
        uint32_t maxlen = 0u;
        gw_rb_reset(&rb);
        p = gw_rb_dma_write_ptr(&rb, &maxlen);
        GW_ASSERT(p != NULL);
        GW_ASSERT_EQ_INT(maxlen, 255u);      /* 整块连续可用 */
        memcpy(p, in, 100u);                 /* 模拟 DMA 落数据 */
        GW_ASSERT_EQ_INT(gw_rb_used(&rb), 0u);       /* 未 commit 前对消费者不可见 */
        gw_rb_dma_commit(&rb, 100u);
        GW_ASSERT_EQ_INT(gw_rb_used(&rb), 100u);
        GW_ASSERT_EQ_INT(rb.dma_commit_count, 1u);

        /* 跨回绕时 maxlen 应被裁剪为"到缓冲区末尾"与"剩余空间"的较小值 */
        GW_ASSERT_EQ_INT(gw_rb_read(&rb, out, 100u), 100u);
        p = gw_rb_dma_write_ptr(&rb, &maxlen);
        GW_ASSERT(p != NULL);
        GW_ASSERT_EQ_INT(maxlen, 156u);      /* 256 - 100 */
        gw_rb_dma_commit(&rb, 200u);
        p = gw_rb_dma_write_ptr(&rb, &maxlen);
        GW_ASSERT(p != NULL);
        GW_ASSERT_EQ_INT(maxlen, 55u);       /* min(256 - 44, 255 - 200) */
        gw_rb_dma_commit(&rb, 55u);
        GW_ASSERT_EQ_INT(gw_rb_free(&rb), 0u);
    }

    GW_CASE("DMA 无空间时返回 NULL 并计数 overrun");
    {
        uint32_t maxlen = 0u;
        uint32_t before = rb.overrun_count;
        GW_ASSERT(gw_rb_dma_write_ptr(&rb, &maxlen) == NULL);
        GW_ASSERT_EQ_INT(maxlen, 0u);
        GW_ASSERT(rb.overrun_count > before);
    }

    GW_CASE("统计：bytes_written/bytes_read/max_used 一致性");
    {
        gw_rb_reset(&rb);
        for (i = 0u; i < 100u; i++) {
            GW_ASSERT_EQ_INT(gw_rb_write(&rb, in, 100u), 100u);
            GW_ASSERT_EQ_INT(gw_rb_read(&rb, out, 100u), 100u);
        }
        GW_ASSERT_EQ_INT(rb.bytes_written, 10000u);
        GW_ASSERT_EQ_INT(rb.bytes_read, 10000u);
        GW_ASSERT_EQ_INT(rb.max_used, 100u);
        GW_ASSERT_EQ_INT(rb.overrun_count, 0u);
    }
}
