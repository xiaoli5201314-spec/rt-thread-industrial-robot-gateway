/*
 * mem_pool.c - 定长块内存池实现
 *
 * 数据结构：把所有块串成单链表挂在 free_list 上。
 *   分配 = 摘头, O(1)
 *   释放 = 插头, O(1)（释放前先遍历校验归属, 防止重复释放/野指针）
 * 校验遍历是 O(n) 的, 但只在"错误路径"与测试里承担代价, 正常释放
 * 走的是"范围 + 对齐"快速判断。
 */
#include "mem_pool.h"

#include <string.h>

int gw_mpool_init(gw_mpool_t *pool, const char *name, uint8_t *storage,
                  uint32_t block_size, uint32_t block_count)
{
    uint32_t i;
    uint32_t bs;

    if ((pool == NULL) || (storage == NULL) || (block_size == 0u) ||
        (block_count == 0u)) {
        return GW_ERR_PARAM;
    }

    memset(pool, 0, sizeof(*pool));

    bs = GW_ALIGN_UP(block_size, 8u);
    if (bs < sizeof(gw_mpool_block_t)) {
        bs = (uint32_t)GW_ALIGN_UP(sizeof(gw_mpool_block_t), 8u);
    }

    pool->name        = (name != NULL) ? name : "pool";
    pool->storage     = storage;
    pool->block_size  = bs;
    pool->block_count = block_count;
    pool->free_list   = NULL;

    /* 反向挂链, 使第一个分配的块地址最低, 便于调试时肉眼观察 */
    for (i = block_count; i > 0u; i--) {
        gw_mpool_block_t *b = (gw_mpool_block_t *)(void *)&storage[(i - 1u) * bs];
        b->next = pool->free_list;
        pool->free_list = b;
    }

    /* 内部互斥：保证采集/解析线程跨线程分配释放时链表不被破坏。
     * 使用优先级继承互斥（与 RT-Thread RT_IPC_FLAG_PRIO 等价）。 */
    pool->lock = gw_mutex_create(pool->name, true);
    if (pool->lock == NULL) {
        return GW_ERR_NOMEM;
    }
    return GW_OK;
}

void *gw_mpool_alloc(gw_mpool_t *pool)
{
    gw_mpool_block_t *b;

    if (pool == NULL) {
        return NULL;
    }
    if (pool->lock != NULL) {
        (void)gw_mutex_take(pool->lock, GW_WAIT_FOREVER);
    }

    b = pool->free_list;
    if (b == NULL) {
        pool->alloc_fail++;        /* 池耗尽：这是可靠性事件, 必须计数 */
        if (pool->lock != NULL) {
            (void)gw_mutex_release(pool->lock);
        }
        return NULL;
    }

    pool->free_list = b->next;
    b->next = NULL;
    pool->used_blocks++;
    pool->alloc_count++;
    if (pool->used_blocks > pool->max_used_blocks) {
        pool->max_used_blocks = pool->used_blocks;
        pool->peak_bytes = pool->used_blocks * pool->block_size;
    }

    if (pool->lock != NULL) {
        (void)gw_mutex_release(pool->lock);
    }
    return (void *)b;
}

void *gw_mpool_calloc(gw_mpool_t *pool)
{
    void *p = gw_mpool_alloc(pool);
    if (p != NULL) {
        memset(p, 0, pool->block_size);
    }
    return p;
}

bool gw_mpool_owns(gw_mpool_t *pool, const void *blk)
{
    const uint8_t *p = (const uint8_t *)blk;
    uint32_t       off;

    if ((pool == NULL) || (blk == NULL)) {
        return false;
    }
    if ((p < pool->storage) ||
        (p >= pool->storage + ((size_t)pool->block_size * pool->block_count))) {
        return false;
    }
    off = (uint32_t)(p - pool->storage);
    return (off % pool->block_size) == 0u;
}

static bool mpool_in_free_list(gw_mpool_t *pool, const void *blk)
{
    gw_mpool_block_t *b = pool->free_list;
    uint32_t guard = pool->block_count + 1u;   /* 防御性上限, 防止链表成环死循环 */

    while ((b != NULL) && (guard-- > 0u)) {
        if ((const void *)b == blk) {
            return true;
        }
        b = b->next;
    }
    return false;
}

int gw_mpool_free(gw_mpool_t *pool, void *blk)
{
    gw_mpool_block_t *b;

    if ((pool == NULL) || (blk == NULL)) {
        return GW_ERR_PARAM;
    }

    if (pool->lock != NULL) {
        (void)gw_mutex_take(pool->lock, GW_WAIT_FOREVER);
    }

    if (!gw_mpool_owns(pool, blk)) {
        pool->free_invalid++;
        if (pool->lock != NULL) {
            (void)gw_mutex_release(pool->lock);
        }
        return GW_ERR_NOT_FOUND;
    }

    if (mpool_in_free_list(pool, blk)) {
        pool->free_double++;       /* 重复释放：不破坏链表, 但要留证据 */
        if (pool->lock != NULL) {
            (void)gw_mutex_release(pool->lock);
        }
        return GW_ERR_STATE;
    }

    b = (gw_mpool_block_t *)blk;
    b->next = pool->free_list;
    pool->free_list = b;
    if (pool->used_blocks > 0u) {
        pool->used_blocks--;
    }
    pool->free_count++;

    if (pool->lock != NULL) {
        (void)gw_mutex_release(pool->lock);
    }
    return GW_OK;
}

void gw_mpool_stats(gw_mpool_t *pool, gw_mpool_stats_t *out)
{
    uint32_t used;

    if ((pool == NULL) || (out == NULL)) {
        return;
    }
    if (pool->lock != NULL) {
        (void)gw_mutex_take(pool->lock, GW_WAIT_FOREVER);
    }

    used = pool->used_blocks;

    memset(out, 0, sizeof(*out));
    out->total_blocks    = pool->block_count;
    out->block_size      = pool->block_size;
    out->used_blocks     = used;
    out->max_used_blocks = pool->max_used_blocks;
    out->free_blocks     = pool->block_count - used;
    out->alloc_count     = pool->alloc_count;
    out->free_count      = pool->free_count;
    out->alloc_fail      = pool->alloc_fail;
    out->free_invalid    = pool->free_invalid;
    out->free_double     = pool->free_double;
    out->peak_bytes      = pool->peak_bytes;

    if (pool->lock != NULL) {
        (void)gw_mutex_release(pool->lock);
    }
}

/* ================================================================== */
/* first-fit 堆模拟器                                                  */
/* ================================================================== */
/*
 * 每个块头 16 字节：{ size, is_free }。分配时首次适配 + 切分,
 * 释放时与前后相邻空闲块合并。这就是典型的会产生外部碎片的分配器,
 * 与定长池在同一串操作下做对照。
 */
typedef struct {
    uint32_t size;        /* 负载字节数（不含头部） */
    uint32_t is_free;
    uint32_t magic;       /* 用于识别非法释放 */
    uint32_t pad;
} heap_hdr_t;

#define HEAP_HDR_SIZE  ((uint32_t)sizeof(heap_hdr_t))
#define HEAP_MAGIC     0x47485750u   /* "GHWP" */

static heap_hdr_t *heap_first(gw_heapsim_t *h)
{
    return (heap_hdr_t *)(void *)h->arena;
}

static heap_hdr_t *heap_next(gw_heapsim_t *h, heap_hdr_t *cur)
{
    uint8_t *p = (uint8_t *)cur + HEAP_HDR_SIZE + cur->size;
    if (p >= h->arena + h->size) {
        return NULL;
    }
    return (heap_hdr_t *)(void *)p;
}

int gw_heapsim_init(gw_heapsim_t *h, uint8_t *arena, uint32_t size)
{
    heap_hdr_t *blk;

    if ((h == NULL) || (arena == NULL) || (size < HEAP_HDR_SIZE + 32u)) {
        return GW_ERR_PARAM;
    }
    memset(h, 0, sizeof(*h));
    h->arena = arena;
    h->size  = size;

    blk = heap_first(h);
    blk->size    = size - HEAP_HDR_SIZE;
    blk->is_free = 1u;
    blk->magic   = HEAP_MAGIC;
    blk->pad     = 0u;
    return GW_OK;
}

void *gw_heapsim_alloc(gw_heapsim_t *h, uint32_t size)
{
    heap_hdr_t *cur;
    uint32_t    need;

    if ((h == NULL) || (size == 0u)) {
        return NULL;
    }
    need = GW_ALIGN_UP(size, 8u);
    h->alloc_count++;

    cur = heap_first(h);
    while (cur != NULL) {
        if ((cur->magic != HEAP_MAGIC) || (cur->is_free == 0u) ||
            (cur->size < need)) {
            cur = heap_next(h, cur);
            continue;
        }

        /* 切分：剩余空间足够放一个头部 + 最小负载时才切 */
        if (cur->size >= need + HEAP_HDR_SIZE + 16u) {
            heap_hdr_t *nxt = (heap_hdr_t *)(void *)((uint8_t *)cur +
                                                     HEAP_HDR_SIZE + need);
            nxt->size    = cur->size - need - HEAP_HDR_SIZE;
            nxt->is_free = 1u;
            nxt->magic   = HEAP_MAGIC;
            nxt->pad     = 0u;
            cur->size    = need;
        }
        cur->is_free = 0u;
        return (void *)((uint8_t *)cur + HEAP_HDR_SIZE);
    }

    h->fail_count++;
    return NULL;
}

int gw_heapsim_free(gw_heapsim_t *h, void *p)
{
    heap_hdr_t *cur;
    heap_hdr_t *blk;

    if ((h == NULL) || (p == NULL)) {
        return GW_ERR_PARAM;
    }
    blk = (heap_hdr_t *)(void *)((uint8_t *)p - HEAP_HDR_SIZE);
    if ((blk->magic != HEAP_MAGIC) || (blk->is_free != 0u)) {
        return GW_ERR_NOT_FOUND;
    }
    blk->is_free = 1u;
    h->free_count++;

    /*
     * 合并相邻空闲块：先向后合并, 再从表头开始找到该块的前驱做向前合并。
     * 只做向后合并会留下"前一块空闲但没并上"的碎片, 碎片率统计就会
     * 失真 —— 这正是对照实验要暴露的东西, 所以这里必须做完整合并。
     */
    cur = blk;
    for (;;) {
        heap_hdr_t *nxt = heap_next(h, cur);
        if ((nxt == NULL) || (nxt->magic != HEAP_MAGIC) || (nxt->is_free == 0u)) {
            break;
        }
        cur->size += HEAP_HDR_SIZE + nxt->size;
    }

    {
        heap_hdr_t *prev = heap_first(h);
        while ((prev != NULL) && (prev != blk)) {
            heap_hdr_t *nxt = heap_next(h, prev);
            if (nxt == blk) {
                if ((prev->magic == HEAP_MAGIC) && (prev->is_free != 0u)) {
                    prev->size += HEAP_HDR_SIZE + blk->size;
                }
                break;
            }
            prev = nxt;
        }
    }
    return GW_OK;
}

void gw_heapsim_stats(gw_heapsim_t *h, gw_heapsim_stats_t *out)
{
    heap_hdr_t *cur;
    uint32_t    free_bytes = 0u;
    uint32_t    largest    = 0u;
    uint32_t    used       = 0u;

    if ((h == NULL) || (out == NULL)) {
        return;
    }

    cur = heap_first(h);
    while (cur != NULL) {
        if (cur->magic != HEAP_MAGIC) {
            break;   /* 头部被写坏, 停止统计 */
        }
        if (cur->is_free != 0u) {
            free_bytes += cur->size;
            if (cur->size > largest) {
                largest = cur->size;
            }
        } else {
            used += cur->size;
        }
        cur = heap_next(h, cur);
    }

    memset(out, 0, sizeof(*out));
    out->arena_size          = h->size;
    out->used_bytes          = used;
    out->free_bytes          = free_bytes;
    out->largest_free_block  = largest;
    out->alloc_count         = h->alloc_count;
    out->fail_count          = h->fail_count;
    out->frag_ratio_x100     = (free_bytes > 0u)
                             ? ((free_bytes - largest) * 100u / free_bytes)
                             : 0u;
}
