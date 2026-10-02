/*
 * mem_pool.h - 定长块内存池 + 堆碎片对照实验工具
 *
 * 为什么不用 malloc（简历"用内存池替代频繁动态分配避免堆碎片"的落地）：
 *   - 协议栈里有三类对象是"尺寸设计期已知、生命周期短、频率高"的：
 *     帧描述符、ADU 缓冲、参数快照。对它们使用通用堆只会引入
 *     外部碎片与不确定的最坏分配时间, 收益为零；
 *   - 定长块池用空闲链表串起所有块, 分配/释放都是 O(1) 且时间恒定,
 *     非常适合放在实时链路的热路径上；
 *   - 池内存在编译期静态数组中, 跑飞时也不会把别的模块的堆踩坏。
 *
 * gw_heapsim_* 是一段**仅用于对照实验**的 first-fit 堆模拟器：
 * 单元测试用同一串分配/释放序列分别打在两套分配器上, 用数据说明
 * "定长池不产生外部碎片"这个结论, 而不是只写在文档里。
 */
#ifndef GW_MEM_POOL_H
#define GW_MEM_POOL_H

#include "gw_types.h"
#include "port_rtos.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 块数据结构（内嵌在空闲块中, 不额外占内存） */
typedef struct gw_mpool_block {
    struct gw_mpool_block *next;
} gw_mpool_block_t;

typedef struct {
    const char    *name;
    uint8_t       *storage;
    uint32_t       block_size;      /* 对齐到 8 字节                    */
    uint32_t       block_count;
    gw_mpool_block_t *free_list;
    gw_mutex_t    *lock;            /* 内部互斥, 保证多线程分配安全      */

    /* 运行统计：诊断线程周期性上报, 也是测试断言的依据 */
    uint32_t used_blocks;
    uint32_t max_used_blocks;
    uint32_t alloc_count;
    uint32_t free_count;
    uint32_t alloc_fail;            /* 耗尽次数（0 才说明容量设计合理）  */
    uint32_t free_invalid;          /* 非法指针释放次数（内存踩踏预警）  */
    uint32_t free_double;           /* 重复释放次数                     */
    uint32_t peak_bytes;
} gw_mpool_t;

typedef struct {
    uint32_t total_blocks;
    uint32_t block_size;
    uint32_t used_blocks;
    uint32_t max_used_blocks;
    uint32_t free_blocks;
    uint32_t alloc_count;
    uint32_t free_count;
    uint32_t alloc_fail;
    uint32_t free_invalid;
    uint32_t free_double;
    uint32_t peak_bytes;
} gw_mpool_stats_t;

/* 计算容纳 count 个 block_size 字节块所需的存储字节数 */
static inline uint32_t gw_mpool_storage_size(uint32_t block_size, uint32_t count)
{
    return GW_ALIGN_UP(block_size, 8u) * count;
}

int      gw_mpool_init(gw_mpool_t *pool, const char *name, uint8_t *storage,
                       uint32_t block_size, uint32_t block_count);
void    *gw_mpool_alloc(gw_mpool_t *pool);          /* 失败返回 NULL（池耗尽）*/
void    *gw_mpool_calloc(gw_mpool_t *pool);         /* 分配并清零 */
int      gw_mpool_free(gw_mpool_t *pool, void *blk);/* GW_OK / GW_ERR_NOT_FOUND */
bool     gw_mpool_owns(gw_mpool_t *pool, const void *blk);
void     gw_mpool_stats(gw_mpool_t *pool, gw_mpool_stats_t *out);

/* ================================================================== */
/* first-fit 堆模拟器（仅用于碎片对照实验）                            */
/* ================================================================== */
typedef struct {
    uint8_t       *arena;
    uint32_t       size;
    uint32_t       alloc_count;
    uint32_t       fail_count;
    uint32_t       free_count;
} gw_heapsim_t;

typedef struct {
    uint32_t arena_size;
    uint32_t used_bytes;
    uint32_t free_bytes;
    uint32_t largest_free_block;
    uint32_t alloc_count;
    uint32_t fail_count;
    /* 外部碎片率 = (空闲总量 - 最大空闲块) / 空闲总量 * 100 */
    uint32_t frag_ratio_x100;
} gw_heapsim_stats_t;

int      gw_heapsim_init(gw_heapsim_t *h, uint8_t *arena, uint32_t size);
void    *gw_heapsim_alloc(gw_heapsim_t *h, uint32_t size);
int      gw_heapsim_free(gw_heapsim_t *h, void *p);
void     gw_heapsim_stats(gw_heapsim_t *h, gw_heapsim_stats_t *out);

#ifdef __cplusplus
}
#endif

#endif /* GW_MEM_POOL_H */
