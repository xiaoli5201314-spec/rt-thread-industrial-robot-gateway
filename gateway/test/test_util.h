/*
 * test_util.h - 极简测试框架
 *
 * 不引入任何第三方测试库（避免版权与依赖问题）, 只用一个全局计数器
 * 加几条宏。断言失败会打印文件/行号/表达式与实参值, 便于直接定位。
 */
#ifndef GW_TEST_UTIL_H
#define GW_TEST_UTIL_H

#include <stdio.h>
#include <string.h>

extern int g_assert_total;
extern int g_assert_failed;
extern int g_case_total;

#define GW_SUITE(name) \
    do { printf("\n===== SUITE: %s =====\n", (name)); } while (0)

#define GW_CASE(name) \
    do { g_case_total++; printf("  -- %s\n", (name)); } while (0)

#define GW_ASSERT(cond)                                                        \
    do {                                                                       \
        g_assert_total++;                                                      \
        if (!(cond)) {                                                         \
            g_assert_failed++;                                                 \
            printf("     [FAIL] %s:%d  %s\n", __FILE__, __LINE__, #cond);      \
        }                                                                      \
    } while (0)

#define GW_ASSERT_EQ_INT(actual, expect)                                       \
    do {                                                                       \
        long _a = (long)(actual);                                              \
        long _e = (long)(expect);                                              \
        g_assert_total++;                                                      \
        if (_a != _e) {                                                        \
            g_assert_failed++;                                                 \
            printf("     [FAIL] %s:%d  %s == %s  (actual=%ld expect=%ld)\n",   \
                   __FILE__, __LINE__, #actual, #expect, _a, _e);              \
        }                                                                      \
    } while (0)

#define GW_ASSERT_EQ_MEM(actual, expect, len)                                  \
    do {                                                                       \
        g_assert_total++;                                                      \
        if (memcmp((actual), (expect), (len)) != 0) {                          \
            g_assert_failed++;                                                 \
            printf("     [FAIL] %s:%d  memcmp(%s, %s, %s) != 0\n",             \
                   __FILE__, __LINE__, #actual, #expect, #len);                \
        }                                                                      \
    } while (0)

#define GW_ASSERT_STR(actual, expect)                                          \
    do {                                                                       \
        g_assert_total++;                                                      \
        if (strcmp((actual), (expect)) != 0) {                                 \
            g_assert_failed++;                                                 \
            printf("     [FAIL] %s:%d  \"%s\" != \"%s\"\n",                    \
                   __FILE__, __LINE__, (actual), (expect));                    \
        }                                                                      \
    } while (0)

/* 十六进制打印缓冲区, 调试用 */
static inline void gw_test_hexdump(const char *tag, const unsigned char *p, int n)
{
    int i;
    printf("     %s [%d]:", tag, n);
    for (i = 0; i < n; i++) {
        printf(" %02X", p[i]);
    }
    printf("\n");
}

/* 各测试套件入口 */
void test_crc_and_types(void);
void test_modbus_rtu(void);
void test_modbus_tcp(void);
void test_frame_sync(void);
void test_mem_pool(void);
void test_ring_buffer(void);
void test_device_model(void);
void test_bus_health(void);
void test_canopen(void);
void test_param_store(void);
void test_port_objects(void);
void test_priority_inherit(void);

#endif /* GW_TEST_UTIL_H */
