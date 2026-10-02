/*
 * port_hw_stub.c - PC 端硬件桩
 *
 * 提供三类替身, 让上层代码在 PC 上具备"真实可跑"的外设语义：
 *   1) 物理链路：回环（进程内可控从站）、stdio 管道、TCP socket
 *   2) 非易失存储：4KB 内存镜像, 可模拟"写一半掉电"
 *   3) 硬件故障信号：可由测试注入的位图
 *
 * 目标板上这些函数由真实驱动实现（见 port_rtthread.c 的设计说明）,
 * 因此上层代码里不会出现任何 "if (host)" 的平台判断。
 */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "port_hw.h"
#include "port_rtos.h"
#include "gateway_config.h"
#include "gw_log.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <pthread.h>

/* ================================================================== */
/* 1. 物理链路 - 回环                                                  */
/* ================================================================== */
#define LOOPBACK_RX_CAP 2048u

typedef struct {
    gw_link_slave_fn   slave_fn;
    void              *ctx;
    uint8_t            rx[LOOPBACK_RX_CAP];
    size_t             rx_head;
    size_t             rx_tail;
    pthread_mutex_t    lock;
    pthread_cond_t     cv;
    uint32_t           req_count;
    uint32_t           silent_count;   /* 从站选择不应答的次数（模拟掉线）*/
} loopback_priv_t;

static int loopback_open(gw_link_t *l, const char *cfg)
{
    GW_UNUSED(l);
    GW_UNUSED(cfg);
    return GW_OK;
}

static int loopback_send(gw_link_t *l, const uint8_t *data, size_t len,
                         int32_t timeout_ms)
{
    loopback_priv_t *p = (loopback_priv_t *)l->priv;
    uint8_t resp[GW_MAX_ADU_TCP];
    int     n;

    GW_UNUSED(timeout_ms);

    if ((p == NULL) || (data == NULL) || (len == 0u)) {
        return GW_ERR_PARAM;
    }
    if (len > GW_MAX_ADU_RTU) {
        return GW_ERR_OVERFLOW;
    }

    memset(resp, 0, sizeof(resp));
    n = p->slave_fn(p->ctx, data, len, resp, sizeof(resp));
    p->req_count++;

    if (n <= 0) {
        p->silent_count++;
        return (int)len;   /* 从站静默：视为发送成功但无响应 */
    }

    pthread_mutex_lock(&p->lock);
    {
        size_t i;
        for (i = 0; i < (size_t)n; i++) {
            size_t next = (p->rx_head + 1u) % LOOPBACK_RX_CAP;
            if (next == p->rx_tail) {
                break;   /* 环形满, 丢弃多余字节（模拟接收溢出） */
            }
            p->rx[p->rx_head] = resp[i];
            p->rx_head = next;
        }
    }
    pthread_cond_broadcast(&p->cv);
    pthread_mutex_unlock(&p->lock);

    return (int)len;
}

static int loopback_recv(gw_link_t *l, uint8_t *buf, size_t cap, int32_t timeout_ms)
{
    loopback_priv_t *p = (loopback_priv_t *)l->priv;
    size_t got = 0;
    int rc = GW_OK;

    if ((p == NULL) || (buf == NULL) || (cap == 0u)) {
        return GW_ERR_PARAM;
    }

    pthread_mutex_lock(&p->lock);
    while (p->rx_tail == p->rx_head) {
        if (timeout_ms == GW_NO_WAIT) {
            rc = GW_ERR_TIMEOUT;
            break;
        }
        if (timeout_ms == GW_WAIT_FOREVER) {
            pthread_cond_wait(&p->cv, &p->lock);
        } else {
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_sec  += (time_t)(timeout_ms / 1000);
            ts.tv_nsec += (long)((timeout_ms % 1000) * 1000000);
            if (ts.tv_nsec >= 1000000000L) {
                ts.tv_sec += 1;
                ts.tv_nsec -= 1000000000L;
            }
            if (pthread_cond_timedwait(&p->cv, &p->lock, &ts) == ETIMEDOUT) {
                rc = GW_ERR_TIMEOUT;
                break;
            }
        }
    }

    if (rc == GW_OK) {
        while ((got < cap) && (p->rx_tail != p->rx_head)) {
            buf[got++] = p->rx[p->rx_tail];
            p->rx_tail = (p->rx_tail + 1u) % LOOPBACK_RX_CAP;
        }
    }
    pthread_mutex_unlock(&p->lock);

    return (rc == GW_OK) ? (int)got : rc;
}

static void loopback_close(gw_link_t *l)
{
    GW_UNUSED(l);
}

gw_link_t *gw_link_loopback_create(gw_link_slave_fn slave_fn, void *ctx)
{
    gw_link_t       *l;
    loopback_priv_t *p;

    l = (gw_link_t *)calloc(1, sizeof(*l));
    p = (loopback_priv_t *)calloc(1, sizeof(*p));
    if ((l == NULL) || (p == NULL)) {
        free(l);
        free(p);
        return NULL;
    }
    p->slave_fn = slave_fn;
    p->ctx      = ctx;
    pthread_mutex_init(&p->lock, NULL);
    pthread_cond_init(&p->cv, NULL);

    l->name  = "loopback";
    l->open  = loopback_open;
    l->send  = loopback_send;
    l->recv  = loopback_recv;
    l->close = loopback_close;
    l->priv  = p;
    return l;
}

/* ================================================================== */
/* 2. 物理链路 - stdio 管道                                            */
/* ================================================================== */
static int stdio_open(gw_link_t *l, const char *cfg)
{
    GW_UNUSED(l);
    GW_UNUSED(cfg);
    return GW_OK;
}

static int stdio_send(gw_link_t *l, const uint8_t *data, size_t len, int32_t timeout_ms)
{
    size_t written;
    GW_UNUSED(l);
    GW_UNUSED(timeout_ms);

    if ((data == NULL) || (len == 0u)) {
        return GW_ERR_PARAM;
    }
    written = fwrite(data, 1u, len, stdout);
    fflush(stdout);
    return (written == len) ? (int)len : GW_ERR_IO;
}

static int stdio_recv(gw_link_t *l, uint8_t *buf, size_t cap, int32_t timeout_ms)
{
    struct pollfd pfd;
    int           pr;
    ssize_t       n;

    GW_UNUSED(l);
    if ((buf == NULL) || (cap == 0u)) {
        return GW_ERR_PARAM;
    }

    pfd.fd      = STDIN_FILENO;
    pfd.events  = POLLIN;
    pfd.revents = 0;

    pr = poll(&pfd, 1, (timeout_ms == GW_WAIT_FOREVER) ? -1 : (int)timeout_ms);
    if (pr == 0) {
        return GW_ERR_TIMEOUT;
    }
    if (pr < 0) {
        return (errno == EINTR) ? GW_ERR_TIMEOUT : GW_ERR_IO;
    }

    n = read(STDIN_FILENO, buf, cap);
    if (n == 0) {
        return GW_ERR_IO;      /* 对端关闭 */
    }
    if (n < 0) {
        return GW_ERR_IO;
    }
    return (int)n;
}

static void stdio_close(gw_link_t *l)
{
    GW_UNUSED(l);
}

gw_link_t *gw_link_stdio_create(void)
{
    gw_link_t *l = (gw_link_t *)calloc(1, sizeof(*l));
    if (l == NULL) {
        return NULL;
    }
    l->name  = "stdio";
    l->open  = stdio_open;
    l->send  = stdio_send;
    l->recv  = stdio_recv;
    l->close = stdio_close;
    return l;
}

/* ================================================================== */
/* 3. 物理链路 - TCP                                                   */
/* ================================================================== */
typedef struct {
    int fd;
    int listen_fd;
} tcp_priv_t;

static void tcp_apply_nodelay(int fd)
{
    int one = 1;
    (void)setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
}

static int tcp_open(gw_link_t *l, const char *cfg)
{
    tcp_priv_t       *p = (tcp_priv_t *)l->priv;
    struct sockaddr_in sa;
    char              host[64];
    int               port = 0;
    const char       *colon;
    int               fd;

    if ((p == NULL) || (cfg == NULL)) {
        return GW_ERR_PARAM;
    }

    memset(&sa, 0, sizeof(sa));
    sa.sin_family = AF_INET;

    colon = strrchr(cfg, ':');
    if (colon != NULL) {
        size_t hl = (size_t)(colon - cfg);
        if (hl >= sizeof(host)) {
            hl = sizeof(host) - 1u;
        }
        memcpy(host, cfg, hl);
        host[hl] = '\0';
        port     = atoi(colon + 1);
    } else {
        snprintf(host, sizeof(host), "0.0.0.0");
        port = atoi(cfg);
    }
    if (port <= 0) {
        return GW_ERR_PARAM;
    }
    sa.sin_port = htons((uint16_t)port);

    if (strcmp(host, "listen") == 0) {
        int one = 1;
        sa.sin_addr.s_addr = htonl(INADDR_ANY);
        fd = socket(AF_INET, SOCK_STREAM, 0);
        if (fd < 0) {
            return GW_ERR_IO;
        }
        (void)setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        if (bind(fd, (struct sockaddr *)&sa, sizeof(sa)) != 0) {
            GW_LOGE("LINK", "tcp bind :%d failed: %s", port, strerror(errno));
            close(fd);
            return GW_ERR_IO;
        }
        if (listen(fd, 1) != 0) {
            close(fd);
            return GW_ERR_IO;
        }
        p->listen_fd = fd;
        GW_LOGI("LINK", "tcp listen on 0.0.0.0:%d, waiting for slave...", port);
        p->fd = accept(fd, NULL, NULL);
        if (p->fd < 0) {
            return GW_ERR_IO;
        }
        tcp_apply_nodelay(p->fd);
        GW_LOGI("LINK", "slave connected");
        return GW_OK;
    }

    if (inet_pton(AF_INET, host, &sa.sin_addr) != 1) {
        /* 允许 "localhost" 之类的写法 */
        if (strcmp(host, "localhost") == 0) {
            sa.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        } else {
            return GW_ERR_PARAM;
        }
    }

    fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        return GW_ERR_IO;
    }
    if (connect(fd, (struct sockaddr *)&sa, sizeof(sa)) != 0) {
        GW_LOGE("LINK", "tcp connect %s:%d failed: %s", host, port, strerror(errno));
        close(fd);
        return GW_ERR_IO;
    }
    tcp_apply_nodelay(fd);
    p->fd = fd;
    return GW_OK;
}

static int tcp_send(gw_link_t *l, const uint8_t *data, size_t len, int32_t timeout_ms)
{
    tcp_priv_t *p = (tcp_priv_t *)l->priv;
    struct pollfd pfd;
    size_t sent = 0;

    GW_UNUSED(timeout_ms);
    if ((p == NULL) || (p->fd < 0) || (data == NULL)) {
        return GW_ERR_IO;
    }

    while (sent < len) {
        ssize_t n = send(p->fd, data + sent, len - sent, MSG_NOSIGNAL);
        if (n > 0) {
            sent += (size_t)n;
            continue;
        }
        if ((n < 0) && ((errno == EAGAIN) || (errno == EWOULDBLOCK))) {
            pfd.fd     = p->fd;
            pfd.events = POLLOUT;
            if (poll(&pfd, 1, 500) <= 0) {
                return GW_ERR_TIMEOUT;
            }
            continue;
        }
        if ((n < 0) && (errno == EINTR)) {
            continue;
        }
        return GW_ERR_IO;
    }
    return (int)sent;
}

static int tcp_recv(gw_link_t *l, uint8_t *buf, size_t cap, int32_t timeout_ms)
{
    tcp_priv_t   *p = (tcp_priv_t *)l->priv;
    struct pollfd pfd;
    int           pr;
    ssize_t       n;

    if ((p == NULL) || (p->fd < 0) || (buf == NULL) || (cap == 0u)) {
        return GW_ERR_IO;
    }

    pfd.fd     = p->fd;
    pfd.events = POLLIN;
    pr = poll(&pfd, 1, (timeout_ms == GW_WAIT_FOREVER) ? -1 : (int)timeout_ms);
    if (pr == 0) {
        return GW_ERR_TIMEOUT;
    }
    if (pr < 0) {
        return (errno == EINTR) ? GW_ERR_TIMEOUT : GW_ERR_IO;
    }
    if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        return GW_ERR_IO;
    }

    n = recv(p->fd, buf, cap, 0);
    if (n == 0) {
        return GW_ERR_IO;      /* 对端关闭：等价于设备掉线 */
    }
    if (n < 0) {
        return GW_ERR_IO;
    }
    return (int)n;
}

static void tcp_close(gw_link_t *l)
{
    tcp_priv_t *p = (tcp_priv_t *)l->priv;
    if (p != NULL) {
        if (p->fd >= 0) {
            close(p->fd);
            p->fd = -1;
        }
        if (p->listen_fd >= 0) {
            close(p->listen_fd);
            p->listen_fd = -1;
        }
    }
}

static gw_link_t *tcp_link_create(const char *nname)
{
    gw_link_t  *l = (gw_link_t *)calloc(1, sizeof(*l));
    tcp_priv_t *p = (tcp_priv_t *)calloc(1, sizeof(*p));
    if ((l == NULL) || (p == NULL)) {
        free(l);
        free(p);
        return NULL;
    }
    p->fd        = -1;
    p->listen_fd = -1;
    l->name  = nname;
    l->open  = tcp_open;
    l->send  = tcp_send;
    l->recv  = tcp_recv;
    l->close = tcp_close;
    l->priv  = p;
    return l;
}

gw_link_t *gw_link_tcp_listen_create(void)
{
    return tcp_link_create("tcp-listen");
}

gw_link_t *gw_link_tcp_connect_create(void)
{
    return tcp_link_create("tcp-connect");
}

/* ================================================================== */
/* 4. 物理链路 - CAN（PC 桩）                                          */
/* ================================================================== */
static int can_send(gw_link_t *l, const uint8_t *data, size_t len, int32_t timeout_ms)
{
    GW_UNUSED(l); GW_UNUSED(data); GW_UNUSED(len); GW_UNUSED(timeout_ms);
    return GW_ERR_UNSUPPORTED;   /* PC 端无 CAN 控制器, 由测试直接调用帧解析 */
}

static int can_recv(gw_link_t *l, uint8_t *buf, size_t cap, int32_t timeout_ms)
{
    GW_UNUSED(l); GW_UNUSED(buf); GW_UNUSED(cap); GW_UNUSED(timeout_ms);
    return GW_ERR_UNSUPPORTED;
}

static int can_open(gw_link_t *l, const char *cfg)
{
    GW_UNUSED(l); GW_UNUSED(cfg);
    return GW_OK;
}

static void can_close(gw_link_t *l)
{
    GW_UNUSED(l);
}

gw_link_t *gw_link_can_create(void)
{
    gw_link_t *l = (gw_link_t *)calloc(1, sizeof(*l));
    if (l == NULL) {
        return NULL;
    }
    l->name  = "can-stub";
    l->open  = can_open;
    l->send  = can_send;
    l->recv  = can_recv;
    l->close = can_close;
    return l;
}

/* ================================================================== */
/* 统一入口                                                            */
/* ================================================================== */
int gw_link_open(gw_link_t *l, const char *cfg)
{
    if ((l == NULL) || (l->open == NULL)) {
        return GW_ERR_PARAM;
    }
    return l->open(l, cfg);
}

int gw_link_send(gw_link_t *l, const uint8_t *data, size_t len, int32_t timeout_ms)
{
    if ((l == NULL) || (l->send == NULL)) {
        return GW_ERR_PARAM;
    }
    return l->send(l, data, len, timeout_ms);
}

int gw_link_recv(gw_link_t *l, uint8_t *buf, size_t cap, int32_t timeout_ms)
{
    if ((l == NULL) || (l->recv == NULL)) {
        return GW_ERR_PARAM;
    }
    return l->recv(l, buf, cap, timeout_ms);
}

void gw_link_close(gw_link_t *l)
{
    if ((l != NULL) && (l->close != NULL)) {
        l->close(l);
    }
}

/* ================================================================== */
/* 5. 非易失存储桩                                                     */
/* ================================================================== */
static uint8_t  s_nv[GW_NV_TOTAL_SIZE];
static bool     s_nv_inited      = false;
static bool     s_nv_torn_write  = false;
static uint32_t s_nv_erase_count = 0;
static uint32_t s_nv_write_count = 0;

static void nv_ensure(void)
{
    if (!s_nv_inited) {
        memset(s_nv, 0xFF, sizeof(s_nv));   /* NOR Flash 擦除后为 0xFF */
        s_nv_inited = true;
    }
}

int gw_nv_read(uint32_t offset, uint8_t *buf, size_t len)
{
    nv_ensure();
    if ((buf == NULL) || ((size_t)offset + len > GW_NV_TOTAL_SIZE)) {
        return GW_ERR_PARAM;
    }
    memcpy(buf, &s_nv[offset], len);
    return GW_OK;
}

int gw_nv_write(uint32_t offset, const uint8_t *buf, size_t len)
{
    size_t n;

    nv_ensure();
    if ((buf == NULL) || ((size_t)offset + len > GW_NV_TOTAL_SIZE)) {
        return GW_ERR_PARAM;
    }

    n = len;
    if (s_nv_torn_write) {
        /* 模拟"擦除后刚开始编程就掉电"：只写进去 8 个字节。
         * 这是双槽设计必须扛住的最坏情况 —— 槽头 CRC 直接不成立,
         * 上电时该槽被判为无效, 自动回退到另一个槽。 */
        n = (len > 8u) ? 8u : len / 2u;
        GW_LOGW("NV", "simulated power loss: %u/%u bytes programmed",
                (unsigned)n, (unsigned)len);
    }

    /* NOR 语义：只能把 1 写成 0, 除非先擦除 */
    {
        size_t i;
        for (i = 0; i < n; i++) {
            s_nv[offset + i] &= buf[i];
        }
    }
    s_nv_write_count++;
    return (n == len) ? GW_OK : GW_ERR_IO;
}

int gw_nv_erase(uint32_t offset, size_t len)
{
    nv_ensure();
    if ((size_t)offset + len > GW_NV_TOTAL_SIZE) {
        return GW_ERR_PARAM;
    }
    memset(&s_nv[offset], 0xFF, len);
    s_nv_erase_count++;
    return GW_OK;
}

void gw_nv_simulate_torn_write(bool enable)
{
    s_nv_torn_write = enable;
}

uint32_t gw_nv_get_erase_count(void)
{
    return s_nv_erase_count;
}

uint32_t gw_nv_get_write_count(void)
{
    return s_nv_write_count;
}

/* ================================================================== */
/* 6. 硬件故障信号桩                                                   */
/* ================================================================== */
static uint32_t s_hw_faults = 0u;
static bool     s_pwr_fail  = false;

uint32_t gw_hw_get_faults(void)
{
    return s_hw_faults;
}

void gw_hw_clear_faults(uint32_t mask)
{
    s_hw_faults &= ~mask;
}

void gw_hw_inject_fault(uint32_t mask)
{
    s_hw_faults |= mask;
}

bool gw_hw_power_fail_pending(void)
{
    return s_pwr_fail;
}

void gw_hw_inject_power_fail(bool pending)
{
    s_pwr_fail = pending;
}
