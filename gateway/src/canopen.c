/*
 * canopen.c - CANopen 协议实现
 *
 * 只实现工程里真正用到的子集：预定义连接集、NMT、心跳、SDO（加速+分段）、
 * PDO 映射解码、EMCY。这样代码量可控、每一行都能在面试里讲清楚,
 * 也避免引入"抄来的但没人看得懂"的协议栈。
 */
#include "canopen.h"

#include <string.h>

/* ================================================================== */
/* COB-ID 解码                                                         */
/* ================================================================== */
const char *gw_canopen_cob_str(gw_canopen_cob_t c)
{
    switch (c) {
    case GW_COB_NMT:       return "NMT";
    case GW_COB_SYNC:      return "SYNC";
    case GW_COB_EMCY:      return "EMCY";
    case GW_COB_TIME:      return "TIME";
    case GW_COB_TPDO1:     return "TPDO1";
    case GW_COB_RPDO1:     return "RPDO1";
    case GW_COB_TPDO2:     return "TPDO2";
    case GW_COB_RPDO2:     return "RPDO2";
    case GW_COB_TPDO3:     return "TPDO3";
    case GW_COB_RPDO3:     return "RPDO3";
    case GW_COB_TPDO4:     return "TPDO4";
    case GW_COB_RPDO4:     return "RPDO4";
    case GW_COB_SDO_TX:    return "SDO_TX";
    case GW_COB_SDO_RX:    return "SDO_RX";
    case GW_COB_HEARTBEAT: return "HEARTBEAT";
    case GW_COB_LSS_TX:    return "LSS_TX";
    case GW_COB_LSS_RX:    return "LSS_RX";
    default:               return "UNKNOWN";
    }
}

int gw_canopen_decode_cob(uint32_t cob_id, gw_canopen_cob_t *type,
                          uint8_t *node_id)
{
    uint32_t base;
    uint8_t  node;

    if ((type == NULL) || (node_id == NULL)) {
        return GW_ERR_PARAM;
    }
    if (cob_id > 0x7FFu) {
        return GW_ERR_UNSUPPORTED;      /* 仅处理 CANopen 标准 11 位 ID */
    }

    *node_id = 0u;

    if (cob_id == GW_COB_NMT) {
        *type = GW_COB_NMT;
        return GW_OK;
    }
    if (cob_id == GW_COB_SYNC) {
        *type = GW_COB_SYNC;
        return GW_OK;
    }
    if (cob_id == GW_COB_TIME) {
        *type = GW_COB_TIME;
        return GW_OK;
    }
    if ((cob_id >= GW_COB_LSS_TX) && (cob_id <= GW_COB_LSS_RX)) {
        *type = (cob_id == GW_COB_LSS_TX) ? GW_COB_LSS_TX : GW_COB_LSS_RX;
        return GW_OK;
    }

    /* 其余功能码的低 7 位为节点号, 高 4 位为功能基址 */
    node = (uint8_t)(cob_id & 0x7Fu);
    base = cob_id & 0x780u;

    switch (base) {
    case 0x080u: *type = GW_COB_EMCY;      *node_id = node; return GW_OK;
    case 0x100u: *type = GW_COB_TIME;                 return GW_OK;
    case 0x180u: *type = GW_COB_TPDO1;     *node_id = node; return GW_OK;
    case 0x200u: *type = GW_COB_RPDO1;     *node_id = node; return GW_OK;
    case 0x280u: *type = GW_COB_TPDO2;     *node_id = node; return GW_OK;
    case 0x300u: *type = GW_COB_RPDO2;     *node_id = node; return GW_OK;
    case 0x380u: *type = GW_COB_TPDO3;     *node_id = node; return GW_OK;
    case 0x400u: *type = GW_COB_RPDO3;     *node_id = node; return GW_OK;
    case 0x480u: *type = GW_COB_TPDO4;     *node_id = node; return GW_OK;
    case 0x500u: *type = GW_COB_RPDO4;     *node_id = node; return GW_OK;
    case 0x580u: *type = GW_COB_SDO_TX;    *node_id = node; return GW_OK;
    case 0x600u: *type = GW_COB_SDO_RX;    *node_id = node; return GW_OK;
    case 0x700u: *type = GW_COB_HEARTBEAT; *node_id = node; return GW_OK;
    default:
        *type = GW_COB_UNKNOWN;
        return GW_ERR_NOT_FOUND;
    }
}

int gw_canopen_dispatch(const gw_can_frame_t *f, gw_canopen_cob_t *type,
                        uint8_t *node_id, gw_canopen_stats_t *stats)
{
    gw_canopen_cob_t t = GW_COB_UNKNOWN;
    uint8_t          n = 0u;
    int              rc;

    if ((f == NULL) || (type == NULL) || (node_id == NULL)) {
        return GW_ERR_PARAM;
    }
    if (f->dlc > GW_CAN_MAX_DLC) {
        if (stats != NULL) {
            stats->frames_bad_dlc++;
        }
        return GW_ERR_PARAM;
    }

    rc = gw_canopen_decode_cob(f->id, &t, &n);
    *type    = t;
    *node_id = n;

    if (stats == NULL) {
        return rc;
    }

    stats->frames_total++;
    switch (t) {
    case GW_COB_NMT:       stats->frames_nmt++;       break;
    case GW_COB_SYNC:      stats->frames_sync++;      break;
    case GW_COB_EMCY:      stats->frames_emcy++;      break;
    case GW_COB_TPDO1: case GW_COB_TPDO2:
    case GW_COB_TPDO3: case GW_COB_TPDO4:
    case GW_COB_RPDO1: case GW_COB_RPDO2:
    case GW_COB_RPDO3: case GW_COB_RPDO4:
        stats->frames_pdo++;                          break;
    case GW_COB_SDO_TX: case GW_COB_SDO_RX:
        stats->frames_sdo++;                          break;
    case GW_COB_HEARTBEAT: stats->frames_heartbeat++; break;
    default:               stats->frames_unknown++;   break;
    }
    return rc;
}

/* ================================================================== */
/* NMT                                                                */
/* ================================================================== */
int gw_canopen_nmt_build(uint8_t command, uint8_t node_id, gw_can_frame_t *out)
{
    if (out == NULL) {
        return GW_ERR_PARAM;
    }
    if (node_id > 0x7Fu) {
        return GW_ERR_PARAM;
    }
    memset(out, 0, sizeof(*out));
    out->id       = GW_COB_NMT;
    out->extended = false;
    out->dlc      = 2u;
    out->data[0]  = command;
    out->data[1]  = node_id;              /* 0 = 广播 */
    return GW_OK;
}

int gw_canopen_nmt_parse(const gw_can_frame_t *f, uint8_t *command, uint8_t *node_id)
{
    if ((f == NULL) || (command == NULL) || (node_id == NULL)) {
        return GW_ERR_PARAM;
    }
    if ((f->id != GW_COB_NMT) || (f->dlc < 2u)) {
        return GW_ERR_PARAM;
    }
    *command = f->data[0];
    *node_id = f->data[1];
    return GW_OK;
}

/* ================================================================== */
/* 心跳                                                                */
/* ================================================================== */
int gw_canopen_hb_init(gw_canopen_hb_t *hb)
{
    if (hb == NULL) {
        return GW_ERR_PARAM;
    }
    memset(hb, 0, sizeof(*hb));
    return GW_OK;
}

int gw_canopen_hb_find(gw_canopen_hb_t *hb, uint8_t node_id)
{
    uint32_t i;
    if (hb == NULL) {
        return GW_ERR_PARAM;
    }
    for (i = 0u; i < GW_CANOPEN_MAX_NODES; i++) {
        if (hb->nodes[i].active && (hb->nodes[i].node_id == node_id)) {
            return (int)i;
        }
    }
    return -1;
}

int gw_canopen_hb_register(gw_canopen_hb_t *hb, uint8_t node_id, uint32_t timeout_ms)
{
    uint32_t i;

    if (hb == NULL) {
        return GW_ERR_PARAM;
    }
    if (gw_canopen_hb_find(hb, node_id) >= 0) {
        return GW_ERR_STATE;               /* 重复注册 */
    }
    for (i = 0u; i < GW_CANOPEN_MAX_NODES; i++) {
        if (!hb->nodes[i].active) {
            hb->nodes[i].active       = true;
            hb->nodes[i].node_id      = node_id;
            hb->nodes[i].timeout_ms   = (timeout_ms == 0u) ? GW_HEARTBEAT_TIMEOUT_MS
                                                           : timeout_ms;
            hb->nodes[i].last_seen_ms = gw_port_tick_ms();
            hb->nodes[i].state        = GW_NMT_STATE_BOOTUP;
            hb->nodes[i].online       = false;
            return (int)i;
        }
    }
    return GW_ERR_FULL;
}

int gw_canopen_hb_feed(gw_canopen_hb_t *hb, const gw_can_frame_t *f, uint32_t now_ms)
{
    gw_canopen_cob_t type = GW_COB_UNKNOWN;
    uint8_t          node = 0u;
    int              idx;

    if ((hb == NULL) || (f == NULL)) {
        return GW_ERR_PARAM;
    }
    if (gw_canopen_decode_cob(f->id, &type, &node) != GW_OK) {
        return GW_ERR_NOT_FOUND;
    }
    if (type != GW_COB_HEARTBEAT) {
        return GW_ERR_UNSUPPORTED;
    }

    idx = gw_canopen_hb_find(hb, node);
    if (idx < 0) {
        return GW_ERR_NOT_FOUND;
    }

    hb->nodes[idx].last_seen_ms = now_ms;
    hb->nodes[idx].heartbeat_count++;
    if (f->dlc >= 1u) {
        hb->nodes[idx].state = f->data[0];
    }
    if (!hb->nodes[idx].online) {
        hb->nodes[idx].online = true;
        hb->online_count++;
    }
    return idx;
}

int gw_canopen_hb_check(gw_canopen_hb_t *hb, uint32_t now_ms)
{
    uint32_t i;
    int      newly_offline = 0;

    if (hb == NULL) {
        return GW_ERR_PARAM;
    }
    for (i = 0u; i < GW_CANOPEN_MAX_NODES; i++) {
        gw_canopen_hb_node_t *n = &hb->nodes[i];
        if (!n->active) {
            continue;
        }
        if ((now_ms - n->last_seen_ms) > n->timeout_ms) {
            if (n->online) {
                n->online = false;
                if (hb->online_count > 0u) {
                    hb->online_count--;
                }
                newly_offline++;
            }
            n->timeout_count++;
            hb->total_timeouts++;
        }
    }
    return newly_offline;
}

/* ================================================================== */
/* 对象字典                                                            */
/* ================================================================== */
uint32_t gw_od_type_size(gw_od_type_t t)
{
    switch (t) {
    case GW_OD_U8:  return 1u;
    case GW_OD_U16: return 2u;
    case GW_OD_I16: return 2u;
    case GW_OD_U32: return 4u;
    case GW_OD_I32: return 4u;
    case GW_OD_F32: return 4u;
    case GW_OD_STRING: return 0u;     /* 变长 */
    default: return 0u;
    }
}

const char *gw_od_type_str(gw_od_type_t t)
{
    switch (t) {
    case GW_OD_U8:     return "UINT8";
    case GW_OD_U16:    return "UINT16";
    case GW_OD_U32:    return "UINT32";
    case GW_OD_I16:    return "INT16";
    case GW_OD_I32:    return "INT32";
    case GW_OD_F32:    return "REAL32";
    case GW_OD_STRING: return "VISIBLE_STRING";
    default:           return "UNKNOWN";
    }
}

const gw_od_entry_t *gw_od_lookup(const gw_od_t *od, uint16_t index, uint8_t subindex)
{
    uint32_t i;

    if (od == NULL) {
        return NULL;
    }
    for (i = 0u; i < od->count; i++) {
        if ((od->entries[i].index == index) &&
            (od->entries[i].subindex == subindex)) {
            return &od->entries[i];
        }
    }
    return NULL;
}

int gw_od_read_u32(const gw_od_entry_t *e, uint32_t *out)
{
    if ((e == NULL) || (out == NULL) || (e->storage == NULL)) {
        return GW_ERR_PARAM;
    }
    if ((e->access & GW_OD_ACCESS_RO) == 0u) {
        return GW_ERR_STATE;              /* 只写对象不可读 */
    }

    switch (e->type) {
    case GW_OD_U8:  *out = *(const uint8_t *)e->storage;  return GW_OK;
    case GW_OD_U16: *out = *(const uint16_t *)e->storage; return GW_OK;
    case GW_OD_I16: *out = (uint32_t)(int32_t)(*(const int16_t *)e->storage);
                    return GW_OK;
    case GW_OD_U32: *out = *(const uint32_t *)e->storage; return GW_OK;
    case GW_OD_I32: *out = (uint32_t)(*(const int32_t *)e->storage); return GW_OK;
    case GW_OD_F32: {
        float f = *(const float *)e->storage;
        memcpy(out, &f, sizeof(f));
        return GW_OK;
    }
    default:
        return GW_ERR_UNSUPPORTED;
    }
}

int gw_od_write_u32(const gw_od_entry_t *e, uint32_t value)
{
    if ((e == NULL) || (e->storage == NULL)) {
        return GW_ERR_PARAM;
    }
    if ((e->access & GW_OD_ACCESS_WO) == 0u) {
        return GW_ERR_STATE;              /* 只读对象不可写 */
    }

    switch (e->type) {
    case GW_OD_U8:  *(uint8_t *)e->storage  = (uint8_t)value;        return GW_OK;
    case GW_OD_U16: *(uint16_t *)e->storage = (uint16_t)value;       return GW_OK;
    case GW_OD_I16: *(int16_t *)e->storage  = (int16_t)(value & 0xFFFFu);
                    return GW_OK;
    case GW_OD_U32: *(uint32_t *)e->storage = value;                 return GW_OK;
    case GW_OD_I32: *(int32_t *)e->storage  = (int32_t)value;        return GW_OK;
    case GW_OD_F32: {
        float f;
        memcpy(&f, &value, sizeof(f));
        *(float *)e->storage = f;
        return GW_OK;
    }
    default:
        return GW_ERR_UNSUPPORTED;
    }
}

/* ================================================================== */
/* SDO                                                                */
/* ================================================================== */
static int sdo_build(uint8_t node_id, bool to_slave, uint8_t cs,
                     uint16_t index, uint8_t subindex, uint32_t value,
                     uint8_t nbytes, gw_can_frame_t *out)
{
    if (out == NULL) {
        return GW_ERR_PARAM;
    }
    if (nbytes > 4u) {
        return GW_ERR_PARAM;
    }

    memset(out, 0, sizeof(*out));
    out->id  = (uint32_t)((to_slave ? GW_COB_SDO_RX : GW_COB_SDO_TX) | node_id);
    out->dlc = 8u;
    out->data[0] = cs;
    out->data[1] = (uint8_t)(index & 0xFFu);
    out->data[2] = (uint8_t)((index >> 8) & 0xFFu);
    out->data[3] = subindex;
    /* 小端填充：不足 4 字节时高位补 0（按 CiA 301, 未用字节无意义）*/
    out->data[4] = (uint8_t)(value & 0xFFu);
    out->data[5] = (uint8_t)((value >> 8) & 0xFFu);
    out->data[6] = (uint8_t)((value >> 16) & 0xFFu);
    out->data[7] = (uint8_t)((value >> 24) & 0xFFu);
    return GW_OK;
}

int gw_sdo_build_download_expedited(uint8_t node_id, uint16_t index,
                                    uint8_t subindex, uint32_t value, uint8_t size,
                                    gw_can_frame_t *out)
{
    uint8_t cs;

    switch (size) {
    case 1u: cs = GW_SDO_CS_DOWNLOAD_EXP_1B; break;
    case 2u: cs = GW_SDO_CS_DOWNLOAD_EXP_2B; break;
    case 3u: cs = GW_SDO_CS_DOWNLOAD_EXP_3B; break;
    case 4u: cs = GW_SDO_CS_DOWNLOAD_EXP_4B; break;
    default: return GW_ERR_PARAM;
    }
    return sdo_build(node_id, true, cs, index, subindex, value, size, out);
}

int gw_sdo_build_upload_request(uint8_t node_id, uint16_t index,
                                uint8_t subindex, gw_can_frame_t *out)
{
    return sdo_build(node_id, true, GW_SDO_CS_UPLOAD_INIT, index, subindex,
                     0u, 0u, out);
}

int gw_sdo_parse(const gw_can_frame_t *f, gw_sdo_result_t *out)
{
    uint8_t cs;
    uint8_t n;

    if ((f == NULL) || (out == NULL)) {
        return GW_ERR_PARAM;
    }
    memset(out, 0, sizeof(*out));
    if (f->dlc < 8u) {
        return GW_ERR_PARAM;
    }

    cs = f->data[0];

    if (cs == GW_SDO_CS_ABORT) {
        out->valid      = true;
        out->aborted    = true;
        out->index      = (uint16_t)((uint16_t)f->data[1] |
                                     ((uint16_t)f->data[2] << 8));
        out->subindex   = f->data[3];
        out->abort_code = (uint32_t)f->data[4] | ((uint32_t)f->data[5] << 8) |
                          ((uint32_t)f->data[6] << 16) |
                          ((uint32_t)f->data[7] << 24);
        return GW_ERR_EXCEPTION;
    }

    switch (cs) {
    case GW_SDO_CS_UPLOAD_EXP_1B: n = 1u; break;
    case GW_SDO_CS_UPLOAD_EXP_2B: n = 2u; break;
    case GW_SDO_CS_UPLOAD_EXP_3B: n = 3u; break;
    case GW_SDO_CS_UPLOAD_EXP_4B: n = 4u; break;
    case GW_SDO_CS_DOWNLOAD_EXP_1B: n = 1u; break;
    case GW_SDO_CS_DOWNLOAD_EXP_2B: n = 2u; break;
    case GW_SDO_CS_DOWNLOAD_EXP_3B: n = 3u; break;
    case GW_SDO_CS_DOWNLOAD_EXP_4B: n = 4u; break;
    case GW_SDO_CS_DOWNLOAD_ACK:
    case GW_SDO_CS_UPLOAD_REQ_SEG:
        out->valid    = true;
        out->index    = (uint16_t)((uint16_t)f->data[1] |
                                   ((uint16_t)f->data[2] << 8));
        out->subindex = f->data[3];
        out->size     = 0u;
        out->value    = 0u;
        return GW_OK;
    default:
        return GW_ERR_UNSUPPORTED;
    }

    out->valid    = true;
    out->index    = (uint16_t)((uint16_t)f->data[1] |
                               ((uint16_t)f->data[2] << 8));
    out->subindex = f->data[3];
    out->size     = n;
    out->value    = (uint32_t)f->data[4] | ((uint32_t)f->data[5] << 8) |
                    ((uint32_t)f->data[6] << 16) | ((uint32_t)f->data[7] << 24);
    return GW_OK;
}

int gw_sdo_build_initiate_download(uint8_t node_id, uint16_t index,
                                   uint8_t subindex, uint32_t total_size,
                                   gw_can_frame_t *out)
{
    return sdo_build(node_id, true, GW_SDO_CS_DOWNLOAD_INIT_SEG, index, subindex,
                     total_size, 4u, out);
}

int gw_sdo_build_segment(uint8_t node_id, bool toggle, const uint8_t *data,
                         uint8_t len, gw_can_frame_t *out)
{
    uint8_t i;

    if ((out == NULL) || (data == NULL) || (len == 0u) || (len > 7u)) {
        return GW_ERR_PARAM;
    }
    memset(out, 0, sizeof(*out));
    out->id  = (uint32_t)(GW_COB_SDO_RX | node_id);
    out->dlc = 8u;
    /* CS = c(toggle) << 4 | n, n = 7 - 有效字节数；位置 1 表示还有后续段 */
    out->data[0] = (uint8_t)((toggle ? 0x10u : 0x00u) | (uint8_t)(7u - len));
    for (i = 0u; i < len; i++) {
        out->data[1u + i] = data[i];
    }
    return GW_OK;
}

int gw_sdo_parse_segment(const gw_can_frame_t *f, bool *toggle, uint8_t *out,
                         uint8_t *out_len)
{
    uint8_t cs;
    uint8_t n;
    uint8_t i;

    if ((f == NULL) || (toggle == NULL) || (out == NULL) || (out_len == NULL)) {
        return GW_ERR_PARAM;
    }
    if (f->dlc < 8u) {
        return GW_ERR_PARAM;
    }

    cs = f->data[0];
    *toggle = (cs & 0x10u) != 0u;
    n = (uint8_t)(cs & 0x0Fu);
    if (n > 7u) {
        return GW_ERR_PARAM;
    }
    *out_len = (uint8_t)(7u - n);
    for (i = 0u; i < *out_len; i++) {
        out[i] = f->data[1u + i];
    }
    return GW_OK;
}

int gw_sdo_build_segment_ack(uint8_t node_id, bool toggle, gw_can_frame_t *out)
{
    if (out == NULL) {
        return GW_ERR_PARAM;
    }
    memset(out, 0, sizeof(*out));
    out->id  = (uint32_t)(GW_COB_SDO_TX | node_id);
    out->dlc = 8u;
    out->data[0] = (uint8_t)(GW_SDO_CS_DOWNLOAD_ACK |
                             (toggle ? 0x10u : 0x00u));
    return GW_OK;
}

/* ================================================================== */
/* PDO 映射                                                            */
/* ================================================================== */
int gw_pdo_decode_to_od(const gw_pdo_map_t *map, const gw_od_t *od,
                        const uint8_t *data, uint32_t len)
{
    uint32_t offset_bits = 0u;
    uint8_t  i;

    if ((map == NULL) || (od == NULL) || (data == NULL)) {
        return GW_ERR_PARAM;
    }

    for (i = 0u; i < map->count; i++) {
        const gw_pdo_entry_t *pe = &map->entries[i];
        const gw_od_entry_t  *oe;
        uint32_t              v = 0u;
        uint8_t               k;

        if ((pe->bits != 8u) && (pe->bits != 16u) && (pe->bits != 32u)) {
            return GW_ERR_UNSUPPORTED;
        }
        if (((offset_bits + pe->bits) / 8u) > len) {
            return GW_ERR_PARAM;         /* 过程数据长度不足 */
        }

        /* CANopen PDO 为大端（Motorola）字节序 */
        for (k = 0u; k < (pe->bits / 8u); k++) {
            uint32_t byte_index = (offset_bits / 8u) + k;
            v = (v << 8) | (uint32_t)data[byte_index];
        }
        offset_bits += pe->bits;

        oe = gw_od_lookup(od, pe->index, pe->subindex);
        if (oe == NULL) {
            return GW_ERR_NOT_FOUND;
        }
        if (gw_od_write_u32(oe, v) != GW_OK) {
            return GW_ERR_STATE;
        }
    }
    return GW_OK;
}

/* ================================================================== */
/* EMCY                                                               */
/* ================================================================== */
int gw_canopen_emcy_parse(const gw_can_frame_t *f, gw_emcy_t *out)
{
    gw_canopen_cob_t type = GW_COB_UNKNOWN;
    uint8_t          node = 0u;
    uint8_t          i;

    if ((f == NULL) || (out == NULL)) {
        return GW_ERR_PARAM;
    }
    if (gw_canopen_decode_cob(f->id, &type, &node) != GW_OK) {
        return GW_ERR_NOT_FOUND;
    }
    if (type != GW_COB_EMCY) {
        return GW_ERR_UNSUPPORTED;
    }
    if (f->dlc < 2u) {
        return GW_ERR_PARAM;
    }

    memset(out, 0, sizeof(*out));
    out->node_id    = node;
    /* EMCY 错误码为小端（CiA 301：低字节在前） */
    out->error_code = (uint16_t)((uint16_t)f->data[0] |
                                 ((uint16_t)f->data[1] << 8));
    out->error_register = (f->dlc >= 3u) ? f->data[2] : 0u;
    for (i = 0u; (i < 5u) && ((uint32_t)(3u + i) < f->dlc); i++) {
        out->vendor_data[i] = f->data[3u + i];
    }
    return GW_OK;
}
