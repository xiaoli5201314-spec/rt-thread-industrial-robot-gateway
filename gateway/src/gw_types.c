/*
 * gw_types.c - 枚举转字符串
 *
 * 把枚举到字符串的映射集中在一处, 日志、诊断上报、测试失败信息
 * 全部复用, 避免出现"日志里打印的是数字, 只有作者看得懂"的情况。
 */
#include "gw_types.h"

const char *gw_status_str(gw_status_t st)
{
    switch (st) {
    case GW_OK:              return "OK";
    case GW_ERR:             return "ERR";
    case GW_ERR_PARAM:       return "ERR_PARAM";
    case GW_ERR_TIMEOUT:     return "ERR_TIMEOUT";
    case GW_ERR_CRC:         return "ERR_CRC";
    case GW_ERR_STATE:       return "ERR_STATE";
    case GW_ERR_NOMEM:       return "ERR_NOMEM";
    case GW_ERR_IO:          return "ERR_IO";
    case GW_ERR_NOT_FOUND:   return "ERR_NOT_FOUND";
    case GW_ERR_FULL:        return "ERR_FULL";
    case GW_ERR_EMPTY:       return "ERR_EMPTY";
    case GW_ERR_BUSY:        return "ERR_BUSY";
    case GW_ERR_EXCEPTION:   return "ERR_EXCEPTION";
    case GW_ERR_UNSUPPORTED: return "ERR_UNSUPPORTED";
    case GW_ERR_OVERFLOW:    return "ERR_OVERFLOW";
    case GW_ERR_NOT_READY:   return "ERR_NOT_READY";
    default:                 return "ERR_UNKNOWN";
    }
}

const char *gw_protocol_str(gw_protocol_t p)
{
    switch (p) {
    case GW_PROTO_MODBUS_RTU: return "MODBUS_RTU";
    case GW_PROTO_MODBUS_TCP: return "MODBUS_TCP";
    case GW_PROTO_CANOPEN:    return "CANOPEN";
    case GW_PROTO_DEVICENET:  return "DEVICENET";
    case GW_PROTO_ETHERNET_IP:return "ETHERNET_IP";
    default:                  return "PROTO_UNKNOWN";
    }
}

const char *gw_dev_state_str(gw_dev_state_t s)
{
    switch (s) {
    case GW_DEV_UNKNOWN:  return "UNKNOWN";
    case GW_DEV_ONLINE:   return "ONLINE";
    case GW_DEV_DEGRADED: return "DEGRADED";
    case GW_DEV_OFFLINE:  return "OFFLINE";
    case GW_DEV_ISOLATED: return "ISOLATED";
    default:              return "STATE_UNKNOWN";
    }
}

const char *gw_bus_state_str(gw_bus_state_t s)
{
    switch (s) {
    case GW_BUS_OK:       return "OK";
    case GW_BUS_WARN:     return "WARN";
    case GW_BUS_DEGRADED: return "DEGRADED";
    case GW_BUS_ISOLATED: return "ISOLATED";
    case GW_BUS_FAULT:    return "FAULT";
    default:              return "BUS_UNKNOWN";
    }
}
