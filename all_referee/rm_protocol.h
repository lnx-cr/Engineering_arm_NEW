#ifndef RM_PROTOCOL_H
#define RM_PROTOCOL_H

#include <stdint.h>

#pragma pack(push, 1)
// 帧头结构
typedef struct {
    uint8_t  sof;           // 固定 0xA5
    uint16_t data_length;   // 数据段长度（小端）
    uint8_t  seq;           // 包序号
    uint8_t  crc8;          // 帧头CRC8（本代码中不使用）
} frame_header_t;

// 完整数据包结构（用于解析）
typedef struct {
    frame_header_t header;
    uint16_t       cmd_id;  // 命令码（小端）
    uint8_t        data[1]; // 实际长度 = header.data_length，用1兼容C89
} rm_packet_t;
#pragma pack(pop)

#endif