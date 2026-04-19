#ifndef RM_PARSER_H
#define RM_PARSER_H

#include <stdint.h>
#include "rm_protocol.h"   // 引入 frame_header_t 定义

// ========== 全局变量声明（方便调试） ==========
extern volatile uint32_t g_rx_write_idx;   // 环形缓冲区写入索引
extern volatile uint32_t g_rx_read_idx;    // 环形缓冲区读取索引
extern uint8_t g_rx_buffer[];              // 环形缓冲区数组（大小在c文件定义）

// 解析状态枚举
typedef enum {
    PARSE_IDLE,
    PARSE_HEADER,
    PARSE_CMD_DATA
} parse_state_t;

extern parse_state_t g_parse_state;        // 当前解析状态
extern frame_header_t g_current_header;    // 正在解析的帧头
extern uint32_t g_bytes_expected;          // 当前状态还需读取的字节数

// ========== 函数接口 ==========
void rm_parser_init(void);
void rm_parser_feed_data(const uint8_t *data, uint32_t len);
void rm_parser_process(void);

// 用户回调：当收到命令码为 0x0302 的数据包时调用
void on_custom_controller_data(const uint8_t *data, uint16_t len);

#endif