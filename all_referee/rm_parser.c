#include "rm_parser.h"
#include "bsp_uart6.h"
#include "rm_protocol.h"
#include <string.h>

// 环形缓冲区大小（2的幂次）
#define RX_BUF_SIZE  128

// 全局变量定义
uint8_t g_rx_buffer[RX_BUF_SIZE];
volatile uint32_t g_rx_write_idx = 0;
volatile uint32_t g_rx_read_idx = 0;

int ax,bx;
parse_state_t g_parse_state = PARSE_IDLE;
frame_header_t g_current_header;
uint32_t g_bytes_expected = 0;

// 临时存放整包数据（最大支持 512 字节）
static uint8_t s_packet_buf[256];

// 从环形缓冲区读取一个字节（返回1成功，0无数据）
static uint8_t read_byte(uint8_t *byte)
{
    if (g_rx_read_idx == g_rx_write_idx) return 0;
    *byte = g_rx_buffer[g_rx_read_idx];
    g_rx_read_idx = (g_rx_read_idx + 1) % RX_BUF_SIZE;
    return 1;
}

// 从环形缓冲区读取多个字节（返回实际读取的字节数）
static uint32_t read_bytes(uint8_t *dst, uint32_t len)
{
    uint32_t i = 0;
    while (i < len && g_rx_read_idx != g_rx_write_idx) {
        dst[i] = g_rx_buffer[g_rx_read_idx];
        g_rx_read_idx = (g_rx_read_idx + 1) % RX_BUF_SIZE;
        i++;
    }
    return i;
}

// 处理一个完整的数据包（已确认帧头SOF正确、命令ID正确）
static void process_packet(uint8_t *pkt_buf, uint32_t pkt_len)
{
    rm_packet_t *pkt = (rm_packet_t*)pkt_buf;
    // 仅处理命令码 0x0302
    if (pkt->cmd_id == 0x0302) {
				ax++;
        uint16_t data_len = pkt->header.data_length;
				
        if (data_len <= 30) {
					bx++;
					memcpy(&Data_float[0],pkt->data,24);
					on_custom_controller_data(pkt->data, data_len);
        }
    }
    // 其他命令码忽略
}

// 初始化解析器（重置所有状态）
void rm_parser_init(void)
{
    g_parse_state = PARSE_IDLE;
    g_rx_read_idx = 0;
    g_rx_write_idx = 0;
    memset((void*)&g_current_header, 0, sizeof(g_current_header));
    g_bytes_expected = 0;
}

// 将新接收的数据喂入环形缓冲区（由DMA中断调用）
void rm_parser_feed_data(const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        uint32_t next = (g_rx_write_idx + 1) % RX_BUF_SIZE;
        // 如果缓冲区满，覆盖旧数据（简单处理，实际可加标志）
        if (next != g_rx_read_idx) {
            g_rx_buffer[g_rx_write_idx] = data[i];
            g_rx_write_idx = next;
        }
    }
}

// 主循环解析任务
void rm_parser_process(void)
{
    while (1) {
        switch (g_parse_state) {
            case PARSE_IDLE:
            {
                uint8_t byte;
                if (!read_byte(&byte)) return;   // 无数据
                if (byte == 0xA5) {               // 找到帧头
                    g_current_header.sof = byte;
                    g_parse_state = PARSE_HEADER;
                    g_bytes_expected = sizeof(frame_header_t) - 1; // 剩余帧头字节数
                }
                break;
            }

            case PARSE_HEADER:
            {
                if (g_bytes_expected > 0) {
                    uint8_t *dst = (uint8_t*)&g_current_header + (sizeof(frame_header_t) - g_bytes_expected);
                    uint32_t rd = read_bytes(dst, g_bytes_expected);
                    if (rd < g_bytes_expected) return;  // 数据不足，等待下次
                    g_bytes_expected = 0;
                }
                // 注意：此处跳过CRC8校验，直接信任帧头
                // 计算整包长度 = 帧头 + 命令码(2) + data_length + CRC16(2)（但CRC16我们也不校验）
                uint32_t total_len = sizeof(frame_header_t) + 2 + g_current_header.data_length + 2;
                if (total_len > sizeof(s_packet_buf)) {
                    // 长度异常，重新同步
                    g_parse_state = PARSE_IDLE;
                    break;
                }
                // 复制已读取的帧头到临时缓冲区
                memcpy(s_packet_buf, &g_current_header, sizeof(frame_header_t));
                // 读取命令码 + 数据 + CRC16（虽然不校验，但需要跳过CRC16字段）
                uint32_t remaining = total_len - sizeof(frame_header_t);
                if (read_bytes(s_packet_buf + sizeof(frame_header_t), remaining) < remaining) {
                    // 数据不足，放弃本次（重新同步）
                    g_parse_state = PARSE_IDLE;
                    break;
                }
                // 跳过CRC16校验，直接处理包
                process_packet(s_packet_buf, total_len);
                g_parse_state = PARSE_IDLE;
                break;
            }

            default:
                g_parse_state = PARSE_IDLE;
                break;
        }
    }
}