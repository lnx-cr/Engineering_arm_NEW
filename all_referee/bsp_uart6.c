#include "main.h"
#include "rm_parser.h"
#include "crc.h"
#include "bsp_uart6.h"
#include <string.h>
#include "usart.h"
#include "dma.h"

uint8_t dma_rx_buffer[DMA_RX_BUF_SIZE];
float Data_float[10];
// 用户回调实现：处理自定义控制器数据
void on_custom_controller_data(const uint8_t *data, uint16_t len)
{
//	memcpy(&Data_float[0],data,24);
    // 在这里处理接收到的数据（例如保存到全局数组、控制电机等）
    // data 指向临时缓冲区，如需长期保存请复制
    // 示例：打印第一个字节
    // printf("Received 0x0302 data, len=%d, first byte=0x%02X\r\n", len, data[0]);
}

// DMA 半满中断回调
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6) {
        rm_parser_feed_data(dma_rx_buffer, DMA_RX_BUF_SIZE / 2);
    }
}

// DMA 全满中断回调
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6) {
        rm_parser_feed_data(dma_rx_buffer + DMA_RX_BUF_SIZE / 2, DMA_RX_BUF_SIZE / 2);
    }
}