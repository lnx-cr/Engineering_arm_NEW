#ifndef BSP_UART6_H
#define BSP_UART6_H

// DMA 接收缓冲区（大小必须与环形缓冲区匹配或更大，这里使用与环形缓冲区相同大小）
#define DMA_RX_BUF_SIZE  128

extern uint8_t dma_rx_buffer[DMA_RX_BUF_SIZE];
extern float Data_float[10];

void on_custom_controller_data(const uint8_t *data, uint16_t len);

#endif