#ifndef __BSP_UART6_H
#define __BSP_UART6_H

#include "stm32f4xx_hal.h"

extern void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void usart6_tx_dma_enable(uint8_t *data, uint16_t len);

#endif
