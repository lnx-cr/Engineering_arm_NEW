#ifndef __TASK_REFEREE_H
#define __TASK_REFEREE_H
#include "stm32f4xx_hal.h"
#include "fifo.h"

#define USART_RX_BUF_LENGHT     512
#define REFEREE_FIFO_BUF_LENGTH 1024

extern fifo_s_t referee_fifo;
extern uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
extern uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];
void referee_unpack_fifo_data(void);
void referee_init(void);

#endif
