#ifndef _MOTOR_CAN_H
#define _MOTOR_CAN_H
#include "main.h"
#include "drv_can.h"
#include "motor.h"

extern CAN_TxHeaderTypeDef  tx_3508_message_L;
extern CAN_TxHeaderTypeDef  tx_3508_message_H;
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void set_motor_voltage(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4);
extern uint8_t              can_send_3508_L[8];
extern uint8_t              can_send_3508_H[8];

#endif
