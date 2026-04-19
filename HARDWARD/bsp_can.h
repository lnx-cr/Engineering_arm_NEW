#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "struct_typedef.h"

#define FEEDBACK_ID_BASE      0x205
#define CAN_CONTROL_ID_BASE   0x1ff
#define CAN_CONTROL_ID_EXTEND 0x2ff
#define MOTOR_MAX_NUM         7

extern void can_filter_init(void);
void set_motor_voltage(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4);
#endif
