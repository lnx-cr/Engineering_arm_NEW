#include "drv_can.h"
#include "motor.h"
#include "motor_can.h"
CAN_TxHeaderTypeDef  tx_3508_message_L;
CAN_TxHeaderTypeDef  tx_3508_message_H;
uint8_t            can_send_3508_L[8];//3508的ID为1-4的数据的目标值储存
uint8_t            can_send_3508_H[8];//3508的ID为5-8的数据的目标值储存
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    tx_3508_message_L.StdId = CAN_CHASSIS_ALL_ID;
    tx_3508_message_L.IDE = CAN_ID_STD;
    tx_3508_message_L.RTR = CAN_RTR_DATA;
		tx_3508_message_L.ExtId = 0;
    tx_3508_message_L.DLC = 0x08;
//		chassis_tx_message.TransmitGlobalTime = DISABLE;
    can_send_3508_L[0] = motor1 >> 8;
    can_send_3508_L[1] = motor1;
    can_send_3508_L[2] = motor2 >> 8;
    can_send_3508_L[3] = motor2;
    can_send_3508_L[4] = motor3 >> 8;
    can_send_3508_L[5] = motor3;
    can_send_3508_L[6] = motor4 >> 8;
    can_send_3508_L[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan2, &tx_3508_message_L, can_send_3508_L, &send_mail_box);
}
void set_motor_voltage(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4)//6020电机的输出
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
    
  tx_header.StdId = (id_range == 0)?(0x1FE):(0x2FE);
  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = 8;

  tx_data[0] = (v1>>8)&0xff;
  tx_data[1] =    (v1)&0xff;
  tx_data[2] = (v2>>8)&0xff;
  tx_data[3] =    (v2)&0xff;
  tx_data[4] = (v3>>8)&0xff;
  tx_data[5] =    (v3)&0xff;
  tx_data[6] = (v4>>8)&0xff;
  tx_data[7] =    (v4)&0xff;
  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0); 
}

