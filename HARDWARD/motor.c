#include "motor.h"
#include "arm_math.h"
#include <math.h>
#include "dm_motor_drv.h"
#include "dm_motor_ctrl.h"
int  motor_2006_pos[3];
int Cache_2006[3]={0};
uint8_t flag_zhuazi=0;
motor_measure_t motor_chassis[7];
uint8_t motor_chassis_count[7]={0},flag_start_2006[7]={0};
float speed_try,pos_try;
void get_motor_measure(motor_measure_t *ptr,uint8_t *data)                                    
    {                                                                   
        (ptr)->last_ecd = (ptr)->ecd;                                   
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  
        (ptr)->temperate = (data)[6];                                   
			}//电机读取  3508,6020的返回值数据一致,2006无温度
		

//输入为Pitch电机6020的反馈
void CAN1_Motor_Call_Back(Struct_CAN_Rx_Buffer *Rx_Buffer)
{
	uint8_t *Rx_Data = Rx_Buffer->Data;
	switch (Rx_Buffer->Header.StdId)//switch部分的代码可以通用
    {
			//以下为大疆电机接收部分
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
				case CAN_YAW_MOTOR_ID:
        case CAN_PIT_MOTOR_ID:
        {
					static uint8_t i = 0;
					//get motor id
					i = Rx_Buffer->Header.StdId - CAN_3508_M1_ID;
					get_motor_measure(&motor_chassis[i], Rx_Data);//motor_chassis中ID1-4为3508的数组，5为Pitch中6020的储存数据
					
            break;
        }
				case 0x114://用于双板通信
				{
					break;
				}
				//以下为达妙电机接收部分，case后面的名字可根据需求更改
				case DM_YAW:
				{
					dm_motor_fbdata(&motor[Motor1], Rx_Data);  break;
				}
				case DM_Pitch_largh:
				{
					dm_motor_fbdata(&motor[Motor2], Rx_Data);  break;
				}
				case DM_Pitch_short:
				{
					dm_motor_fbdata(&motor[Motor3], Rx_Data);  break;
				}
				case DM_Roll:
				{
					dm_motor_fbdata(&motor[Motor4], Rx_Data);  
					break;
				}
//				case zhuazi:
//				{
//					dm_motor_fbdata(&motor[Motor5], Rx_Data);  
//					break;
//				}
        default:
        {
            break;
        }
    }
}

void CAN2_Motor_Call_Back(Struct_CAN_Rx_Buffer *Rx_Buffer)
{
	uint8_t *Rx_Data = Rx_Buffer->Data;
	switch (Rx_Buffer->Header.StdId)//switch部分的代码可以通用
    {

				case zhuazi:
				{
					dm_motor_fbdata(&motor[Motor7], Rx_Data);  
					break;
				}
				case diff_x:
				{
					dm_motor_fbdata(&motor[Motor6], Rx_Data);  
					break;
				}
				case diff_y:
				{
					dm_motor_fbdata(&motor[Motor5], Rx_Data);  
					break;
				}
        default:
        {
            break;
        }
    }
}
const motor_measure_t *get_gimbal_motor_measure_point_bo(void)
{
    return &motor_chassis[4];
}
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[5];
}
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_chassis[6];
}
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}
