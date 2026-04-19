/* USER CODE BEGIN Header */
/*该项目为秦风战队2026赛季工程挑战赛最终版代码
该项目使用c板为主控板，用freertos为模板，共分为8个任务，每个任务的内容在对应的任务前进行说明，
机器人使用全部为达妙电机。控制逻辑大部分为重补,阻力补偿加pid，只有大臂上使用了前馈来帮助pid。*/
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ins_task.h"
#include "drv_can.h"
#include "stdio.h"
#include "pid.h"
#include "string.h"
#include <stdlib.h>
#include <stdarg.h>
#include "motor.h"
#include "motor_can.h"
#include "dm_motor_drv.h"
#include "dm_motor_ctrl.h"
#include "properties.h"
#include "protect_Test.h"
#include "bsp_uart6.h"
#include "rm_parser.h"
#include "crc.h"
//#include <math.h>

#include "queue.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
pid_type_def motor_pid[7];//存储电机的pid值
pid_type_def motor_short_pid[3];//小臂pid值
pid_type_def motor_large_pid[3];//大臂pid值
pid_type_def diff_joint_speed_pid[4];
pid_type_def diff_joint_pos_pid[4];
pid_type_def motor_yaw_pid[2];
int16_t yaw_now;
uint8_t CAN2_YAW[3];
//#include "bsp_system.h"
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask04 */
osThreadId_t myTask04Handle;
const osThreadAttr_t myTask04_attributes = {
  .name = "myTask04",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for myTask05 */
osThreadId_t myTask05Handle;
const osThreadAttr_t myTask05_attributes = {
  .name = "myTask05",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for myTask06 */
osThreadId_t myTask06Handle;
const osThreadAttr_t myTask06_attributes = {
  .name = "myTask06",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for myTask07 */
osThreadId_t myTask07Handle;
const osThreadAttr_t myTask07_attributes = {
  .name = "myTask07",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for myTask08 */
osThreadId_t myTask08Handle;
const osThreadAttr_t myTask08_attributes = {
  .name = "myTask08",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
};
/* Definitions for myQueue02 */
osMessageQueueId_t myQueue02Handle;
const osMessageQueueAttr_t myQueue02_attributes = {
  .name = "myQueue02"
};
/* Definitions for myQueue03 */
osMessageQueueId_t myQueue03Handle;
const osMessageQueueAttr_t myQueue03_attributes = {
  .name = "myQueue03"
};
/* Definitions for myQueue04 */
osMessageQueueId_t myQueue04Handle;
const osMessageQueueAttr_t myQueue04_attributes = {
  .name = "myQueue04"
};
/* Definitions for myQueue05 */
osMessageQueueId_t myQueue05Handle;
const osMessageQueueAttr_t myQueue05_attributes = {
  .name = "myQueue05"
};
/* Definitions for myQueue06 */
osMessageQueueId_t myQueue06Handle;
const osMessageQueueAttr_t myQueue06_attributes = {
  .name = "myQueue06"
};
/* Definitions for myQueue07 */
osMessageQueueId_t myQueue07Handle;
const osMessageQueueAttr_t myQueue07_attributes = {
  .name = "myQueue07"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);
void StartTask06(void *argument);
void StartTask07(void *argument);
void StartTask08(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of myQueue01 */
  myQueue01Handle = osMessageQueueNew (16, sizeof(uint16_t), &myQueue01_attributes);

  /* creation of myQueue02 */
  myQueue02Handle = osMessageQueueNew (16, sizeof(uint16_t), &myQueue02_attributes);

  /* creation of myQueue03 */
  myQueue03Handle = osMessageQueueNew (16, sizeof(uint16_t), &myQueue03_attributes);

  /* creation of myQueue04 */
  myQueue04Handle = osMessageQueueNew (16, sizeof(uint16_t), &myQueue04_attributes);

  /* creation of myQueue05 */
  myQueue05Handle = osMessageQueueNew (16, sizeof(uint16_t), &myQueue05_attributes);

  /* creation of myQueue06 */
  myQueue06Handle = osMessageQueueNew (16, sizeof(uint16_t), &myQueue06_attributes);

  /* creation of myQueue07 */
  myQueue07Handle = osMessageQueueNew (16, sizeof(uint16_t), &myQueue07_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
	//static const RC_Ctl_t dummy = {0};           // 零开销，只编译期用
//	myQueue01Handle = osMessageQueueNew (256, sizeof(RC), &myQueue01_attributes);
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(StartTask03, NULL, &myTask03_attributes);

  /* creation of myTask04 */
  myTask04Handle = osThreadNew(StartTask04, NULL, &myTask04_attributes);

  /* creation of myTask05 */
  myTask05Handle = osThreadNew(StartTask05, NULL, &myTask05_attributes);

  /* creation of myTask06 */
  myTask06Handle = osThreadNew(StartTask06, NULL, &myTask06_attributes);

  /* creation of myTask07 */
  myTask07Handle = osThreadNew(StartTask07, NULL, &myTask07_attributes);

  /* creation of myTask08 */
  myTask08Handle = osThreadNew(StartTask08, NULL, &myTask08_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
float speed_now,speed_set,tor_speed,tor_ref,POS_now,POS_set,mo3_t,pit_try,roll_try,derta_1,start_pitch,start_roll,large_pitch_pos,pos_4310;
char pitch_try;
float short_pitch_zero,pitch_zero,POS_2,error,Centrifugal_force_3,cos_roll,diff_joint_cp;
float roll_set,speed2_last,speed3_last,tor2,speed_zhuazi_last;
int16_t zhuazi_cur=-2000,speed_zhuazi,count_short=0,pos_zhuazi,send_2006_output[2];
uint8_t Data[8],flag_chasu_reset=1,differential_count=0;
float zhongbu3;
float diff_joint_speed[2],diff_joint_speed_ref[2],diff_joint_pos[2],diff_joint_pos_ref[2],speed_2006_left,speed_2006_right;//索引0为绕x轴旋转，索引1为y绕轴旋转
static uint8_t flag_arm;

/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
//该任务首先使能全部的电机，具有顺序。该项目只用于控制大臂，小臂上的8009p和roll轴上的dm4310。
//由于时间紧急，只有roll轴和小臂上的初始化位置有先后顺序，先使roll轴回到初始点，再使小pitch回到初始点
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	RC RC_Ctl_dsp;
	uint8_t flag_start=0,count_limit=0;
	
	uint16_t count=0;
	
	CAN_Init(&hcan1,CAN1_Motor_Call_Back);
	CAN_Init(&hcan2,CAN2_Motor_Call_Back);
	dm_motor_init();
	INS_Init();
	
	while(count<1200)//先等待陀螺仪稳定，待陀螺仪稳定后才会继续任务
	{
		INS_Task();
		count++;
	}
//	roll_set=INS.Roll;
	start_roll=INS.Roll;
//	dm_motor_enable(&hcan1, &motor[Motor1]);//达妙电机初始化
	
	while(motor[Motor2].para.state==0)
	{
		dm_motor_enable(&hcan1, &motor[Motor2]);
		HAL_Delay(100);
	}
	while(motor[Motor5].para.state==0)
	{
		dm_motor_enable(&hcan2, &motor[Motor5]);
		HAL_Delay(100);
	}
	while(motor[Motor6].para.state==0)
	{
		dm_motor_enable(&hcan2, &motor[Motor6]);
		HAL_Delay(100);
	}
	while(motor[Motor7].para.state==0)
	{
		dm_motor_enable(&hcan2, &motor[Motor7]);
		HAL_Delay(100);
	}
	while(motor[Motor4].para.state==0)
	{
		dm_motor_enable(&hcan1, &motor[Motor4]);
		HAL_Delay(100);
	}
	while(motor[Motor3].para.state==0)
	{
		dm_motor_enable(&hcan1, &motor[Motor3]);
		HAL_Delay(100);
	}
	while(motor[Motor1].para.state==0)
	{
		dm_motor_enable(&hcan1, &motor[Motor1]);
		HAL_Delay(100);
	}

//	osDelay(10);
//	dm_motor_enable(&hcan1, &motor[Motor1]);
//	osDelay(10);
	
	
	
//	short_pitch_zero=motor[Motor3].para.pos;
	roll_set=roll_initial;//设置小pitch轴初始化角度
//	save_pos_zero(&hcan1,motor[Motor4].id,1);//编码器初始为0
//	save_pos_zero(&hcan1,motor[Motor3].id,1);
//	save_pos_zero(&hcan1,motor[Motor2].id,1);
//	save_pos_zero(&hcan1,motor[Motor1].id,1);
	osDelay(2);
	
	
	pid_init(&motor_pid[2],11.f,0.f,0.f,0,0);//左边2006速度环
	pid_init(&motor_pid[3],11.f,0.f,0.f,0,0);//右边2006速度环
	
	pid_init(&motor_short_pid[0],1.2f,0.004f,0.3f,0.027f,9);//小臂速度环
	pid_init(&motor_short_pid[1],0.154f,0.f,0.04f,0.f,20);//小臂位置环
	
	pid_init(&motor_large_pid[0],1.35f,0.001f,0.f,0.02f,9);//大臂速度环
	pid_init(&motor_large_pid[1],8.f,0.03f,3.1f,0.1f,3.f);//大臂位置环
	
	HAL_UART_Receive_DMA(&huart3,Rx_data,18);
	
	HAL_UART_Receive_DMA(&huart6, dma_rx_buffer, DMA_RX_BUF_SIZE);
	rm_parser_init();
//	osDelay(500);
  /* Infinite loop */
  for(;;)
  {
		if(flag_start==2)
		{

			if((rc_Ctrl.s2==2)&&(rc_Ctrl.s1!=1))
			{
				flag_arm=1;//使用遥控器控制YAW和Pitch_large
			}
			else if((rc_Ctrl.s1==3)&&(rc_Ctrl.s2==3))
			{
				flag_arm=2;//使用遥控器控制其他几个轴
			}
			else if(rc_Ctrl.s1==1)
			{
				if(rc_Ctrl.s2==1)
				{
					flag_arm=3;//使用键鼠模式
				}
				else
				{
					flag_arm=4;//机械臂停止运动
				}
//				flag_arm=4;
			}
			
			//以下内容为给机械臂设置目标值
			switch(flag_arm)
			{
				case 1:
				{
					motor[Motor2].ctrl.pos_set+=rc_Ctrl.ch1/100000.f;
					LIMIT_MIN_MAX(motor[Motor2].ctrl.pos_set,Pitch_large_MIN,Pitch_large_MAX);
//					motor[Motor2].ctrl.vel_set=rc_Ctrl.ch1*0.0015f;
//					if((motor[Motor2].para.pos>Pitch_large_MAX)||(motor[Motor2].para.pos<Pitch_large_MIN))
//					{
//						motor[Motor2].ctrl.vel_set=0;
//					}
					break;
				}
				case 2:
				{
					motor[Motor4].ctrl.pos_set+=rc_Ctrl.ch0/50000.f;
					LIMIT_MIN_MAX(motor[Motor4].ctrl.pos_set,-12.4f,12.4f);
					roll_set+=rc_Ctrl.ch1/1000.f;
//					motor[Motor3].ctrl.vel_set=rc_Ctrl.ch1*0.006f;
//					motor[Motor3].ctrl.pos_set+=rc_Ctrl.ch1/50000.f;
//					motor[Motor3].ctrl.pos_set=PI*(roll_set-INS.Roll)/180.f+short_pitch_zero;
					motor[Motor3].ctrl.kp_set=0;
					LIMIT_MIN_MAX(roll_set,-60.f,70.f);
//					if(INS.Roll>80.f)
//					{
//						motor[Motor3].ctrl.vel_set=0;
//					}
					
					
					break;
				}
				case 3:
				{
					roll_set=roll_initial+Data_float[1];
					motor[Motor4].ctrl.pos_set=row_dm+Data_float[4];
					motor[Motor2].ctrl.pos_set=L_Pitch_dm+Data_float[2];
					LIMIT_MIN_MAX(motor[Motor4].ctrl.pos_set,-12.4f,12.4f);
					LIMIT_MIN_MAX(roll_set,-60.f,70.f);
					LIMIT_MIN_MAX(motor[Motor2].ctrl.pos_set,Pitch_large_MIN,Pitch_large_MAX);
					break;
				}
				case 4:
				{
					break;
				}
				default:
				{
					break;
				}
			}
			if(rc_Ctrl.s2!=0)
			{
//				speed_zhuazi=rc_Ctrl.ch4*0.255f;
//				pos_zhuazi+=rc_Ctrl.ch4*0.0255f;
//				LIMIT_MIN_MAX(pos_zhuazi,ZHUAZI_POS_MIN,ZHUAZI_POS_MAX);
			}
			
		}
//		else if(flag_start==0)
//		{
//			motor[Motor4].ctrl.pos_set=row_dm;//初始化电机零点
//		}
//		else if(flag_start==1)
//		{
//			motor[Motor3].ctrl.pos_set=S_Pitch_dm;
//		}
		//以下为roll轴的控制逻辑
		motor[Motor4].ctrl.vel_set=(motor[Motor4].ctrl.pos_set-motor[Motor4].para.pos)*10.5f;
		LIMIT_MIN_MAX(motor[Motor4].ctrl.vel_set,-motor[Motor4].tmp.VMAX,motor[Motor4].tmp.VMAX);//注意，达妙电机本身的示例函数没有自动限幅的功能
		motor[Motor4].ctrl.tor_set=(motor[Motor4].ctrl.vel_set-motor[Motor4].para.vel)*1.0f;
//		motor[Motor4].ctrl.tor_set=0;
//		motor_Compensation(&motor[Motor4],ROLL_CP,0.1f);	
		LIMIT_MIN_MAX(motor[Motor4].ctrl.tor_set,-motor[Motor4].tmp.TMAX,motor[Motor4].tmp.TMAX);
		
		
		speed2_last=0.7*speed2_last+0.3*motor[Motor2].para.vel;//对两个电机进行简易滤波
		speed3_last=0.7*speed3_last+0.3*motor[Motor3].para.vel;
		//以下为小pitch轴的控制逻辑
		motor[Motor3].ctrl.vel_set=PID_calc(&motor_short_pid[1],roll_set,INS.Roll)+motor[Motor2].para.vel;
		LIMIT_MIN_MAX(motor[Motor3].ctrl.vel_set,-motor[Motor3].tmp.VMAX,motor[Motor3].tmp.VMAX);
		motor[Motor3].ctrl.tor_set=PID_calc(&motor_short_pid[0],motor[Motor3].ctrl.vel_set,speed3_last);
		cos_roll=arm_cos_f32(PI*INS.Roll/180.f);
		zhongbu3=Force_cp_short_pitch*cos_roll;
		motor[Motor3].ctrl.tor_set+=zhongbu3;//改点
		
		if(flag_zhuazi==1)//判断是否有夹起物块，并进行重力补偿
		{
			diff_joint_cp=0.175f*(Force_cp_4310+Force_cp_4310_bowl)*arm_cos_f32(INS.Roll/57.295779513f-(motor[Motor5].para.pos-diff_y_dm));
			diff_joint_cp+=Force_cp_Energy_Unit*0.33f*cos_roll;
		}
		else
		{
			diff_joint_cp=0.175f*Force_cp_4310*arm_cos_f32(INS.Roll/57.295779513f-(motor[Motor5].para.pos-diff_y_dm));
		}

		
		motor[Motor3].ctrl.tor_set+=diff_joint_cp;
//		motor[Motor3].ctrl.tor_set+=(motor_chassis[0].given_current-motor_chassis[2].given_current)*0.00018f;
		motor_Compensation(&motor[Motor3],S_Pitch_cp,0.1f);
		LIMIT_MIN_MAX(motor[Motor3].ctrl.tor_set,-motor[Motor3].tmp.TMAX,motor[Motor3].tmp.TMAX);
		
		//以下为大pitch轴的控制逻辑
		motor[Motor2].ctrl.vel_set=PID_calc(&motor_large_pid[1],motor[Motor2].ctrl.pos_set,motor[Motor2].para.pos);		
		LIMIT_MIN_MAX(motor[Motor2].ctrl.vel_set,-motor[Motor2].tmp.VMAX,motor[Motor2].tmp.VMAX);
		POS_2=1.97f+motor[Motor2].para.pos;
		
		large_pitch_pos=arm_cos_f32(POS_2);
		motor[Motor2].ctrl.tor_set=PID_calc(&motor_large_pid[0],motor[Motor2].ctrl.vel_set,speed2_last);
		motor[Motor2].ctrl.tor_set+=(Force_cp_large_pitch*large_pitch_pos-zhongbu3-diff_joint_cp);//改点
		motor[Motor2].ctrl.tor_set+=(motor[Motor2].ctrl.vel_set*0.08f);//这是大臂上的重补
//		motor[Motor2].ctrl.tor_set=(Force_cp_large_pitch*large_pitch_pos+0.0001332f*Force_cp_4310*large_pitch_pos-zhongbu3-diff_joint_cp);
		if(flag_zhuazi==1)
		{
			motor[Motor2].ctrl.tor_set+=Force_cp_Energy_Unit*0.37f*large_pitch_pos;
		}
		//此处的前馈是用于补偿阻力
		if(motor[Motor2].ctrl.vel_set>0.1)
		{
			motor[Motor2].ctrl.tor_set+=L_Pitch_cp;
		}
		else if(motor[Motor2].ctrl.vel_set<(-0.1))
		{
			motor[Motor2].ctrl.tor_set-=L_Pitch_cp;
		}
		else 
		{
			motor[Motor2].ctrl.tor_set+=(L_Pitch_cp*motor[Motor2].ctrl.vel_set/0.1f);
		}
//		motor_Compensation(&motor[Motor2],L_Pitch_cp,0.15f);
		LIMIT_MIN_MAX(motor[Motor2].ctrl.tor_set,-motor[Motor2].tmp.TMAX,motor[Motor2].tmp.TMAX);
		

		speed_now=motor[Motor2].para.vel;//全部为调试变量
		speed_set=motor[Motor2].ctrl.vel_set;
//		speed_now=motor_chassis[2].speed_rpm;
//		speed_set=speed_zhuazi;
//		tor_ref=motor_chassis[2].given_current;
//		tor_speed=zhuazi_cur;
		tor_speed=motor[Motor2].ctrl.tor_set;
		tor_ref  =motor[Motor2].para.tor;
		POS_set  =motor[Motor2].ctrl.pos_set;
		POS_now  =motor[Motor2].para.pos;
		roll_try=INS.Roll/PI;
		
		
//		if(motor_chassis[2].given_current<-1200)
//		{
//			count_limit++;
//		}
//		
//		if(flag_chasu_reset==1)
//		{
//			send_2006_output[0]=-2000;
//			send_2006_output[1]=2000;
//		}
//		if((motor_chassis[2].given_current>1600)&&(motor_chassis[0].given_current<-1600))
//		{
//			differential_count++;
//		}
//		if((differential_count>10)&&(flag_chasu_reset==1))
//		{
//			send_2006_output[0]=0;
//			send_2006_output[1]=0;
//			flag_chasu_reset=0;
//			flag_start=3;
//			
//		}
//		if(flag_start==0)
//		{

//			speed_zhuazi=PID_calc(&motor_pid[1],pos_zhuazi,motor_2006_pos[0]);
//			zhuazi_cur=PID_calc(&motor_pid[0],speed_zhuazi,speed_zhuazi_last);
//			if(pos_zhuazi>7000)
//			{
//				zhuazi_cur+=7000;
//			}
			
			
//			LIMIT_MIN_MAX(zhuazi_cur,-9800,9800);
//		}
//		POS_now=motor_2006_pos[1];
//		send_2006_output[0]=0;
//		send_2006_output[1]=0;

		
			
		
		if(((motor[Motor4].para.pos<(0.1f+motor[Motor4].ctrl.pos_set))&&(motor[Motor4].para.pos>(-0.1f+motor[Motor4].ctrl.pos_set)))&&(flag_start==0))
		{
			flag_start=1;
		}
		else if(((INS.Roll<(1.f+roll_set))&&(INS.Roll>(-1.f+roll_set)))&&(flag_start==1))
		{
			count_short++;
			if(count_short>20)
			{
				flag_start=2;
				roll_set=INS.Roll;
			}
//			pitch_zero=motor[Motor3].para.pos;
//			start_roll=roll_set;
//			short_pitch_zero=motor[Motor3].para.pos;
		}
		else if((flag_start==2)&&(count_limit>10))
		{
			flag_start=3;
			speed_zhuazi=0;
//			zero_2006[0]=motor_chassis[2].ecd;
		}
		
//		tor_speed=motor[Motor3].ctrl.tor_set+Force_cp_short_pitch*arm_cos_f32(PI*INS.Pitch/180.f);
//		motor[Motor3].ctrl.tor_set=0;
		
		switch(flag_arm)
		{
			case 4:
			{
				mit_ctrl(&hcan1,&motor[Motor4],motor[Motor4].id,0.f,0.f,0.f,0.f,0.f);
				mit_ctrl(&hcan1,&motor[Motor3],motor[Motor3].id,0.f,0.f,0.f,0.f,0.f);
				osDelay(1);
				mit_ctrl(&hcan1,&motor[Motor2],motor[Motor2].id,0.f,0,0.f,0,0.f);
//				mit_ctrl(&hcan1,&motor[Motor1],motor[Motor1].id,0.f,0,0.f,0,0.f);
				break;
			}
			default:
			{
				dm_motor_ctrl_send(&hcan1,&motor[Motor4]);
				dm_motor_ctrl_send(&hcan1,&motor[Motor3]);
				dm_motor_ctrl_send(&hcan1,&motor[Motor2]);
				
//				mit_ctrl(&hcan1,&motor[Motor4],motor[Motor4].id,0.f,0.f,0.f,0.f,0.f);
//				mit_ctrl(&hcan1,&motor[Motor3],motor[Motor3].id,0.f,0.f,0.f,0.f,0.f);
//				mit_ctrl(&hcan1,&motor[Motor2],motor[Motor2].id,0.f,0,0.f,0,0.f);
				break;
			}
		}
		
		
		
    osDelay(4);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */

int a;
char flag_bihe=0;
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
//遥控器值的接收和处理
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
//	RC rc_Ctrl;//作为中间变量，传递处理过的遥控器值
	memset(&Data, 0, sizeof(Data[8]));
  /* Infinite loop */
  for(;;)
  {
		a++;
		RC_Ctl.rc.ch0 = (Rx_data[0]| (Rx_data[1] << 8)) & 0x07ff;      
		RC_Ctl.rc.ch1 = ((Rx_data[1] >> 3) | (Rx_data[2] << 5)) & 0x07ff;       
		RC_Ctl.rc.ch2 = ((Rx_data[2] >> 6) | (Rx_data[3] << 2) | (Rx_data[4] << 10)) & 0x07ff;          
		RC_Ctl.rc.ch3 = ((Rx_data[4] >> 1) | (Rx_data[5] << 7)) & 0x07ff;           
		RC_Ctl.rc.s1  = ((Rx_data[5] >> 4)& 0x000C) >> 2;                           
		RC_Ctl.rc.s2  = ((Rx_data[5] >> 4)& 0x0003);  
	
		RC_Ctl.rc.ch4 = Rx_data[16] | (Rx_data[17] << 8);
		RC_Ctl.mouse.x = Rx_data[6] | (Rx_data[7] << 8);                    //!< Mouse X axis        
		RC_Ctl.mouse.y = Rx_data[8] | (Rx_data[9] << 8);                    //!< Mouse Y axis      
		RC_Ctl.mouse.z = Rx_data[10] | (Rx_data[11] << 8);                  //!< Mouse Z axis         
		RC_Ctl.mouse.press_l = Rx_data[12];                                        //!< Mouse Left Is Press      
		RC_Ctl.mouse.press_r = Rx_data[13];                                        //!< Mouse Right Is Press 
		RC_Ctl.key.v = Rx_data[14] | (Rx_data[15] << 8);   			//!< KeyBoard value
		
		RC_Ctl.kb.bit.W=(RC_Ctl.key.v>>0)&0x0001;
		RC_Ctl.kb.bit.S=(RC_Ctl.key.v>>1)&0x0001;
		RC_Ctl.kb.bit.A=(RC_Ctl.key.v>>2)&0x0001;
		RC_Ctl.kb.bit.D=(RC_Ctl.key.v>>3)&0x0001;
		RC_Ctl.kb.bit.SHIFT=(RC_Ctl.key.v>>4)&0x0001;
		RC_Ctl.kb.bit.Q=(RC_Ctl.key.v>>6)&0x0001;
		RC_Ctl.kb.bit.E=(RC_Ctl.key.v>>7)&0x0001;
		RC_Ctl.kb.bit.V=(RC_Ctl.key.v>>14)&0x0001;
		RC_Ctl.kb.bit.B=(RC_Ctl.key.v>>15)&0x0001;
		
		if(RC_Ctl.kb.bit.B==1)
		{
			flag_bihe=1;
		}
		else if(RC_Ctl.kb.bit.V==1)
		{
			flag_bihe=0;
		}
		
		rc_Ctrl.ch0=RC_Ctl.rc.ch0-1024;
		rc_Ctrl.ch1=RC_Ctl.rc.ch1-1024;
		rc_Ctrl.ch2=RC_Ctl.rc.ch2-1024;
		rc_Ctrl.ch3=RC_Ctl.rc.ch3-1024;
		rc_Ctrl.ch4=RC_Ctl.rc.ch4-1024;
		rc_Ctrl.s1=RC_Ctl.rc.s1;
		rc_Ctrl.s2=RC_Ctl.rc.s2;
		
//		rm_parser_process();
//		xQueueSend(myQueue01Handle,&RC_Ctl.rc.ch0,portMAX_DELAY);		
    osDelay(10);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
int b;

/**

* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
//陀螺仪值的处理
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
	
  /* Infinite loop */
  for(;;)
  {
		INS_Task();
		osDelay(3);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
int16_t vx,vy,v_w;
uint16_t v_x,v_y,v_W;
/**
* @brief Function implementing the myTask04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
//双板通信数据处理
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
	
  /* Infinite loop */
  for(;;)
  {
		if((RC_Ctl.rc.s2!=1)||(RC_Ctl.rc.s1!=1))
		{
			if((RC_Ctl.rc.ch2<1685)&&(RC_Ctl.rc.ch2>363))
			{
				Data[0]=RC_Ctl.rc.ch2;
				Data[1]=RC_Ctl.rc.ch2>>8;
				Data[2]=RC_Ctl.rc.ch3;
				Data[3]=RC_Ctl.rc.ch3>>8;
				Data[4]=RC_Ctl.rc.ch0;
				Data[5]=RC_Ctl.rc.ch0>>8;
				Data[6]=(RC_Ctl.rc.s2<<2)|RC_Ctl.rc.s1;
			}
		}
		else
		{
			if(RC_Ctl.kb.bit.W==1)
			{
				vx=200;
			}
			else if(RC_Ctl.kb.bit.S==1)
			{
				vx=-200;
			}
			else
			{
				vx=0;
			}
			if(RC_Ctl.kb.bit.A==1)
			{
				vy=-200;
			}
			else if(RC_Ctl.kb.bit.D==1)
			{
				vy=200;
			}
			else
			{
				vy=0;
			}
			if(RC_Ctl.kb.bit.Q==1)
			{
				v_W=-120;
			}
			else if(RC_Ctl.kb.bit.E==1)
			{
				v_W=120;
			}
			else
			{
				v_W=0;
			}
			if(RC_Ctl.kb.bit.SHIFT==1)
			{
				vy*=2;
				vx*=2;
				v_W*=2;
			}
			v_w=1024+v_W;
			
//			v_w=1024+RC_Ctl.mouse.x;
			v_x=vx+1024;
			v_y=vy+1024;
			Data[0]=v_y;
			Data[1]=v_y>>8;
			Data[2]=v_x;
			Data[3]=v_x>>8;
			Data[4]=v_w;
			Data[5]=v_w>>8;
			Data[6]=(RC_Ctl.rc.s2<<2)|RC_Ctl.rc.s1;
		}
		CAN_Send_Data(&hcan1,0x115,Data,8);
		//建立队列，让该任务中的变量能够在别的任务中使用，先进先出，使得任务在使用变量时更安全，可理解为建立一个缓存区
//		xQueueSend(myQueue01Handle,&rc_ts,portMAX_DELAY);
    osDelay(20);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
float chasu_speed,chasu_speed_ref,opuput_chasu,chasu_pos,chasu_pos_ref;
float last_diff_speed[2];
/**
* @brief Function implementing the myTask05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
//差速关节处的dm4310处理
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
	pid_init(&diff_joint_speed_pid[0],0.5f,0.006f,0,0.03f,9);// x轴旋转速度环
	pid_init(&diff_joint_speed_pid[1],0.5f,0.01f,0.f,0.06f,9);// y轴旋转速度环
	
	pid_init(&diff_joint_pos_pid[0],8.f,0.005f,0,0.15f,25);//x轴旋转位置环
	pid_init(&diff_joint_pos_pid[1],17.f,0.005f,0,0.3f,20);//y轴旋转位置环
	
  /* Infinite pid_init(&diff_joint_speed_pid[0],0,0,0,0,0);//右边速度环loop */
  for(;;)
  {
		if((RC_Ctl.rc.ch2!=0)&&(RC_Ctl.rc.s1==3))
		{
//			diff_joint_pos[0]+=rc_Ctrl.ch2/660.f;
//			diff_joint_pos[1]+=rc_Ctrl.ch3/660.f;
//			LIMIT_MIN_MAX(diff_joint_pos[0],-180,180);
//			LIMIT_MIN_MAX(diff_joint_pos[1],-70,70);
//			diff_joint_speed[0] =rc_Ctrl.ch3*1.5f;
//			diff_joint_speed[1] =-rc_Ctrl.ch3*1.5f;
			motor[Motor5].ctrl.pos_set-=rc_Ctrl.ch3/33000.f;
//			motor[Motor5].ctrl.vel_set=-rc_Ctrl.ch3/330.f;
			LIMIT_MIN_MAX(motor[Motor5].ctrl.pos_set,(diff_y_dm-1.6f),(diff_y_dm+1.6f));
			
			motor[Motor6].ctrl.pos_set+=rc_Ctrl.ch2/66000.f;
			LIMIT_MIN_MAX(motor[Motor6].ctrl.pos_set,(diff_x_dm-3.13f),(diff_x_dm+3.13f));
//			motor[Motor6].ctrl.vel_set=rc_Ctrl.ch2/330.f;
			
//			diff_joint_speed[1]+=rc_Ctrl.ch2*1.5f;
		}
		else if((RC_Ctl.rc.s1==1)&&(RC_Ctl.rc.s2==1))
		{
			motor[Motor5].ctrl.pos_set=diff_y_dm+Data_float[3];
			motor[Motor6].ctrl.pos_set=diff_x_dm+Data_float[0];
			LIMIT_MIN_MAX(motor[Motor5].ctrl.pos_set,(diff_y_dm-1.5f),(diff_y_dm+1.5f));
			LIMIT_MIN_MAX(motor[Motor6].ctrl.pos_set,(diff_x_dm-3.13f),(diff_x_dm+3.13f));
			
		}
//		motor[Motor5].ctrl.tor_set=PID_calc(&diff_joint_speed_pid[0],motor[Motor5].ctrl.vel_set,motor[Motor5].para.vel);

//		diff_joint_speed_ref[0]=-(motor_chassis[0].speed_rpm+motor_chassis[2].speed_rpm)/36.f;
//		diff_joint_speed_ref[1]=(motor_chassis[0].speed_rpm-motor_chassis[2].speed_rpm)/36.f;
//		diff_joint_pos_ref[0]=(-motor_2006_pos[2]-motor_2006_pos[1])/3276.8f;
//		diff_joint_pos_ref[1]=-12+(-motor_2006_pos[2]+motor_2006_pos[1])/1638.4f;
		motor[Motor6].ctrl.vel_set=PID_calc(&diff_joint_pos_pid[0],motor[Motor6].ctrl.pos_set,motor[Motor6].para.pos);
		motor[Motor6].ctrl.tor_set=PID_calc(&diff_joint_speed_pid[0],motor[Motor6].ctrl.vel_set,motor[Motor6].para.vel);
		
		motor[Motor5].ctrl.vel_set=PID_calc(&diff_joint_pos_pid[1],motor[Motor5].ctrl.pos_set,motor[Motor5].para.pos);
		motor[Motor5].ctrl.tor_set=PID_calc(&diff_joint_speed_pid[1],motor[Motor5].ctrl.vel_set,motor[Motor5].para.vel);
//		motor[Motor5].ctrl.tor_set+=(motor[Motor5].ctrl.vel_set*0.1f);
//		
//		diff_joint_speed[0]=-PID_calc(&diff_joint_pos_pid[0],diff_joint_pos[0],diff_joint_pos_ref[0]);
//		diff_joint_speed[1]=diff_joint_speed[0];
//		
//		diff_joint_pos_pid[1].out=PID_calc(&diff_joint_pos_pid[1],diff_joint_pos[1],diff_joint_pos_ref[1]);
//		
//		diff_joint_speed[0]+=diff_joint_pos_pid[1].out;
//		diff_joint_speed[1]-=diff_joint_pos_pid[1].out;
//		
//		last_diff_speed[0]=last_diff_speed[0]*0.7f+motor_chassis[0].speed_rpm*0.3f;
//		last_diff_speed[1]=last_diff_speed[1]*0.7f+motor_chassis[2].speed_rpm*0.3f;
//		
//		send_2006_output[0]=PID_calc(&diff_joint_speed_pid[0],diff_joint_speed[0],last_diff_speed[0]);
//		send_2006_output[1]=PID_calc(&diff_joint_speed_pid[1],diff_joint_speed[1],last_diff_speed[1]);
//		
////		send_2006_output[0]=0;
////		send_2006_output[1]=0;
//		//差速关节重补
//差速关节pitch上的重补需要对机器人进行dh参数的标定并且进行旋转矩阵的计算
		pos_4310=arm_cos_f32(-INS.Pitch/57.29578f)*arm_cos_f32(-INS.Roll/57.29578f)*arm_cos_f32(motor[Motor5].para.pos-diff_y_dm)-arm_sin_f32(INS.Roll/57.29578f)*arm_sin_f32(-motor[Motor5].para.pos+diff_y_dm);
		motor[Motor5].ctrl.tor_set-=Force_cp_4310*pos_4310; //改点
//		send_2006_output[0]+=Force_cp_4310*pos_4310;
//		send_2006_output[1]-=Force_cp_4310*pos_4310;
//		
//		
//		
//		motor[Motor6].ctrl.tor_set=0;
		motor_Compensation(&motor[Motor5],differential_joint_y,differential_joint_y_speed);
		motor_Compensation(&motor[Motor6],differential_joint_x,differential_joint_x_speed);
//		
		if(flag_zhuazi==1)
		{
			motor[Motor5].ctrl.tor_set-=Force_cp_4310_bowl*pos_4310;
//			send_2006_output[0]=8500;
//			send_2006_output[1]=-8500;
		}
		
		LIMIT_MIN_MAX(motor[Motor5].ctrl.tor_set,-9,9);
		LIMIT_MIN_MAX(motor[Motor6].ctrl.tor_set,-9,9);
		
		//以下变量仅在调试使用
		chasu_pos=motor[Motor5].ctrl.pos_set;
		chasu_pos_ref=motor[Motor5].para.pos;
		chasu_speed=motor[Motor5].ctrl.vel_set;
		chasu_speed_ref=motor[Motor5].para.vel;
		opuput_chasu=motor[Motor5].ctrl.tor_set;
		
//		if(flag_zhuazi==1)
//		{
//			motor[Motor5].ctrl.tor_set=5.f;
//		}
		dm_motor_ctrl_send(&hcan2,&motor[Motor6]);
		dm_motor_ctrl_send(&hcan2,&motor[Motor5]);
//		if(motor[Motor5].para.state==1)
//		{
//			mit_ctrl(&hcan2,&motor[Motor5],motor[Motor5].id,0.f,0.f,0.f,0.f,0.f);
//			mit_ctrl(&hcan2,&motor[Motor6],motor[Motor6].id,0.f,0.f,0.f,0.f,0.f);
//		}
		
//		CAN_cmd_chassis(send_2006_output[0],0,send_2006_output[1],0);//2006发送,差速关节
    osDelay(3);
  }
  /* USER CODE END StartTask05 */
}

/* USER CODE BEGIN Header_StartTask06 */
float speed_ref_zhuazi,speed_set_zhuazi,tor_set,tor_ref_zhuazi,zhuazi_pos,zhuazi_pos_ref;
/**
* @brief Function implementing the myTask06 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask06 */
//爪子处的处理
void StartTask06(void *argument)
{
  /* USER CODE BEGIN StartTask06 */
	pid_init(&motor_pid[0],0.4f,0.f,0.f,0.f,9);//爪子开收速度环
	pid_init(&motor_pid[1],9.f,0.f,0.f,0.f,9.f);//爪子开收位置环
  /* Infinite loop */
  for(;;)
  {
		if((RC_Ctl.rc.ch4!=0)&&(RC_Ctl.rc.s1==3))
		{
//			if(rc_Ctrl.ch4>500)
//			{
//				motor[Motor7].ctrl.pos_set=1.5f;
//			}
//			else if(rc_Ctrl.ch4<(-500))
//			{
//				motor[Motor7].ctrl.pos_set=0.2f;
//			}
			if(rc_Ctrl.ch4>(-670))
			{
				motor[Motor7].ctrl.pos_set+=(rc_Ctrl.ch4/66000.f);
			}
			LIMIT_MIN_MAX(motor[Motor7].ctrl.pos_set,ZHUAZI_POS_MIN,ZHUAZI_POS_MAX);
//			motor[Motor7].ctrl.vel_set=rc_Ctrl.ch4/330.f;
		}
		else if((RC_Ctl.rc.s1==1)&&(RC_Ctl.rc.s2==1))
		{
			if(flag_bihe==1)
			{
				motor[Motor7].ctrl.pos_set=-0.63f;
			}
			else if(flag_bihe==0)
			{
				motor[Motor7].ctrl.pos_set=-1.45f;
			}
			LIMIT_MIN_MAX(motor[Motor7].ctrl.pos_set,ZHUAZI_POS_MIN,ZHUAZI_POS_MAX);
		}
		motor[Motor7].ctrl.vel_set=PID_calc(&motor_pid[1],motor[Motor7].ctrl.pos_set,motor[Motor7].para.pos);
		speed_zhuazi_last=0.7f*speed_zhuazi_last+0.3f*motor[Motor7].para.vel;
		motor[Motor7].ctrl.tor_set=PID_calc(&motor_pid[0],motor[Motor7].ctrl.vel_set,speed_zhuazi_last);
		if((((motor[Motor7].para.tor>0.6f)&&(motor[Motor7].para.pos>-0.9f))&&(flag_zhuazi==0))&&(motor[Motor7].ctrl.pos_set>-0.9f))
		{
			flag_zhuazi=1;
		}
		else if((flag_zhuazi==1)&&(motor[Motor7].ctrl.pos_set<-1))
		{
			flag_zhuazi=0;
		}
		if(flag_zhuazi==1)
		{
			motor[Motor7].ctrl.tor_set=Force_advance_zhuazi;
		}
//		motor[Motor7].ctrl.tor_set=0;
		if(motor[Motor7].para.pos<(-1.36f))
		{
			motor_Compensation(&motor[Motor7],Zhuaizi_cp_big,0.1f);
		}
		else
		{
			motor_Compensation(&motor[Motor7],Zhuaizi_cp,0.1f);
		}
		
		zhuazi_pos=motor[Motor7].ctrl.pos_set;
		zhuazi_pos_ref=motor[Motor7].para.pos;
		speed_set_zhuazi=motor[Motor7].ctrl.vel_set;
		speed_ref_zhuazi=motor[Motor7].para.vel;
		tor_set=motor[Motor7].ctrl.tor_set;
		tor_ref_zhuazi=motor[Motor7].para.tor;
		dm_motor_ctrl_send(&hcan2,&motor[Motor7]);
//		if(motor[Motor5].para.state==1)
//		{
//			mit_ctrl(&hcan2,&motor[Motor7],motor[Motor7].id,0.f,0.f,0.f,0.f,0.f);
//		}
    osDelay(3);
  }
  /* USER CODE END StartTask06 */
}

/* USER CODE BEGIN Header_StartTask07 */
/**
* @brief Function implementing the myTask07 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask07 */
//YAW轴控制
void StartTask07(void *argument)
{
  /* USER CODE BEGIN StartTask07 */
	pid_init(&motor_yaw_pid[0],1.25f,0.04f,0.2f,0.08f,9);
	pid_init(&motor_yaw_pid[1],5.2f,0.f,0.f,0.f,20);
	
  /* Infinite loop */
  for(;;)
  {
		if((flag_arm==1)&&(RC_Ctl.rc.s1!=2))//判断数据来源
		{
			motor[Motor1].ctrl.pos_set+=(RC_Ctl.rc.ch0-1024)/28000.f;
			LIMIT_MIN_MAX(motor[Motor1].ctrl.pos_set,(motor_YAW_zero-3.f),(motor_YAW_zero+3.f));
//			motor[Motor1].ctrl.vel_set=-YAW_speed*(RC_Ctl.rc.ch0-1024)/660.f;
		}
//		else if(flag_arm==3)
		else if((RC_Ctl.rc.s1==1)&&(RC_Ctl.rc.s2==1))
		{
			motor[Motor1].ctrl.pos_set=motor_YAW_zero+Data_float[5];
			LIMIT_MIN_MAX(motor[Motor1].ctrl.pos_set,(motor_YAW_zero-3.f),(motor_YAW_zero+3.f));
		}
		
		motor[Motor1].ctrl.vel_set=PID_calc(&motor_yaw_pid[1],motor[Motor1].ctrl.pos_set,motor[Motor1].para.pos);
		motor[Motor1].ctrl.tor_set=PID_calc(&motor_yaw_pid[0],motor[Motor1].ctrl.vel_set,motor[Motor1].para.vel);
		
//		motor[Motor1].ctrl.tor_set=0;
		motor_Compensation(&motor[Motor1],YAW_cp,0.1f);
		if(flag_arm==4)
		{
			mit_ctrl(&hcan1,&motor[Motor1],motor[Motor1].id,0.f,0,0.f,0,0.f);
		}
		else
		{
//			mit_ctrl(&hcan1,&motor[Motor1],motor[Motor1].id,0.f,0,0.f,0,0.f);
			dm_motor_ctrl_send(&hcan1,&motor[Motor1]);
		}
    osDelay(5);
  }
  /* USER CODE END StartTask07 */
}

/* USER CODE BEGIN Header_StartTask08 */
/**
* @brief Function implementing the myTask08 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask08 */
void StartTask08(void *argument)
{
  /* USER CODE BEGIN StartTask08 */
  /* Infinite loop */
  for(;;)
  {
		rm_parser_process();//自定义控制器数据接收，参考上海大学文档以及AI
    osDelay(3);
  }
  /* USER CODE END StartTask08 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

