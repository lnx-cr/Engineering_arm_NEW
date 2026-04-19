#include "main.h"
#include "protect_Test.h"
#include "motor.h"
#include "motor_can.h"
#include "string.h"
#include "stdbool.h"


PRO_motor pt_motor[7]; 
/**
************************************************************************
**
*@Brief:用于初始化电机保护参数
*@param:输入为电流值，电压值和最大，最小位置
*@Note:
*@RetVal:
************************************************************************
**/
void pro_Init(PRO_motor *pro,float tor,float vel,float pos_max,float pos_min)
{
	memset(pro, 0, sizeof(*pro));//初始化结构体，将内部变量全附0
	pro->tor_safe=tor;
	pro->vel_safe=vel;
	pro->pos.max=pos_max;
	pro->pos.min=pos_min;
	pro->flag=1;
}

/**
************************************************************************
*@Brief:用于检测电机是否处于非正常状态，若电机非正常，则保持该位置不动
*@param:输入为每个电机结构体的指针
*@Note:通过速度和电流来判断，速度大于一定值或电流大于一定值，判断为非正常
*@RetVal:反馈值为1或0,1为正常
************************************************************************
**/
void test_normal(motor_t *motor,PRO_motor *pro)
{
	if((motor->para.tor>pro->tor_safe)&&(motor->para.vel>pro->vel_safe ))
	{
		pro->flag= 0;
	}
	pro->flag= 1;
}

/**
************************************************************************
**
*@Brief:进行常规的阻力补偿
*@param:输入电机的结构体以及需要的补偿值和速度的线性区域
*@Note:
*@RetVal:
************************************************************************
**/
void motor_Compensation(motor_t *motor,float cps,float scope)
{
	float cps_real=1;
	if(motor->para.vel>scope)
	{
		cps_real=cps;
	}
	else if(motor->para.vel<(-scope))
	{
		cps_real=-cps;
	}
	else 
	{
		cps_real=cps*motor->para.vel/scope;
	}
	motor->ctrl.tor_set+=cps_real;
}

/**
************************************************************************
**
*@Brief:作为
*@param:
*@Note:
*@RetVal:
************************************************************************

**/

