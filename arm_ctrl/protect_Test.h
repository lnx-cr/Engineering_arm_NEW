#ifndef PROTECT_TEST_H
#define PROTECT_TEST_H
#include "struct_typedef.h"
#include "dm_motor_drv.h"
#include "dm_motor_ctrl.h"
typedef struct
{ 
		float tor_safe;
		float vel_safe;
		struct
		{
			float max;
			float min;
		}pos;
		uint8_t flag;
}PRO_motor;
extern PRO_motor pt_motor[7];
void pro_Init(PRO_motor *pro,float tor,float vel,float pos_max,float pos_min);
void test_normal(motor_t *motor,PRO_motor *pro);
void motor_Compensation(motor_t *motor,float cps,float scope);
#endif
