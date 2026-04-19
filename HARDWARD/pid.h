#ifndef __PID_H_
#define __PID_H_
 
#include "main.h"
#include "usart.h"
#include <string.h>

typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct _pid_struct_t
{
  float kp;
  float ki;
  float kd;
  float i_max;
  float out_max;
  
  float ref;      // target value
  float fdb;      // feedback value
  float err[2];   // error and last error

  float p_out;
  float i_out;
  float d_out;
  float output;
}pid_struct_t;

typedef struct
{
    uint8_t mode;
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次

} pid_type_def;

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         none
  */
//extern void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);

/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
extern fp32 PID_calc(pid_type_def *pid,fp32 set,fp32 ref);

/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
extern void PID_clear(pid_type_def *pid);

//void motor_pid_init(void);
extern pid_type_def gimbal_pitch_speed_pid;
extern pid_type_def gimbal_pitch_angle_pid;
extern pid_type_def gimbal_yaw_speed_pid;
extern pid_type_def gimbal_yaw_angle_pid;
extern pid_type_def shoot_left_speed_pid;
extern pid_type_def shoot_right_speed_pid;
extern pid_type_def shoot_poker_pid;
extern pid_type_def chassis1_pid;
extern pid_type_def chassis2_pid;
extern pid_type_def chassis3_pid;
extern pid_type_def chassis4_pid;
extern pid_type_def miniturn_pid;
extern pid_type_def miniturn_speed_pid;
extern pid_type_def follow_pid;
extern pid_type_def visual_x_pid;
extern pid_type_def visual_y_pid;
extern void pid_init(pid_type_def *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max);
							
#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))
							
#endif
			  