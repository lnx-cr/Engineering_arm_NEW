#include "pid.h"
 
 
//云台电机结构体
pid_type_def gimbal_pitch_speed_pid;
pid_type_def gimbal_pitch_angle_pid;
pid_type_def gimbal_yaw_speed_pid;
pid_type_def gimbal_yaw_angle_pid;
//发射机构结构体
pid_type_def shoot_left_speed_pid;
pid_type_def shoot_right_speed_pid;
pid_type_def shoot_poker_pid;
//底盘电机结构体
pid_type_def chassis1_pid;
pid_type_def chassis2_pid;
pid_type_def chassis3_pid;
pid_type_def chassis4_pid;
//小陀螺结构体
pid_type_def miniturn_pid;
pid_type_def miniturn_speed_pid;
//底盘随动结构体
pid_type_def follow_pid;
//视觉跟踪结构体
pid_type_def visual_x_pid;
pid_type_def visual_y_pid;
//******************************//
//PID值写入
fp32 miniturn_mum[3]={0.02,0.00001,1};
fp32 miniturn_speed_mum[3]={100,30,0};

fp32 visual_x_num[3]={0.06,0,0.09};//0.09,0,0.09
fp32 visual_y_num[3]={0.06,0,0.09};

fp32 shoot_poker_mum[3]={1,0.1,0.1};
fp32 shoot_left_mum[3]={20,1,0.8};
fp32 shoot_right_mum[3]={20,1,0.8};

fp32 gimbal_pitch_speed_mum[3]={50,15,5};//60,25,25
fp32 gimbal_pitch_angle_mum[3]={0.1,0.003,2.5};//0.08,0.003,1.8

fp32 gimbal_yaw_speed_mum[3]={70,6,14};//70 6 14  //{70,30,20}
fp32 gimbal_yaw_angle_mum[3]={0.5,0,4.2};//0.5 0 5 //0.5，0,4.2//{0.02,0,0.011}

fp32 chassis1_mum[3]={5,0.5,0.1};
fp32 chassis2_mum[3]={5,0.5,0.1};
fp32 chassis3_mum[3]={5,0.5,0.1};
fp32 chassis4_mum[3]={5,0.5,0.1};

fp32 follow_num[3]={1.5,0,0};
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }


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
void pid_init(pid_type_def *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max)
{
  pid->Kp      = kp;
  pid->Ki      = ki;
  pid->Kd      = kd;
  pid->max_iout   = i_max;
  pid->max_out = out_max;
}

/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
fp32 PID_calc(pid_type_def *pid,fp32 set, fp32 ref )
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}

//void motor_pid_init()
//{
//	 //发射机构PID初始化
//	 PID_init(&shoot_poker_pid,PID_DELTA,shoot_poker_mum, 16000, 2000);
//	 PID_init(&shoot_left_speed_pid,PID_DELTA,shoot_left_mum, 16000, 2000);
//	 PID_init(&shoot_right_speed_pid,PID_DELTA,shoot_right_mum, 16000, 2000);
//	 //***************************************************//
//	 //云台电机PID初始化
//     PID_init(&gimbal_pitch_speed_pid,PID_POSITION,gimbal_pitch_speed_mum, 16000, 12000);
//	 PID_init(&gimbal_pitch_angle_pid,PID_POSITION,gimbal_pitch_angle_mum, 16000, 10000);
//	 PID_init(&gimbal_yaw_speed_pid,PID_POSITION,gimbal_yaw_speed_mum, 15000, 8000);
//	 PID_init(&gimbal_yaw_angle_pid,PID_POSITION,gimbal_yaw_angle_mum, 20000, 10000);
//	 //***************************************************//
//	 //底盘电机PID初始化
//	 PID_init(&chassis1_pid,PID_DELTA,chassis1_mum, 16000, 2000);
//	 PID_init(&chassis2_pid,PID_DELTA,chassis2_mum, 16000, 2000);
//	 PID_init(&chassis3_pid,PID_DELTA,chassis3_mum, 16000, 2000);
//	 PID_init(&chassis4_pid,PID_DELTA,chassis4_mum, 16000, 2000);
//	 //***************************************************//
//	 //小陀螺和底盘随动PID初始化
//	 PID_init(&miniturn_pid,PID_POSITION,miniturn_mum,16000,10000);
//	 PID_init(&miniturn_speed_pid,PID_POSITION,miniturn_speed_mum,16000,10000);
//	 PID_init(&follow_pid,PID_POSITION,follow_num,6000,1500);
//	 
//	 PID_init(&visual_x_pid,PID_POSITION,visual_x_num,6000,1500);
//	 PID_init(&visual_y_pid,PID_POSITION,visual_y_num,6000,1500);
//}


 