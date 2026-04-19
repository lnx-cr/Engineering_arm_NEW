#ifndef PROPERTIES_H
#define PROPERTIES_H

#define PI 3.14159265358979323846f
//////****各轴限位*****///
#define ROLL_POS_MAX 2.15f //目前无用
#define ROLL_POS_MIN -3.8f //目前无用

#define Pitch_short_MIN 2.35f
#define Pitch_short_MAX 4.43f ///还需更改

#define Pitch_large_MIN -1.66f
#define Pitch_large_MAX 0.8f

#define YAW_middle 3.51f //电机YAW轴的中值
#define short_pitch_middle 0.f

#define ZHUAZI_POS_MAX -0.6f
#define ZHUAZI_POS_MIN -1.45f

/////****各个轴的阻力补偿*****////
#define ROLL_CP (0.5f)
#define S_Pitch_cp (0.1f)
#define L_Pitch_cp (0.1f)
#define Zhuaizi_cp (0.08f)
#define Zhuaizi_cp_big (0.14f)
#define YAW_cp  0.18f
/////****各个臂的重力补偿******////
//#define Force_cp_short_pitch  (0.716f)
#define Force_cp_short_pitch  (1.186f) //low
//#define Force_cp_short_pitch  (0.766f)
//#define Force_cp_large_pitch  (1.11856f)
#define Force_cp_large_pitch  (3.4584f)

#define Force_cp_4310   0.93f
//#define Force_cp_large_pitch  (2.58f)

#define differential_joint_x 0.08f
#define differential_joint_x_speed 0.15f
#define differential_joint_y 0.15f
#define differential_joint_y_speed 0.1f
/////****爪子力矩的前馈量******////
#define Force_advance_zhuazi 3.f

//////****抓到物块后的重力补偿******//
#define Force_cp_4310_bowl 0.82f  //用4310夹取
#define Force_cp_Energy_Unit 0.6f  //mg

////****各个电机初始编码器值(也可以理解为默认0点)*****/////单位为rad
#define S_Pitch_dm (3.2f)
#define L_Pitch_dm (-1.55f)
#define row_dm -0.25134f
#define zhuazi_dm (-1.36f)
#define diff_x_dm (1.882f) //id 6
#define diff_y_dm (-0.1404f) ///id 5

#define roll_initial 20

#define motor_YAW_zero -1.2f
#define YAW_speed 2.f  //测试使用

#endif
