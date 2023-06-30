#include "chassis_motor.h"
#include "pid.h"
#include "motor_can.h"

/********************数据信息和控制信息********************/
motor_data_t 		chassis_motor_data[4];				//存放电机数据信息
pid_type_def 		chassis_motor_speed_pid[4];		//存放电机控制信息
pid_type_def 		chassis_motor_follow_gimbal_angle_pid;	//存放电机控制信息
int16_t Chassis_Follow_Gimbal_Offset_Ecd=6980;//底盘跟随云台的6020目标编码器角度

/********************电机控制PID参数********************/
fp32 chassis_motor_speed_pid_Kp				=9.856f;
fp32 chassis_motor_speed_pid_Ki				=0.981f;
fp32 chassis_motor_speed_pid_Kd				=0.1f;//0.1
fp32 chassis_motor_speed_pid_max_out	=7000.0f;//16200.0f
fp32 chassis_motor_speed_pid_max_iout	=5000.0f;
fp32 chassis_motor_speed_PID[3];

fp32 chassis_motor_follow_gimbal_angle_pid_Kp=4.5f;
fp32 chassis_motor_follow_gimbal_angle_pid_Ki=0.0f;
fp32 chassis_motor_follow_gimbal_angle_pid_Kd=8.0f;
fp32 chassis_motor_follow_gimbal_angle_pid_max_out=12000;//12000;彭总害怕小陀螺结束之后底盘跟随云台太猛把超电烧了，所以这边限一限;MD不限了，鼠标根本跟不上
fp32 chassis_motor_follow_gimbal_angle_pid_max_iout=500;
fp32 chassis_motor_follow_gimbal_angle_PID[3];




/**	[1]							底盘电机初始化
  * brief         	初始化电机的数据信息和控制信息
  * postscript										*/
void chassis_motor_init(void)
{
	/*(1)	初始化底盘电机速度环pid*/
	chassis_motor_speed_PID[0]	=	chassis_motor_speed_pid_Kp;
	chassis_motor_speed_PID[1]	=	chassis_motor_speed_pid_Ki;
	chassis_motor_speed_PID[2]	=	chassis_motor_speed_pid_Kd;
	
	PID_init(	&chassis_motor_speed_pid[0],	PID_DELTA,	chassis_motor_speed_PID,	chassis_motor_speed_pid_max_out,	chassis_motor_speed_pid_max_iout);
	PID_init(	&chassis_motor_speed_pid[1],	PID_DELTA,	chassis_motor_speed_PID,	chassis_motor_speed_pid_max_out,	chassis_motor_speed_pid_max_iout);
	PID_init(	&chassis_motor_speed_pid[2],	PID_DELTA,	chassis_motor_speed_PID,	chassis_motor_speed_pid_max_out,	chassis_motor_speed_pid_max_iout);
	PID_init(	&chassis_motor_speed_pid[3],	PID_DELTA,	chassis_motor_speed_PID,	chassis_motor_speed_pid_max_out,	chassis_motor_speed_pid_max_iout);
	
	/*(2)	初始化底盘跟随云台位置环pid*/
	chassis_motor_follow_gimbal_angle_PID[0] = chassis_motor_follow_gimbal_angle_pid_Kp;
	chassis_motor_follow_gimbal_angle_PID[1] = chassis_motor_follow_gimbal_angle_pid_Ki;
	chassis_motor_follow_gimbal_angle_PID[2] = chassis_motor_follow_gimbal_angle_pid_Kd;
	
	PID_init(	&chassis_motor_follow_gimbal_angle_pid,	PID_POSITION,	chassis_motor_follow_gimbal_angle_PID,	chassis_motor_follow_gimbal_angle_pid_max_out,	chassis_motor_follow_gimbal_angle_pid_max_iout);
}


/**[2]						根据目标速度计算目标电流
  * brief																
	* postscript																																	*/
void calculate_chassis_motor_target_current_with_target_speed(void)
{
		chassis_motor_data[0].target_current	=	PID_calc(	&chassis_motor_speed_pid[0],	chassis_motor_data[0].speed_rpm,	chassis_motor_data[0].target_speed_rpm);
			
		chassis_motor_data[1].target_current	=	PID_calc(	&chassis_motor_speed_pid[1],	chassis_motor_data[1].speed_rpm,	chassis_motor_data[1].target_speed_rpm);	
	
		chassis_motor_data[2].target_current	=	PID_calc(	&chassis_motor_speed_pid[2],	chassis_motor_data[2].speed_rpm,	chassis_motor_data[2].target_speed_rpm);
			
		chassis_motor_data[3].target_current	=	PID_calc(	&chassis_motor_speed_pid[3],	chassis_motor_data[3].speed_rpm,	chassis_motor_data[3].target_speed_rpm);	
}

/**[3]						根据目标位置计算目标速度
  * brief																
	* postscript																																	*/
extern int16_t chassis_follow_gimbal_target_rotation_speed_z;
extern int16_t chassis_follow_gimbal_ecd;
void calculate_chassis_follow_gimbal_motor_speed_with_target_ecd(void)
{
		chassis_follow_gimbal_target_rotation_speed_z	=	-	PID_calc(	&chassis_motor_follow_gimbal_angle_pid, 	chassis_follow_gimbal_ecd,		Chassis_Follow_Gimbal_Offset_Ecd);//不加负号就成正反馈了
}

/**	[4]							
  * brief         	底盘运动学解算
	* param[in]				chassis_target_speed_x：前进速度
	* param[in]				chassis_target_speed_y：横移速度（遥控器左摇左移，右摇右移）
	* param[in]				chassis_target_rotation_speed_z：旋转速度(逆时针拨滚轮，车逆时针转)
	* postscript			注意，设置轮子速度为正，是相对电机来说正向转动，由于安装原因，轮子相对于地面不一定是我们认为的正转					*/
void calculate_chassis_motor_target_speed_with_kinematics_solution(int16_t chassis_target_speed_x, int16_t chassis_target_speed_y,int16_t chassis_target_rotation_speed_z)
{
		chassis_motor_data[3].target_speed_rpm=+chassis_target_speed_x+chassis_target_speed_y-chassis_target_rotation_speed_z;//左前轮，ID=4
		chassis_motor_data[2].target_speed_rpm=-chassis_target_speed_x+chassis_target_speed_y-chassis_target_rotation_speed_z;//右前轮，ID=3
		chassis_motor_data[1].target_speed_rpm=-chassis_target_speed_x-chassis_target_speed_y-chassis_target_rotation_speed_z;//右后轮，ID=2
		chassis_motor_data[0].target_speed_rpm=+chassis_target_speed_x-chassis_target_speed_y-chassis_target_rotation_speed_z;//左后轮，ID=1
}

