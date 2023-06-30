#include "aim_motor.h"
#include "motor_can.h"
#include "pid.h"
#include "remote_control.h"
motor_data_t 		aim_motor_data;		
pid_type_def		aim_motor_encoder_angle_pid;	
pid_type_def 		aim_motor_encoder_speed_pid;

fp32 aim_motor_encoder_speed_pid_Kp				=3.0f;
fp32 aim_motor_encoder_speed_pid_Ki				=0.5f;
fp32 aim_motor_encoder_speed_pid_Kd				=0.0f;
fp32 aim_motor_encoder_speed_pid_max_out	=2000.0f;
fp32 aim_motor_encoder_speed_pid_max_iout	=1000.0f;
fp32 aim_motor_encoder_speed_PID[3]; 

fp32 aim_motor_encoder_angle_pid_Kp				=0.2f;
fp32 aim_motor_encoder_angle_pid_Ki				=0.0f;
fp32 aim_motor_encoder_angle_pid_Kd				=1.0f;
fp32 aim_motor_encoder_angle_pid_max_out	=1000.0f;
fp32 aim_motor_encoder_angle_pid_max_iout	=0.0f;
fp32 aim_motor_encoder_angle_PID[3]; 

int16_t KEY_R;
int16_t KEY_R_last;
int16_t KEY_F;
int16_t KEY_F_last;
int16_t KEY_G;
int16_t KEY_G_last;

/*****2006参数宏定义*****/
#define Delta_2006_Ecd_After_One_Round 8192			//3508转一圈之后编码器数值改变量
#define Reduction_Ratio_of_2006_Motor 36		//3508电机的减速比(宏定义的数字参与运算，后面不加分号，不然乘法运算时会被当成指针)
#define Delta_Total_2006_Ecd_After_One_Round Delta_2006_Ecd_After_One_Round*Reduction_Ratio_of_2006_Motor//用成宏定义避免精度损失

RC_ctrl_t rc_ctrl_last3;
uint8_t aim_on_off_flag=1;//上电之前要复位到开镜的位置
int64_t aim_motor_offset_total_ecd=0;			 //上拨弹盘复位偏置总编码器角度；复位偏置总编码器角度，也就是从这个总编码器角度基础上转固定角度
void Aim_Motor_Control()
{
	KEY_R=((rc_ctrl.key.v & KEY_PRESSED_OFFSET_R) >>8);
	KEY_F=((rc_ctrl.key.v & KEY_PRESSED_OFFSET_F) >>9);
	KEY_G=((rc_ctrl.key.v & KEY_PRESSED_OFFSET_G) >>10);
	
	if(KEY_F!=1 && KEY_G!=1)
	{
		if(KEY_R_last!=1 && KEY_R==1)//R键开关镜
			aim_on_off_flag=!aim_on_off_flag;

		if(aim_on_off_flag==0)
			aim_motor_data.target_total_ecd=-Delta_Total_2006_Ecd_After_One_Round*90.0f/360.0f+aim_motor_offset_total_ecd;
		if(aim_on_off_flag==1)
			aim_motor_data.target_total_ecd=aim_motor_offset_total_ecd;
		
		calculate_aim_motor_current_with_target_total_angle();
	}
	if(KEY_F==1)
	{
		aim_motor_data.target_speed_rpm=-200;
		calculate_aim_motor_current_with_target_total_speed();
	}
	if(KEY_G==1)
	{
		aim_motor_data.target_speed_rpm=200;
		calculate_aim_motor_current_with_target_total_speed();
	}
	
	if(	( (KEY_F_last!=0) && (KEY_F==0) ) || (	(KEY_G_last!=0) && (KEY_G==0)	)		)//如果松开按键
	{
		aim_motor_offset_total_ecd=aim_motor_data.total_ecd;
		aim_motor_data.target_total_ecd=aim_motor_data.total_ecd;
		aim_on_off_flag=1;
	}
	
	KEY_R_last=KEY_R;
	KEY_F_last=KEY_F;
	KEY_G_last=KEY_G;
	rc_ctrl_last3=rc_ctrl;
}


void calculate_aim_motor_current_with_target_total_angle()
{
		aim_motor_data.target_speed_rpm=PID_calc(	&aim_motor_encoder_angle_pid,	aim_motor_data.total_ecd, aim_motor_data.target_total_ecd);

		aim_motor_data.target_current= PID_calc(	&aim_motor_encoder_speed_pid,	aim_motor_data.speed_rpm,	aim_motor_data.target_speed_rpm);
}

void calculate_aim_motor_current_with_target_total_speed()
{
		aim_motor_data.target_current= PID_calc(	&aim_motor_encoder_speed_pid,	aim_motor_data.speed_rpm,	aim_motor_data.target_speed_rpm);
}

void aim_motor_init()
{
	aim_motor_encoder_angle_PID[0]	=	aim_motor_encoder_angle_pid_Kp;
	aim_motor_encoder_angle_PID[1]	=	aim_motor_encoder_angle_pid_Ki;
	aim_motor_encoder_angle_PID[2]	=	aim_motor_encoder_angle_pid_Kd;
	PID_init(	&aim_motor_encoder_angle_pid,	PID_POSITION,	aim_motor_encoder_angle_PID,	aim_motor_encoder_angle_pid_max_out,	aim_motor_encoder_angle_pid_max_iout);
	
	aim_motor_encoder_speed_PID[0]	=	aim_motor_encoder_speed_pid_Kp;
	aim_motor_encoder_speed_PID[1]	=	aim_motor_encoder_speed_pid_Ki;
	aim_motor_encoder_speed_PID[2]	=	aim_motor_encoder_speed_pid_Kd;
	PID_init(	&aim_motor_encoder_speed_pid,	PID_POSITION,	aim_motor_encoder_speed_PID,	aim_motor_encoder_speed_pid_max_out,	aim_motor_encoder_speed_pid_max_iout);
	
}
