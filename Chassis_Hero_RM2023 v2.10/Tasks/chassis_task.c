#include "chassis_task.h"
#include "FreeRTOS.h"
#include "task.h"

#include "bsp_uart.h"
#include "chassis_motor.h"
#include "motor_can.h"
#include "can_transmit.h"
#include "math.h"
#include "vofa.h"

#include "arm_math.h"

#include "Referee.h"

#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)

/*(1)	底盘运动模式和底盘目标速度*/
uint8_t ChassisMode;
int16_t hero_target_forward_speed_x,hero_target_right_speed_y,hero_target_rotation_speed_z;
int16_t chassis_target_forward_speed_x,chassis_target_right_speed_y,chassis_target_rotation_speed_z;
int16_t chassis_follow_gimbal_target_rotation_speed_z;
/*(2)	底盘电机的目标速度和电流*/
fp32 chassis_motor_speed_set[4];

int16_t chassis_follow_gimbal_ecd;
extern motor_data_t yaw_motor_data;
extern motor_data_t chassis_motor_data[4];				//存放电机数据信息
extern int16_t Chassis_Follow_Gimbal_Offset_Ecd;

float Angle;
float Sin;
float Cos;
	
	
extern ext_game_robot_state_t Game_Robot_State;
/* RM2022超级电容通信协议 */
float Chassis_Capacitance;
float Chassis_CapPower;

uint8_t diaotou_flag;
void chassis_task(void const * argument)
{	
	chassis_motor_init();//底盘电机初始化
	
	while(1)
	{		
		
		/*(1)	can接收云台遥控器传来的底盘运动模式和目标速度*/
/*			在"can_receive.c"里面的接收函数
				case 0x150:						
				{
					ChassisMode=rx_data[0];
					hero_target_forward_speed_x=(((int16_t)rx_data[1]<<8)|rx_data[2]);
					hero_target_right_speed_y=(((int16_t)rx_data[3]<<8)|rx_data[4]);
					hero_target_rotation_speed_z=(((int16_t)rx_data[5]<<8)|rx_data[6]);
					break;
				}
*/		
		
		/*(2)	根据底盘不同模式，运动学解算出电机目标速度*/
		if(ChassisMode == RC_SW_DOWN)//底盘无力模式
		{
			chassis_motor_data[0].target_current=0;//目标电流为0，电机无力
			chassis_motor_data[1].target_current=0;
			chassis_motor_data[2].target_current=0;
			chassis_motor_data[3].target_current=0;
		}
		
		if(ChassisMode ==	RC_SW_MID)//底盘云台运动分离，底盘自由运动
		{
//			chassis_target_forward_speed_x=hero_target_forward_speed_x;
//			chassis_target_right_speed_y=hero_target_right_speed_y;
//			chassis_target_rotation_speed_z=hero_target_rotation_speed_z;
			
			Angle = (yaw_motor_data.ecd - Chassis_Follow_Gimbal_Offset_Ecd) * 2.0 * 3.1415926 / 8192.0;
			Sin   = arm_sin_f32(Angle);
			Cos   = arm_cos_f32(Angle);
		
			chassis_target_forward_speed_x=Cos*hero_target_forward_speed_x+Sin*hero_target_right_speed_y;
			chassis_target_right_speed_y=-Sin*hero_target_forward_speed_x+Cos*hero_target_right_speed_y;
			chassis_target_rotation_speed_z=hero_target_rotation_speed_z;
			
			calculate_chassis_motor_target_speed_with_kinematics_solution(chassis_target_forward_speed_x,chassis_target_right_speed_y,chassis_target_rotation_speed_z);//运动学解算出电机目标速度
			calculate_chassis_motor_target_current_with_target_speed();//计算电机目标电流
		}
		
		if(ChassisMode == RC_SW_UP)//底盘跟随云台
		{
			Angle = (yaw_motor_data.ecd - Chassis_Follow_Gimbal_Offset_Ecd) * 2.0 * 3.1415926 / 8192.0;
			Sin   = arm_sin_f32(Angle);
			Cos   = arm_cos_f32(Angle);
		
			chassis_target_forward_speed_x=Cos*hero_target_forward_speed_x+Sin*hero_target_right_speed_y;
			chassis_target_right_speed_y=-Sin*hero_target_forward_speed_x+Cos*hero_target_right_speed_y;
			chassis_target_rotation_speed_z=hero_target_rotation_speed_z;
			
			if(hero_target_rotation_speed_z==0)//如果没开小陀螺，底盘跟随云台
			{
				chassis_follow_gimbal_ecd=yaw_motor_data.ecd;
				if(Chassis_Follow_Gimbal_Offset_Ecd -  chassis_follow_gimbal_ecd > +4096) chassis_follow_gimbal_ecd += 8192;
				if(Chassis_Follow_Gimbal_Offset_Ecd -  chassis_follow_gimbal_ecd < -4096) chassis_follow_gimbal_ecd -= 8192;
			
				if(	abs(Chassis_Follow_Gimbal_Offset_Ecd -  chassis_follow_gimbal_ecd) >=10)
					calculate_chassis_follow_gimbal_motor_speed_with_target_ecd();
				else	
					chassis_follow_gimbal_target_rotation_speed_z=0;
			
				calculate_chassis_motor_target_speed_with_kinematics_solution(	chassis_target_forward_speed_x,	chassis_target_right_speed_y,chassis_follow_gimbal_target_rotation_speed_z);//运动学解算出电机目标速度
				calculate_chassis_motor_target_current_with_target_speed();//计算电机目标电流
			}
			else//如果开小陀螺，按小陀螺的转速转
			{		
				if(hero_target_forward_speed_x==0	&& hero_target_right_speed_y==0)//如果只转小陀螺，不移动,这时候可以让小陀螺转的快一点，所以单列了这种情况
				{
//				chassis_target_rotation_speed_z=2*chassis_target_rotation_speed_z;//小陀螺速度加倍
				calculate_chassis_motor_target_speed_with_kinematics_solution(chassis_target_forward_speed_x,chassis_target_right_speed_y,chassis_target_rotation_speed_z);//运动学解算出电机目标速度
				calculate_chassis_motor_target_current_with_target_speed();//计算电机目标电流
				}
				else//如果边转小陀螺边动
				{				
//					chassis_target_rotation_speed_z=2*chassis_target_rotation_speed_z;//小陀螺速度加倍
					calculate_chassis_motor_target_speed_with_kinematics_solution(chassis_target_forward_speed_x,chassis_target_right_speed_y,1.5f*chassis_target_rotation_speed_z);//运动学解算出电机目标速度
					calculate_chassis_motor_target_current_with_target_speed();//计算电机目标电流
				}
			}
			
		}

//		Chassis_PowerLimitControl();
																	//这个限制函数是根据读取的裁判系统的功率有没有达到功率限制的四分之三，但是超电充电本质上还是在用底盘功率，相当于一用超电就进限制，限的太狠，
																	//2023.6.3打西工大爬坡的时候发现爬的贼慢，超电还有50%多但是爬不上去，彭总和王一卜按照步兵的经验觉得就是这个限制
																	//我怕之前带着限制函数测的底盘速度系数，去掉限制函数之后跑的太快会耗超电超的太猛
			Chassis_PowerCapControl();
		
		send_data_to_vofa4(	Power_Heat_Data.chassis_power,Game_Robot_State.chassis_power_limit,Chassis_Capacitance,Chassis_CapPower);
		send_motor_3508current_through_CAN1(chassis_motor_data[0].target_current,chassis_motor_data[1].target_current,chassis_motor_data[2].target_current,chassis_motor_data[3].target_current);
//		send_data_to_vofa4(chassis_motor_data[0].speed_rpm,chassis_motor_data[1].speed_rpm,chassis_motor_data[2].speed_rpm,chassis_motor_data[3].speed_rpm);
		vTaskDelay(2);
	}
	
}

void Chassis_PowerLimitControl(void)
{
	float total_current_limit = 0.0f;
	float total_current       = 0.0f;
	float power_scale         = 0.0f;
	float current_scale       = 0.0f;
	
	if(Game_Robot_State.chassis_power_limit == 0)//读出来是零说明没接裁判系统，就不做限制
	{
		total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
	}
  else
	{
		if(Power_Heat_Data.chassis_power_buffer < WARNING_POWER_BUFF)//如果底盘功率缓冲已经过低，低于警戒值了
		{
			if(Power_Heat_Data.chassis_power_buffer > 5.0f)//如果底盘功率缓冲超过5焦耳
			{
					power_scale = Power_Heat_Data.chassis_power_buffer / WARNING_POWER_BUFF;
			}
			else//如果底盘功率缓冲低于5焦耳
			{
					power_scale = 5.0f / WARNING_POWER_BUFF;
			}
			total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * power_scale;
		}
		else//如果底盘功率缓冲还比较多
		{
			if(Power_Heat_Data.chassis_power > Game_Robot_State.chassis_power_limit * 0.75f)//如果底盘功率超过底盘功率限制的四分之三
			{
				if(Power_Heat_Data.chassis_power < Game_Robot_State.chassis_power_limit)//如果还没有超功率限制
				{
					power_scale = (Game_Robot_State.chassis_power_limit - Power_Heat_Data.chassis_power) / (Game_Robot_State.chassis_power_limit - Game_Robot_State.chassis_power_limit*0.75f);	
				}
				else//如果超功率限制了
				{
					power_scale = 0.0f;
				}
				
				total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT * power_scale;
			}
			else//如果底盘功率低于底盘功率限制的四分之三，电流限制就放开一点
			{
				total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT;
			}
		}
	}
	
	total_current += fabs((float)chassis_motor_data[0].target_current);
	total_current += fabs((float)chassis_motor_data[1].target_current);
	total_current += fabs((float)chassis_motor_data[2].target_current);
	total_current += fabs((float)chassis_motor_data[3].target_current);

  if(total_current > total_current_limit)
  {
		current_scale = total_current_limit / total_current;
		chassis_motor_data[0].target_current *= current_scale;
		chassis_motor_data[1].target_current *= current_scale;
		chassis_motor_data[2].target_current *= current_scale;
		chassis_motor_data[3].target_current *= current_scale;
  }
}
float power_scale_watch;
void Chassis_PowerCapControl(void)
{
	float total_current_limit = 0.0f;
	float total_current       = 0.0f;
	float power_scale         = 0.0f;
	float current_scale       = 0.0f;
	
	if(Chassis_Capacitance == 0)
	{
		total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
	}
  else
	{
		if(Chassis_Capacitance < 0.32f)//卡卡又让改成0.6和1.2//彭总又让我改回0.32f	//老超电是0.32f
		{
			power_scale = Chassis_Capacitance / 0.32f;
			total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * power_scale;
			power_scale_watch=power_scale;
		}
		else
		{
			total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
		}
	}
	
	total_current += fabs((float)chassis_motor_data[0].target_current);
	total_current += fabs((float)chassis_motor_data[1].target_current);
	total_current += fabs((float)chassis_motor_data[2].target_current);
	total_current += fabs((float)chassis_motor_data[3].target_current);

  if(total_current > total_current_limit)
  {
		current_scale = total_current_limit / total_current;
		chassis_motor_data[0].target_current *= current_scale;
		chassis_motor_data[1].target_current *= current_scale;
		chassis_motor_data[2].target_current *= current_scale;
		chassis_motor_data[3].target_current *= current_scale;
  }
}

