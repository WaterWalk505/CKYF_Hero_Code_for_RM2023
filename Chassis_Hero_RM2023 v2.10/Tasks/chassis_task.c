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

/*(1)	�����˶�ģʽ�͵���Ŀ���ٶ�*/
uint8_t ChassisMode;
int16_t hero_target_forward_speed_x,hero_target_right_speed_y,hero_target_rotation_speed_z;
int16_t chassis_target_forward_speed_x,chassis_target_right_speed_y,chassis_target_rotation_speed_z;
int16_t chassis_follow_gimbal_target_rotation_speed_z;
/*(2)	���̵����Ŀ���ٶȺ͵���*/
fp32 chassis_motor_speed_set[4];

int16_t chassis_follow_gimbal_ecd;
extern motor_data_t yaw_motor_data;
extern motor_data_t chassis_motor_data[4];				//��ŵ��������Ϣ
extern int16_t Chassis_Follow_Gimbal_Offset_Ecd;

float Angle;
float Sin;
float Cos;
	
	
extern ext_game_robot_state_t Game_Robot_State;
/* RM2022��������ͨ��Э�� */
float Chassis_Capacitance;
float Chassis_CapPower;

uint8_t diaotou_flag;
void chassis_task(void const * argument)
{	
	chassis_motor_init();//���̵����ʼ��
	
	while(1)
	{		
		
		/*(1)	can������̨ң���������ĵ����˶�ģʽ��Ŀ���ٶ�*/
/*			��"can_receive.c"����Ľ��պ���
				case 0x150:						
				{
					ChassisMode=rx_data[0];
					hero_target_forward_speed_x=(((int16_t)rx_data[1]<<8)|rx_data[2]);
					hero_target_right_speed_y=(((int16_t)rx_data[3]<<8)|rx_data[4]);
					hero_target_rotation_speed_z=(((int16_t)rx_data[5]<<8)|rx_data[6]);
					break;
				}
*/		
		
		/*(2)	���ݵ��̲�ͬģʽ���˶�ѧ��������Ŀ���ٶ�*/
		if(ChassisMode == RC_SW_DOWN)//��������ģʽ
		{
			chassis_motor_data[0].target_current=0;//Ŀ�����Ϊ0���������
			chassis_motor_data[1].target_current=0;
			chassis_motor_data[2].target_current=0;
			chassis_motor_data[3].target_current=0;
		}
		
		if(ChassisMode ==	RC_SW_MID)//������̨�˶����룬���������˶�
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
			
			calculate_chassis_motor_target_speed_with_kinematics_solution(chassis_target_forward_speed_x,chassis_target_right_speed_y,chassis_target_rotation_speed_z);//�˶�ѧ��������Ŀ���ٶ�
			calculate_chassis_motor_target_current_with_target_speed();//������Ŀ�����
		}
		
		if(ChassisMode == RC_SW_UP)//���̸�����̨
		{
			Angle = (yaw_motor_data.ecd - Chassis_Follow_Gimbal_Offset_Ecd) * 2.0 * 3.1415926 / 8192.0;
			Sin   = arm_sin_f32(Angle);
			Cos   = arm_cos_f32(Angle);
		
			chassis_target_forward_speed_x=Cos*hero_target_forward_speed_x+Sin*hero_target_right_speed_y;
			chassis_target_right_speed_y=-Sin*hero_target_forward_speed_x+Cos*hero_target_right_speed_y;
			chassis_target_rotation_speed_z=hero_target_rotation_speed_z;
			
			if(hero_target_rotation_speed_z==0)//���û��С���ݣ����̸�����̨
			{
				chassis_follow_gimbal_ecd=yaw_motor_data.ecd;
				if(Chassis_Follow_Gimbal_Offset_Ecd -  chassis_follow_gimbal_ecd > +4096) chassis_follow_gimbal_ecd += 8192;
				if(Chassis_Follow_Gimbal_Offset_Ecd -  chassis_follow_gimbal_ecd < -4096) chassis_follow_gimbal_ecd -= 8192;
			
				if(	abs(Chassis_Follow_Gimbal_Offset_Ecd -  chassis_follow_gimbal_ecd) >=10)
					calculate_chassis_follow_gimbal_motor_speed_with_target_ecd();
				else	
					chassis_follow_gimbal_target_rotation_speed_z=0;
			
				calculate_chassis_motor_target_speed_with_kinematics_solution(	chassis_target_forward_speed_x,	chassis_target_right_speed_y,chassis_follow_gimbal_target_rotation_speed_z);//�˶�ѧ��������Ŀ���ٶ�
				calculate_chassis_motor_target_current_with_target_speed();//������Ŀ�����
			}
			else//�����С���ݣ���С���ݵ�ת��ת
			{		
				if(hero_target_forward_speed_x==0	&& hero_target_right_speed_y==0)//���ֻתС���ݣ����ƶ�,��ʱ�������С����ת�Ŀ�һ�㣬���Ե������������
				{
//				chassis_target_rotation_speed_z=2*chassis_target_rotation_speed_z;//С�����ٶȼӱ�
				calculate_chassis_motor_target_speed_with_kinematics_solution(chassis_target_forward_speed_x,chassis_target_right_speed_y,chassis_target_rotation_speed_z);//�˶�ѧ��������Ŀ���ٶ�
				calculate_chassis_motor_target_current_with_target_speed();//������Ŀ�����
				}
				else//�����תС���ݱ߶�
				{				
//					chassis_target_rotation_speed_z=2*chassis_target_rotation_speed_z;//С�����ٶȼӱ�
					calculate_chassis_motor_target_speed_with_kinematics_solution(chassis_target_forward_speed_x,chassis_target_right_speed_y,1.5f*chassis_target_rotation_speed_z);//�˶�ѧ��������Ŀ���ٶ�
					calculate_chassis_motor_target_current_with_target_speed();//������Ŀ�����
				}
			}
			
		}

//		Chassis_PowerLimitControl();
																	//������ƺ����Ǹ��ݶ�ȡ�Ĳ���ϵͳ�Ĺ�����û�дﵽ�������Ƶ��ķ�֮�������ǳ����籾���ϻ������õ��̹��ʣ��൱��һ�ó���ͽ����ƣ��޵�̫�ݣ�
																	//2023.6.3�����������µ�ʱ�����������������绹��50%�൫��������ȥ�����ܺ���һ�����ղ����ľ�����þ����������
																	//����֮ǰ�������ƺ�����ĵ����ٶ�ϵ����ȥ�����ƺ���֮���ܵ�̫���ĳ��糬��̫��
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
	
	if(Game_Robot_State.chassis_power_limit == 0)//����������˵��û�Ӳ���ϵͳ���Ͳ�������
	{
		total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
	}
  else
	{
		if(Power_Heat_Data.chassis_power_buffer < WARNING_POWER_BUFF)//������̹��ʻ����Ѿ����ͣ����ھ���ֵ��
		{
			if(Power_Heat_Data.chassis_power_buffer > 5.0f)//������̹��ʻ��峬��5����
			{
					power_scale = Power_Heat_Data.chassis_power_buffer / WARNING_POWER_BUFF;
			}
			else//������̹��ʻ������5����
			{
					power_scale = 5.0f / WARNING_POWER_BUFF;
			}
			total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * power_scale;
		}
		else//������̹��ʻ��廹�Ƚ϶�
		{
			if(Power_Heat_Data.chassis_power > Game_Robot_State.chassis_power_limit * 0.75f)//������̹��ʳ������̹������Ƶ��ķ�֮��
			{
				if(Power_Heat_Data.chassis_power < Game_Robot_State.chassis_power_limit)//�����û�г���������
				{
					power_scale = (Game_Robot_State.chassis_power_limit - Power_Heat_Data.chassis_power) / (Game_Robot_State.chassis_power_limit - Game_Robot_State.chassis_power_limit*0.75f);	
				}
				else//���������������
				{
					power_scale = 0.0f;
				}
				
				total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT * power_scale;
			}
			else//������̹��ʵ��ڵ��̹������Ƶ��ķ�֮�����������ƾͷſ�һ��
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
		if(Chassis_Capacitance < 0.32f)//�������øĳ�0.6��1.2//���������ҸĻ�0.32f	//�ϳ�����0.32f
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

