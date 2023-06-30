#include "chassis_motor.h"
#include "pid.h"
#include "motor_can.h"

/********************������Ϣ�Ϳ�����Ϣ********************/
motor_data_t 		chassis_motor_data[4];				//��ŵ��������Ϣ
pid_type_def 		chassis_motor_speed_pid[4];		//��ŵ��������Ϣ
pid_type_def 		chassis_motor_follow_gimbal_angle_pid;	//��ŵ��������Ϣ
int16_t Chassis_Follow_Gimbal_Offset_Ecd=6980;//���̸�����̨��6020Ŀ��������Ƕ�

/********************�������PID����********************/
fp32 chassis_motor_speed_pid_Kp				=9.856f;
fp32 chassis_motor_speed_pid_Ki				=0.981f;
fp32 chassis_motor_speed_pid_Kd				=0.1f;//0.1
fp32 chassis_motor_speed_pid_max_out	=7000.0f;//16200.0f
fp32 chassis_motor_speed_pid_max_iout	=5000.0f;
fp32 chassis_motor_speed_PID[3];

fp32 chassis_motor_follow_gimbal_angle_pid_Kp=4.5f;
fp32 chassis_motor_follow_gimbal_angle_pid_Ki=0.0f;
fp32 chassis_motor_follow_gimbal_angle_pid_Kd=8.0f;
fp32 chassis_motor_follow_gimbal_angle_pid_max_out=12000;//12000;���ܺ���С���ݽ���֮����̸�����̨̫�Ͱѳ������ˣ����������һ��;MD�����ˣ�������������
fp32 chassis_motor_follow_gimbal_angle_pid_max_iout=500;
fp32 chassis_motor_follow_gimbal_angle_PID[3];




/**	[1]							���̵����ʼ��
  * brief         	��ʼ�������������Ϣ�Ϳ�����Ϣ
  * postscript										*/
void chassis_motor_init(void)
{
	/*(1)	��ʼ�����̵���ٶȻ�pid*/
	chassis_motor_speed_PID[0]	=	chassis_motor_speed_pid_Kp;
	chassis_motor_speed_PID[1]	=	chassis_motor_speed_pid_Ki;
	chassis_motor_speed_PID[2]	=	chassis_motor_speed_pid_Kd;
	
	PID_init(	&chassis_motor_speed_pid[0],	PID_DELTA,	chassis_motor_speed_PID,	chassis_motor_speed_pid_max_out,	chassis_motor_speed_pid_max_iout);
	PID_init(	&chassis_motor_speed_pid[1],	PID_DELTA,	chassis_motor_speed_PID,	chassis_motor_speed_pid_max_out,	chassis_motor_speed_pid_max_iout);
	PID_init(	&chassis_motor_speed_pid[2],	PID_DELTA,	chassis_motor_speed_PID,	chassis_motor_speed_pid_max_out,	chassis_motor_speed_pid_max_iout);
	PID_init(	&chassis_motor_speed_pid[3],	PID_DELTA,	chassis_motor_speed_PID,	chassis_motor_speed_pid_max_out,	chassis_motor_speed_pid_max_iout);
	
	/*(2)	��ʼ�����̸�����̨λ�û�pid*/
	chassis_motor_follow_gimbal_angle_PID[0] = chassis_motor_follow_gimbal_angle_pid_Kp;
	chassis_motor_follow_gimbal_angle_PID[1] = chassis_motor_follow_gimbal_angle_pid_Ki;
	chassis_motor_follow_gimbal_angle_PID[2] = chassis_motor_follow_gimbal_angle_pid_Kd;
	
	PID_init(	&chassis_motor_follow_gimbal_angle_pid,	PID_POSITION,	chassis_motor_follow_gimbal_angle_PID,	chassis_motor_follow_gimbal_angle_pid_max_out,	chassis_motor_follow_gimbal_angle_pid_max_iout);
}


/**[2]						����Ŀ���ٶȼ���Ŀ�����
  * brief																
	* postscript																																	*/
void calculate_chassis_motor_target_current_with_target_speed(void)
{
		chassis_motor_data[0].target_current	=	PID_calc(	&chassis_motor_speed_pid[0],	chassis_motor_data[0].speed_rpm,	chassis_motor_data[0].target_speed_rpm);
			
		chassis_motor_data[1].target_current	=	PID_calc(	&chassis_motor_speed_pid[1],	chassis_motor_data[1].speed_rpm,	chassis_motor_data[1].target_speed_rpm);	
	
		chassis_motor_data[2].target_current	=	PID_calc(	&chassis_motor_speed_pid[2],	chassis_motor_data[2].speed_rpm,	chassis_motor_data[2].target_speed_rpm);
			
		chassis_motor_data[3].target_current	=	PID_calc(	&chassis_motor_speed_pid[3],	chassis_motor_data[3].speed_rpm,	chassis_motor_data[3].target_speed_rpm);	
}

/**[3]						����Ŀ��λ�ü���Ŀ���ٶ�
  * brief																
	* postscript																																	*/
extern int16_t chassis_follow_gimbal_target_rotation_speed_z;
extern int16_t chassis_follow_gimbal_ecd;
void calculate_chassis_follow_gimbal_motor_speed_with_target_ecd(void)
{
		chassis_follow_gimbal_target_rotation_speed_z	=	-	PID_calc(	&chassis_motor_follow_gimbal_angle_pid, 	chassis_follow_gimbal_ecd,		Chassis_Follow_Gimbal_Offset_Ecd);//���Ӹ��žͳ���������
}

/**	[4]							
  * brief         	�����˶�ѧ����
	* param[in]				chassis_target_speed_x��ǰ���ٶ�
	* param[in]				chassis_target_speed_y�������ٶȣ�ң������ҡ���ƣ���ҡ���ƣ�
	* param[in]				chassis_target_rotation_speed_z����ת�ٶ�(��ʱ�벦���֣�����ʱ��ת)
	* postscript			ע�⣬���������ٶ�Ϊ��������Ե����˵����ת�������ڰ�װԭ����������ڵ��治һ����������Ϊ����ת					*/
void calculate_chassis_motor_target_speed_with_kinematics_solution(int16_t chassis_target_speed_x, int16_t chassis_target_speed_y,int16_t chassis_target_rotation_speed_z)
{
		chassis_motor_data[3].target_speed_rpm=+chassis_target_speed_x+chassis_target_speed_y-chassis_target_rotation_speed_z;//��ǰ�֣�ID=4
		chassis_motor_data[2].target_speed_rpm=-chassis_target_speed_x+chassis_target_speed_y-chassis_target_rotation_speed_z;//��ǰ�֣�ID=3
		chassis_motor_data[1].target_speed_rpm=-chassis_target_speed_x-chassis_target_speed_y-chassis_target_rotation_speed_z;//�Һ��֣�ID=2
		chassis_motor_data[0].target_speed_rpm=+chassis_target_speed_x-chassis_target_speed_y-chassis_target_rotation_speed_z;//����֣�ID=1
}

