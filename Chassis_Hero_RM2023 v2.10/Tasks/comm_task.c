#include "comm_task.h"
#include "FreeRTOS.h"
#include "task.h"

#include "motor_can.h"
#include "can_transmit.h"
#include "Referee.h"

extern ext_shoot_data_t    Shoot_Data;
extern motor_data_t	chassis_motor_data[4];		
float dansu=1.52f;

	
void comm_task(void const * argument)
{
	while(1)
	{
		
		send_power_information_to_Cap_through_CAN1();//�򳬵緢�Ӳ���ϵͳ�����Ĺ���������Ϣ
		vTaskDelay(5);
		
	 send_referee_data_through_CAN2();	
		vTaskDelay(5);
		
		send_remain_HP_through_CAN2();//���ͻ�����ʣ��Ѫ��
		vTaskDelay(5);
		
	 send_one_float_data_through_CAN2(Shoot_Data.bullet_speed);
		
//	 send_one_float_data_through_CAN2(dansu);
		
		vTaskDelay(20);//֮ǰ2msһ���������̨C��������������ͨ�������ǲ�Ҫ̫���аɣ�����Ƶ�ʵ�һ�㷢
	}

}
