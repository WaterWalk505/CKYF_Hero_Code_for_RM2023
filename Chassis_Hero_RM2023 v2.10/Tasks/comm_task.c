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
		
		send_power_information_to_Cap_through_CAN1();//向超电发从裁判系统读到的功率热量信息
		vTaskDelay(5);
		
	 send_referee_data_through_CAN2();	
		vTaskDelay(5);
		
		send_remain_HP_through_CAN2();//发送机器人剩余血量
		vTaskDelay(5);
		
	 send_one_float_data_through_CAN2(Shoot_Data.bullet_speed);
		
//	 send_one_float_data_through_CAN2(dansu);
		
		vTaskDelay(20);//之前2ms一发会干扰云台C板控制其他电机，通信任务还是不要太集中吧，或者频率低一点发
	}

}
