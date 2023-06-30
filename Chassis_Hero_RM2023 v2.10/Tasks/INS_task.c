#include "INS_task.h"
#include "FreeRTOS.h"
#include "task.h"//����vTaskDelay()
#include "bsp_imu.h"
#include "main.h"//ʶ�����ź궨��


extern imu_t              imu;

void led_off(void)
{
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6|LED_GREEN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
}


void INS_task(void const * argument)
{
	led_off();
	
	/*(1) ��ʼ��MPU6500��IST8310*/
	mpu_device_init();
	/*(2) ��ʼ����Ԫ��*/
	init_quaternion();	
	
	
	while(1)
	{
		/*(3) ��ȡ�����Ĵ��������ݲ����µ�imu����*/
		mpu_get_data();
		/*(4) ��̬���������Ԫ��*/
		imu_ahrs_update();
		/*(5)	������̬��*/
		imu_attitude_update(); 
		
		HAL_Delay(5);			
	
		vTaskDelay(2);
	}
	
}

