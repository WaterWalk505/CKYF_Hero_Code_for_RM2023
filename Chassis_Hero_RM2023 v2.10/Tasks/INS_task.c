#include "INS_task.h"
#include "FreeRTOS.h"
#include "task.h"//调用vTaskDelay()
#include "bsp_imu.h"
#include "main.h"//识别引脚宏定义


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
	
	/*(1) 初始化MPU6500和IST8310*/
	mpu_device_init();
	/*(2) 初始化四元数*/
	init_quaternion();	
	
	
	while(1)
	{
		/*(3) 获取处理后的传感器数据并更新到imu变量*/
		mpu_get_data();
		/*(4) 姿态解算更新四元数*/
		imu_ahrs_update();
		/*(5)	计算姿态角*/
		imu_attitude_update(); 
		
		HAL_Delay(5);			
	
		vTaskDelay(2);
	}
	
}

