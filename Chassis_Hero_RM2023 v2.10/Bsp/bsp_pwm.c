#include "bsp_pwm.h"
#include "tim.h"

void PWM_SetDuty(TIM_HandleTypeDef *tim,uint32_t tim_channel,float duty){
	
	switch(tim_channel){	
		case TIM_CHANNEL_1: tim->Instance->CCR1 = (PWM_RESOLUTION*duty) - 1;break;
		case TIM_CHANNEL_2: tim->Instance->CCR2 = (PWM_RESOLUTION*duty) - 1;break;
		case TIM_CHANNEL_3: tim->Instance->CCR3 = (PWM_RESOLUTION*duty) - 1;break;
		case TIM_CHANNEL_4: tim->Instance->CCR4 = (PWM_RESOLUTION*duty) - 1;break;
	}
	
}

void PWM_init(void)
{
/* Open 4 sets of PWM waves respectively */
		/**TIM2 GPIO Configuration    
	PA1     ------> TIM2_CH2
	PA0     ------> TIM2_CH1
	PA2     ------> TIM2_CH3
	PA3     ------> TIM2_CH4 
	*/
	
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);

	/**TIM4 GPIO Configuration    
	PD15     ------> TIM4_CH4
	PD14     ------> TIM4_CH3
	PD13     ------> TIM4_CH2
	PD12     ------> TIM4_CH1 
	*/
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);

	/**TIM5 GPIO Configuration    
	PI0     ------> TIM5_CH4
	PH12     ------> TIM5_CH3
	PH11     ------> TIM5_CH2
	PH10     ------> TIM5_CH1 
	*/

	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_4);
	
	
	/**TIM8 GPIO Configuration    
	PI7     ------> TIM8_CH3
	PI6     ------> TIM8_CH2
	PI5     ------> TIM8_CH1
	PI2     ------> TIM8_CH4 
	*/
		
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
	
	
	PWM_SetDuty(&htim8,TIM_CHANNEL_1,0.10);
	PWM_SetDuty(&htim8,TIM_CHANNEL_2,0.20);
	PWM_SetDuty(&htim8,TIM_CHANNEL_3,0.30);
	PWM_SetDuty(&htim8,TIM_CHANNEL_4,0.40);
}
