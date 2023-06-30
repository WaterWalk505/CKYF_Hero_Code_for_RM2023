#include "bsp_buzzer.h"
#include "main.h"

extern TIM_HandleTypeDef htim4;

/**
  * @brief          配置定时器参数产生PWM，来打开蜂鸣器
  * @param[in]      psc:psc分频系数
	* @param[in]      arr:arr重载值
	* @param[in]      cce:ccr比较值
  * @retval         none
  */
void buzzer_on(uint16_t psc,uint16_t arr, uint16_t ccr)
{
    __HAL_TIM_PRESCALER(&htim4, psc);
		__HAL_TIM_SetAutoreload(&htim4,arr);							//PWM的频率=168M/(psc+1)(arr+1)
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, ccr);	//PWM占空比=(ccr-1)/arr
	
}

/**
  * @brief          使用循环计数延迟一段时间
  * @param[in]      us:us微秒
  * @retval         none
  */
void buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}


