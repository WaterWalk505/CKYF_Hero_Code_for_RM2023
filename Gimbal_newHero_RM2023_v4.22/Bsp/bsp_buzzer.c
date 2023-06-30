#include "bsp_buzzer.h"
#include "main.h"

extern TIM_HandleTypeDef htim4;

/**
  * @brief          ���ö�ʱ����������PWM�����򿪷�����
  * @param[in]      psc:psc��Ƶϵ��
	* @param[in]      arr:arr����ֵ
	* @param[in]      cce:ccr�Ƚ�ֵ
  * @retval         none
  */
void buzzer_on(uint16_t psc,uint16_t arr, uint16_t ccr)
{
    __HAL_TIM_PRESCALER(&htim4, psc);
		__HAL_TIM_SetAutoreload(&htim4,arr);							//PWM��Ƶ��=168M/(psc+1)(arr+1)
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, ccr);	//PWMռ�ձ�=(ccr-1)/arr
	
}

/**
  * @brief          ʹ��ѭ�������ӳ�һ��ʱ��
  * @param[in]      us:us΢��
  * @retval         none
  */
void buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}


