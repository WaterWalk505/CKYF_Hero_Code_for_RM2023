/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       bsp_uart.c
 * @brief      this file contains rc data receive and processing function
 * @note       
 * @Version    V1.0.0
 * @Date       Jan-30-2018
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */
                                                                                                              
#include "string.h"
#include "stdlib.h"
#include "bsp_uart.h"
#include "usart.h"
#include "main.h"


uint8_t   dbus_buf[DBUS_BUFLEN];
rc_info_t rc;

/************************[1]	���ڼ�����DMA��ʼ������*************************************/
/**[1]
  * brief   ��ʼ��dbus uart�豸*/
void dbus_uart_init(void)
{
	/* (1) �������ڿ����ж� */
	__HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);
	__HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);
	
	/*(2)	����ʹ�ô���DMA����*/
	uart_receive_dma_no_it(&DBUS_HUART, dbus_buf, DBUS_MAX_LEN);
}


/**[1.1]
  * @brief      ʹ��ȫ�ִ����ж�(uart it),��ʹ��DMA���䣨���԰ɣ�����ô�о�����,���ź�������֪��ѽ��
  * @param[in]  huart: uart IRQHandler id
  * @param[in]  pData: receive buff 
  * @param[in]  Size:  buff size
  * @retval     set success or fail
  */
static int uart_receive_dma_no_it(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{
  uint32_t tmp1 = 0;
	
	/*(1)	��ȡ���ڽ���״̬*/
  tmp1 = huart->RxState;
	
	/*(2)	��������ѳ�ʼ����׼������������DMA����	*/
	if (tmp1 == HAL_UART_STATE_READY)//�����ѳ�ʼ����׼������
	{
		/*(2.1)	������ջ���Ϊ�ջ��߻����СΪ0*/
		if ((pData == NULL) || (Size == 0))
		{
			return HAL_ERROR;
		}

		/*(2.2)	����һ�´��ڽ��յĵ�ַ�ʹ�С*/
		huart->pRxBuffPtr = pData;
		huart->RxXferSize = Size;
		huart->ErrorCode  = HAL_UART_ERROR_NONE;

		/*(2.3) ʹ��DMA��(����˵�ò���DMA�������)*/
		HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);

		/*(2.4) ͨ������UART CR3�Ĵ����е�DMARλ�����ý����������DMA����*/
		SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

		return HAL_OK;
	}
	
	else
	{
		return HAL_BUSY;
	}
	
}


/************************[2]	�����жϺ���*************************************/

/**[2]
  * brief 			This function handles USART1 global interrupt.
	* postscript	��"stm32f4xx_it.c"�������	*/
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	uart_receive_handler(&huart1);
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**[2.1]
  * brief     	uart�ж�ʱ�ص��˺���
  * param[in]   huart: uart IRQHandler id
  * postscript	�ڴ���1���ж�������ô˺���	*/
void uart_receive_handler(UART_HandleTypeDef *huart)
{  
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) && 
			__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
	{
		uart_rx_idle_callback(huart);
	}
}

/**[2.1.1]
  * @brief      uart���յ�һ֡���ݺ���������жϱ�־
  * @param[in]  huart: uart IRQHandler id
  * @retval  
  */
static void uart_rx_idle_callback(UART_HandleTypeDef* huart)
{
	/* ��������жϱ�־���������ǿ����ж� */
	__HAL_UART_CLEAR_IDLEFLAG(huart);

	/* ��������ж��еĽ������� */
	if (huart == &DBUS_HUART)
	{
		/* ���DMA������ɱ�־ */
		__HAL_DMA_DISABLE(huart->hdmarx);

		/* ��DMA����dbus������dbus_buf */
		if ((DBUS_MAX_LEN - dma_current_data_counter(huart->hdmarx->Instance)) == DBUS_BUFLEN)
		{
			rc_callback_handler(&rc, dbus_buf);	
		}
		
		/* ����dma���� */
		__HAL_DMA_SET_COUNTER(huart->hdmarx, DBUS_MAX_LEN);
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
}



/**[2.1.1.1]
  * @brief     	���ص�ǰDMAy Streamx������ʣ�����ݵ�Ԫ��������
  * @param[in]  dma_stream: DMAy Streamx��y����ѡ����DMA1����DMA2��x����ѡ����Stream��0��7
  * @retval     ��ǰDMAy Streamx������ʣ�����ݵ�Ԫ��������
  */
uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream)
{
  /* ����DMAy Streamx��ʣ�����ݵ�Ԫ�� */
  return ((uint16_t)(dma_stream->NDTR));
}

/**[2.1.1.2]
  * @brief       ������յ�rc����
  * @param[out]  rc:   �����Ѵ���rc���ݵĽṹ
  * @param[in]   buff: ����ԭʼrc���ݵĻ�����
  * @retval 
  */
void rc_callback_handler(rc_info_t *rc, uint8_t *buff)
{
	
  rc->ch1 = (buff[0] | buff[1] << 8) & 0x07FF;
  rc->ch1 -= 1024;
  rc->ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
  rc->ch2 -= 1024;
  rc->ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
  rc->ch3 -= 1024;
  rc->ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
  rc->ch4 -= 1024;

  rc->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
  rc->sw2 = (buff[5] >> 4) & 0x0003;
  
  if ((abs(rc->ch1) > 660) || \
      (abs(rc->ch2) > 660) || \
      (abs(rc->ch3) > 660) || \
      (abs(rc->ch4) > 660))
  {
    memset(rc, 0, sizeof(rc_info_t));
  }		
}


