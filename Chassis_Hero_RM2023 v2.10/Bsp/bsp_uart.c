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

/************************[1]	串口及串口DMA初始化函数*************************************/
/**[1]
  * brief   初始化dbus uart设备*/
void dbus_uart_init(void)
{
	/* (1) 开启串口空闲中断 */
	__HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);
	__HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);
	
	/*(2)	设置使用串口DMA传输*/
	uart_receive_dma_no_it(&DBUS_HUART, dbus_buf, DBUS_MAX_LEN);
}


/**[1.1]
  * @brief      使用全局串口中断(uart it),不使用DMA传输（不对吧，我怎么感觉反了,看着函数名就知道呀）
  * @param[in]  huart: uart IRQHandler id
  * @param[in]  pData: receive buff 
  * @param[in]  Size:  buff size
  * @retval     set success or fail
  */
static int uart_receive_dma_no_it(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{
  uint32_t tmp1 = 0;
	
	/*(1)	获取串口接收状态*/
  tmp1 = huart->RxState;
	
	/*(2)	如果外设已初始化并准备就绪，开启DMA传输	*/
	if (tmp1 == HAL_UART_STATE_READY)//外设已初始化并准备就绪
	{
		/*(2.1)	如果接收缓存为空或者缓存大小为0*/
		if ((pData == NULL) || (Size == 0))
		{
			return HAL_ERROR;
		}

		/*(2.2)	配置一下串口接收的地址和大小*/
		huart->pRxBuffPtr = pData;
		huart->RxXferSize = Size;
		huart->ErrorCode  = HAL_UART_ERROR_NONE;

		/*(2.3) 使能DMA流(不是说好不用DMA传输的吗)*/
		HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);

		/*(2.4) 通过设置UART CR3寄存器中的DMAR位，启用接收器请求的DMA传输*/
		SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

		return HAL_OK;
	}
	
	else
	{
		return HAL_BUSY;
	}
	
}


/************************[2]	串口中断函数*************************************/

/**[2]
  * brief 			This function handles USART1 global interrupt.
	* postscript	从"stm32f4xx_it.c"搞过来的	*/
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
  * brief     	uart中断时回调此函数
  * param[in]   huart: uart IRQHandler id
  * postscript	在串口1的中断里面调用此函数	*/
void uart_receive_handler(UART_HandleTypeDef *huart)
{  
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) && 
			__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
	{
		uart_rx_idle_callback(huart);
	}
}

/**[2.1.1]
  * @brief      uart接收到一帧数据后清除空闲中断标志
  * @param[in]  huart: uart IRQHandler id
  * @retval  
  */
static void uart_rx_idle_callback(UART_HandleTypeDef* huart)
{
	/* 清除空闲中断标志，避免总是空闲中断 */
	__HAL_UART_CLEAR_IDLEFLAG(huart);

	/* 处理空闲中断中的接收数据 */
	if (huart == &DBUS_HUART)
	{
		/* 清除DMA传输完成标志 */
		__HAL_DMA_DISABLE(huart->hdmarx);

		/* 从DMA处理dbus的数据dbus_buf */
		if ((DBUS_MAX_LEN - dma_current_data_counter(huart->hdmarx->Instance)) == DBUS_BUFLEN)
		{
			rc_callback_handler(&rc, dbus_buf);	
		}
		
		/* 重启dma传输 */
		__HAL_DMA_SET_COUNTER(huart->hdmarx, DBUS_MAX_LEN);
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
}



/**[2.1.1.1]
  * @brief     	返回当前DMAy Streamx传输中剩余数据单元的数量。
  * @param[in]  dma_stream: DMAy Streamx，y用来选择是DMA1还是DMA2，x用来选择是Stream的0到7
  * @retval     当前DMAy Streamx传输中剩余数据单元的数量。
  */
uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream)
{
  /* 返回DMAy Streamx的剩余数据单元数 */
  return ((uint16_t)(dma_stream->NDTR));
}

/**[2.1.1.2]
  * @brief       处理接收的rc数据
  * @param[out]  rc:   保存已处理rc数据的结构
  * @param[in]   buff: 保存原始rc数据的缓冲区
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


