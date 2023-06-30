/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       bsp_uart.h
 * @brief      this file contains the common defines and functions prototypes for 
 *             the bsp_uart.c driver
 * @note         
 * @Version    V1.0.0
 * @Date       Jan-30-2018      
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */

#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "usart.h"

#define UART_RX_DMA_SIZE (1024)
#define DBUS_MAX_LEN     (50)
#define DBUS_BUFLEN      (18)
#define DBUS_HUART       huart1 /* for dji remote controler reciever */
/** 
  * @brief  remote control information
  */
typedef __packed struct
{
  /* rocker channel information */
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t ch4;

  /* left and right lever information */
  uint8_t sw1;
  uint8_t sw2;
} rc_info_t;

void uart_receive_handler(UART_HandleTypeDef *huart);
void dbus_uart_init(void);
void USART_SendData(USART_TypeDef* USARTx, uint16_t Data);

static int uart_receive_dma_no_it(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size);
static void uart_rx_idle_callback(UART_HandleTypeDef* huart);
uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream);
void rc_callback_handler(rc_info_t *rc, uint8_t *buff);
#endif

