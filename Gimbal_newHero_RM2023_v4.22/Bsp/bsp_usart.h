#ifndef BSP_USART_H
#define BSP_USART_H

#include "main.h"

void usart1_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
void usart1_tx_dma_enable(uint8_t *data, uint16_t len);
void usart6_tx_dma_enable(uint8_t *data, uint16_t len);

#endif
