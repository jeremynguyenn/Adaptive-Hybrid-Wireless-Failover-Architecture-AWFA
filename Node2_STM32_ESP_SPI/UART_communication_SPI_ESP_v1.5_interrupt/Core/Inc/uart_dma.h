#ifndef __UART_DMA_H__
#define __UART_DMA_H__
#include <stdint.h>
#include "stm32f4xx.h"

#define UART_DATA_BUFF_SIZE		5

void uart2_rx_tx_init(void);
void dma1_init(void);
void dma1_stream5_uart_rx_config(void);
void dma1_stream6_uart_tx_config(uint32_t msg_to_snd, uint32_t msg_len);

// ==============================================
// NEW: Export tất cả biến global để main.c và các file khác có thể dùng
extern char uart_data_buffer[UART_DATA_BUFF_SIZE];
extern uint8_t g_rx_cmplt;
extern uint8_t g_tx_cmplt;
extern uint8_t g_uart_cmplt;

void uart_dma_send_string(const char *str);
// ==============================================

#endif
