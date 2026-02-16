// uart_dma.h (Tăng buffer size để tốt hơn.)
// ===== NEW IMPROVED: Increased buffer size; Added config struct for flexibility.
// ===== FIXED: Added #include <stdbool.h> for bool type.
// ===== NEW ADDED: Changed functions to uart1, dma2; Added debug functions declarations; Added globals for connection monitoring.

#ifndef __UART_DMA_H__
#define __UART_DMA_H__

#include <stdint.h>
#include <stdbool.h>  // ===== FIXED: Include for bool type
#include "stm32f4xx.h"

#define UART_DATA_BUFF_SIZE     512   // ===== NEW IMPROVED: Increased to 512 for larger data

extern char g_frame_data[256];        // ===== NEW ADDED: For parsed frame data

extern volatile uint8_t g_timer_tick;  // ===== NEW ADDED: For TIM3 periodic tick

// ===== NEW IMPROVED: Config struct for UART
typedef struct {
    uint32_t baudrate;
    uint8_t word_length;  // 0:8-bit, 1:9-bit
    uint8_t parity;       // 0:none, 1:even, 2:odd
    uint8_t stop_bits;    // 0:1, 1:0.5, 2:2, 3:1.5
    bool use_crc;         // Enable CRC parser
} UART_Config_t;

void debug_uart_init(void);  // ===== NEW ADDED: Debug USART2 init
void debug_send(const char *str);  // ===== NEW ADDED: Debug send function

void uart1_rx_tx_init(void);  // ===== NEW ADDED: Changed to uart1; Note: Can modify to take UART_Config_t* config
void dma2_init(void);  // ===== NEW ADDED: Changed to dma2
void dma2_stream2_uart_rx_config(void);  // ===== NEW ADDED: Changed to stream2
void dma2_stream7_uart_tx_config(uint32_t msg_to_snd, uint32_t msg_len);  // ===== NEW ADDED: Changed to stream7

#endif
