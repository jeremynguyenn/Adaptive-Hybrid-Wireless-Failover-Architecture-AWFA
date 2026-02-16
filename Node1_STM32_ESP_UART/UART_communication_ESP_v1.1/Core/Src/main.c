// main.c (Sử dụng g_frame_data để echo, và thêm null terminate chính xác; Tăng buffer để tránh overflow.)
// ===== NEW IMPROVED: Added CRC validation before echo; Handled different error types; Added reset on persistent errors.
// ===== FIXED: Simplified error handling for basic test, only hardware error.
// ===== NEW ADDED: Changed to USART1 for communication with ESP32-S3; Added connection monitoring with PING/PONG protocol; Added debug output to USART2 polling; Integrated TIM3 for periodic PING sending; Added flags for connection status and timer tick.
// ===== NEW FIXED: Removed duplicate definition of g_timer_tick to fix multiple definition error; now using extern from header.
// ===== FIXED: No changes needed to main logic, but ensure clock is correct for baud rate.
//[Author: Nguyen Trung Nhan]
#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>
#include "uart_dma.h"

extern volatile uint8_t g_rx_cmplt;
extern volatile uint8_t g_tx_cmplt;
extern volatile uint8_t g_uart_error;
extern volatile uint16_t g_rx_len;

extern char g_frame_data[256];  // ===== NEW ADDED: Use parsed frame data

char msg_buff[300];  // ===== NEW ADDED: Increased size to avoid sprintf overflow warning

volatile uint8_t connected = 0;  // ===== NEW ADDED: Flag to track connection status
volatile uint8_t miss_count = 0;  // ===== NEW ADDED: Count missed PONG responses

int main(void)
{
    debug_uart_init();  // ===== NEW ADDED: Initialize USART2 for debug output to Realterm
    uart1_rx_tx_init();  // ===== NEW ADDED: Changed to uart1 for ESP32 communication
    dma2_init();  // ===== NEW ADDED: Changed to dma2 for USART1 DMA
    dma2_stream2_uart_rx_config();  // ===== NEW ADDED: Changed to stream2 for RX

    debug_send("[Author: Nguyen Trung Nhan]System Ready...\r\n");  // ===== NEW ADDED: Send initial message via debug UART

    while(1)
    {
        if(g_uart_error)
        {
            // ===== FIXED: Simplified for basic test//[Author: Nguyen Trung Nhan]

            debug_send("UART ERROR: Hardware!\r\n");  // ===== NEW ADDED: Send error via debug UART
            g_uart_error = 0;
            // ===== NEW IMPROVED: Auto-recovery on error//[Author: Nguyen Trung Nhan]

            dma2_stream2_uart_rx_config();  // ===== NEW ADDED: Re-init RX DMA (changed to stream2)
        }

        if(g_rx_cmplt)
        {
            g_frame_data[g_rx_len] = '\0';

            // ===== FIXED: No CRC check for basic test//[Author: Nguyen Trung Nhan]


            if(strcmp(g_frame_data, "PONG") == 0)  // ===== NEW ADDED: Check for PONG response from ESP32
            {
                miss_count = 0;
                if(!connected)
                {
                    connected = 1;
                    debug_send("[Author: Nguyen Trung Nhan]Connected to ESP32\r\n");  // ===== NEW ADDED: Display connected status
                }
            }
            else
            {
                debug_send("[Author: Nguyen Trung Nhan]Received invalid: ");  // ===== NEW ADDED: Display invalid response
                debug_send(g_frame_data);
                debug_send("\r\n");
            }

            g_rx_cmplt = 0;
        }
//[Author: Nguyen Trung Nhan]
        if(g_timer_tick)  // ===== NEW ADDED: Handle periodic tick from TIM3
        {
            g_timer_tick = 0;
            sprintf(msg_buff, "PING\r\n");
            g_tx_cmplt = 0;
            dma2_stream7_uart_tx_config((uint32_t)msg_buff, strlen(msg_buff));  // ===== NEW ADDED: Send PING via TX DMA (changed to stream7)
            while(!g_tx_cmplt){}
            miss_count++;
            if(miss_count > 3 && connected)
            {
                connected = 0;
                debug_send("Disconnected from ESP32\r\n");  // ===== NEW ADDED: Display disconnected status
            }
        }
    }
}
