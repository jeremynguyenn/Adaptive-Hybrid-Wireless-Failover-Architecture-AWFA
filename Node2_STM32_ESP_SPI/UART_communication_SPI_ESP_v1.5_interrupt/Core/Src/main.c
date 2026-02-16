#include "stm32f4xx.h"
#include "uart_dma.h"
#include "spi_dma.h"
#include "dma.h"          // giữ để compile (không dùng)
//[Author: Nguyen Trung Nhan]
// Buffer test SPI
uint8_t tx_buffer[4] = {0x11, 0x22, 0x33, 0x44};   // NEW: dữ liệu dễ kiểm tra echo
uint8_t rx_buffer[4];

// ==============================================
// NEW: Hàm in buffer dưới dạng hex qua UART DMA//[Author: Nguyen Trung Nhan]
static void uart_print_buffer(const char *prefix, uint8_t *buf, uint16_t len)
{
    uart_dma_send_string((char*)prefix);
    for(uint16_t i = 0; i < len; i++)
    {
        char hex[4] = {0};
        uint8_t high = buf[i] >> 4;
        uint8_t low  = buf[i] & 0x0F;

        hex[0] = (high < 10) ? '0' + high : 'A' + high - 10;
        hex[1] = (low  < 10) ? '0' + low  : 'A' + low  - 10;
        hex[2] = ' ';
        hex[3] = '\0';

        uart_dma_send_string(hex);
    }
    uart_dma_send_string("\r\n");
}
// ==============================================//[Author: Nguyen Trung Nhan]

int main(void)
{
    // ==============================================//[Author: Nguyen Trung Nhan]
    // UART monitor
    uart2_rx_tx_init();
    dma1_init();
    dma1_stream5_uart_rx_config();          // comment nếu không cần RX UART
    uart_dma_send_string("=== STM32F411RE + ESP32-S3 SPI DMA ===\r\n");
    uart_dma_send_string("UART monitor ready...\r\n");
    // ==============================================

    // ==============================================//[Author: Nguyen Trung Nhan]
    // SPI + DMA
    spi1_dma_init();
    spi_cs_init();
    dma2_stream3_spi_tx_init();
    dma2_stream2_spi_rx_init();
    // ==============================================//[Author: Nguyen Trung Nhan]

    uart_dma_send_string("SPI initialized. Starting communication with ESP32-S3...\r\n");

    // ===== NEW IMPROVED: Thêm biến để theo dõi trạng thái kết nối
    static uint8_t connected = 0;

    while(1)
    {
        spi_dma_test_transfer(tx_buffer, rx_buffer, 4);

        // =====//[Author: Nguyen Trung Nhan] NEW IMPROVED: Kiểm tra dữ liệu nhận được để xác nhận kết nối (dựa trên pattern echo +1 từ ESP32)
        if (rx_buffer[0] == tx_buffer[0] + 1 &&
            rx_buffer[1] == tx_buffer[1] + 1 &&
            rx_buffer[2] == tx_buffer[2] + 1)
        {
            if (!connected)
            {
                uart_dma_send_string("Connected to ESP32-S3 successfully!\r\n");
                connected = 1;
            }
        }
        else
        {
            if (connected)
            {
                uart_dma_send_string("Connection to ESP32-S3 lost!\r\n");
                connected = 0;
            }
        }

        // In dữ liệu nhận được từ ESP32//[Author: Nguyen Trung Nhan]
        uart_print_buffer("RX from ESP32: ", rx_buffer, 4);

        // NEW: Tăng byte đầu tiên để thấy thay đổi mỗi lần
        tx_buffer[0]++;
        if(tx_buffer[0] == 0) tx_buffer[0] = 0x11;   // tránh 0x00

        // Delay ~100ms (để ESP32 không bị flood Serial)//[Author: Nguyen Trung Nhan]
        for(volatile uint32_t d = 0; d < 800000; d++);
    }
}
