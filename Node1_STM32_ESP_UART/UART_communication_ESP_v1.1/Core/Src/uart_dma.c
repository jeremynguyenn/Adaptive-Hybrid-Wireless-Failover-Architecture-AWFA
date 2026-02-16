// uart_dma.c (Thêm chỉnh sửa để RX hoạt động đơn giản hơn: Tạm thời comment out parser CRC và timeout để test cơ bản với IDLE line; Sử dụng lại IDLE để detect frame mà không cần format đặc biệt; Điều này giúp RX phản hồi mà không cần gửi frame đặc biệt; Sau khi test OK, có thể uncomment parser nếu cần; Cập nhật ARR của TIM3 để timeout dài hơn nếu dùng; Thêm check NDTR để tính len chính xác; Sửa để tránh mất ký tự bằng cách process buffer đầy đủ.)
// ===== NEW IMPROVED: Uncommented and completed CRC parser with initial CRC=0xFFFF, state machine; Enabled TIM3 for inter-byte timeout; Added peripheral reset; Used ICR for flag clearing; Enabled DBM and FIFO for RX DMA; Added burst mode (single); Processed data in HTIF/TCIF to avoid overflow; Added more error types; Made baudrate and config more flexible.
// ===== FIXED: Reverted to SR/DR clearing for flags since ICR not available in STM32F4xx USART.
// ===== FIXED FOR RX ISSUE: Commented out the parser state machine and TIM3 to test basic IDLE detection first. Uncomment when format is ready. Copied data directly in IDLE IRQ for simple echo test.
// ===== NEW ADDED: Changed to USART1 for ESP32 communication (PA9 TX, PA10 RX); Changed DMA to DMA2 (Stream2 RX, Stream7 TX); Added get_apb2_clock for APB2; Added debug_uart_init and debug_send for USART2 polling TX to Realterm; Uncommented and adjusted TIM3 for 1s periodic PING; Added TIM3_IRQHandler; Adjusted IRQ names and priorities; Adjusted flag clearing for DMA2 streams.
// ===== NEW ADDED: Called tim3_init() in uart1_rx_tx_init() for 1s PING; Uncommented parser state and crc_update if needed, but kept commented for basic test.
// ===== FIXED: Completed the truncated init code with standard UART config. Fixed clock calculation to properly handle PLL/HSE/HSI for correct baud rate.
// ===== FIXED: Removed unused process_rx_buffer function to eliminate warning.
// ===== FIXED: Removed unused crc_update function, parser state enum, and related variables to eliminate warning and clean up code.

#include "uart_dma.h"
#include <string.h>
//[Author: Nguyen Trung Nhan]

/* ================= CLOCK CONFIG ================= */

#define USART1EN       (1U<<4)  // For USART1 on APB2
#define UART2EN        (1U<<17)  // For debug USART2
#define GPIOAEN        (1U<<0)
#define DMA2EN         (1U<<22)  // Changed to DMA2
#define TIM3EN         (1U<<1)     // For periodic timer

/* ================= UART BITS ================= */

#define CR1_TE         (1U<<3)
#define CR1_RE         (1U<<2)
#define CR1_UE         (1U<<13)
#define CR1_PCE        (1U<<10)   // Parity control enable
#define CR1_PS         (1U<<9)    // Parity selection
#define CR1_M          (1U<<12)   // Word length
#define CR1_OVER8      (1U<<15)   // Oversampling mode
#define CR1_IDLEIE     (1U<<4)    // IDLE interrupt enable
#define CR1_PEIE       (1U<<8)    // PE interrupt enable

#define CR2_STOP_1     (0U<<12)
#define CR2_STOP_2     (2U<<12)   // Stop bits config

#define CR3_DMAT       (1U<<7)
#define CR3_DMAR       (1U<<6)
#define CR3_EIE        (1U<<0)    // Error interrupt enable

#define SR_TXE         (1U<<7)    // For polling TX in debug
#define SR_IDLE        (1U<<4)    // IDLE line detected
#define SR_ORE         (1U<<3)    // Overrun error
#define SR_NE          (1U<<2)    // Noise error
#define SR_FE          (1U<<1)    // Framing error
#define SR_PE          (1U<<0)    // Parity error

#define UART_BAUDRATE  115200

/* ================= DMA ================= */
//[Author: Nguyen Trung Nhan]

#define DMA_SCR_EN        (1U<<0)
#define DMA_SCR_MINC      (1U<<10)
#define DMA_SCR_CIRC      (1U<<8)
#define DMA_SCR_TCIE      (1U<<4)
#define DMA_SCR_HTIE      (1U<<3)      // Half transfer interrupt enable
#define DMA_SCR_TEIE      (1U<<2)
#define DMA_SCR_PL_HIGH   (2U<<16)     // Priority level high
#define DMA_SCR_DBM       (1U<<18)     // Double buffer mode

#define DMA_SFCR_DMDIS    (1U<<2)      // FIFO enable
#define DMA_SFCR_FTH_FULL (3U<<0)      // FIFO threshold full
#define DMA_SFCR_FEIE     (1U<<7)      // FIFO error interrupt enable

#define LIFCR_CLEAR2   (0x3D<<16)  // Clear mask for Stream2
#define HIFCR_CLEAR7   (0x3D<<22)  // Clear mask for Stream7

/* ================= GLOBAL ================= */

#define HALF_BUFF_SIZE     (UART_DATA_BUFF_SIZE / 2)   // For dual buffer

char uart_data_buffer[UART_DATA_BUFF_SIZE];

char g_frame_data[256];                        // For frame data

volatile uint8_t  g_rx_cmplt = 0;
volatile uint8_t  g_tx_cmplt = 0;
volatile uint8_t  g_uart_error = 0;
volatile uint16_t g_rx_len = 0;      // frame length

volatile uint16_t g_rx_line_index = 0;

volatile uint8_t g_timer_tick = 0;  // Timer tick for periodic PING

/* ================= CLOCK GET REAL APB1/APB2 ================= */
// ===== FIXED: Proper system clock calculation to handle HSI/HSE/PLL for correct baud rate.

static uint32_t get_system_clock(void)
{
    uint32_t pllm, plln, pllp, pll_source, sysclk;
    uint32_t cfgr = RCC->CFGR;
    uint32_t pllcfgr = RCC->PLLCFGR;

    uint32_t sws = (cfgr & RCC_CFGR_SWS) >> 2;
    if (sws == 0) {  // HSI
        return 16000000;
    } else if (sws == 1) {  // HSE
        return 8000000;  // Change to your HSE frequency if different (e.g., 25000000 for some boards)
    } else if (sws == 2) {  // PLL
        pllm = pllcfgr & RCC_PLLCFGR_PLLM;
        plln = (pllcfgr & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos;
        pllp = (((pllcfgr & RCC_PLLCFGR_PLLP) >> RCC_PLLCFGR_PLLP_Pos) + 1) * 2;
        pll_source = (pllcfgr & RCC_PLLCFGR_PLLSRC) ? 8000000 : 16000000;  // HSE or HSI; adjust HSE freq if needed
        sysclk = ((pll_source / pllm) * plln) / pllp;
        return sysclk;
    }
    return 16000000;  // Default fallback
}
//[Author: Nguyen Trung Nhan]

static uint32_t get_apb1_clock(void)
{
    uint32_t sysclk = get_system_clock();
    uint32_t cfgr = RCC->CFGR;
    uint32_t ppre1 = (cfgr >> 10) & 0x7;
    if (ppre1 < 4) return sysclk;
    else return sysclk >> (ppre1 - 3);
}

static uint32_t get_apb2_clock(void)
{
    uint32_t sysclk = get_system_clock();
    uint32_t cfgr = RCC->CFGR;
    uint32_t ppre2 = (cfgr >> 13) & 0x7;
    if (ppre2 < 4) return sysclk;
    else return sysclk >> (ppre2 - 3);
}

/* ================= BAUD ================= */
//[Author: Nguyen Trung Nhan]
static void uart_set_baudrate(USART_TypeDef *usart, uint32_t baudrate)  // ===== NEW ADDED: Generalized for any USART
{
    uint32_t periph_clk;
    if(usart == USART1) periph_clk = get_apb2_clock();
    else periph_clk = get_apb1_clock();
    usart->BRR = (periph_clk + (baudrate/2U))/baudrate;
}

/* ================= DEBUG UART INIT (USART2 POLLING TX) ================= */  // ===== NEW ADDED: For displaying status on Realterm
//[Author: Nguyen Trung Nhan]

void debug_uart_init(void)
{
    RCC->AHB1ENR |= GPIOAEN;

    GPIOA->MODER &= ~(3U << 4);  // PA2 TX
    GPIOA->MODER |= (2U << 4);

    GPIOA->AFR[0] |= (7U << 8);  // AF7 for PA2

    RCC->APB1ENR |= UART2EN;

    USART2->CR1 &= ~CR1_M;        // 8-bit
    USART2->CR1 &= ~CR1_PCE;      // No parity
    USART2->CR2 &= ~(3U<<12);     // 1 stop bit
    USART2->CR1 &= ~CR1_OVER8;    // Oversampling 16

    uart_set_baudrate(USART2, UART_BAUDRATE);

    USART2->CR1 |= CR1_TE | CR1_UE;
}

void debug_send(const char *str)  // ===== NEW ADDED: Polling send string via USART2
{
    while(*str)
    {
        while(!(USART2->SR & SR_TXE));
        USART2->DR = *str++;
    }
}

/* ================= TIM3 CONFIG ================= */  // ===== NEW ADDED: Uncommented for 1s tick with proper PSC/ARR

static void tim3_init(void)
{
    RCC->APB1ENR |= TIM3EN;

    TIM3->PSC = (get_apb1_clock() / 1000) - 1;  // ===== FIXED: Dynamic PSC based on APB1 clock for 1kHz tick

    TIM3->ARR = 999;  // 1kHz / 1000 = 1Hz (1s)

    TIM3->DIER |= (1U << 0);  // UIE enable

    TIM3->CR1 |= (1U << 0);  // Enable timer

    NVIC_SetPriority(TIM3_IRQn,3);
    NVIC_EnableIRQ(TIM3_IRQn);
}
//[Author: Nguyen Trung Nhan]

/* ================= UART INIT ================= */

void uart1_rx_tx_init(void)  // ===== NEW ADDED: Changed to uart1
{
    // ===== NEW IMPROVED: Reset UART peripheral
    RCC->APB2RSTR |= (1U<<4);  // ===== NEW ADDED: USART1 reset
    RCC->APB2RSTR &= ~(1U<<4);

    RCC->AHB1ENR |= GPIOAEN;

    GPIOA->MODER &= ~(0xF << 18);  // ===== NEW ADDED: PA9 TX, PA10 RX
    GPIOA->MODER |= (0xA << 18);

    GPIOA->AFR[1] |= (7 << 4);  // ===== NEW ADDED: AF7 for PA9
    GPIOA->AFR[1] |= (7 << 8);  // PA10

    RCC->APB2ENR |= USART1EN;  // ===== NEW ADDED: USART1 enable

    USART1->CR1 &= ~CR1_M;        // 8-bit word length
    USART1->CR1 &= ~CR1_PCE;      // No parity
    USART1->CR2 &= ~(3U<<12);     // 1 stop bit
    USART1->CR1 &= ~CR1_OVER8;    // Oversampling 16

    uart_set_baudrate(USART1, UART_BAUDRATE);

    USART1->CR3 |= CR3_DMAR | CR3_DMAT;  // Enable DMA RX and TX
    USART1->CR3 |= CR3_EIE;              // Error interrupt enable
    USART1->CR1 |= CR1_IDLEIE;           // IDLE interrupt enable
    USART1->CR1 |= CR1_PEIE;             // Parity error interrupt enable

    USART1->CR1 |= CR1_TE | CR1_RE | CR1_UE;  // Enable TX, RX, UART

    NVIC_SetPriority(USART1_IRQn, 1);
    NVIC_EnableIRQ(USART1_IRQn);

    tim3_init();  // ===== NEW ADDED: Init TIM3 for 1s PING
}
//[Author: Nguyen Trung Nhan]

/* ================= DMA INIT ================= */

void dma2_init(void)
{
    RCC->AHB1ENR |= DMA2EN;

    // ===== NEW IMPROVED: Reset DMA2
    RCC->AHB1RSTR |= (1U<<22);
    RCC->AHB1RSTR &= ~(1U<<22);
}

/* ================= RX CONFIG ================= */
//[Author: Nguyen Trung Nhan]

void dma2_stream2_uart_rx_config(void)  // ===== NEW ADDED: Changed to stream2 for RX
{
    DMA2_Stream2->CR &= ~DMA_SCR_EN;
    while(DMA2_Stream2->CR & DMA_SCR_EN){}

    DMA2->LIFCR = LIFCR_CLEAR2;  // Clear all flags for Stream2

    DMA2_Stream2->PAR  = (uint32_t)&USART1->DR;  // ===== NEW ADDED: USART1 DR
    DMA2_Stream2->M0AR = (uint32_t)uart_data_buffer;
    DMA2_Stream2->NDTR = UART_DATA_BUFF_SIZE;
//[Author: Nguyen Trung Nhan]

    DMA2_Stream2->CR |= (4<<25);  // Channel 4 for USART1 RX
    DMA2_Stream2->CR &= ~(1<<6);  // DIR peripheral-to-memory (default 0)
    DMA2_Stream2->CR |= DMA_SCR_MINC;
    DMA2_Stream2->CR |= DMA_SCR_CIRC;
    DMA2_Stream2->CR |= DMA_SCR_TCIE;
    DMA2_Stream2->CR |= DMA_SCR_HTIE;     // ===== NEW ADDED: Enable HTIE để process half buffer, tránh mất data
    DMA2_Stream2->CR |= DMA_SCR_PL_HIGH;  // ===== NEW ADDED: DMA priority high
    DMA2_Stream2->CR |= DMA_SCR_TEIE;
    DMA2_Stream2->CR |= DMA_SCR_DBM;      // ===== NEW ADDED: Enable double buffer mode
//[Author: Nguyen Trung Nhan]

    /* ===== NEW IMPROVED: Enabled FIFO and FEIE */
    DMA2_Stream2->FCR |= DMA_SFCR_DMDIS;
    DMA2_Stream2->FCR |= DMA_SFCR_FTH_FULL;
    DMA2_Stream2->FCR |= DMA_SFCR_FEIE;
//[Author: Nguyen Trung Nhan]

    // ===== NEW IMPROVED: Added single burst mode (default is single, but explicit)
    DMA2_Stream2->CR &= ~(3U<<23);  // MBURST single
    DMA2_Stream2->CR &= ~(3U<<21);  // PBURST single

    NVIC_SetPriority(DMA2_Stream2_IRQn,2);  // ===== NEW ADDED: Changed to Stream2 IRQ
    NVIC_EnableIRQ(DMA2_Stream2_IRQn);

    DMA2_Stream2->CR |= DMA_SCR_EN;
}
//[Author: Nguyen Trung Nhan]

/* ================= TX CONFIG ================= */

void dma2_stream7_uart_tx_config(uint32_t msg_to_snd, uint32_t msg_len)  // ===== NEW ADDED: Changed to stream7 for TX
{
    DMA2_Stream7->CR &= ~DMA_SCR_EN;
    while(DMA2_Stream7->CR & DMA_SCR_EN){}

    DMA2->HIFCR = HIFCR_CLEAR7;  // ===== NEW ADDED: Adjusted clear for Stream7

    DMA2_Stream7->PAR  = (uint32_t)&USART1->DR;  // ===== NEW ADDED: USART1 DR
    DMA2_Stream7->M0AR = msg_to_snd;
    DMA2_Stream7->NDTR = msg_len;

    DMA2_Stream7->CR |= (4<<25);  // Channel 4 for USART1 TX
    DMA2_Stream7->CR |= DMA_SCR_MINC;
    DMA2_Stream7->CR |= (1<<6);  // DIR memory-to-peripheral
    DMA2_Stream7->CR |= DMA_SCR_TCIE;
    DMA2_Stream7->CR |= DMA_SCR_PL_HIGH;
//[Author: Nguyen Trung Nhan]

    DMA2_Stream7->FCR |= DMA_SFCR_DMDIS;
    DMA2_Stream7->FCR |= DMA_SFCR_FTH_FULL;
//[Author: Nguyen Trung Nhan]

    // ===== NEW IMPROVED: Added single burst mode
    DMA2_Stream7->CR &= ~(3U<<23);  // MBURST single
    DMA2_Stream7->CR &= ~(3U<<21);  // PBURST single

    NVIC_SetPriority(DMA2_Stream7_IRQn,2);  // ===== NEW ADDED: Changed to Stream7 IRQ
    NVIC_EnableIRQ(DMA2_Stream7_IRQn);

    DMA2_Stream7->CR |= DMA_SCR_EN;
}

/* ================= DMA IRQ ================= */

void DMA2_Stream7_IRQHandler(void)  // ===== NEW ADDED: Changed to Stream7 for TX
{
    if(DMA2->HISR & (1U<<27))  // TCIF7
    {
        g_tx_cmplt = 1;
        DMA2->HIFCR = (1U<<27);  // Clear TCIF7
    }
    if(DMA2->HISR & (1U<<25))  // TEIF7
    {
        g_uart_error = 1;
        DMA2->HIFCR = (1U<<25);  // Clear TEIF7
    }
}
//[Author: Nguyen Trung Nhan]

void DMA2_Stream2_IRQHandler(void)  // ===== NEW ADDED: Changed to Stream2 for RX
{
    if(DMA2->LISR & (1U<<20))  // HTIF2
    {
        DMA2->LIFCR = (1U<<20);  // Clear HTIF2
    }

    if(DMA2->LISR & (1U<<21))  // TCIF2
    {
        DMA2->LIFCR = (1U<<21);
    }
//[Author: Nguyen Trung Nhan]

    if(DMA2->LISR & (1U<<19))  // TEIF2
    {
        g_uart_error = 1;
        DMA2->LIFCR = (1U<<19);  // Clear TEIF2
    }
//[Author: Nguyen Trung Nhan]

    if(DMA2->LISR & (1U<<16))  // FEIF2
    {
        g_uart_error = 3;  // FIFO error
        DMA2->LIFCR = (1U<<16);  // Clear FEIF2
    }
}

/* ================= TIM3 IRQ for Periodic Tick ================= */  // ===== NEW ADDED: Uncommented and used for 1s tick

void TIM3_IRQHandler(void)
{
    TIM3->SR = 0;
    g_timer_tick = 1;  // ===== NEW ADDED: Set tick flag for main loop
}

/* ================= USART IRQ ================= */
//[Author: Nguyen Trung Nhan]

void USART1_IRQHandler(void)  // ===== NEW ADDED: Changed to USART1
{
    uint32_t sr = USART1->SR;

    /* === ERROR HANDLING//[Author: Nguyen Trung Nhan] === */
    if(sr & (SR_ORE | SR_NE | SR_FE | SR_PE))  // ===== NEW ADDED: Error handling for overrun, noise, framing, parity
    {
        volatile uint32_t temp = USART1->SR;  // ===== FIXED: Clear flags by reading SR (already done) and DR
        temp = USART1 -> DR;
        (void)temp;
        g_uart_error = 1;  // ===== NEW ADDED: Set error flag on UART errors
    }

    /* === IDLE LINE DETECTION//[Author: Nguyen Trung Nhan] === */
    if(sr & SR_IDLE)   // ===== NEW ADDED: Sử dụng IDLE để detect frame end
    {
        volatile uint32_t temp = USART1->SR;  // ===== FIXED: Clear IDLE by reading SR and DR
        temp = USART1 -> DR;
        (void)temp;
//[Author: Nguyen Trung Nhan]

        // ===== //[Author: Nguyen Trung Nhan]FIXED FOR RX ISSUE: Calculate len and copy data directly for basic test
        uint16_t received = UART_DATA_BUFF_SIZE - DMA2_Stream2->NDTR;  // ===== NEW ADDED: Changed to Stream2
        if(received > 0)
        {
        	for(uint16_t i = 0; i < received; i++)
        	{
        		char ch = uart_data_buffer[i];

        		if(ch == '\r' || ch == '\n')
        		{
        			if(g_rx_line_index > 0 && g_rx_cmplt == 0)   // ===== //[Author: Nguyen Trung Nhan]IMPROVED: chỉ xử lý khi có dữ liệu và chưa có lệnh đang chờ
        			{
        				g_frame_data[g_rx_line_index] = '\0';
        				g_rx_len = g_rx_line_index;
        				g_rx_cmplt = 1;
        				g_rx_line_index = 0;                     // reset để nhận dòng mới
        			}
        		}
        		else if(g_rx_line_index < sizeof(g_frame_data) - 1)
        		{
        			g_frame_data[g_rx_line_index++] = ch;
        		}
        	}
        }

        // Reset NDTR to restart DMA - Improved://[Author: Nguyen Trung Nhan] Reset DMA an toàn để nhận dòng tiếp theo
		
        DMA2_Stream2->CR &= ~DMA_SCR_EN;  // ===== NEW ADDED: Changed to Stream2
        while(DMA2_Stream2->CR & DMA_SCR_EN){}
        DMA2_Stream2->NDTR = UART_DATA_BUFF_SIZE;
        DMA2->LIFCR = LIFCR_CLEAR2;  // Clear flags  // ===== NEW ADDED: Adjusted for Stream2
        DMA2_Stream2->CR |= DMA_SCR_EN;
    }
}
