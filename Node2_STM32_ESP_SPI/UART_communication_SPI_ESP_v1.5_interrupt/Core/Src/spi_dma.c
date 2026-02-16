#include "spi_dma.h"
#include "uart_dma.h"  // ===== NEW IMPROVED: Added include to fix implicit declaration of uart_dma_send_string
//[Author: Nguyen Trung Nhan]
/*
 *  SPI1 Master DMA giao tiếp với ESP32-S3
 *  PA5 → GPIO18 (SCK)
 *  PA7 → GPIO23 (MOSI)
 *  PA6 ← GPIO19 (MISO)
 *  PA4 → GPIO5  (CS)
 *  GND chung
 */

#define GPIOAEN			(1U<<0)
#define SPI1EN			(1U<<12)
//[Author: Nguyen Trung Nhan]
#define CR1_SSM			(1U<<9)
#define CR1_SSI			(1U<<8)
#define CR1_MSTR		(1U<<2)
#define CR1_CPOL		(1U<<1)
#define CR1_PHA			(1U<<0)
#define CR2_TXDMAEN		(1U<<1)
#define CR2_RXDMAEN		(1U<<0)
#define CR1_SPE			(1U<<6)

#define DMA2EN				(1U<<22)
#define DMA_SCR_EN  		(1U<<0)
#define DMA_SCR_MINC		(1U<<10)
#define DMA_SCR_PINC		(1U<<9)
#define DMA_SCR_CIRC		(1U<<8)
#define DMA_SCR_TCIE		(1U<<4)
#define DMA_SCR_TEIE		(1U<<2)
#define DMA_SFCR_DMDIS		(1U<<2)

// ===== //[Author: Nguyen Trung Nhan]NEW IMPROVED: Thêm biến global flags cho interrupt DMA SPI
volatile uint8_t g_spi_tx_cmplt = 0;
volatile uint8_t g_spi_rx_cmplt = 0;
volatile uint8_t g_spi_tx_error = 0;
volatile uint8_t g_spi_rx_error = 0;

void spi1_dma_init(void)
{
	/************* GPIO Configuration *****************/
	RCC->AHB1ENR |= GPIOAEN;
//[Author: Nguyen Trung Nhan]
	// PA5 SCK, PA6 MISO, PA7 MOSI
	GPIOA->MODER &= ~(3U<<10); GPIOA->MODER |= (2U<<10);   // AF
	GPIOA->MODER &= ~(3U<<12); GPIOA->MODER |= (2U<<12);
	GPIOA->MODER &= ~(3U<<14); GPIOA->MODER |= (2U<<14);

	GPIOA->AFR[0] |= (5U<<20);   // AF5 cho SPI1
	GPIOA->AFR[0] |= (5U<<24);
	GPIOA->AFR[0] |= (5U<<28);

	/************* SPI Configuration *****************/
	RCC->APB2ENR |= SPI1EN;
//[Author: Nguyen Trung Nhan]
	SPI1->CR1 |= CR1_SSM | CR1_SSI | CR1_MSTR;           // master, software NSS
	SPI1->CR1 |= CR1_CPOL | CR1_PHA;                     // Mode 3
	SPI1->CR1 |= (1U<<3);                                // BR = /4 (PCLK/4)
	SPI1->CR1 &= ~(1U<<4);
	SPI1->CR1 &= ~(1U<<5);

	SPI1->CR2 |= CR2_RXDMAEN | CR2_TXDMAEN;
	SPI1->CR1 |= CR1_SPE;                                // enable SPI
}

// ==============================================
// NEW: Chân CS PA4 cho ESP32
void spi_cs_init(void)
{
	RCC->AHB1ENR |= GPIOAEN;
	GPIOA->MODER &= ~(3U<<8);
	GPIOA->MODER |=  (1U<<8);      // output
	GPIOA->OTYPER &= ~(1U<<4);     // push-pull
	GPIOA->OSPEEDR |= (3U<<8);     // high speed
	GPIOA->ODR |= (1U<<4);         // CS idle = HIGH
}
//[Author: Nguyen Trung Nhan]
void spi_cs_set(void)    { GPIOA->ODR |=  (1U<<4); }
void spi_cs_reset(void)  { GPIOA->ODR &= ~(1U<<4); }
// ==============================================
// ===== //[Author: Nguyen Trung Nhan]NEW IMPROVED: Bật interrupt cho DMA streams (TCIE, TEIE) và NVIC
void dma2_stream3_spi_tx_init(void)
{
	RCC->AHB1ENR |= DMA2EN;
	DMA2_Stream3->CR = 0;
	while(DMA2_Stream3->CR & DMA_SCR_EN);

	DMA2_Stream3->CR |= DMA_SCR_MINC;
	DMA2_Stream3->CR |= (3U<<25);           // Channel 3
	DMA2_Stream3->CR |= (1U<<6);            // Mem to Periph
	DMA2_Stream3->CR |= DMA_SCR_TCIE | DMA_SCR_TEIE;  // ===== NEW IMPROVED: Bật interrupt TC và TE
	DMA2_Stream3->FCR |= DMA_SFCR_DMDIS | (3U<<0);

	NVIC_EnableIRQ(DMA2_Stream3_IRQn);   // ===== NEW IMPROVED: Bật NVIC cho IRQ
}

void dma2_stream2_spi_rx_init(void)
{
	RCC->AHB1ENR |= DMA2EN;
	DMA2_Stream2->CR = 0;
	while(DMA2_Stream2->CR & DMA_SCR_EN);

	DMA2_Stream2->CR |= DMA_SCR_MINC;
	DMA2_Stream2->CR |= (3U<<25);           // Channel 3
	DMA2_Stream2->CR &= ~(3U<<6);           // Periph to Mem
	DMA2_Stream2->CR |= DMA_SCR_TCIE | DMA_SCR_TEIE;  // ===== NEW IMPROVED: Bật interrupt TC và TE
	DMA2_Stream2->FCR |= DMA_SFCR_DMDIS | (3U<<0);

	NVIC_EnableIRQ(DMA2_Stream2_IRQn);   // ===== NEW IMPROVED: Bật NVIC cho IRQ
}
//[Author: Nguyen Trung Nhan]
void dma2_stream3_spi_transfer(uint32_t msg_to_send, uint32_t msg_len)
{
	DMA2->LIFCR = (1U<<27) | (1U<<25);      // clear TC/TE for stream3
	DMA2_Stream3->PAR  = (uint32_t)&(SPI1->DR);
	DMA2_Stream3->M0AR = msg_to_send;
	DMA2_Stream3->NDTR = msg_len;
	DMA2_Stream3->CR  |= DMA_SCR_EN;
}
//[Author: Nguyen Trung Nhan]
void dma2_stream2_spi_receive(uint32_t received_msg, uint32_t msg_len)
{
	DMA2->LIFCR = (1U<<21) | (1U<<19);      // clear TC/TE for stream2
	DMA2_Stream2->PAR  = (uint32_t)&(SPI1->DR);
	DMA2_Stream2->M0AR = received_msg;
	DMA2_Stream2->NDTR = msg_len;
	DMA2_Stream2->CR  |= DMA_SCR_EN;
}
//[Author: Nguyen Trung Nhan]
// ==============================================
// NEW: Hàm chuyển dữ liệu SPI DMA + interrupt (thay vì polling)
// ===== NEW IMPROVED: Thay polling NDTR bằng chờ flags từ interrupt, thêm kiểm tra lỗi
void spi_dma_test_transfer(uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len)
{
	spi_cs_reset();

    // ===== NEW IMPROVED: Reset flags trước khi bắt đầu transfer
    g_spi_tx_cmplt = 0;
    g_spi_rx_cmplt = 0;
    g_spi_tx_error = 0;
    g_spi_rx_error = 0;

	dma2_stream3_spi_transfer((uint32_t)tx_buf, len);
	dma2_stream2_spi_receive((uint32_t)rx_buf, len);

	// ===== NEW IMPROVED: Chờ flags từ interrupt thay vì polling NDTR
    while(!g_spi_tx_cmplt || !g_spi_rx_cmplt);

    // ===== NEW IMPROVED: Kiểm tra lỗi từ DMA
    if (g_spi_tx_error || g_spi_rx_error) {
        uart_dma_send_string("SPI DMA Error!\r\n");  // Thông báo lỗi qua UART
        // Có thể thêm reset DMA hoặc xử lý khác
    }

	// //[Author: Nguyen Trung Nhan]===== NEW IMPROVED: Thêm chờ SPI không busy để đảm bảo dữ liệu flush hết
	while(SPI1->SR & (1U<<7));  // SR_BSY = 1U<<7

	spi_cs_set();

	// clear flags (dù đã clear trong handler, để chắc chắn)
	DMA2->LIFCR = (1U<<27)|(1U<<25)|(1U<<21)|(1U<<19);
}

// //[Author: Nguyen Trung Nhan]===== NEW IMPROVED: Thêm IRQ handlers cho DMA SPI
void DMA2_Stream3_IRQHandler(void)
{
    if (DMA2->LISR & (1U<<27)) {  // TCIF3
        g_spi_tx_cmplt = 1;
        DMA2->LIFCR |= (1U<<27);
    }
    if (DMA2->LISR & (1U<<25)) {  // TEIF3
        g_spi_tx_error = 1;
        DMA2->LIFCR |= (1U<<25);
    }
}
//[Author: Nguyen Trung Nhan]
void DMA2_Stream2_IRQHandler(void)
{
    if (DMA2->LISR & (1U<<21)) {  // TCIF2
        g_spi_rx_cmplt = 1;
        DMA2->LIFCR |= (1U<<21);
    }
    if (DMA2->LISR & (1U<<19)) {  // TEIF2
        g_spi_rx_error = 1;
        DMA2->LIFCR |= (1U<<19);
    }
}
// //[Author: Nguyen Trung Nhan]==============================================
