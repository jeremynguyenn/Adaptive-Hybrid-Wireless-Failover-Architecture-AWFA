#include "uart_dma.h"

#define UART2EN			(1U<<17)
#define GPIOAEN			(1U<<0)

#define CR1_TE			(1U<<3)
#define CR1_RE			(1U<<2)
#define CR1_UE			(1U<<13)
#define SR_TXE			(1U<<7)

#define CR3_DMAT		(1U<<7)
#define CR3_DMAR		(1U<<6)
#define SR_TC			(1U<<6)
#define CR1_TCIE		(1U<<6)

#define UART_BAUDRATE	115200
#define CLK				16000000

#define DMA1EN			    (1U<<21)
#define DMA_SCR_EN  		(1U<<0)
#define DMA_SCR_MINC		(1U<<10)
#define DMA_SCR_PINC		(1U<<9)
#define DMA_SCR_CIRC		(1U<<8)
#define DMA_SCR_TCIE		(1U<<4)
#define DMA_SCR_TEIE		(1U<<2)
#define DMA_SFCR_DMDIS		(1U<<2)

#define HIFCR_CDMEIF5		(1U<<8)
#define HIFCR_CTEIF5		(1U<<9)
#define HIFCR_CTCIF5		(1U<<11)

#define HIFCR_CDMEIF6		(1U<<18)
#define HIFCR_CTEIF6		(1U<<19)
#define HIFCR_CTCIF6		(1U<<21)

#define HIFSR_TCIF5		(1U<<11)
#define HIFSR_TCIF6		(1U<<21)

static uint16_t compute_uart_bd(uint32_t periph_clk, uint32_t baudrate);
static void uart_set_baudrate(uint32_t periph_clk, uint32_t baudrate);

char uart_data_buffer[UART_DATA_BUFF_SIZE];

uint8_t g_rx_cmplt;
uint8_t g_tx_cmplt;
uint8_t g_uart_cmplt;

// ==============================================
// NEW: Hàm gửi chuỗi qua UART DMA để monitor trạng thái
void uart_dma_send_string(const char *str)
{
	if(str == NULL) return;
	uint32_t len = 0;
	while(str[len] != '\0' && len < 200) len++;   // an toàn
	dma1_stream6_uart_tx_config((uint32_t)str, len);
	while(g_tx_cmplt == 0);   // chờ DMA hoàn tất
	g_tx_cmplt = 0;
}
// ==============================================

void uart2_rx_tx_init(void)
{
	/*************Configure UART GPIO pin********************/
	RCC->AHB1ENR |= GPIOAEN;

	GPIOA->MODER &= ~(1U<<4); GPIOA->MODER |=  (1U<<5);   // PA2 TX
	GPIOA->MODER &= ~(1U<<6); GPIOA->MODER |=  (1U<<7);   // PA3 RX

	GPIOA->AFR[0] |= (0x7U<<8);   // AF7 cho PA2
	GPIOA->AFR[0] |= (0x7U<<12);  // AF7 cho PA3

	/*************Configure UART Module********************/
	RCC->APB1ENR |= UART2EN;
	uart_set_baudrate(CLK,UART_BAUDRATE);

	USART2->CR3 = CR3_DMAT | CR3_DMAR;
	USART2->CR1 = CR1_TE | CR1_RE;
	USART2->SR  &= ~SR_TC;
	USART2->CR1 |= CR1_TCIE;
	USART2->CR1 |= CR1_UE;

	NVIC_EnableIRQ(USART2_IRQn);
}

void dma1_init(void)
{
	RCC->AHB1ENR |= DMA1EN;
	NVIC_EnableIRQ(DMA1_Stream6_IRQn);
}

void dma1_stream5_uart_rx_config(void)
{
	DMA1_Stream5->CR &= ~DMA_SCR_EN;
	while(DMA1_Stream5->CR & DMA_SCR_EN);
	DMA1->HIFCR = HIFCR_CDMEIF5 | HIFCR_CTEIF5 | HIFCR_CTCIF5;

	DMA1_Stream5->PAR  = (uint32_t)&(USART2->DR);
	DMA1_Stream5->M0AR = (uint32_t)uart_data_buffer;
	DMA1_Stream5->NDTR = UART_DATA_BUFF_SIZE;

	DMA1_Stream5->CR &= ~((3U<<25)|(3U<<27));
	DMA1_Stream5->CR |=  (1U<<27);           // Channel 4
	DMA1_Stream5->CR |= DMA_SCR_MINC | DMA_SCR_TCIE | DMA_SCR_CIRC;

	DMA1_Stream5->CR &= ~(3U<<6);            // Periph to Mem
	DMA1_Stream5->CR |= DMA_SCR_EN;

	NVIC_EnableIRQ(DMA1_Stream5_IRQn);
}

void dma1_stream6_uart_tx_config(uint32_t msg_to_snd, uint32_t msg_len)
{
	DMA1_Stream6->CR &= ~DMA_SCR_EN;
	while(DMA1_Stream6->CR & DMA_SCR_EN);
	DMA1->HIFCR = HIFCR_CDMEIF6 | HIFCR_CTEIF6 | HIFCR_CTCIF6;

	DMA1_Stream6->PAR  = (uint32_t)&(USART2->DR);
	DMA1_Stream6->M0AR = msg_to_snd;
	DMA1_Stream6->NDTR = msg_len;

	DMA1_Stream6->CR &= ~((3U<<25)|(3U<<27));
	DMA1_Stream6->CR |=  (1U<<27);           // Channel 4
	DMA1_Stream6->CR |= DMA_SCR_MINC;
	DMA1_Stream6->CR |= (1U<<6);             // Mem to Periph
	DMA1_Stream6->CR |= DMA_SCR_TCIE;
	DMA1_Stream6->CR |= DMA_SCR_EN;
}

static uint16_t compute_uart_bd(uint32_t periph_clk, uint32_t baudrate)
{
	return ((periph_clk + (baudrate/2U)) / baudrate);
}

static void uart_set_baudrate(uint32_t periph_clk, uint32_t baudrate)
{
	USART2->BRR = compute_uart_bd(periph_clk, baudrate);
}

void DMA1_Stream6_IRQHandler(void)
{
	if(DMA1->HISR & HIFSR_TCIF6)
	{
		g_tx_cmplt = 1;
		DMA1->HIFCR |= HIFCR_CTCIF6;
	}
}

void DMA1_Stream5_IRQHandler(void)
{
	if(DMA1->HISR & HIFSR_TCIF5)
	{
		g_rx_cmplt = 1;
		DMA1->HIFCR |= HIFCR_CTCIF5;
	}
}

void USART2_IRQHandler(void)
{
	g_uart_cmplt = 1;
	USART2->SR &= ~SR_TC;
}
