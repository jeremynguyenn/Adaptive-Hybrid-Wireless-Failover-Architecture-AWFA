// dma.c (No changes needed as this is unrelated to UART DMA improvements.)
// ===== NEW IMPROVED: Added full IRQ handler for DMA2_Stream0; Added reset; Added error handling.

#include "dma.h"
//[Author: Nguyen Trung Nhan]

#define DMA2EN				(1U<<22)
#define DMA_SCR_EN  		(1U<<0)
#define DMA_SCR_MINC		(1U<<10)
#define DMA_SCR_PINC		(1U<<9)
#define DMA_SCR_TCIE		(1U<<4)
#define DMA_SCR_TEIE		(1U<<2)
#define DMA_SFCR_DMDIS		(1U<<2)
//[Author: Nguyen Trung Nhan]

volatile uint8_t g_dma_mem2mem_cmplt = 0;
volatile uint8_t g_dma_mem2mem_error = 0;
//[Author: Nguyen Trung Nhan]

void dma2_mem2mem_config(void)
{
    // ===== NEW IMPROVED: Reset DMA2
    RCC->AHB1RSTR |= (1U<<22);
    RCC->AHB1RSTR &= ~(1U<<22);

	/*Enable clock access to the dma module*/
	RCC->AHB1ENR |= DMA2EN;

	/*Disable dma stream*/
	DMA2_Stream0->CR = 0;

	/*Wait until stream is disabled*/
	while((DMA2_Stream0->CR & DMA_SCR_EN)){}

	/*Configure dma parameters*/

	/*Set MSIZE i.e Memory data size to half-word*/
	DMA2_Stream0->CR |= (1U<<13);
	DMA2_Stream0->CR &= ~(1U<<14);

	/*Set PSIZE i.e Peripheral data size to half-word*/
	DMA2_Stream0->CR |= (1U<<11);
	DMA2_Stream0->CR &= ~(1U<<12);

	/*Enable memory addr increment*/
	DMA2_Stream0->CR |=DMA_SCR_MINC;

	/*Enable peripheral addr increment*/
	DMA2_Stream0->CR |=DMA_SCR_PINC;

	/*Select mem-to-mem transfer //[Author: Nguyen Trung Nhan]*/
	DMA2_Stream0->CR &= ~(1U<<6);
	DMA2_Stream0->CR |= (1U<<7);

	/*Enable transfer complete interrupt*/
	DMA2_Stream0->CR |= DMA_SCR_TCIE;

	/*Enable transfer error interrupt*/
	DMA2_Stream0->CR |= DMA_SCR_TEIE;
//[Author: Nguyen Trung Nhan]

	/*Disable direct mode*/
	DMA2_Stream0->FCR |=DMA_SFCR_DMDIS;

	/*Set DMA FIFO threshold*/
	DMA2_Stream0->FCR |=(1U<<0);
	DMA2_Stream0->FCR |=(1U<<1);

	/*Enable DMA interrupt in NVIC//[Author: Nguyen Trung Nhan]*/
	NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}
//[Author: Nguyen Trung Nhan]



void dma_transfer_start(uint32_t src_buff, uint32_t dest_buff, uint32_t len)
{
	/*Set peripheral address//[Author: Nguyen Trung Nhan]*/
	DMA2_Stream0->PAR = src_buff;

	/*Set memory address*/
	DMA2_Stream0->M0AR = dest_buff;

	/*Set transfer length//[Author: Nguyen Trung Nhan]*/
	DMA2_Stream0->NDTR = len;

	/*Enable dma stream*/
	DMA2_Stream0->CR |= DMA_SCR_EN;

}

// =====//[Author: Nguyen Trung Nhan] NEW IMPROVED: Added IRQ handler
void DMA2_Stream0_IRQHandler(void)
{
    if (DMA2->LISR & LISR_TCIF0) {
        g_dma_mem2mem_cmplt = 1;
        DMA2->LIFCR |= LIFCR_CTCIF0;
    }
    if (DMA2->LISR & LISR_TEIF0) {
        g_dma_mem2mem_error = 1;
        DMA2->LIFCR |= LIFCR_CTEIF0;
    }
}
