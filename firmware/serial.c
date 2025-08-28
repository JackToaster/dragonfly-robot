#include "serial.h"
#include "ch32fun.h"
#include "ch32v20xhw.h"


void serial_init() {
	// USART3

	// Enable UART (GPIOB already enabled)
	RCC->APB1PCENR |= RCC_APB1Periph_USART3;

	// Push-Pull, 10MHz Output on D5, with AutoFunction
	funPinMode(PB10, GPIO_CFGLR_OUT_10Mhz_AF_PP); // USART3-TX
	funPinMode(PB11, GPIO_CFGLR_IN_FLOAT); // USART3-RX

	// Setup UART for Tx 8n1
	USART3->CTLR1 = USART_WordLength_8b | USART_Parity_Odd | USART_Mode_Tx | USART_Mode_Rx | USART_IT_RXNE;
	USART3->CTLR2 = USART_StopBits_1 | USART_HardwareFlowControl_None;
	// Enable Tx DMA event
	USART3->CTLR3 = USART_DMAReq_Tx ;

	// Set baud rate and enable UART
	USART3->BRR = ((FUNCONF_SYSTEM_CORE_CLOCK)/2 + (UART_BR)/2) / (UART_BR);
	USART3->CTLR1 |= CTLR1_UE_Set;

	NVIC_EnableIRQ(USART3_IRQn);

	// USART4
	// Enable UART and GPIOD
	RCC->APB1PCENR |= RCC_APB1Periph_UART4; // UART4 is actually a USART on v203

	// Push-Pull, 10MHz Output on D5, with AutoFunction
	AFIO->PCFR2 |= GPIO_FullRemap_USART4;
	funPinMode(PB0, GPIO_CFGLR_OUT_50Mhz_AF_PP); // USART3-TX
	funPinMode(PB5, GPIO_CFGLR_IN_FLOAT); // USART3-RX

	// Setup UART for Tx 8n1
	UART4->CTLR1 = USART_WordLength_8b | USART_Parity_No | USART_Mode_Tx;
	UART4->CTLR2 = USART_StopBits_1;
	// Enable Tx DMA event
	UART4->CTLR3 = USART_DMAReq_Tx;

	// Set baud rate and enable UART
	UART4->BRR = ((FUNCONF_SYSTEM_CORE_CLOCK / 2) + (UART_BR)/2) / (UART_BR);
	UART4->CTLR1 |= CTLR1_UE_Set;
}

void dma_uart_setup(void)
{
	// Enable DMA peripheral
	RCC->AHBPCENR |= RCC_AHBPeriph_SRAM | RCC_AHBPeriph_DMA1;

	// Disable channel just in case there is a transfer in progress
	DMA1_Channel2->CFGR &= ~DMA_CFGR1_EN;

	// USART3 TX uses DMA channel 2
	DMA1_Channel2->PADDR = (uint32_t)&USART3->DATAR;
	// MEM2MEM: 0 (memory to peripheral)
	// PL: 0 (low priority since UART is a relatively slow peripheral)
	// MSIZE/PSIZE: 0 (8-bit)
	// MINC: 1 (increase memory address)
	// CIRC: 0 (one shot)
	// DIR: 1 (read from memory)
	// TEIE: 0 (no tx error interrupt)
	// HTIE: 0 (no half tx interrupt)
	// TCIE: 1 (transmission complete interrupt enable)
	// EN: 0 (do not enable DMA yet)
	DMA1_Channel2->CFGR = DMA_CFGR1_MINC | DMA_CFGR1_DIR | DMA_CFGR1_TCIE | DMA_Priority_VeryHigh;

	// Enable channel 4 interrupts
	NVIC_EnableIRQ(DMA1_Channel2_IRQn);
}

// DMA transfer completion interrupt. It will fire when the DMA transfer is
// complete. We use it just to blink the LED
__attribute__((interrupt)) __attribute__((section(".srodata")))
void DMA1_Channel2_IRQHandler(void)
{
	// Clear flag
	DMA1->INTFCR |= DMA_CTCIF2;
}

void dma_uart_tx(const void *data, uint32_t len)
{
	// Disable DMA channel (just in case a transfer is pending)
	DMA1_Channel2->CFGR &= ~DMA_CFGR1_EN;
	// Set transfer length and source address
	DMA1_Channel2->CNTR = len;
	DMA1_Channel2->MADDR = (uint32_t)data;
	// Enable DMA channel to start the transfer
	DMA1_Channel2->CFGR |= DMA_CFGR1_EN;
}


UartRxBufferT uart3_rxbuf;
volatile uint8_t _nothing;
// UART3 RXNE Interrupt
__attribute__((interrupt)) __attribute__((section(".srodata")))
void USART3_IRQHandler(void)
{
	// copy to buffer
	uart3_rxbuf.buffer[uart3_rxbuf.write_addr] = (uint8_t) (USART3->DATAR & (uint16_t)0x01FF);
	uart3_rxbuf.write_addr = (uart3_rxbuf.write_addr + 1) % UART_RX_BUF_SIZE;



	uart3_rxbuf.overrun |= uart3_rxbuf.write_addr == uart3_rxbuf.read_addr;

	// Clear flag
	USART3->STATR &= ~USART_FLAG_RXNE;
	_nothing = USART3->DATAR; // extra read
}


void clear_buf(UartRxBufferT *buf) {
	buf->overrun = 0;
	buf->read_addr = buf->write_addr;
}

uint32_t bytes_available(UartRxBufferT* buf) {
	if(buf->overrun) {
		clear_buf(buf);
	}
	return (buf->write_addr + UART_RX_BUF_SIZE - buf->read_addr) % UART_RX_BUF_SIZE;
}

// return pointer to a contiguous buffer of data, null terminated
uint8_t* read_data(UartRxBufferT* buf, uint32_t len) {
	uint32_t end_addr = (buf->read_addr + len) % UART_RX_BUF_SIZE;
	uint8_t* write_ptr = buf->b2;
	// we do a fun lil bespoke memcpy
	for(;buf->read_addr != end_addr; buf->read_addr = (buf->read_addr + 1) % UART_RX_BUF_SIZE) {
		*write_ptr++ = buf->buffer[buf->read_addr];
	}
	*write_ptr = 0; // null term
	return buf->b2;
}
