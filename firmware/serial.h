#pragma once
#include "ch32fun.h"

#define UART_BR 420000 // default for CRSF?

#define UART_RX_BUF_SIZE 256

typedef struct UartRxBufferT {
	volatile uint8_t buffer[UART_RX_BUF_SIZE];
	uint8_t b2[UART_RX_BUF_SIZE + 1];
	volatile uint32_t write_addr;
	volatile uint32_t read_addr;
	volatile uint8_t overrun;

} UartRxBufferT;

extern UartRxBufferT uart3_rxbuf;

void serial_init();

void dma_uart_setup(void);

void dma_uart_tx(const void *data, uint32_t len);

void clear_buf(UartRxBufferT *buf);

uint32_t bytes_available(UartRxBufferT* buf);
// return pointer to a contiguous buffer of data
uint8_t* read_data(UartRxBufferT* buf, uint32_t len);
