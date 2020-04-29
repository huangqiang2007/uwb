#ifndef _SPIDRV_H_
#define _SPIDRV_H_

#include "em_usart.h"

#define SPI_CLK 200000

#define UWB_MAX_SPI_LEN 127
typedef struct {
	int txlen;
	int rxlen;
	uint8_t txBuf[UWB_MAX_SPI_LEN+3];
	uint8_t *rxBuf;
	volatile bool recvDone;
	volatile bool sendDone;
	volatile bool uwbIRQOccur;
} SPITransDes_t;

SPITransDes_t g_spiTransDes;

USART_TypeDef *usart_spi;

extern void SPIConfig(uint32_t spiclk);
extern void SPISendByte(uint8_t ucData);
extern int SPISendNbytes(uint8_t *str, int n);
extern uint8_t SPIRecvNBytes(uint8_t dst[], int n);
extern uint8_t USARTSpiTransfer(USART_TypeDef *usart, uint8_t data);

void initTransferDma(void);
void initReceiveDma(void);
void spiTransferForRead(SPITransDes_t *spiTransDes, uint8_t *txbuf, int txlen,
		uint8_t *rxbuf, int rxlen);
void spiTransferForWrite(SPITransDes_t *spiTransDes, uint8_t *txbuf, int txlen);
void SPIDMAInit(void);
void initUSART1 (int SpiClk);

#endif
