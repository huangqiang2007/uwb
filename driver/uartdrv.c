#include <stdint.h>
#include <stddef.h>
#include "em_core.h"
#include "em_device.h"
#include "em_usart.h"
#include "em_gpio.h"
#include "main.h"
#include "uartdrv.h"

#define UART_FRAMR_QUEUE_LEN_10 10

/*
 * uart frame queue
 * @num: data item counter
 * @in: enqueue index
 * @out: dequeue index
 * @RS422frame[]: array for storing items
 * */
typedef struct {
	volatile int8_t num;
	int8_t in, out;
	UartFrame RS422frame[UART_FRAMR_QUEUE_LEN_10];
}UartFrameQueueDef;

UartFrameQueueDef uartFrameQueue = {0};

static bool UartFrameEnqueueEmpty(void)
{
	if (uartFrameQueue.num == 0)
		return true;
	else
		return false;
}

static bool UartFrameEnqueueFull(void)
{
	if (uartFrameQueue.num == UART_FRAMR_QUEUE_LEN_10)
		return true;
	else
		return false;
}

int UartFrameEnqueue(UartFrame *uFrame)
{
	if (UartFrameEnqueueFull())
		return -1;

	uartFrameQueue.RS422frame[uartFrameQueue.in] = *uFrame;
	uartFrameQueue.in++;
	if (uartFrameQueue.in > UART_FRAMR_QUEUE_LEN_10 - 1)
		uartFrameQueue.in = 0;

	CORE_CriticalDisableIrq();
	uartFrameQueue.num += 1;
	CORE_CriticalEnableIrq();

	return 0;
}

UartFrame* UartFrameDequeue(void)
{
	UartFrame *uFrame = NULL;

	if (UartFrameEnqueueEmpty())
		goto out;

	uFrame = &uartFrameQueue.RS422frame[uartFrameQueue.out];
	uartFrameQueue.out++;
	if (uartFrameQueue.out > UART_FRAMR_QUEUE_LEN_10 - 1)
		uartFrameQueue.out = 0;

	CORE_CriticalDisableIrq();
	uartFrameQueue.num -= 1;
	CORE_CriticalEnableIrq();

out:
	return uFrame;
}

/*
 * Declare a circular buffer structure to use for Rx and Tx queues
 * */
#define BUFFERSIZE 480

volatile static struct circularBuffer
{
  uint8_t  data[BUFFERSIZE];  /* data buffer */
  uint32_t rdI;               /* read index */
  uint32_t wrI;               /* write index */
  uint32_t pendingBytes;      /* count of how many bytes are not yet handled */
  bool     overflow;          /* buffer overflow indicator */
} rxBuf, txBuf = { {0}, 0, 0, 0, false };

/* Setup UART0 in async mode for RS232*/
static USART_TypeDef *uart = USART0;
static USART_InitAsync_TypeDef uartInit = USART_INITASYNC_DEFAULT;

/*
 * uartSetup function
 */
void uartSetup(void)
{
	/*
	 * Enable clock for GPIO module (required for pin configuration)
	 * */
	/*
	 * Configure GPIO pins
	 * */
	GPIO_PinModeSet(gpioPortE, 10, gpioModePushPull, 1);
	GPIO_PinModeSet(gpioPortE, 11, gpioModeInput, 1);

	/*
	 * Prepare struct for initializing UART in asynchronous mode
	 * */
	uartInit.enable       = usartDisable;   /* Don't enable UART upon intialization */
	uartInit.refFreq      = 0;              /* Provide information on reference frequency. When set to 0, the reference frequency is */
	uartInit.baudrate     = 256000;         /* Baud rate *///115200 transfers to 148720
	uartInit.oversampling = usartOVS8;     /* Oversampling. Range is 4x, 6x, 8x or 16x */
	uartInit.databits     = usartDatabits8; /* Number of data bits. Range is 4 to 10 */
	uartInit.parity       = usartNoParity; /* Parity mode */
	uartInit.stopbits     = usartStopbits1; /* Number of stop bits. Range is 0 to 2 */
	uartInit.mvdis        = false;          /* Disable majority voting */
	uartInit.prsRxEnable  = false;          /* Enable USART Rx via Peripheral Reflex System */
	uartInit.prsRxCh      = usartPrsRxCh0;  /* Select PRS channel if enabled */

	/*
	 * Initialize USART with uartInit struct
	 * */
	USART_InitAsync(uart, &uartInit);

	/*
	 * Prepare UART Rx and Tx interrupts
	 * */
	USART_IntClear(uart, _USART_IFC_MASK);
	USART_IntEnable(uart, USART_IEN_RXDATAV);
	NVIC_ClearPendingIRQ(USART0_RX_IRQn);
//	NVIC_SetPriority(USART0_RX_IRQn,3);
	NVIC_ClearPendingIRQ(USART0_TX_IRQn);
//	NVIC_SetPriority(USART0_TX_IRQn,5);
	NVIC_EnableIRQ(USART0_RX_IRQn);
	NVIC_EnableIRQ(USART0_TX_IRQn);

	/*
	 * Enable I/O pins at UART1 location #2
	 * */
	uart->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN | USART_ROUTE_LOCATION_LOC0;

	/*
	 * Enable UART
	 * */
	USART_Enable(uart, usartEnable);
}


/*
 * @brief  uartGetChar function
 *
 *  Note that if there are no pending characters in the receive buffer, this
 *  function will hang until a character is received.
 *
 * */
uint8_t uartGetChar( )
{
	uint8_t ch;

	/*
	 * Check if there is a byte that is ready to be fetched. If no byte is ready, wait for incoming data
	 * */
	if (rxBuf.pendingBytes < 1) {
		while (rxBuf.pendingBytes < 1) ;
	}

	/* Copy data from buffer */
	ch = rxBuf.data[rxBuf.rdI];
	rxBuf.rdI = (rxBuf.rdI + 1) % BUFFERSIZE;

	/* Decrement pending byte counter */
	rxBuf.pendingBytes--;

	return ch;
}


/*
 * @brief  uartGetChar function
 *
 *  Note that if there are no pending characters in the receive buffer, this
 *  function will return 0, else return 1.
 * */
uint32_t uartReadChar(uint8_t *data)
{
	/*
	 * Check if there is a byte that is ready to be fetched. If no byte is ready, return 0
	 * */
	if (rxBuf.pendingBytes < 1){
		return 0 ;
	}

	/* Copy data from buffer */
	*data = rxBuf.data[rxBuf.rdI];
	rxBuf.rdI = (rxBuf.rdI + 1) % BUFFERSIZE;

	/* Decrement pending byte counter */
	rxBuf.pendingBytes--;

	return 1;
}

/******************************************************************************
 * @brief  uartPutChar function
 *
 *****************************************************************************/
void uartPutChar(uint8_t ch)
{
	/* Check if Tx queue has room for new data */
	if ((txBuf.pendingBytes + 1) > BUFFERSIZE) {
		/* Wait until there is room in queue */
		while ((txBuf.pendingBytes + 1) > BUFFERSIZE) ;
	}

	/* Copy ch into txBuffer */
	txBuf.data[txBuf.wrI] = ch;
	txBuf.wrI = (txBuf.wrI + 1) % BUFFERSIZE;

	/* Increment pending byte counter */
	txBuf.pendingBytes++;

	/* Enable interrupt on USART TX Buffer*/
	USART_IntEnable(uart, USART_IEN_TXBL);
}

/******************************************************************************
 * @brief  uartPutData function
 *
 *****************************************************************************/
void uartPutData(volatile uint8_t * dataPtr, uint32_t dataLen)
{
	uint32_t i = 0;

	/* Check if buffer is large enough for data */
	if (dataLen > BUFFERSIZE)
		return;

	/* Check if buffer has room for new data */
	if ((txBuf.pendingBytes + dataLen) > BUFFERSIZE) {
		/* Wait until room */
		while ((txBuf.pendingBytes + dataLen) > BUFFERSIZE);
	}

	/* Fill dataPtr[0:dataLen-1] into txBuffer */
	while (i < dataLen) {
		txBuf.data[txBuf.wrI] = *(dataPtr + i);
		txBuf.wrI = (txBuf.wrI + 1) % BUFFERSIZE;
		i++;
	}

	/* Increment pending byte counter */
	txBuf.pendingBytes += dataLen;

	/* Enable interrupt on USART TX Buffer*/
	USART_IntEnable(uart, USART_IEN_TXBL);
}

/*
 * @brief  uartGetData function
 * */
uint32_t uartGetData(uint8_t * dataPtr, uint32_t dataLen)
{
	uint32_t i = 0;

	/* Wait until the requested number of bytes are available */
	if (rxBuf.pendingBytes < dataLen) {
		while (rxBuf.pendingBytes < dataLen) ;
	}

	if (dataLen == 0) {
		dataLen = rxBuf.pendingBytes;
	}

	/* Copy data from Rx buffer to dataPtr */
	while (i < dataLen) {
		*(dataPtr + i) = rxBuf.data[rxBuf.rdI];
		rxBuf.rdI      = (rxBuf.rdI + 1) % BUFFERSIZE;
		i++;
	}

	/* Decrement pending byte counter */
	rxBuf.pendingBytes -= dataLen;

	return i;
}

/*
 * @brief UART0 RX IRQ Handler
 *
 * Set up the interrupt prior to use
 *
 * Note that this function handles overflows in a very simple way.
 * */
void USART0_RX_IRQHandler(void)
{
	/* Check for RX data valid interrupt */
	if (uart->IF & USART_IF_RXDATAV) {
		/* Copy data into RX Buffer */
		uint8_t rxData = USART_Rx(uart);
		rxBuf.data[rxBuf.wrI] = rxData;
		rxBuf.wrI             = (rxBuf.wrI + 1) % BUFFERSIZE;
		rxBuf.pendingBytes++;

		/* Flag Rx overflow */
		if (rxBuf.pendingBytes > BUFFERSIZE) {
			rxBuf.overflow = true;
		}
	}
}

/*
 * @brief UART0 TX IRQ Handler
 *
 * Set up the interrupt prior to use
 * */
void USART0_TX_IRQHandler(void)
{
	/* Check TX buffer level status */
	if (uart->IF & USART_IF_TXBL) {
		if (txBuf.pendingBytes > 0) {
			/* Transmit pending character */
			USART_Tx(uart, txBuf.data[txBuf.rdI]);
			txBuf.rdI = (txBuf.rdI + 1) % BUFFERSIZE;
			txBuf.pendingBytes--;
		}

		/* Disable Tx interrupt if no more bytes in queue */
		if (txBuf.pendingBytes == 0) {
			USART_IntDisable(uart, USART_IEN_TXBL);
		}
	}
}
