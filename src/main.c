#include <string.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_adc.h"
#include "em_cmu.h"
#include "timer.h"
#include "udelay.h"
#include "hal-config.h"
#include "main.h"
#include "mainctrl.h"
#include "uartdrv.h"
#include "spidrv.h"
#include "Typedefs.h"
#include "libdw1000.h"
#include "em_dma.h"

//extern volatile int8_t g_slaveWkup;

void clockConfig(void)
{
	CMU_ClockEnable(cmuClock_GPIO, true);

	timer_init();
	Delay_ms(5);
	GPIO_PinModeSet(gpioPortA, 1, gpioModePushPull, 1);
	Delay_ms(5);

	SystemCoreClockUpdate();

	/*
	 * chose external crystal oscillator as clock source.
	 * */
	CMU_OscillatorEnable(cmuOsc_HFXO, true, true);
	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
	CMU_OscillatorEnable(cmuOsc_HFRCO, false, false);

	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);
	CMU_OscillatorEnable(cmuSelect_ULFRCO, true, true);
	CMU_ClockEnable(cmuClock_HFLE, true);

	/*
	 * Enable clocks required
	 * */
	//CMU_ClockDivSet(cmuClock_HF, cmuClkDiv_2);
	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_USART0, true);
	CMU_ClockEnable(cmuClock_TIMER0, true);
	CMU_ClockEnable(cmuClock_TIMER1, true);
	CMU_ClockEnable(cmuClock_DMA, true);
}

#if 0

int main(void)
{
	bool recvDone = false;

	// Initialize chip
	CHIP_Init();

	powerADandUWB(1);
//	spiDMA_test();
	globalInit();
	clockConfig();
	SPIDMAInit();
	//uart_test();
	dwSpiRead(&g_dwDev, 0x00, 0x00, g_dwDev.networkAndAddress, 4);
	g_dwDev.networkAndAddress[0]= 01;
	g_dwDev.networkAndAddress[1]= 02;
	g_dwDev.networkAndAddress[2]= 03;
	g_dwDev.networkAndAddress[3]= 04;

	dwWriteNetworkIdAndDeviceAddress(&g_dwDev);
	memset(g_dwDev.networkAndAddress, 0x00, sizeof(g_dwDev.networkAndAddress));
	dwReadNetworkIdAndDeviceAddress(&g_dwDev);

	dwDeviceInit(&g_dwDev);
  	UDELAY_Calibrate();
  	Delay_ms(500);

	// Place breakpoint here and observe RxBuffer
	// RxBuffer should contain 0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9
	while(1) {
		recvDone = g_spiTransDes.recvDone;
	}
}
#elif 1

void spiDMA_test(dwDevice_t *dev)
{
	uint8_t buf[4] = {0};

	while(1) {
		dwSpiRead(dev, 0x00, 0x00, buf, 4);
		buf[0] = 0x01;
		buf[1] = 0x02;
		buf[2] = 0x03;
		buf[3] = 0x04;
//		dwSpiWrite(dev, 0x03, 0x00, buf, 4);
//		buf[0] = 0x05;
//		buf[1] = 0x06;
//		buf[2] = 0x07;
//		buf[3] = 0x08;
//		dwSpiWrite(dev, 0x03, 0x00, buf, 4);
//		buf[0] = 9;
//		buf[1] = 10;
//		buf[2] = 11;
//		buf[3] = 12;
//		dwSpiWrite(dev, 0x03, 0x00, buf, 4);
		memset(buf, 0x00, 4);
		Delay_ms(1);
		dwSpiRead(dev, 0x03, 0x00, buf, 4);
		dwSpiRead(dev, 0x04, 0x00, buf, 4);
	}
}
int main(void)
{
	/*
	 * Chip errata
	 * */
	CHIP_Init();//32000000

	/*
	 * global var init
	 * */
	globalInit();

	/*
	 * config needed clock
	 * */
	clockConfig();

	/*
	 * Timer init
	 * */
	timer_init();

	/*
	 * power down AD
	 * */
	powerADandUWB(0);
	/*
	 * SPI master config
	 * */
	//SPIConfig(SPI_CLK);
	SPIDMAInit();

	/*
	 * RS422 Uart init for delivering converted data
	 * */
	uartSetup();

	//LDO;
	SET_NUM = 5; //set number
	MAIN_NODE_ID = (SET_NUM - 1) << 2;
	//spiDMA_test(&g_dwDev);

	/*
	 * config one DMA channel for transferring ADC sample results
	 * to specific RAM buffer.
	 * */
	//DMAConfig();

	/*
	 * DW100 wireless device init
	 * */
	GPIO_PinModeSet(gpioPortC, 13, gpioModePushPull, 1);
	Delay_ms(2);
	GPIO_PinModeSet(gpioPortC, 13, gpioModePushPull, 0);

	GPIO_PinModeSet(gpioPortA, 2, gpioModePushPull, 0);
	Delay_ms(5);
	GPIO_PinModeSet(gpioPortA, 2, gpioModePushPull, 1);

	dwDeviceInit(&g_dwDev);

  	UDELAY_Calibrate();
  	Delay_ms(500);

	dwNewReceive(&g_dwDev);
	dwStartReceive(&g_dwDev);

	while (1) {
		switch(g_cur_mode)
		{
			case MAIN_WKUPMODE:
				WakeupSlave(&g_dwDev);
				break;

			case MAIN_SAMPLEMODE:
				RecvFromSlave(&g_dwDev);
				break;
			case MAIN_SYNCMODE:
				SyncSlave(&g_dwDev);
				break;
			case MAIN_IDLEMODE:
//				if (!pollSleepCMD(&g_dwDev))
				g_cur_mode = MAIN_WKUPMODE;
				break;

			case MAIN_SLEEPMODE:
				g_cur_mode = MAIN_SLEEPMODE;
				break;

			default:
				g_cur_mode = MAIN_WKUPMODE;
				break;
		}
	}
}
#endif
