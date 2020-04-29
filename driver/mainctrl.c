#include <stdbool.h>
#include <string.h>
#include "em_usart.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_timer.h"
#include "timer.h"
#include "uartdrv.h"
#include "mainctrl.h"
#include "libdw1000.h"
#include "libdw1000Types.h"
#include "main.h"

volatile uint8_t g_slaveStatus = 0;
uint8_t cnt[4];

void globalInit(void)
{
	memset((void *)&g_mainCtrlFr, 0x00, sizeof(g_mainCtrlFr));
	memset((void *)&g_recvSlaveFr, 0x00, sizeof(g_recvSlaveFr));
	memset((void *)&g_dwDev , 0x00, sizeof(g_dwDev));
//	memset((void *)&g_dwMacFrameSend, 0x00, sizeof(g_dwMacFrameSend));
//	memset((void *)&g_dwMacFrameRecv, 0x00, sizeof(g_dwMacFrameRecv));
	memset(&cnt, 0x00, sizeof(cnt));

	g_dataRecvDone = false;
	g_dataRecvSleep = false;
	g_slaveWkup = false;

	g_cur_mode = MAIN_WKUPMODE;
}

void powerADandUWB(uint8_t master)
{
	if (master == 1) {
		GPIO_PinModeSet(gpioPortA, 0, gpioModePushPull, 1);
	} else {
		GPIO_PinModeSet(gpioPortA, 0, gpioModePushPull, 0);
	}
}

/*
 * all 5 bytes data add each other and get the 16bits sum.
 * low 8bits store into crc0, high 8bits store into crc1.
 * */
uint16_t CalFrameCRC(uint8_t data[], int len)
{
	int i = 0;
	uint16_t crc_sum = 0;

	for (; i < len; i++)
		crc_sum += data[i];

	return crc_sum;
}

void InitFrame(struct MainCtrlFrame *mainCtrlFr, uint8_t src, uint8_t slave, uint8_t type, uint32_t TSTD)
{
	uint16_t data_crc = 0;
	mainCtrlFr->head0 = 0xaa;
	mainCtrlFr->head1 = 0x55;
	mainCtrlFr->len = FRAME_LEN; //after frameType and before CRC
	mainCtrlFr->frameID = slave & 0x7;

	//mainCtrlFr->serial = g_recvSlaveFr.serial + 1;
	mainCtrlFr->serial = 0;

//	mainCtrlFr->frameCtrl_blank[0] = 0xf3;
//	mainCtrlFr->frameCtrl_blank[1] = 0x19;
//	mainCtrlFr->frameCtrl_blank[2] = 0x01;

//	mainCtrlFr->frameCtrl_blank[0] = TOA;
//	mainCtrlFr->frameCtrl_blank[1] = TOA>>8;
	mainCtrlFr->frameCtrl_blank[2] = TSTD;
	mainCtrlFr->adcIndex = TSTD>>8;
	mainCtrlFr->frameCtrl = (slave & 0x7) + src;

	mainCtrlFr->frameType = type & 0xf;
	memset(mainCtrlFr->data, 0x00, FRAME_DATA_LEN);

	data_crc = CalFrameCRC(mainCtrlFr->data, FRAME_DATA_LEN);
	mainCtrlFr->crc0 = data_crc & 0xff;
//	mainCtrlFr->crc1 = (data_crc >> 8) & 0xff;
}

int checkSlaveWkup(struct MainCtrlFrame *mainCtrlFr, struct MainCtrlFrame *recvSlaveFr)
{
	/*
	 * send owner, slave ID and back token, all match, then it indicates
	 * slave is waken up.
	 * */
	if (((recvSlaveFr->frameCtrl & 0xff) == (mainCtrlFr->frameCtrl & 0xff))
		&& ((recvSlaveFr->frameType & 0x0f) == (mainCtrlFr->frameType & 0x0f) + 1))
		return 0;
	else
		return -1;
}

/*
 * send a frame to slave and poll the corresponding back token.
 * @src: frame source, main node, manual node or slave node
 * @slave: talk to which slave node
 * @type£º frame type
 *
 * @ret: -1: talk timeout; 0: talk successfully.
 * */
uint8_t TalktoSlave(dwDevice_t *dev, uint8_t src, uint8_t slave, uint8_t type, uint32_t TSTD)
{
	int8_t ret = -1;
//	uint16_t pan_id = PAN_ID1, dest_addr = SLAVE_ADDR1 + (slave - 1), source_addr = CENTER_ADDR1;

	InitFrame(&g_mainCtrlFr, src, slave, type, TSTD);

	/*
	 * to do:  wireless send logic
	 * */
	g_dataRecvDone = false;
	g_dataRecvSleep = false;

	g_cmd_feedback_timeout = g_Ticks + CMD_FEEDBACK_TIMEOUT;

//	dwTxBufferFrameEncode(&g_dwMacFrameSend, 1, 0, pan_id, dest_addr,
//		source_addr, (uint8_t *)&g_mainCtrlFr, sizeof(g_mainCtrlFr));
//	dwSendData(&g_dwDev, (uint8_t *)&g_dwMacFrameSend, sizeof(g_dwMacFrameSend));
//	dwSendData(&g_dwDev, (uint8_t *)&g_mainCtrlFr, sizeof(g_mainCtrlFr));
	dwSendData(&g_dwDev, (uint8_t *)&g_mainCtrlFr, 11);

	while (g_Ticks < g_cmd_feedback_timeout) {
		if (g_dataRecvDone == true) {
			if (g_dataRecvSleep == true) {
				return 10;
			}
			ret = checkSlaveWkup(&g_mainCtrlFr, &g_recvSlaveFr);
			if (ret < 0)
				ret = -1;
			break;
		}
	}

	if (ret < 0)
		ret = -1;

	return ret;
}

/*
 * try to wake up all slave, woken slaves are marked in 'g_slaveStatus' var.
 * */
void WakeupSlave(dwDevice_t *dev)
{
	//int i = 0;
	int ret = -1;
	g_wakup_timeout = g_Ticks + WAKUP_DURATION;

	/*
	 * wake up duration is 10 minutes
	 * */
	while (g_Ticks < g_wakup_timeout) {
		for (i = 0; i < SLAVE_NUMS; i++) {
			/*
			 * reset command timeout
			 * */
			ret = TalktoSlave(dev, MAIN_NODE_ID, i + 1, ENUM_SAMPLE_SET, 0);
			if (ret == 0){
				g_slaveStatus |= (1 << i);
			}
			else if (ret == 10){
				pollSleepCMD(dev);
				return;
			}
//			frm_cnt[i] = frm_cnt[i] + 1;
			while (g_Ticks < g_cmd_feedback_timeout) ;

		}

		/*
		 *
		 * all slaves are waken up
		 * */
		if ((g_slaveStatus & SLAVE_WKUP_MSK) == SLAVE_WKUP_MSK)
			break;
	}

	/*
	 * if it exists woken slaves, it can begin to fetch data from slaves.
	 * */
	if ((g_slaveStatus & SLAVE_WKUP_MSK) > 0) {
		g_slaveWkup = true;
		g_cur_mode = MAIN_SYNCMODE;
	}
	memset(&cnt, 0x00, sizeof(cnt));
	Delay_ms(100);
}
/*
 * scan all slaves and fetch sample data.
 * */

void RecvFromSlave(dwDevice_t *dev)
{
	//uint16_t crc_sum = 0;
	//int i = 0;
	int ret = -1;
	uint8_t src = 0;
	uint8_t ENUM_TYPE = ENUM_SAMPLE_DATA;

	//static uint16_t max=0;
	//static uint32_t time_us = 0;
	//uint8_t static err_cnt = 0;
	/*
	 * scan each slave and receive sample data
	 * */

	src = MAIN_NODE_ID;

	for (i = 0; i < SLAVE_NUMS; i++) {
		if ((g_slaveStatus & (1 << i)) == (1 << i)) {
			//time_us = g_Ticks * MS_COUNT + TIMER_CounterGet(TIMER0); //get the initial time
			ENUM_TYPE = ENUM_SAMPLE_DATA;
			if (cnt[i] != 0){
				//err_cnt++;
				ENUM_TYPE = ENUM_REPEAT_DATA;
			}

			ret = TalktoSlave(dev, src, i + 1, ENUM_TYPE, time_std[i]);
			if (ret == 0) {
				//dwGetRawReceiveTimestamp(dev, &g_dwTime);
				//TOA_T[i] = g_Ticks * MS_COUNT + TIMER_CounterGet(TIMER0) - timer_cnt[i]; //get the TOA_T time
				time_std[i] = g_recvSlaveFr.frameCtrl_blank[2] + (g_recvSlaveFr.adcIndex<<8);

				if (cnt[i] != 0)
					cnt[i]--;

				//crc_sum = CalFrameCRC(g_recvSlaveFr.data, FRAME_DATA_LEN);
				if (g_recvSlaveFr.head0 == 0xaa && g_recvSlaveFr.head1 == 0x55){
					//if( g_recvSlaveFr.crc0 == (crc_sum & 0xff) && g_recvSlaveFr.len == 0){
					if(g_recvSlaveFr.len == 0){
						while (g_Ticks < g_cmd_feedback_timeout);
						continue;
					}
					uartPutData((uint8_t *)&g_recvSlaveFr, sizeof(struct MainCtrlFrame));
					//TryQuickUART_Tx(g_recvSlaveFr);

				}
			}
			else {
//				err_cnt++;
//				if (err_cnt>100){
//					break;
//				}
				cnt[i]++;
				if (cnt[i] > 50){
					cnt[i] = 0;
					g_slaveStatus &= ~(1 << i);
				}
				/*
				 * if all slave is offline, reset flag 'g_slaveWkup' to begin wakeup logic.
				 * */
				if ((g_slaveStatus & SLAVE_WKUP_MSK) == 0) {
					g_slaveWkup = false;
					g_cur_mode = MAIN_IDLEMODE;
				}
			}
		}
		else{
			ret = TalktoSlave(dev, MAIN_NODE_ID, i + 1, ENUM_SAMPLE_SET, 0);
			if (ret == 0){
				g_slaveStatus |= (1 << i);
				Delay_ms(20);
				g_cur_mode = MAIN_WKUPMODE;
			}
		}
		while (g_Ticks < g_cmd_feedback_timeout);
	}
}

void SyncSlave(dwDevice_t *dev)
{
//	uint32_t time_us;
//	memset(timer_cnt, 0, 4);
//	memset(TOA_T, 0, 4);
//	memset(TD, 0, 4);
	Delay_ms(3);
	i = 0;
	InitFrame(&g_mainCtrlFr, 0, 0, ENUM_SLAVE_SYNC, 0);
//	g_cmd_feedback_timeout = g_Ticks + CMD_FEEDBACK_TIMEOUT;
	dwSendData(&g_dwDev, (uint8_t *)&g_mainCtrlFr, sizeof(g_mainCtrlFr));
//	while (g_Ticks < g_cmd_feedback_timeout) {
//	}

	Delay_ms(3);
	memset(time_std, 0, 4);
	g_cur_mode = MAIN_SAMPLEMODE;

//	g_cmd_sync_timeout = g_Ticks + SYNC_CMD_TIMEOUT;
}

void sleepSlave(dwDevice_t *dev)
{
	uint16_t crc_sum = 0;
	int i = 0, ret = -1;
	g_wakup_timeout = g_Ticks + SLEEP_DURATION;
	g_slaveStatus = 0x0F;
	g_slaveWkup = true;
	/*
	 * wake up duration is 10 minutes
	 * */
	while (g_Ticks < g_wakup_timeout) {
		/*
		 * scan each slave plus main node, and send sleep command to each one.
		 * */
		for (i = 0; i < SLAVE_NUMS; i++) {
			if ((g_slaveStatus & (1 << i)) == (1 << i)) {
				ret = TalktoSlave(dev, MAIN_NODE_ID, i + 1, ENUM_SLAVE_SLEEP, 0);
				if (ret == 0) {
					cnt[i] = 0;
					crc_sum = CalFrameCRC(g_recvSlaveFr.data, FRAME_DATA_LEN);

					g_cmd_wake_wait_time = g_Ticks + WAKE_CMD_WAIT_TIME;

					if (g_recvSlaveFr.head0 == 0xaa && g_recvSlaveFr.head1 == 0x55){
						if( g_recvSlaveFr.crc0 == (crc_sum & 0xff) && g_recvSlaveFr.len == 0){

						//g_recvSlaveFr.serial = g_recvSlaveFr.serial - 1;
						continue;
						}
						g_slaveStatus &= ~(1 << i);
						uartPutData((uint8_t *)&g_recvSlaveFr, sizeof(struct MainCtrlFrame));
					}
				}
				else {
					cnt[i] += 1;
					if (cnt[i] > 50){
						cnt[i] = 0;
						g_slaveStatus &= ~(1 << i);
					}
				}
			}
//			frm_cnt[i] = frm_cnt[i] + 1;
		}
	}

	if ((g_slaveStatus & SLAVE_WKUP_MSK) == 0) {
		g_slaveWkup = false;
		Delay_ms(100);
		return;
	}
}

int checkSleepCMD(struct MainCtrlFrame *recvSlaveFr)
{
	/*
	 * send owner, slave ID and back token, all match, then it indicates
	 * slave is waken up.
	 * */
	if ((recvSlaveFr->frameType == ENUM_SLAVE_SLEEP) && ((recvSlaveFr->frameCtrl) == 0))
		return 0;
	else
		return -1;
}

bool pollSleepCMD(dwDevice_t *dev)
{
//	int timeout = 500;
	int8_t ret = false;

//	g_dataRecvDone = false;
//	dwNewReceive(dev);
//	dwStartReceive(dev);
//
//	/*
//	 * if it doesn't receive SLEEP command during 'timeout' time window,
//	 * it will keep original work mode. otherwise change work mode  to
//	 * MAIN_SLEEPMODE.
//	 * */
//	while (timeout--) {
//		if (g_dataRecvDone == true) {
//			if (!checkSleepCMD(&g_recvSlaveFr)) {
		for (int i=0;i<10;i++){
			InitFrame(&g_mainCtrlFr, 0, 0, ENUM_SLAVE_SLEEP_TOKEN, 0);
			dwSendData_noTurnon(&g_dwDev, (uint8_t *)&g_mainCtrlFr, sizeof(g_mainCtrlFr));
			Delay_ms(100);
		}
		Delay_ms(5000);
		while(g_slaveWkup){
			sleepSlave(&g_dwDev);
		}

		g_cur_mode = MAIN_SLEEPMODE;

		dwIdle(&g_dwDev);
		Delay_ms(2);
		ret = true;
//				break;
//			}
//		}

		delayms(2);
//	}

	return ret;
}
