#include "main.h"
#include "em_device.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_system.h"
#include "em_timer.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "timer.h"

// Freq = 25M
#define TOP 25000
#define MS_COUNT  3125  //25000000 / 8 / 1000
#define MAX_MS    20    //65535 / MS_COUNT
#define MAX_TICK (0xFFFFFFF0 - WAKUP_DURATION)

volatile bool Timer1_overflow;
volatile uint32_t g_Ticks = 0;
volatile uint32_t tx_start_times = 0;
volatile uint32_t tx_finish_times = 0;
/**************************************************************************//**
 * @brief TIMER0_IRQHandler
 * Interrupt Service Routine TIMER0 Interrupt Line
 *****************************************************************************/
void TIMER0_IRQHandler(void)
{
	/* Clear flag for TIMER0 overflow interrupt */
	TIMER_IntClear(TIMER0, TIMER_IF_OF);
	g_Ticks++;
	if (g_Ticks > 0xFFFFFFF0 - WAKUP_DURATION)
		g_Ticks = 0;
}

/**************************************************************************//**
 * @brief TIMER0_IRQHandler
 * Interrupt Service Routine TIMER0 Interrupt Line
 *****************************************************************************/
void TIMER1_IRQHandler(void)
{
	/* Clear flag for TIMER0 overflow interrupt */
	TIMER_IntClear(TIMER1, TIMER_IF_OF);
	Timer1_overflow = true;
}


void setupTimer0(void)
{
	/* Enable clock for TIMER0 module */
	CMU_ClockEnable(cmuClock_TIMER0, true);

	/* Select TIMER0 parameters */
	TIMER_Init_TypeDef timerInit =
	{
		.enable     = true,
		.debugRun   = true,
		.prescale   = timerPrescale8,
		.clkSel     = timerClkSelHFPerClk,
		.fallAction = timerInputActionNone,
		.riseAction = timerInputActionNone,
		.mode       = timerModeUp,
		.dmaClrAct  = false,
		.quadModeX4 = false,
		.oneShot    = false,
		.sync       = false,
	};

	/* Enable overflow interrupt */
	TIMER_IntEnable(TIMER0, TIMER_IF_OF);

	/* Enable TIMER0 interrupt vector in NVIC */
	NVIC_EnableIRQ(TIMER0_IRQn);

	/* Set TIMER Top value */
	//TIMER_TopSet(TIMER0, TOP);
	TIMER_TopSet(TIMER0, MS_COUNT ); //1 ms

	/* Configure TIMER */
	TIMER_Init(TIMER0, &timerInit);
}

void setupTimer1(void)
{
	/* Enable clock for TIMER0 module */
	CMU_ClockEnable(cmuClock_TIMER1, true);

	/* Select TIMER0 parameters */
	TIMER_Init_TypeDef timerInit =
	{
		.enable     = false,
		.debugRun   = true,
		.prescale   = timerPrescale8,
		.clkSel     = timerClkSelHFPerClk,
		.fallAction = timerInputActionNone,
		.riseAction = timerInputActionNone,
		.mode       = timerModeUp,
		.dmaClrAct  = false,
		.quadModeX4 = false,
		.oneShot    = true,
		.sync       = false,
	};

	/* Enable overflow interrupt */
	TIMER_IntEnable(TIMER1, TIMER_IF_OF);

	/* Enable TIMER0 interrupt vector in NVIC */
	NVIC_EnableIRQ(TIMER1_IRQn);

	/* Set TIMER Top value */
	//  TIMER_TopSet(TIMER1, TOP);

	/* Configure TIMER */
	TIMER_Init(TIMER1, &timerInit);
}

void timer_init(void)
{
	g_Ticks = 0;

	setupTimer0();
	setupTimer1();
}

/*
 * precondition: timer0's interrupt interval is 1ms
 * */
void delayms(uint32_t ms)
{
	uint32_t ticks = 0;

	ticks = ms + g_Ticks;

	if (ticks > MAX_TICK) {
		ticks = ticks - MAX_TICK;
		while (g_Ticks < MAX_TICK);
		while (g_Ticks == MAX_TICK || g_Ticks < ticks);
	} else {
		while (g_Ticks < ticks);
	}
}

void Delay_us(uint32_t us)
{
  uint32_t countMax;
  countMax = us * 25;
  /* Set TIMER value */
  TIMER_CounterSet(TIMER0, 95);
  while (TIMER_CounterGet(TIMER0) < countMax);
}

/* max 20ms */
void __Delay_ms(uint32_t ms)
{
	Timer1_overflow = false;

	/* Set TIMER value */
	TIMER_CounterSet(TIMER1, 0);
	/* Set TIMER Top value */
	TIMER_TopSet(TIMER1, MS_COUNT * ms);
	/* Enable TIMER */
	TIMER_Enable(TIMER1, true);

	while (Timer1_overflow == false);
	/* Disable TIMER */
	TIMER_Enable(TIMER1, false);
}

void Delay_ms(uint32_t ms)
{
	if (ms < MAX_MS) {
		__Delay_ms(ms);
		return;
	}

	int n = ms / MAX_MS;
	while(n-- > 0) {
		__Delay_ms(MAX_MS);
	}

	if (ms % MAX_MS > 0)
		__Delay_ms(ms % MAX_MS);
}
