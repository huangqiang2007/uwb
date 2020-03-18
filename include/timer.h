#ifndef __INLCUDE_TIMER_H_
#define __INLCUDE_TIMER_H_

#include <stdint.h>
#include <stdlib.h>


#define SLEEP_DURATION 10000 /* wake up time 610 second*/
#define SLEEP_DURATION 10000 /* wake up time 610 second*/
/*
 * UnWake Slave CMD timeout judge
 *
 * represent 2 * 1ms = 2ms timeout
 * */
#define UNWAKE_CMD_TIMEOUT 20 /* unwake slave timeout 2ms*/
volatile uint32_t g_cmd_unwake_timeout; /* the count of 1ms unit */

/*
 * Sync Slave ADC time
 *
 * represent 20000 * 1ms = 20s timeout
 * */
#define SYNC_CMD_TIMEOUT 2000 /* unwake slave timeout 2ms*/
volatile uint32_t g_cmd_sync_timeout; /* the count of 1ms unit */



/*
 * Wake Slave CMD wait time judge
 *
 * represent 1 * 1ms = 1ms timeout
 * */
#define WAKE_CMD_WAIT_TIME 4 /* wake slave cmd wait time 2ms*/
volatile uint32_t g_cmd_wake_wait_time; /* the count of 1ms unit */

/*
 * CMD feedback timeout judge
 *
 * represent 4 * 1ms = 4ms timeout
 * */
//#define CMD_FEEDBACK_TIMEOUT 4 /* receive wait response timeout 10ms*/
volatile uint32_t g_cmd_feedback_timeout; /* the count of 1ms unit */
volatile uint8_t CMD_FEEDBACK_TIMEOUT; /* the count of 1ms unit */

/*
 * wakeup duration
 *
 * 600000 * 1ms = 10 minutes
 * */
#define WAKUP_DURATION 61000//000 /* wake up time 610 second*/
volatile uint32_t g_wakup_timeout; /* the count of 1ms unit */

/*
 * slave does not receive new CMD during the below duration, enter into sleep mode.
 *
 * 30000 * 1ms = 5 minutes
 * */
#define IDLE_JUDGE 300000
volatile uint32_t g_idle_judge;

void setupTimer0(void);
void setupTimer1(void);
extern void Delay_ms(uint32_t ms);
extern void Delay_us(uint32_t us);
extern void timer_init(void);
extern void delayms(uint32_t ms);

#endif /* INLCUDE_TIMER_H_ */
