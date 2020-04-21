#ifndef INCLUDE_MAIN_H_
#define INCLUDE_MAIN_H_

#include <stdint.h>
//#define POWER_WAKEUP //供电MCU休眠模式

volatile uint8_t MAIN_NODE_ID;
volatile uint8_t SET_NUM;


/** DMA channel used for scan sequence sampling adc channel 2, 3 and 4. */
#define DMA_CHANNEL    	2
#define NUM_SAMPLES    	3
//#define BATT_MIN_VOL	2080//电池欠压最小值3.8v
#define BATT_MIN_VOL	1980//电池欠压最小值3.62v

#define PORT_CapSelect           					 		 gpioPortA
#define PIN_Sensor_1AxisAccelerometer_CapSelect1            0//
#define PIN_Sensor_1AxisAccelerometer_CapSelect2            1//
#define PIN_Sensor_1AxisAccelerometer_GainSelect0           2//


#define PORT_CurrentSelect           				 		 gpioPortB
#define PIN_Sensor_Temperature_BiasVoltSelect0         		7//////
#define PIN_Sensor_Temperature_BiasVoltSelect1         		8//////
#define PIN_Sensor_Temperature_GainSelect0             		11//
#define PIN_Sensor_Temperature_GainSelect1             		13//

#define PORT_GainSelect										 gpioPortC
#define PIN_Sensor_1AxisAccelerometer_GainSelect1            0//
#define PIN_Sensor_1AxisAccelerometer_GainSelect2            1//
#define PIN_Pressure_GainSelect0                             13//
#define PIN_Pressure_GainSelect1                             14//
#define PIN_Pressure_GainSelect2                             15//

#define PORT_Temperature_Pressure							 gpioPortD
#define PIN_Sensor_Temperature_GainSelect2					 4//
#define PIN_Pressure_CapSelect1 							 6
#define PIN_Pressure_CapSelect2								 7

#define PORT_UART           						 		 gpioPortE
#define PIN_RX            									 11
#define PIN_TX            									 10
#endif /* INCLUDE_MAIN_H_ */
