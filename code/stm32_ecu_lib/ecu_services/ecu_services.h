#ifndef _ecuServices
#define _ecuServices



/*
 * Defines the services provided to the ECU software by the stm32 peripherals and drivers.
 * Acts as an interface between the stm32 hardware/firmware to minimise the hardware/firmware
 * dependency in the ECU software and also to minimise the amount of user code in main.c



This software/firmware source code or executable program is copyright of
Just Technology (North West) Ltd (http://www.just-technology.co.uk) 2020

This software/firmware source code or executable program is provided as free software:
you can redistribute it and/or modify it under the terms of the GNU General Public License
as published by the Free Software Foundation, either version 3 of the License, or (at your
option) any later version. The license is available at https://www.gnu.org/licenses/gpl-3.0.html

The software/firmware source code or executable program is distributed in the hope that
it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
or FITNESS FOR A PARTICULAR PURPOSE.
*/



#include "main.h"
#include "async_serial.h"


/*
 *  Peripherals used defined below. Plus ADC 1 & 2, that are initialised & controlled using HAL.
 *  And USART1 & 2 used for aux & host comms.
 *
 */
#define CRANKSHAFT_TRIGGER_TIMER	TIM2
#define IGNITION_TIMER 				TIM5
#define INJECTION_TIMER_A			TIM1
#define INJECTION_TIMER_B			TIM8
#define PWM_TIMER 					TIM3


/*
 * The following ISRs must be inserted into stm32xxxx_it.c at the appropriate callback:
 *
 */
extern void ecuISRTimerTick(void);
extern void ecuISRIgnitionTimer(void);
extern void ecuISRInjectionATimer(void);
extern void ecuISRInjectionBTimer(void);
extern void ecuISRHostUART(void);
extern void ecuISRAuxUART(void);
extern void ecuISRcrankshaftTrigger(void);


// set host & aux serial data rate
// data rates are currently set in cubeMX
#define HOST_DATA_RATE 115200
#define AUX_DATA_RATE 9600


// PWM outputs
extern void setDutyCyclePWM1(float dc);
extern void setDutyCyclePWM2(float dc);

// injector & ignition pin mapping
typedef struct {
	GPIO_TypeDef *port;
	uint16_t pin;
} GPIOPin;
extern void setIOPinMapping(void);
extern GPIOPin injectorIO[4];
extern GPIOPin coilIO[2];

// three 16 bit timers are allocated to injection & ignition timing,
// configured at 1uS resolution (1Mhz timer clock), in single pulse, count-up mode
extern void startInjectionTimerA(uint16_t delay1, uint16_t delay2, void (*callback1)(), void (*callback2)());
extern void startInjectionTimerB(uint16_t delay1, uint16_t delay2, void (*callback1)(), void (*callback2)());
extern void startIgnitionTimer(uint16_t period, void (*callback)());

extern void hostPrint(char *txBuffer, int strLen);
extern void auxPrint(char *txBuffer, int strLen);

// The host serial receive buffer need to be sized to accept the largest single message - an NVM message.
// 64 values each with format -9999.9 (7 chars x 64 = 448 chars) + preamble (sn10,0#) (8 chars) + 64 separators + CRLF + NULL
// Total size of rx buffer should be 448 + 8 + 64 + 3 ~ 500
#define HOST_RX_BUFFER_SIZE 500
extern char hostRxBuffer[HOST_RX_BUFFER_SIZE];
extern asseControlData hostIO;

// The auxiliary serial input buffer is sized to accept simple commands
#define AUX_RX_BUFFER_SIZE 50
extern char auxRxBuffer[AUX_RX_BUFFER_SIZE];
extern asseControlData auxIO;

// ADC
// define the raw data index for each analog signal
#define ADC_MAP 		3
#define ADC_LAMBDA		1
#define ADC_TPSV		2
#define ADC_VOLTAGE		0
#define ADC_AIR_TEMP	4
#define ADC_ENG_TEMP	5

// analog raw data store
extern uint16_t adcRawData[6];

// analogue services

extern int startADCConversion(void);
extern int waitForADCCompletion(void);
extern void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef * hadc);

// Initialises / starts all of the services provided by ecu_services.c
extern void ecuServicesStart(void);



#endif



/*+++REVISION_HISTORY+++
1)	04 Nov 2020	Replaces host & aux serial comms mechanics with the async_serial package.
+++REVISION_HISTORY_ENDS+++*/
