/*
 * Provides the services, references and low-level functions required by the ECU software.
 *
 *
 * *** In stm32xxxx_it.h remember to make sure the HAL ISRs are not permitted for timers 2, 4, 5 & 9 and UARTS 1 & 6
 *
 *
 *
This software/firmware source code or executable program is copyright of
Just Technology (North West) Ltd (http://www.just-technology.co.uk) 2021

This software/firmware source code or executable program is provided as free software:
you can redistribute it and/or modify it under the terms of the GNU General Public License
as published by the Free Software Foundation, either version 3 of the License, or (at your
option) any later version. The license is available at https://www.gnu.org/licenses/gpl-3.0.html

The software/firmware source code or executable program is distributed in the hope that
it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
or FITNESS FOR A PARTICULAR PURPOSE.
*/

#include "ecu_services.h"
#include "utility_functions.h"
#include "scheduler.h"
#include "trigger_wheel_handler.h"
#include "global.h"


/*
 * Interrupt Priorities, from highest to lowest:
 *
 * Peripheral	Function			Pri	Sub
 * ----------	--------			---	---
 * TIM2			Crankshaft Pulse	 0	 0
 * TIM9			Ignition			 1   0
 * TIM4			Injection A			 1   1
 * TIM10			Injection B			 1   2
 * TIM11			Injection C			 1   3
 * TIM13			Injection D			 1   4
 * DMA1 CH1		Support for ADC		 2 	 0
 * USART6		Host Comms			 2	 1
 * USART1		Aux Comms			 2	 2
 * Systick		Cyclic				 3	 0
 *
 */

void setInterruptPriorities(){

    HAL_NVIC_SetPriority(TIM2_IRQn, 				0, 0);
	HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 		1, 0);
	HAL_NVIC_SetPriority(TIM4_IRQn, 		1, 1);
//	HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 1, 2);
//	HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 1, 3);
//	HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 1, 4)
	HAL_NVIC_SetPriority(TIM5_IRQn, 				1, 2);
//		HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 		1, 1);
//	HAL_NVIC_SetPriority(TIM7_IRQn, 	1, 3);
	//	HAL_NVIC_SetPriority(TIM8_CC_IRQn, 	1, 2);
	
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 		2, 0);
    HAL_NVIC_SetPriority(USART2_IRQn, 				2, 1);
    HAL_NVIC_SetPriority(USART1_IRQn, 				2, 2);
    HAL_NVIC_SetPriority(SysTick_IRQn, 				3, 0);
  
	
	
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
	HAL_NVIC_EnableIRQ(TIM5_IRQn);
	HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
	
	HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
	//HAL_NVIC_EnableIRQ(TIM8_CC_IRQn);
	HAL_NVIC_EnableIRQ(TIM4_IRQn);
	HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
	HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
	HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
	
	
//	HAL_NVIC_EnableIRQ(TIM5_IRQn);
//	HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
//	HAL_NVIC_EnableIRQ(TIM7_IRQn);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    HAL_NVIC_EnableIRQ(SysTick_IRQn);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}


/*
 * 1mS timer tick from systick ISR in stm32xxxx_it.h
 * Provide the timebase for the system scheduler.
 *
 */
void ecuISRTimerTick(){
	// run the scheduler
	scTimerTick();
}


/*
 * The following array holds the mapping of injector & ignition IO to port & pin numbers.
 * A specific injector or coil is referenced by the array index - makes it easy to manage
 * sequencing in trigger_wheel_handler.
 *
 * **** Note there are only two injector outputs available in this configuration ****
 *
 * The pin/port assignments are defined in cubeMX and made available in main.h
 *
 */

GPIOPin injectorIO[4];
GPIOPin coilIO[4];

void setIOPinMapping(){
	injectorIO[0].port = Injector_A_GPIO_Port;
	injectorIO[0].pin = Injector_A_Pin;
	injectorIO[1].port = Injector_B_GPIO_Port;
	injectorIO[1].pin = Injector_B_Pin;
	injectorIO[2].port = Injector_C_GPIO_Port;
	injectorIO[2].pin = Injector_C_Pin;
	injectorIO[3].port = Injector_D_GPIO_Port;
	injectorIO[3].pin = Injector_D_Pin;
	coilIO[0].port = Coil_A_GPIO_Port;
	coilIO[0].pin = Coil_A_Pin;
	coilIO[1].port = Coil_B_GPIO_Port;
	coilIO[1].pin = Coil_B_Pin;
	coilIO[2].port = Coil_C_GPIO_Port;
	coilIO[2].pin = Coil_C_Pin;
	coilIO[3].port = Coil_D_GPIO_Port;
	coilIO[3].pin = Coil_D_Pin;
}


/*
 * Ignition & injection timer services.
 *
 * Note that cubeMX / HAL need to configure these timers as follows:
 *
 * 		Prescaler set to provide 1MHz timer clock input
 * 		One Pulse Mode
 * 		TRGO: Update Event (if applicable)
 *
 * startIgnitionTimer() & startInjectionTimerA/B() provide the functions to time the ignition & injection delays and pulse widths.
 * A separate timer to injector channels A & B are provided. This allows injectors to overlap, i.e. where the span of the injector
 * open duration is greater than 180 degrees.
 *
 *
 */

// references to ignition/injection callbacks
void (*ignitionTimerCallback)() = NULL;
void (*injectionTimerACallback1)() = NULL;
void (*injectionTimerACallback2)() = NULL;
void (*injectionTimerBCallback1)() = NULL;
void (*injectionTimerBCallback2)() = NULL;
void (*injectionTimerCCallback1)() = NULL;
void (*injectionTimerCCallback2)() = NULL;
void (*injectionTimerDCallback1)() = NULL;
void (*injectionTimerDCallback2)() = NULL;

// ignition timer ISR
void ecuISRIgnitionTimer(){
	// clear the interrupt flag
	IGNITION_TIMER->SR = 0;
	// call the callback if not null
	if (ignitionTimerCallback != NULL) {
		ignitionTimerCallback();
	}
}

// injection timer A ISR
void ecuISRInjectionATimer(){
	uint16_t sr = INJECTION_TIMER_A->SR;
	if ( (sr & 1) != 0 ) {
		// update interrupt
		if (injectionTimerACallback2 != NULL) {
			injectionTimerACallback2();
		}
	}
	if ( (sr & 2) != 0 ) {
		// capture & compare interrupt
		if (injectionTimerACallback1 != NULL) {
			injectionTimerACallback1();
		}
	}
	INJECTION_TIMER_A->SR = 0;
}

// injection timer B ISR
void ecuISRInjectionBTimer(){
	uint16_t sr = INJECTION_TIMER_B->SR;
	if ( (sr & 1) != 0 ) {
		// update interrupt
		if (injectionTimerBCallback2 != NULL) {
			injectionTimerBCallback2();
		}
	}
	if ( (sr & 2) != 0 ) {
		// capture & compare interrupt
		if (injectionTimerBCallback1 != NULL) {
			injectionTimerBCallback1();
		}
	}
	INJECTION_TIMER_B->SR = 0;
}

// injection timer C ISR
void ecuISRInjectionCTimer(){
	uint16_t sr = INJECTION_TIMER_C->SR;
	if ( (sr & 1) != 0 ) {
		// update interrupt
		if (injectionTimerCCallback2 != NULL) {
			injectionTimerCCallback2();
		}
	}
	if ( (sr & 2) != 0 ) {
		// capture & compare interrupt
		if (injectionTimerCCallback1 != NULL) {
			injectionTimerCCallback1();
		}
	}
	INJECTION_TIMER_C->SR = 0;
}

// injection timer D ISR
void ecuISRInjectionDTimer(){
	uint16_t sr = INJECTION_TIMER_D->SR;
	if ( (sr & 1) != 0 ) {
		// update interrupt
		if (injectionTimerDCallback2 != NULL) {
			injectionTimerDCallback2();
		}
	}
	if ( (sr & 2) != 0 ) {
		// capture & compare interrupt
		if (injectionTimerDCallback1 != NULL) {
			injectionTimerDCallback1();
		}
	}
	INJECTION_TIMER_D->SR = 0;
}

void initialiseIgnInjTimers(){
	IGNITION_TIMER->SR = 0;				// clear any pending interrupts
	IGNITION_TIMER->DIER = 1;			// enable interrupt on update event
	INJECTION_TIMER_A->SR = 0;
	INJECTION_TIMER_A->DIER = 3;		// enable interrupt on update and capture & compare
	INJECTION_TIMER_B->SR = 0;
	INJECTION_TIMER_B->DIER = 3;
	INJECTION_TIMER_C->SR = 0;
	INJECTION_TIMER_C->DIER = 3;		// enable interrupt on update and capture & compare
	INJECTION_TIMER_D->SR = 0;
	INJECTION_TIMER_D->DIER = 3;
}

/*
 * The injection timers have two callbacks:
 * callback #1 is called after delay1
 * callback #2 is called after delay2
 *
 * This provides he ability to specify the injector ON timing and pulse width
 *
 */
void startInjectionTimerA(uint16_t delay1, uint16_t delay2, void (*callback1)(), void (*callback2)()){
	INJECTION_TIMER_A->CCR1 = delay1;
	INJECTION_TIMER_A->ARR = delay1 + delay2;
	injectionTimerACallback1 = callback1;
	injectionTimerACallback2 = callback2;
	INJECTION_TIMER_A->CR1 |= 1; // enable the timer counter
}

void startInjectionTimerB(uint16_t delay1, uint16_t delay2, void (*callback1)(), void (*callback2)()){
	INJECTION_TIMER_B->CCR1 = delay1;
	INJECTION_TIMER_B->ARR = delay1 + delay2;
	injectionTimerBCallback1 = callback1;
	injectionTimerBCallback2 = callback2;
	INJECTION_TIMER_B->CR1 |= 1; // enable the timer counter
}
void startInjectionTimerC(uint16_t delay1, uint16_t delay2, void (*callback1)(), void (*callback2)()){
	INJECTION_TIMER_C->CCR1 = delay1;
	INJECTION_TIMER_C->ARR = delay1 + delay2;
	injectionTimerCCallback1 = callback1;
	injectionTimerCCallback2 = callback2;
	INJECTION_TIMER_C->CR1 |= 1; // enable the timer counter
}

void startInjectionTimerD(uint16_t delay1, uint16_t delay2, void (*callback1)(), void (*callback2)()){
	INJECTION_TIMER_D->CCR1 = delay1;
	INJECTION_TIMER_D->ARR = delay1 + delay2;
	injectionTimerDCallback1 = callback1;
	injectionTimerDCallback2 = callback2;
	INJECTION_TIMER_D->CR1 |= 1; // enable the timer counter
}

void startIgnitionTimer(uint16_t period, void (*callback)()){
	IGNITION_TIMER->ARR = period;
	ignitionTimerCallback = callback;
	IGNITION_TIMER->CR1 |= 1; // enable the timer counter
}



/*
 * Serial text IO services.
 *
 * UART6 provides serial comms with a host computer via USB
 * UART1 provides an auxiliary ECU data feed to external peripherals
 *
 */

char hostRxBuffer[HOST_RX_BUFFER_SIZE] = {0};
asseControlData hostIO;

char auxRxBuffer[AUX_RX_BUFFER_SIZE] = {0};
asseControlData auxIO;

void startUSARTServices(){
	 asseInitialise(&hostIO, USART1, '#', hostRxBuffer, HOST_RX_BUFFER_SIZE);
	 asseInitialise(&auxIO, USART2, '#', auxRxBuffer, AUX_RX_BUFFER_SIZE);
}

inline void hostPrint(char msg[], int length){
	asseSend(&hostIO, msg, length);
}

inline void ecuISRHostUART(){
	asseISR(&hostIO);
}

inline void auxPrint(char msg[], int length){
	asseSend(&auxIO, msg, length);
}

inline void ecuISRAuxUART(){
	asseISR(&auxIO);
}




/*
 * PWM services on Timer 3 provides PWM output on:
 *
 * Channel 1 - pin PB4
 * Channel 2 - pin PB5
 *
 * The PWM frequency is given by PWMf = Ftc / (Pt + 1) / (ARR + 1), where:
 * Ftc is the timer clock frequency, Pt is the timer pre-scaler and ARR is the counter preload/reload register (2499).
 *
 * Note that cubeMX / HAL must have pre-configured the timer to provide PWM outputs and the prescaler selected
 * to provide a 1Mhz timer input clock.
 *
 * PWMf is therefore set to 1MHz / (2499 + 1) = 1E6 / 2500 = 400Hz
 *
 * PWM duty cycle is set by setting the capture/compare register CCR3, CCR4 as follows:
 *
 * 0% duty cycle 	=> CCR3/4 = 0
 * 50% duty cycle 	=> CCR3/4 = 1249
 * 100% duty cycle 	=> CCR3/4 = 2499
 *
 * Hence to convert Duty Cycle (DC) 0-100% to timer units (T): T = DC * 2499 / 100
 *
 */

// define the PWM duty cycle to timer unit conversion
#define DC_TO_TU_CONVERSION 24.99

// start the PWM timers
void startPWMService(){
	PWM_TIMER->ARR = 2499;
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

// Set the duty cycle on the PWM Pins
void setDutyCyclePWM1(float dc){
	PWM_TIMER->CCR1 = (uint32_t)(limitF(dc, 0, 100) * DC_TO_TU_CONVERSION);
}

void setDutyCyclePWM2(float dc){
	PWM_TIMER->CCR2 = (uint32_t)(limitF(dc, 0, 100) * DC_TO_TU_CONVERSION);
}



/*
 * TIMER #2 provides crankshaft trigger wheel pulse period measurement, using input capture on channel 1, pin PA15.
 * ecuISRcrankshaftTrigger() is called from TIM2_IRQHandler() in stm32xxxx_it.h.
 *
 * This interrupt service function calculates the pulse period then calls the ECU crankshaft pulse handler.
 * As of version 3200.064, a filter is applied to the input capture trigger in the configuration of the timer.
 *
 */

void ecuISRcrankshaftTrigger(){

	static uint32_t crankshaftPulseTime_1 = 0;
	uint32_t period;


	// if TIM2 Channel 1 has invoked the ISR, read the captured data
	// bit 1 of SR = channel 1 interrupt flag
	if ( (CRANKSHAFT_TRIGGER_TIMER->SR & 2) != 0 ) {

		// get the captured time
		// ** note the variables used in the time difference calc must have the same width as the counter - i.e. 16 bits
		uint32_t timeNow = CRANKSHAFT_TRIGGER_TIMER->CCR1;

		// calculate the pulse period
		if(timeNow > crankshaftPulseTime_1)  period = timeNow - crankshaftPulseTime_1;
		 else period = timeNow + (0xffffffff - crankshaftPulseTime_1);
		crankshaftPulseTime_1 = timeNow;

		// call the crankshaft pulse handler function
		crankshaftPulseHandler(period);
	}
}


/*
 *
 *
 * Analog sensor inputs are provided by ADC2 using DMA to obtain each channel
 *
 * Uses CubeMX to configure the ADC & DMA peripherals:
 *
 * 10 bit resolution, 6 channels, each at XXX.X cycles sample time. ADC Clock Prescaler must be set to Asynchronous Divided by, at least, 6.
 * In this configuration, the time taken from before a call to startADCConversion() and after waitForADCCompletion() is around XXX uS.
 * The sequence of conversions is started with a call to HAL_ADC_Start_DMA(). On completion, the value on each pin will be stored
 * in adcRawData[] as shown below:
 *
 *								adcRawData[]
 * FUNCTION	PIN	CHANNEL	RANK	INDEX #
 * --------	---	-------	----	------------
 * MAP		PA0	  IN1	 4			3
 * Lambda	PA1	  IN2	 2			1
 * TPS V	PA6	  IN3	 3			2
 * Voltage	PA7	  IN4	 1			0
 * Air Temp	PA5   IN13	 5			4
 * Eng Temp	PA4	  IN17	 6			5
 *
 * The adcRawData[] index for each function are defined in ecu_service.h
 *
 *
 * *** Note ***
 * Due to a possible bug in the HAL drivers, the following CubeMX settings are required for an F401CC device (even
 * though the ADC is still triggered by software). If DMA continuous isn't selected only one conversion is performed:
 *
 * ADC Settings:
 *
 * 		Continuous Mode: 			Disabled
 * 		DMA Continuous Requests:	Enabled
 *
 * DMA #2 Stream #0
 *
 * 		Mode: 			Continuous
 * 		Data Width: 	Half Word
 *
 *
 */

// raw ADC data store
uint16_t adcRawData[7];

// set by a ADC conversion complete callback when DMA transfer finished
static volatile int adcDataReadyFlag = 0;

// start the conversion
int startADCConversion(){
	adcDataReadyFlag = 0;
	HAL_StatusTypeDef stat = HAL_ADC_Start_DMA(SENSOR_ADC, (uint32_t *)adcRawData, 7);
	return stat;
}

// DMA transfer complete callback
void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef * hadc){
	if (hadc == SENSOR_ADC){
		adcDataReadyFlag = 1;
	}
}

int waitForADCCompletion(){
	uint32_t timeoutCount = 0;
	while (adcDataReadyFlag == 0) {
		// test for time out
		if (++timeoutCount > 2000) {
			// timed out, set the ADC timeout flag in ecuStatus and return timeout code
			SET_ADC_TIMEOUT;
			return -1;
		}
	}
	return 1;
}

/**** XXX not yet sure if calibration applicable - needs research! */
void runADCCalibration(){
//	HAL_ADC_Stop(SENSOR_ADC);
//	HAL_ADCEx_Calibration_Start(SENSOR_ADC, ADC_SINGLE_ENDED);
}



// do all initialisation requirements for all the services provided in here
void ecuServicesStart(){
  // start can1
	HAL_CAN_Start(&hcan1);
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
		{
			Error_Handler();
		}
		
	// run the ADC calibration process
	runADCCalibration();

	// initialise the io mapping for injectors & ignition outputs
	setIOPinMapping();

	// Start the PWM service using Timer #3
	startPWMService();
		
		// start pwm timer for fan control
	HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);

	// Start input capture on timer 2, channel 1 for use by the crankshaft trigger pulse handler.
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);

	// start the USARTs for host & aux comms
	startUSARTServices();

	// Initialise timers used for injection & ignition timing
	initialiseIgnInjTimers();

	// set the interrupt priorities for peripherals used
	setInterruptPriorities();
	

}




/*+++REVISION_HISTORY+++
1) 04 Nov 2020 Replaces host & aux serial comms mechanics with the async_serial package.
2) 12 May 2021 Modified for F401CC MCU
+++REVISION_HISTORY_ENDS+++*/
