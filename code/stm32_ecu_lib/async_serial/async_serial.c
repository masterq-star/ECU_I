/*
 * Asynchronous serial text IO services.
 *
 * Provides asychronous interrupt-driven serial IO. Can be used simultaneously with multiple USARTS.
 *
 * Initial usart configuration should be done with cubeMX/HAL to set up stop bits, baud rate etc. Interrupts
 * must be enabled but the HAL code generation should be inhibited.
 *
 * A data structure type asseControlData must be provided by the user. This is used to hold the RX & TX
 * buffer pointers and control variables for the specified USART / comms channel.
 *
 * In file stm32g4xx_it.c, inside each of the USART ISRs, a call to asseISR() must be inserted, specifying
 * the control data structure for the USART / comms channel.
 *
 * Once the cubeMX & HAL initialisation is complete, the user calls asseInitialise() to initialises the serial channel.
 *
 * After the call to asseInitialise() is complete, data will be received continuously and stored in the specified rx buffer. The calling function
 * should monitor rxMsgLength in the control data structure. When rxMsgLength becomes non-zero, a complete message has been received (a complete message is when
 * the terminating character is encountered) and rxMsgLength is the length of the message.
 *
 * *** Important *** The calling function must set rxMsgLength to zero once the received message has been consumed.
 *
 * A call to asseSend() will send the user specified data message to the designated USART. Note that this function blocks (until timeout expires) until the
 * previous transmission is complete by monitoring txInProgress in the control data structure.
 *
 *
 */

#include "async_serial.h"

uint32_t asseTimeout = 500;		// the maximum wait time in mS


/*
 * Sends the specified message to the USART. Note that characters will be transmitted until the
 * number of characters specified in the buffer length parameter or until a null is encountered.
 * Waits for the previous transmission to end. If the transmission does not end within the timeout
 * period, the function returns and no action is take.
 */
void asseSend(asseControlData *c, char *txBuffer, int bufferLen){
	// block here if there's a transmission in progress
//	uint32_t start = HAL_GetTick();
//	while (c->txInProgress != 0){
//		if ((HAL_GetTick() - start) > asseTimeout){
//			return;
//		}
//	}
//	c->txInProgress = 1;
//	c->txBufferPtr = txBuffer;
//	c->txBufferLength = bufferLen;
//	c->txBufferCount = 1;
	// send the first character in the buffer to start transmission
	while(HAL_UART_Transmit(c->husart, (unsigned char *)txBuffer, bufferLen , 20)== HAL_BUSY);
	c->usart->CR1 |= USART_CR1_RXNEIE;
//	c->txInProgress = 0;
	//c->usart->DR = *c->txBufferPtr;
}


/*
 * USART Interrupt Service Routine
 *
 */
void asseISR(asseControlData *c){

	// test for receive (RXNE flag set)
	if ((c->usart->SR & 0x20) != 0){
		// get the data
		char d = c->usart->DR;
		if ((d >= 32)) {
			// store the character, if room in the buffer
			if (c->rxBufferIndex < c->rxBufferSize) {
				*(c->rxBufferBase + c->rxBufferIndex++) = d;
			}
			// test for terminator
			if (d == c->terminator) {
				// terminator found, so set message ready flag to the length of the message
				c->rxMsgLength = c->rxBufferIndex;
				// and reset the index
				c->rxBufferIndex = 0;
			}
		}
	}
		// end if Rx
//	// test for tx complete (Bit 6 TC: Transmission complete)
//	if ((c->usart->SR & 0x40) != 0){
//		
//		// end transmission when null character encountered
//		if (*c->txBufferPtr == 0){
//			// end of transmission, set the TCCF flag in ICR to clear TC
//			c->usart->SR |= 0x40;
//			c->txInProgress = 0;
//		}
//		else {
//			// if reached the end of the buffer, end the transmission
//			if (c->txBufferCount++ >= c->txBufferLength){
//				c->usart->SR |= 0x40;
//				c->txInProgress = 0;
//			}
//			else{
//				// send the next char
//				c->usart->DR = *c->txBufferPtr++;
//			}
//		}
//	} // end if host tx
}

/*
 * Initialises the serial IO channel
 *
 */
void asseInitialise(asseControlData *c, USART_TypeDef *usart, UART_HandleTypeDef *husart, char terminator, char *rxBuffer, int rxBuffSize){

	// save the recieve buffer address & size
	c->rxBufferBase = rxBuffer;
	c->rxBufferSize = rxBuffSize;

	// save reference to the selected usart
	c->usart = usart;
	c->husart = husart;

	// set the rx terminating character
	c->terminator = terminator;


	// clear UE in CR1 to enable write to configuration registers
	c->usart->CR1 = 0;

	// set bit 12 in CR3 - disable Rx overrun interrupts
	//c->usart->CR3 = 0x1000;
	// set CR1:
	// Bit 6 TCIE: Transmission complete interrupt enable
	// Bit 5 RXNEIE: RX Not Empty interrupt enable
	// Bit 3 TE: Transmitter enable
	// Bit 2 RE: Receiver enable
	// Bit 13 UE: USART enable
	
	
	c->usart->CR1 |= USART_CR1_RXNEIE; // RE=1.. Enable the Receiver
	c->usart->CR1 |= (1<<2); // RE=1.. Enable the Receiver
	c->usart->CR1 |= (1<<3);  // TE=1.. Enable Transmitter
	c->usart->CR1 |= (1<<13);
  
		
}


// code to set baud rate, if required.

//uint32_t usartClockFreq  = 85000000;
//c->usart->BRR = usartClockFreq / dataRate;


/*+++REVISION_HISTORY+++
1) 23 Oct 2020 1st issue.
2) 10 Nov 2020 Uses data structure asseControlData to allow multiple instances of controller.
3) 20 Nov 2020 Introduced timeout in otherwise infinite loop in asseSend().
4) 10 Jan 2021 c->txInProgress set first before all other asseSend() variables.
5) 20 May 2021 asseTimeout changed to uint32_t, consistent with HAL_GetTick().
+++REVISION_HISTORY_ENDS+++*/
