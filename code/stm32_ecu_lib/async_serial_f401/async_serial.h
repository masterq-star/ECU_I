#ifndef _async_serial
#define _async_serial

#include "main.h"



typedef struct {

	// RX meta
	char *rxBufferBase;		// pointer to the rx buffer
	int rxBufferSize;		// the size of the rx buffer
	int rxBufferIndex;		// index into the rx buffer
	int rxMsgLength;		// 0 = message not ready. > 0 = complete message ready and value is the message length
	char terminator;			// terminating character for the rx stream

	// TX meta
	char *txBufferPtr;		// pointer to the transmit buffer
	int txBufferLength;		// tx buff length
	int txBufferCount;		// counts the characters as they are transmitted
	int txInProgress;		// flag indicates tx in progress (1) or idle (0)

	// uart in use
	USART_TypeDef *usart;	// pointer to the usart device

} asseControlData;


extern void asseISR(asseControlData *c);
extern void asseSend(asseControlData *c, char *txBuffer, int bufferLen);
extern void asseInitialise(asseControlData *c, USART_TypeDef * usart, char terminator, char *rxBuffer, int rxBuffSize);

#endif
