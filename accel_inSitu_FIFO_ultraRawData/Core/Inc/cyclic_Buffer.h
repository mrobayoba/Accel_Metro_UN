/*
 * cyclic_Buffer.h
 *
 *  Created on: May 24, 2024
 *      Author: matto
 */

#ifndef INC_CYCLIC_BUFFER_H_
#define INC_CYCLIC_BUFFER_H_

typedef struct{
	uint8_t* bufferData; // array to store data
	uint32_t bufferSize;

	uint8_t* bufferHead; // pointer to the start of the buffer
	uint8_t* bufferTail; // pointer to the last data wrote

} cBuffer_Handler;

void cBuffer_create(cBuffer_Handler* ptrCicularBuffer); // to create the buffer to manage data
void cBuffer_pop(cBuffer_Handler* ptrCicularBuffer); // to remove transmitted data from the cBuffer
void cBuffer_push(cBuffer_Handler* ptrCicularBuffer); // to add data to the cBuffer

#endif /* INC_CYCLIC_BUFFER_H_ */
