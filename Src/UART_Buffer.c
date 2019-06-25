/*
 * UART_Buffer.cpp
 *
 *  Created on: 1 זמגע. 2018 נ.
 *      Author: i-kukharenko
 */

#include "UART_Buffer.h"

unsigned char UART_BUF[UART2_BUFFER_SIZE];
//unsigned char UART_DECODED[UART2_BUFFER_SIZE];
//unsigned char UART_EXECUTED[UART2_BUFFER_SIZE];
unsigned char UART_W_PTR=0;
unsigned char UART_R_PTR=0;

void UART_buff_appennd(uint8_t byte){
	uint8_t temp = (UART_W_PTR+1) & UART2_BUFFER_MASK;
	if(temp != UART_R_PTR){
		UART_W_PTR = temp;
		UART_BUF[UART_W_PTR]=byte;
	}
	else{
		UART_BUF[UART_W_PTR]=0;
	}
}

UART_buff_read_status UART_buf_read(uint8_t * byte){
	if (UART_R_PTR != UART_W_PTR){
		UART_R_PTR++;
		UART_R_PTR &= UART2_BUFFER_MASK;
		*byte = UART_BUF[UART_R_PTR];
		return(READ_OK);
	}
	return(NOTHING_TO_READ);
}
