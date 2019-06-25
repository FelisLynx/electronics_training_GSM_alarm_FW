/*
 * UART_Buffer.h
 *
 *  Created on: 1 זמגע. 2018 נ.
 *      Author: i-kukharenko
 */

#ifndef UART_BUFFER_H_
#define UART_BUFFER_H_

#include <stdint.h>

#define UART2_BUFFER_SIZE 128
#define UART2_BUFFER_MASK 0b1111111

extern unsigned char UART_BUF[UART2_BUFFER_SIZE];
extern unsigned char UART_W_PTR;
extern unsigned char UART_R_PTR;

typedef enum UART_buff_read_status{
	NOTHING_TO_READ,
	READ_OK
}UART_buff_read_status;

//extern void UART_buff_appennd(uint8_t);
//extern UART_buff_read_status UART_buf_read(uint8_t *); // Declared in Main.cpp to work with .cpp (C/C++ mangling problem)
void UART_buff_appennd(uint8_t byte);
UART_buff_read_status UART_buf_read(uint8_t * byte);

#endif /* UART_BUFFER_H_ */
