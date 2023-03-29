/*
 * uart.h
 *
 * Created: 17.02.2022 13:42:09
 *  Author: Ondra
 */ 


#ifndef UART_H_

#define UART_H_
void UART1_init(uint16_t Baudrate);
void UART1_SendChar(uint8_t data);
uint8_t UART1_GetChar( void );
void UART1_SendString(char *text);

void UART0_init(uint16_t Baudrate);
void UART0_SendChar(uint8_t data);
uint8_t UART0_GetChar( void );
void UART0_SendString(char *text);
int printCHAR(char character, FILE *stream);

#endif /* UART_H_ */