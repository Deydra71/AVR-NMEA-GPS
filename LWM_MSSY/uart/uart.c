/*/*
 * uart.c
 *
 * Created: 17.02.2022 13:41:57
 *  Author: Ondra
 */ 

//#include "../makra.h"
#include <avr/io.h>
#include <stdio.h>

void UART1_init(uint16_t Baudrate){
		int ubrr=((F_CPU/16/Baudrate)-1);
		UCSR1C = (1<<UCSZ11)|(1<<UCSZ10);// Async, Parity-Disabled, 1 Stop bit, 8 data bits
		//sbi(UCSR1C,UCSZ11);
		//sbi(UCSR1C,UCSZ10);
		UBRR1H = (uint8_t)(ubrr>>8);
		UBRR1L = (uint8_t)ubrr;
		UCSR1B = (1<<RXEN1)|(1<<TXEN1);// Enable RX/TX
}
void UART0_init(uint16_t Baudrate){
	int ubrr=((F_CPU/16/Baudrate)-1);
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);// Async, Parity-Disabled, 1 Stop bit, 8 data bits
	//sbi(UCSR1C,UCSZ11);
	//sbi(UCSR1C,UCSZ10);
	UBRR0H = (uint8_t)(ubrr>>8);
	UBRR0L = (uint8_t)ubrr;
	UCSR0B = (1<<RXEN0);// Enable RX/TX
}
void UART1_SendChar(uint8_t data)
{
	while ( !( UCSR1A & (1<<UDRE1)) )
	;
	UDR1 = data;
}
void UART0_SendChar(uint8_t data)
{
	while ( !( UCSR0A & (1<<UDRE0)) )
	;
	UDR0 = data;
}
uint8_t UART1_GetChar( void )
{
	while ( !(UCSR1A & (1<<RXC1)) )
	;
	return UDR1;
}
uint8_t UART0_GetChar( void )
{
	while ( !(UCSR0A & (1<<RXC0)) )
	;
	return UDR0;
}

void UART1_SendString(char *text)
{
	while (*text != 0x00)
	{
		UART1_SendChar(*text);
		text++;
	}
}
/*
void UART1_SendData(uint8_t *text)
{
	while (*text != 0x00)
	{
		UART1_SendChar(*text);
		text++;
	}
}
*/
void UART0_SendString(char *text)
{
	while (*text != 0x00)
	{
		UART0_SendChar(*text);
		text++;
	}
}

int printCHAR(char character, FILE *stream)
{
	while ((UCSR1A & (1 << UDRE1)) == 0) {};

	UDR1 = character;

	return 0;
}

