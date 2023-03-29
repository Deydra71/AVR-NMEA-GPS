/*
 * LWM_MSSY.c
 *
 * Created: 6.4.2017 15:42:46
 * Author : Krajsa
 */ 

#include <avr/io.h>
/*- Includes ---------------------------------------------------------------*/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "config.h"
#include "hal.h"
#include "phy.h"
#include "sys.h"
#include "nwk.h"
#include "sysTimer.h"
#include "halBoard.h"
//#include "halUart.h"	// uziti vlastniho uartu
#include "main.h"

#include <util/delay.h>
#include <avr/interrupt.h>
#include "uart/uart.h"

/*- Definitions ------------------------------------------------------------*/

#ifdef NWK_ENABLE_SECURITY
#define APP_BUFFER_SIZE     (NWK_MAX_PAYLOAD_SIZE - NWK_SECURITY_MIC_SIZE)
#else
#define APP_BUFFER_SIZE     NWK_MAX_PAYLOAD_SIZE
#endif

/*- Types ------------------------------------------------------------------*/
typedef enum AppState_t
{
	APP_STATE_INITIAL,
	APP_STATE_IDLE,
	APP_STATE_LWM_SEND_TIME,
	//APP_STATE_LWM_SEND_PULSE,
} AppState_t;

/*- Prototypes -------------------------------------------------------------*/
//static void appSendData(void);
static void appSendPulse(uint16_t dst);
static void appSendTime(uint16_t dst);	// funkce posilani bufferu casu (hhmmss)
void reset_GPS(void);
void board_init(void);					// iniciace desky a periferii
uint8_t Parse_GPRMC_Time(char * GPS_buffer, char * hhmmss);			// vlastni parsovaci funkce pro ziskani casu z GPRMC spravy, vykonava se kazdou 3 sekundu pro demonstraci
																	// prakticky se synchronizace vykona podle potreby a ucelu site prodlouzenim intervalu, ktery normalne byva pres timer a ne delay jako je to resene tady
/*- Variables --------------------------------------------------------------*/
static AppState_t appState = APP_STATE_INITIAL;
static SYS_Timer_t appTimer;
static NWK_DataReq_t appDataReq;
static bool appDataReqBusy = false;

char send_buffer [256] = {'\0'};	//UART buffer
char GPS_buffer[256]  = {'\0'};
char hhmmss[10] = "hhmmss";
uint8_t send_hhmmss[6];
uint8_t send_pulse[6];
uint8_t buffer_index = 0;
volatile uint8_t send_pulse_flag = 0;
/*- Implementations --------------------------------------------------------*/

/*************************************************************************//**
*****************************************************************************/
static void appDataConf(NWK_DataReq_t *req)
{
appDataReqBusy = false;
(void)req;
}

/*************************************************************************//**
*****************************************************************************/
void board_init(){
	cli(); //disable interrupts
//	PB1 = LED4
//	PB2 = LED2
//	PB3 = LED3
	//	init LED
	DDRB =	0b00001110;
	PORTB = 0b00001110;
	//	FIBOCOM RST pin
	DDRE =	0b00000100;
	PORTE = 0b00000100;
		
	UART1_init(38400); //baudrate 38400b/s PC
	UART0_init(9600);  // GPS UART - dle datasheetu
	
	UCSR1B |= (1 << RXCIE1); // UART receive interrupt enable
	UCSR0B |= (1 << RXCIE0); // UART receive interrupt enable

	////	INTERRUPT 5 - GPS status	////
	EIMSK |= (1 << INT5);	// zapnout interrupt 5
	
	// nabezna i sestupna hrana 	
//	EICRB |= (1 << ISC50);

	//nabezna hrana
	EICRB |= (1 << ISC50);
	EICRB |= (1 << ISC51);
	
	//sestupna hrana
//	EICRB |= (1 << ISC51);
	
	
	//	BLIK LED
	uint8_t i = 0;
	while(i < 3){
		_delay_ms(500);
		PORTB = 0b00000000;
		_delay_ms(500);
		PORTB = 0b00001110;
		i++;
	}
	sei(); // enable interrupts
}
/*************************************************************************//**
*****************************************************************************/

void reset_GPS(void) {
	//	FIBOCOM 200 ms reset puls
	_delay_ms(500);
	PORTE = 0b00000000;
	_delay_ms(200);
	PORTE = 0b00000100;
}
/*************************************************************************//**
*****************************************************************************/

static bool appDataInd(NWK_DataInd_t *ind)
{
	return true;
}
/*************************************************************************//**
*****************************************************************************/

static void appSendTime (uint16_t dst)
{
	if (appDataReqBusy)
	return;
	
	appDataReq.dstAddr = dst;
	appDataReq.dstEndpoint = 6;
	appDataReq.srcEndpoint = 6;
	appDataReq.options = NWK_OPT_ENABLE_SECURITY;
	appDataReq.data = send_hhmmss;	//bylo tady pole
	appDataReq.size = 6;
	appDataReq.confirm = appDataConf;
	NWK_DataReq(&appDataReq);
	
	appDataReqBusy = true;
	
}

static void appSendPulse (uint16_t dst)
{
	if (appDataReqBusy)
	return;
	
	appDataReq.dstAddr = dst;
	appDataReq.dstEndpoint = 7;
	appDataReq.srcEndpoint = 7;
	appDataReq.options = NWK_OPT_ENABLE_SECURITY;
	appDataReq.data = send_pulse;	
	appDataReq.size = 1;
	appDataReq.confirm = appDataConf;
	NWK_DataReq(&appDataReq);
	
	appDataReqBusy = true;
	
}

/*************************************************************************//**
*****************************************************************************/
static void appTimerHandler(SYS_Timer_t *timer)
{
//appSendData();
//appSendOK(APP_ADDR-1);
appSendTime(APP_ADDR-1);
(void)timer;

}
/*************************************************************************//**
*****************************************************************************/
static void appInit(void)
{
NWK_SetAddr(APP_ADDR);
NWK_SetPanId(APP_PANID);
PHY_SetChannel(APP_CHANNEL);
#ifdef PHY_AT86RF212
PHY_SetBand(APP_BAND);
PHY_SetModulation(APP_MODULATION);
#endif
PHY_SetRxState(true);

NWK_OpenEndpoint(APP_ENDPOINT, appDataInd);

HAL_BoardInit();

//softwarovy timer
appTimer.interval = APP_FLUSH_TIMER_INTERVAL;
appTimer.mode = SYS_TIMER_INTERVAL_MODE;
appTimer.handler = appTimerHandler;
}

/*************************************************************************//**
*****************************************************************************/

// $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh

// $GPRMC,123519,A,4807.038,N,01131.000,E,0.022,269.131,230394,,,A,C*6A
// $GPRMC,082100.466,V,,,,,,,040522,,,N*43

uint8_t Parse_GPRMC_Time(char * GPS_buffer, char * hhmmss) {
	// GPS_buffer muze byt pozmenen prerusenim z ISR uart0 pocas parsovani - to se nesmi stat,
	// obsah zachycenych sprav je ulozen do parse_buffer, ktery zustane staticky pocas parsovani
	char parse_buffer[256];
	strcpy(parse_buffer, GPS_buffer);
	for (int i = 0; i < 256; i++)
	{
		// min 20 znakova rezerva od pocatku GPRMC spravy po konec GPS_bufferu potrebna pro uspesne parsovani casu
		if (i <= 235) {
			//if ( (parse_buffer[i] == '$') && (parse_buffer[i+5] == 'C')	 && (parse_buffer[i+18] == 'A') ) {		// $GPRMC,hhmmss,A
				if ( (parse_buffer[i] == '$') && (parse_buffer[i+5] == 'C') ) {
				// zapis casoveho useku spravy do pole hhmmcc[], ktere se bude odesilat
				uint8_t j = 0;	// index aktualni cifry casu
				while (j < 6) {
					if ( (parse_buffer[i+7+j] >= '0') && (parse_buffer[i+7+j] <= '9') ) {
						hhmmss[j] = parse_buffer[i+7+j];
						j++;
					}
					else {
						return 0;
					}
				}
				//aktualni cas byl ulozen, dalsi parsovani v ramci bufferu nema smysl, konec
				return 1;
				//hours[0] = parse_buffer[i+7];
				//hours[1] = parse_buffer[i+7];
			}
		}
		// pokud i > 235 dalsi parsovani v aktualnim bufferu neni mozne, konec parsovani
		else {
			return 0;
		}
	}
	return 0;
}

/*************************************************************************//**
*****************************************************************************/

static void APP_TaskHandler(void)
{
	switch (appState)
		{
		case APP_STATE_INITIAL: //nastaveni adresy, site, kanalu
			{
			appInit();
							  //$GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh
			strcpy(GPS_buffer, "$GPRMC,194509.000,A,4042.6142,N,07400.4168,W,2.03,5.84,160412,,,A*778\0");
			send_pulse[0] = 1;
			board_init();
			reset_GPS();
			_delay_ms(1000);
			appState = APP_STATE_IDLE;
			} break;

		case APP_STATE_IDLE:
			_delay_ms(3000);
			if (send_pulse_flag != 0) {
					appSendPulse(0xffff);
					send_pulse_flag = 0;
			}
			strcpy(send_buffer, "\n\rActual GPS_buffer: \n\r");
			UART1_SendString(send_buffer);
			UART1_SendString(GPS_buffer);
		
			if( (Parse_GPRMC_Time(GPS_buffer, hhmmss) ) >= 1) {
				strcpy(send_buffer, "\n\rGPRMC packet ok \n\r");
				UART1_SendString(send_buffer);
				strcpy(send_buffer, "\n\rhhmmss: \n\r");
				UART1_SendString(send_buffer);
				UART1_SendString(hhmmss);
				strcpy(send_buffer, "\n\r");
				appState = APP_STATE_LWM_SEND_TIME;
			}
			else
			{
				strcpy(send_buffer, "\n\r No GPS signal / Invalid packet\n");
				UART1_SendString(send_buffer);
				appState = APP_STATE_IDLE;
			}
		break;
		
		
		case APP_STATE_LWM_SEND_TIME:
			for (uint8_t i=0; i <= 6; i++)
			{
				send_hhmmss[i] = (uint8_t)hhmmss[i];	//parsuje se pole znaku (char), a odesila se pole uint8_t
			}			
			//appSendTime(APP_ADDR-1);
			appSendTime(0xFFFF);
			appState = APP_STATE_IDLE;
		break;
		
			default:
			break;
		}
}


/*************************************************************************//**
*****************************************************************************/
int main(void)
{
SYS_Init();	//vzdy zavolat

while (1)
{
SYS_TaskHandler();		//obsluha stacku, prijimani a odeslani dat, stack, neresit
APP_TaskHandler();		//uzivatelske funkce, stavovy automat
//appSendOK(APP_ADDR-1);
}
}


/************************************************************************/
/* INTERRUPTS                                                           */
/************************************************************************/
ISR(INT5_vect){
	send_pulse_flag = 1;
}

ISR(USART0_RX_vect){
	volatile char c = UDR0;
	
	if (buffer_index <= (sizeof(GPS_buffer)-2) )
	{
		GPS_buffer[buffer_index] = c;
		GPS_buffer[(buffer_index + 1)] = '\0';
		buffer_index++;
	}
	else 
	{
		buffer_index = 0;
	}

}

//USART1 je spojena s PC
ISR(USART1_RX_vect){
	volatile char c = UDR1; //Read the value out of the UART buffer
	char reset_buffer[80];
	if (c == 'r') {
		reset_GPS();
		strcpy(reset_buffer, "\n\rGPS reset\n\r");
		UART1_SendString(reset_buffer);
	}
	if (c == '1') {
		_delay_ms(5000); // pause execution when we send char '1' for debugging
	}
}