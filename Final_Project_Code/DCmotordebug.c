#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "interrupt.h"
#include <math.h>



void configPWM();
void configIO();
void configInterrupts();
void mTimer(int delay);

volatile unsigned char dutyCycle = 0x80; 	// set PWM = 50%
volatile unsigned char DC_cw = 0x02;		// DC clockwise
volatile unsigned char DC_ccw = 0x04		// DC counter-clockwise
volatile unsigned char DC_vccstop = 0x00  	// vcc stop DC motot


void DCMotorCtrl(char sysSTATE);		// STATE = run, stop, pause Button, RampDown Button


int main()
{	
	cli();			// Disable all interrupts
	void configIO();
	void configInterrupts();
	void configPWM();
	void configADC();
	sei();	
	
	return 0;
}



void configInterrupts(){
	EIMSK |= 0b01111111;					// Enable INT0-6

	// Rising edge: INT2 & INT 6
	EICRA |= (_BV(ISC20) | _BV(ISC21));		// INT2 rising edge
	EICRB |= (_BV(ISC60) | _BV(ISC61));		// INT6 rising edge

	// Falling edge: INT0,1,3,4,5
	EICRA |= _BV(ISC01);					// INT0 falling edge
	EICRA |= _BV(ISC11);					// INT1 falling edge
	EICRA |= _BV(ISC31);					// INT3 falling edge

	EICRB |= _BV(ISC41);					// INT4 falling edge
	EICRB |= _BV(ISC51);					// INT5 falling edge

}

void configIO(){
	/* IO Ports Definition */
	DDRA = 0xff;	// PORTA output, Stepper motor drive
	DDRB = 0xff;	// PORTB output, DC motor drive
	DDRC = 0xff;	// PORTC output, LEDs debugging
	DDRD = 0x00;	// PORTD[0,3] input, interrupts
	DDRE = 0x00;	// PORTE[4,7] input, interrupts
	DDRF = 0x00;	// PORTF1 Reflective ADC interupt
}



void configPWM(int duty_cyc){
	TCCR0A |= _BV(WGM00) | _BV(WGM01) | _BV(COM0A1);
	TCCROB |= _BV(CS01);
	TIMSK0 |= _BV(OCIE0A);
	OCR0A = duty_cyc;
}


void DCMotorCtrl(sysSTATE){
	switch (sysSTATE){
		case 0:
			PORTB = DC_cw;
			break;
		case 1:
			PORTB = DC_vccstop;
			break;
		case 2:
			buttonPressSTOP();
			break;
		case 3:
			buttonPressRAMPDOWN();
			break;
	}
}



ISR(INT6_vect){
	DCMotorCtrl(3);
}



void mTimer(int delay){
	/* Set the waveform generation mode (WGM) bit description to clear timer on compare math mode (CTC) only */
	int i;
	TCCR1B |= _BV(WGM12);	// this will set the WGM bits to 0100, WGM is spread over two registers
	OCR1A = 0x03e8;			// 1000 cycles = 1ms, set output register for 1000 cycles
	TCNT1 = 0x0000;			// set the initial value of the timer counter to 0x0000
	TIMSK1 |= TIMSK1 | 0b00000010;	// Enable the output compare interrupt enable

	// clear the timer interrupt flag and begin timer

	TIFR1 |= _BV(OCF1A);
	// polling the timer to determine when the timer has reached 0x03e8=1000

	while(i<delay){// clear the interrupt falg by writting a 1 to the bit
		if ((TIFR1 & 0x02) == 0x02)
		{
			TIFR1 |= _BV(OCF1A);
			i++;
		}
	}
}
