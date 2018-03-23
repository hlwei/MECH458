#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
// define the global variables that can be used in every function ==========
volatile unsigned char ADC_resultH; 
volatile unsigned char ADC_resultL; 
volatile unsigned int ADC_result_flag;
int flag = 0;

// Function declarations
void mTimer(); //timer
void set0();   //set the stepper motot to initial position

void main() {

	char d4, d7, pd;
	cli(); // disable all of the interrupt ==========================
	
	DDRA = 0xFF;// Stepper
	DDRB = 0xFF;// PORTB<->DC motor
	DDRC = 0xFF;// set the PORTC and LEDs on board to display the ADC results
	DDRD = 0x00;// buttons
    DDRF = 0x00;// RL sensor->PORTF1

	// config the external interrupt ====================================== 
	EIMSK |= (_BV(INT0)); // enable INT0
	EICRA |= (_BV(ISC01) ); // rising edge interrupt

	// config ADC
	ADCSRA |= _BV(ADEN);
	ADCSRA |= _BV(ADIE);
	ADMUX  |=  (_BV(REFS0))|(_BV(MUX0))|(_BV(ADLAR));

	// sets the Global Enable for all interrupts ==========================
	sei();

	//DC motor 
	TCCR0A|=_BV(WGM01)|_BV(WGM00)|_BV(COM0A1);
	TCCR0B|=_BV(CS01);
	TIMSK0|=_BV(OCIE0A);
	OCR0A = 0x80;// Control the PWM
	PORTB = 0x04;
    // initialize the ADC, start one conversion at the beginning ==========
	ADCSRA |= _BV(ADSC);

	while (1) 
	{/*
		if(ADC_result_flag)	
		{
			PORTC = (ADC_resultH);
			d4 = (ADC_resultL & 0b10000000);
			d7 = (ADC_resultL & 0b01000000);
			d4 = d4 >> 3;
			d7 = d7 << 1;
			pd = d4 | d7;
			PORTD = pd;
			ADC_result_flag = 0;
		}	*/
	 }
}
	

	// if the bottom is pressed, the motor is paused or cw
	ISR(INT0_vect)
	{
        if(flag == 0){
		PORTB = 0x04;
		//mTimer(5);
		flag = ~flag;
		
		}
		else
		{
		PORTB = 0x06;
		//mTimer(5);
		flag = ~flag;

		}

	}


	ISR(ADC_vect)
	{
		ADC_resultH = ADCH;
		ADC_resultL = ADCL;
		ADC_result_flag = 1;

	}

/* the timer*/
void mTimer(int count){
int i;	
i = 0;

	TCCR1B|=_BV(WGM12);
	OCR1A = 0x0000;
	TCNT1 = 0x0000;
	TIMSK1 = TIMSK1|0b00000010;
	TIFR1 |=_BV(OCF1A);
	
	while(i<count){
		if ((TIFR1 & 0x02) == 0x02){
		TIFR1|=_BV(OCF1A);
		i++;
		}
	}
	return;
}



