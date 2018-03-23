/*########################################################################
# MILESTONE : 4B
# PROGRAM : 4B
# PROJECT : Lab 4B
# GROUP : 
# NAME 1 : Heng-lai Wei, V00884728
# NAME 2 : Zhuo Li, V00834939
# DESC : This program has two main function: 
#1.generate pwm signals; 
#2.red ADC signal;
# DATE:03/02/2018
# REVISED:03/02/2018
########################################################################*/
# include <stdio.h>
# include <stdlib.h>
# include <avr/io.h>
#include<avr/interrupt.h> 

volatile unsigned char ADC_result; 
volatile unsigned int ACD_reslut_flag;
// initilization for the steper motor 

void main(){

	cli();//disable all of the interrupt

	//configure the external interrupt
	EIMSK |= (_BV(INT2));// enable INT2
	EICRA |= (_BV(ISC21)) | _BV(ISC20);

	EIMSK |= (_BV(INT1));// enable INT1
	EICRA |= (_BV(ISC11)); //EICRA external interrupt control register
	// config ADC ========================================================= // by default, the ADC input (analog input is set to be ADC0 / PORTF0 
	
	ADCSRA |= _BV(ADEN); // enable ADC
	ADCSRA |= _BV(ADIE); // enable interrupt of ADC
	ADMUX |= _BV(ADLAR) | _BV(REFS0);

    
	// set the PORTA as output to display the ADC result ==================
	DDRD = 0x00;		// portD:input the interrupt
	DDRC = 0xFF;


	// sets the Global Enable for all interrupts ==========================	
	sei();

	// initialize the ADC, start one conversion at the beginning ==========
	ADCSRA |= _BV(ADSC);
	DDRB = 0xFF;
	TCCR0A|=_BV(WGM01)|_BV(WGM00)|_BV(COM0A1);
	TCCR0B|=_BV(CS01);
	TIMSK0|=_BV(OCIE0A);


    while(1){
		if (ACD_reslut_flag) {
		    OCR0A = ADC_result;
			//if((0x04 & PINA)==0x00)
			PORTB = 0b00000100;
			ACD_reslut_flag = 0x00;
			ADCSRA |= _BV(ADSC);
		}
	}
}

	ISR(INT1_vect)
	{
		PORTB=0b00000100;
	}
// sensor 3: 2nd Optical Inductive, Active HIGH starts AD converstion =======


	ISR(INT2_vect)
	{
		ADCSRA |= _BV(ADSC);
	}


	// when there is a rising edge, we need to do ADC =====================

 	// the interrupt will be trigured if the ADC is done ========================
	ISR(ADC_vect)
	{
		ADC_result = ADCH;
		ACD_reslut_flag = 1;
	}


 




