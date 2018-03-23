
/*########################################################################
# MILESTONE : 4B
# PROGRAM : 4B
# PROJECT : Lab 4B
# GROUP : 
# NAME 1 : Heng-lai Wei, V00884728
# NAME 2 : Zhuo Li, V00834939
# DESC : This program has two main function: 
#1.generate pwm signals; 
6++++++++++++++++++++++++9

#2.red ADC signal;
# DATE:03/02/2018
# REVISED:03/02/2018
########################################################################*/

#include <avr/interrupt.h>
#include <avr/io.h>


// define the global variables that can be used in every function ==========



volatile unsigned char ADC_result; 
volatile unsigned int ACD_reslut_flag;
// initilization for the steper motor 


void main() {

	cli(); // disable all of the interrupt ==========================
	
	
	
	// config the external interrupt ====================================== 
	
	EIMSK |= (_BV(INT2)); // enable INT2
	
	EICRA |= (_BV(ISC21) | _BV(ISC20)); // rising edge interrupt
	// config ADC ========================================================= // by default, the ADC input (analog input is set to be ADC0 / PORTF0 
	
	ADCSRA |= _BV(ADEN); // enable ADC
	ADCSRA |= _BV(ADIE); // enable interrupt of ADC
	ADMUX |= _BV(ADLAR) | _BV(REFS0);
	// set the PORTA as output to display the ADC result ==================
	DDRC = 0xff;
	// sets the Global Enable for all interrupts ==========================
	
	
	
	sei();
	// initialize the ADC, start one conversion at the beginning ==========
	
	
	ADCSRA |= _BV(ADSC);
	DDRB = 0xFF;
	TCCR0A|=_BV(WGM01)|_BV(WGM00)|_BV(COM0A1);
	TCCR0B|=_BV(CS01);
	TIMSK0|=_BV(OCIE0A);
	//OCR0A = 0x80;
	PORTB = 0x02;

	while (1) {

		if (ACD_reslut_flag) {
		    OCR0A = ADC_result;
			PORTC = ADC_result;
			ACD_reslut_flag = 0x00;
			ADCSRA |= _BV(ADSC);
		}
         
	 }
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
 




