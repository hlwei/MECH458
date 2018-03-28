/* University of Victoria 2018 Spring MECH 458 Final Proeject */
/* STUDENT1: Zhuo Li V00885451 */
/* STUDENT2: Henglai Wei V00884728 */




/* Header files */

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "interrupt.h"
#include <math.h>


/* ------------ Subroutines List ------------ */
/* ------------------------------------------ */

/* ----------- Configs ----------- */
/* ------------------------------- */
void configPWM();
void configADC();
void configInterrupts();
void configTimers();

/* --------- System Ctrls --------- */
/* -------------------------------- */
void beltCtrl(char sysSTATE);		// STATE = run, stop, pause Button, RampDown Button
void DCMotorCtrl();
void stepperCtrl(int current_bin, int sort_bin)	
void stepperInit();				// Init by HE sensor



/* -------------- Variables List -------------- */
/* -------------------------------------------- */

/* Motors */
// For DC motor
volatile unsigned char dutyCycle = 0x80; // set PWM = 50%
volatile unsigned char DC_cw = 0x02;		// DC clockwise
volatile unsigned char DC_ccw = 0x04		// DC counter-clockwise
volatile unsigned char DC_vccstop = 0x00  	// vcc stop DC motot

// For stepper motor
volatile unsigned char Steps[4] = [-90, 0, 90, 180]; // to do
volatile unsigned char ObjectReciv;


/* Sensors */
// For optical sensor IO
volatile unsigned int Count_OptIO;
volatile unsigned int EnterFlagl;
// For inductive sensor IN
volatile unsigned int Count_IndIN;
// For optical sensor OR
volatile unsigned int Count_OptOR;

// For reflective sensor RL ADC
volatile unsigned int Count_RefRL;
volatile unsigned char ADC_resultH;
volatile unsigned char ADC_resultL;
volatile unsigned int ADC_result_Flag;

// For optical sensor EX
volatile unsigned int Count_OptEX;
volatile unsigned int ExitFlag;
// For Hall Effect sensor HE
volatile unsigned int HallFlag;

/* System State */
volatile unsigned char sysSTATE = ['run','stop','pause','RampDown'];

/* Cylinder Info Storage */
// For reflective values of different materials
// AL = 0-250;	STL = 251-600;  WhPLT = 601-950;	BlPLT = 951-1024;
#define AL_MAX = 250;
#define STL_MIN = 251;
#define STL_MAX = 600;
#define WPL_MIN = 601;
#define WPL_MAX = 950;
#define BPL_MIN = 951;

// AL = 0; STL = 1; WPL = 2; BPL = 3;
volatile unsigned int Cylinders[48]



/* -------------- Main Loop -------------- */
/* --------------------------------------- */
/* --------------------------------------- */
/* --------------------------------------- */
int main(int argc, char const *argv[])
{
	/* user parameters define here */

	/* IO Ports Definition */
	DDRA = 0xff;	// PORTA output, Stepper motor drive
	DDRB = 0xff;	// PORTB output, DC motor drive
	DDRC = 0xff;	// PORTC output, LEDs debug
	DDRD = 0xf0;	// PORTD[0,3] input, interrupts
	DDRE = 0x00;	// PORTE[4,7] input, interrupts
	DDRF = 0x00;	// PORTF1 Reflective ADC interupt

	cli();			// Disable all interrupts
	configPWM();
	configADC();
	configInterrupts();
	configTimers();
	sei();			// enable all interrupts

	/* DC and Stepper Initialization here */
	stepperInit();

	while(1){
		/* Sorting code here */
	}

	return 0;
}





/* LIGNE */



void configPWM(int duty_cyc){
	TCCR0A |= _BV(WGM00) | _BV(WGM01) | _BV(COM0A1);
	OCR0A = duty_cyc;
	PORTB = dc_clockwise;
}

void configADC(){
	ADCSRA |= _BV(ADEN); 		// enable adc
	ADCSRA |= _BV(ADIE);		// enable interrupt of adc
	ADCSRA |= _BV(ADPS2);
	ADCSRA |= _BV(ADPS0);		// adc scaler 32
	ADMUX |= _BV(REFS0); 
	ADMUX |= _BV(MUX0);		// Avcc refer 3.3 

	ADMUX = ADMUX & 0B11100001;		// PORTF1 for ADC reflective sensor

}

void configInterrupts(){
	EIMSK |= 0b01011111;		// init INT0,1,2,3,4,6
	EICRA |= 0B11101110;		// rising edge INT1,3; falling edge INT2,4
	EICRB |= 0b00100010;		// low for INT4,6
}



/* ----------- Des for Subroutines --------- */
/* ---------------------------------------- */

void configPWM(int duty_cyc){
	TCCR0A |= _BV(WGM00) | _BV(WGM01) | _BV(COM0A1);
	OCR0A = duty_cyc;
	PORTB = dc_clockwise;
}

void configADC(){
	ADCSRA |= _BV(ADEN); 		// enable adc
	ADCSRA |= _BV(ADIE);		// enable interrupt of adc
	ADCSRA |= _BV(ADPS2);
	ADCSRA |= _BV(ADPS0);		// adc scaler 32
	ADMUX |= _BV(REFS0); 
	ADMUX |= _BV(MUX0);		// Avcc refer 3.3 

	ADMUX = ADMUX & 0B11100001;		// PORTF1 for ADC reflective sensor

}

void configInterrupts(){
	EIMSK |= 0b01011111;		// init INT0,1,2,3,4,6
	EICRA |= 0B11101110;		// rising edge INT1,3; falling edge INT2,4
	EICRB |= 0b00100010;		// low for INT4,6
}




/* ----------- Des for interrupts --------- */
/* ---------------------------------------- */
// For ADC conversion, RL, PF1, INTADC
ISR(ADC_vect){					// ADC interrupt for reflecness conversion , PF1

}


// For optical 1, IO, PD0, INT0
ISR(INT0_vect){
	Num_OI = Num_OI + 1;		// optical 1, PD0
}

// For inductive, IN, PD1, INT1
ISR(INT1_vect){
	Num_IN += 1;				// inductive, PD1, for metal cylinders, for falling edge trigger
}

// For optical 2, OR, PD2, INT2
ISR(INT2_vect){					//  optical 2, PD2 
	lowADC=0xFFFF;
	ADCSRA|= _BV(ADSC); 		// once the cyliners arrive the reflective sensor, ADC conversion starts
	NUm_OR += 1;				
	ADCCompleteFlag=0x00; // tell system ADC conversions are occurring
}

// For optical 3, EX, PD3, INT3
ISR(INT3_vect){
	End_travel += 1;
}

// For press buttom to stop/resume the belt, PE4, INT4
ISR(INT4_vect){
	PORTB &= 0b11110000;		// Vcc stop the dc motor, PD0 for falling edge press button
}


// For Hall effect sonsor under the stepper, PE5, INT5
ISR(INT5_vect){
	stepperInitSucc();
	stepperInitFlag = 1;
}

// For timer interrupt
ISR(TIMER1_OVF_vect){
	TIFR1 |= 0x01;
}

