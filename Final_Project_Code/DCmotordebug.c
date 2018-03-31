#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>



void configPWM();
void configIO();
void configInterrupts();
void mTimer(int delay);
void configADC();

volatile unsigned char dutyCycle = 0x40; 	// set PWM = 50%
volatile unsigned char DC_cw = 0x04;		// DC clockwise
volatile unsigned char DC_ccw = 0x02;		// DC counter-clockwise
volatile unsigned char DC_vccstop = 0x00;  	// vcc stop DC motot

/* Cylinder Info Storage */
// For reflective values of different materials
// AL = 0-250;	STL = 251-600;  WhPLT = 601-950;	BlPLT = 951-1024;
#define AL_MAX = 250;
#define STL_MIN = 251;
#define STL_MAX = 600;
#define WPL_MIN = 601;
#define WPL_MAX = 950;
#define BPL_MIN = 951;
#define BPL_MAX = 1024;


/* Sensors */
// For optical sensor IO
volatile unsigned int Count_OptIO = 0x00;
volatile unsigned int Enter_Flag = 0;
// For inductive sensor IN
volatile unsigned int Count_IndIN = 0x00;
volatile unsigned int Inductive_Flag = 0;

// For optical sensor OR
volatile unsigned int Count_OptOR = 0x00;
volatile unsigned int OR_Flag = 0;

// For optical sensor EX
volatile unsigned int Count_OptEX;
volatile unsigned int Exit_Flag = 0;
// For Hall Effect sensor HE
volatile unsigned int Hall_Flag;


// For reflective sensor RL ADC
volatile unsigned int Count_RefRL;
volatile unsigned char ADC_resultH;
volatile unsigned char ADC_resultL;
volatile unsigned int ADC_result;
volatile unsigned int ADC_result_Flag;
volatile unsigned int ADC_min =  1024; // 16_bit to save 10-bit ADC reflectness



void DCMotorCtrl(int sysSTATE);		// STATE = run, stop, pause Button, RampDown Button


int main()
{	
	cli();			// Disable all interrupts
	configIO();
	configInterrupts();
	configPWM(dutyCycle);
	configADC();
	sei();	

	while( OR_Flag == 0){
		PORTB = 0x04;
	
	}
		
	while(1){PORTB = 0x00;}	
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
	TCCR0B |= _BV(CS01);
	TIMSK0 |= _BV(OCIE0A);
	OCR0A = duty_cyc;
}

void configADC(){
	ADCSRA |= _BV(ADEN); 							// enable adc
	ADCSRA |= _BV(ADIE);							// enable interrupt of adc
	//ADCSRA |= (_BV(ADPS2) | _BV(ADPS0));			// adc scaler division factor 32
	//ADCSRA |= _BV(ADSC)		// for the first conversion
	
	ADMUX |= _BV(ADLAR);							// set 10bit ADC value structure, ADCH[7,0] = ADC[9,2], ADCL[7,6] = ADC[1,0]
	ADMUX |= _BV(REFS0); 			// Vcc 3.3v Voltage reference with external capacitor on AREF pin
	ADMUX |= _BV(MUX0);								// channel select, ADC1

	//ADMUX = 0B11100001;		// gen shang mian de san hang dai ma yi yang

}

void DCMotorCtrl(int sysSTATE){
	switch (sysSTATE){
		case 0:
			PORTB = DC_cw;
			break;
		case 1:
			PORTB = DC_vccstop;
			break;
		case 2:
			//buttonPressSTOP();
			break;
		case 3:
			//buttonPressRAMPDOWN();
			break;
	}
}


// For optical 1, IO, PD0, INT0
ISR(INT0_vect){
	Enter_Flag = 1;
	Count_OptIO++;		// optical 1, PD0
}

// For inductive, IN, PD1, INT1
ISR(INT1_vect){
	Count_IndIN++;				// inductive, PD1, for metal cylinders, for falling edge trigger
}


// For optical 2, OR, PD2, INT2, la Fini
ISR(INT2_vect){					//  optical 2, PD2 
	OR_Flag = 1;
	Count_OptOR++;
	ADCSRA |= _BV(ADSC); 		// rising on INT2, start ADC conversion
}

// For optical 3, EX, PD3, INT3
ISR(INT3_vect){
	Exit_Flag = 1;
	Count_OptEX++;
}

// For press buttom low to stop/resume the belt, PE4, INT4
ISR(INT4_vect){
	mTimer(5);
	DCMotorCtrl(1);		// Vcc stop the dc motor, PE4 for falling edge press button
	mTimer(5);
}


// For Hall effect sonsor under the stepper, PE5, INT5
ISR(INT5_vect){

	Hall_Flag = 1;

}

//For press button high, PE6
ISR(INT6_vect){
	DCMotorCtrl(3);
}

// For ADC conversion, RL, PF1, INTADC, la Fini Mar 29, 3.25PM
ISR(ADC_vect){					// ADC interrupt for reflecness conversion , PF1
	
	PORTC = 0x02;
	ADC_resultH = ADCH;
	ADC_resultL = ADCL;

	ADC_resultH = (ADC_resultH & 0b11111111);
	ADC_resultL = (ADC_resultL & 0b11000000);
	ADC_resultL = (ADC_resultL >> 6);

	ADC_result = ADC_resultH;
	ADC_result = (ADC_result << 2);

	ADC_result |= ADC_resultL;

	//Reflectness = ADC_result
	if (ADC_result <= ADC_min){
		ADC_min = ADC_result;
	}

	int outC;
	outC = (0b1111111100 | ADC_min);
	outC = outC >> 2;
	unsigned char OUTC;
	OUTC = outC;
	
	PORTC = ADC_resultH;
	ADC_result_Flag = 1;
}



void mTimer(int delay){
	/* Set the waveform generation mode (WGM) bit description to clear timer on compare math mode (CTC) only */
	int i;
	TCCR1B |= _BV(WGM12);	// this will set the WGM bits to 0100, WGM is spread over two registers
	OCR1A = 0x03e8;			// 1000 cycles = 1ms, set output register for 1000 cycles
	TCNT1 = 0x0000;			// set the initial value of the timer counter to 0x0000
	//TIMSK1 |= TIMSK1 | 0b00000010;	// Enable the output compare interrupt enable

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

