
///2nd case 
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdlib.h>

/* ----------- Configs ----------- */
/* ------------------------------- */
void configIO();
void configInterrupts();
void configPWM();
void configADC();


/* Motors */
// For DC motor
volatile unsigned char dutyCycle = 0x40; 	// set PWM = 50%
volatile unsigned char DC_cw = 0x04;		// DC clockwise
volatile unsigned char DC_ccw = 0x02;		// DC counter-clockwise
volatile unsigned char DC_vccstop = 0x00;  	// vcc stop DC motot


//for stepper
#define STEP1 0b00110000;
#define STEP2 0b00000110;
#define STEP3 0b00101000;
#define STEP4 0b00000101;
#define CLOCKWISE 1;
#define WIDDERSHINS -1;
char stepper_State;			// s1, s2, s3 ou s4
char CurrentPosition;		// AL0, Blk1, Whit2, STL3, Unknown4
char DesiredPosition;



/* Sensors */
// For optical sensor IO
volatile unsigned int Count_OptIO = 0x00;
volatile unsigned int Enter_Flag = 0;
// For inductive sensor IN
volatile unsigned int Count_IndIN = 0x00;
volatile unsigned int Inductive_Flag = 0;

// For optical sensor OR
unsigned int Count_OptOR = 0;
volatile unsigned int OR_Flag = 0;



// For reflective sensor RL ADC
volatile unsigned char ADC_resultH;
volatile unsigned char ADC_resultL;
volatile unsigned int ADC_result;
volatile unsigned int ADC_result_Flag=0;
volatile unsigned int ADC_min =  0xff; // 16_bit to save 10-bit ADC reflectness

// For optical sensor EX
volatile unsigned int Count_OptEX = 0;
volatile unsigned int Exit_Flag = 0;
// For Hall Effect sensor HE
unsigned int Hall_Flag = 0;

// Function declarations==============================================
void mTimer(); //timer1
void stepper_Home();
void stepperRotate();
int stepperSorting(int CurrentPosition, int DesiredPosition);



typedef struct cylinderMATERIAL{
	int inductive;		// 0 non ferrous, 1 ferrous
	int category;		// AL = 0; STL = 2; WPL = 3; BPL = 1; UNKnown = 4;

};
struct cylinderMATERIAL cylin[48];
/* Cylinder Info Storage */
// For reflective values of different materials
// AL = 0-250;	STL = 251-600;  WhPLT = 601-950;	BlPLT = 951-1024;
int AL_MAX = 150;

int STL_MAX = 500;

int WPL_MAX = 750;

int BPL_MAX = 1020;




// Main function======================================================
void main() 
{	
	
	cli();

	configIO();
	configInterrupts();
	configPWM(dutyCycle);
	configADC();

	sei();
	stepper_Home();
	DCMotorCtrl(0);

	while(1){
		if (Enter_Flag == 1){
			Enter_Flag = 0;
		}
		if (Inductive_Flag == 1){
			Inductive_Flag = 0;
		if (Count_OptIO >= 1)
			{
				cylin[Count_OptIO-1].inductive = 1;
			}
		}
        
	/*	if (ADC_result_Flag == 1){
		
			ADC_result_Flag = 0;
		}
		*/
		if (OR_Flag == 1){
			OR_Flag = 0;
		}
		if (Exit_Flag == 1)
		{	
			DCMotorCtrl(1);		// belt stop for 0.5s
			
			DesiredPosition = cylin[Count_OptEX-1].category;
			
		    CurrentPosition = stepperSorting(CurrentPosition, DesiredPosition);
			PORTC = CurrentPosition;
			mTimer(500);
			
		    Exit_Flag = 0;
		}
		DCMotorCtrl(0);

	}

}


/* ------------- Configurations ----------- */
/* ---------------------------------------- */

// Fini Mar 29, 3.25PM
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
	//TIMSK0 |= _BV(OCIE0A);
	OCR0A = duty_cyc;
}

// la Fini, Mar 29, 3PM

void configADC(){
	ADCSRA |= _BV(ADEN); 							// enable adc
	ADCSRA |= _BV(ADIE);							// enable interrupt of adc
	ADCSRA |= (_BV(ADPS2) | _BV(ADPS0));			// adc scaler division factor 32
	//ADCSRA |= _BV(ADSC)		// for the first conversion
	
	ADMUX |= _BV(ADLAR);							// set 10bit ADC value structure, ADCH[7,0] = ADC[9,2], ADCL[7,6] = ADC[1,0]
	ADMUX |= _BV(REFS0); 			// Vcc 3.3v Voltage reference with external capacitor on AREF pin
	ADMUX |= _BV(MUX0);								// channel select, ADC1

}

// la Fini, Mar 29, 3PM
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







/* --------------- Motors Ctrl ------------ */
/* ---------------------------------------- */

void DCMotorCtrl(char sysSTATE){
	switch (sysSTATE){
		case 0:
			PORTB = DC_cw;
			break;
		case 1:
			PORTB = DC_vccstop;
			break;
		/*case 2:
			buttonPressSTOP();
			break;
		case 3:
			buttonPressRAMPDOWN();
			break;*/
	}
}
// stepper go home
void stepper_Home(){
	

    while(Hall_Flag == 0){
			
			if (Hall_Flag == 0){
				PORTA = STEP1;
				mTimer(20);
				stepper_State = 1;
			}
			if (Hall_Flag == 0){
				PORTA = STEP2;
				mTimer(20);
				stepper_State = 2;
			}
			if (Hall_Flag == 0){
				PORTA = STEP3;
				mTimer(20);
				stepper_State = 3;
			}
			if (Hall_Flag == 0){
				PORTA = STEP4;
				mTimer(20);
				stepper_State = 4;
			}
			//stepperRotate(1,1);

		  
	}
	CurrentPosition = 1;
}

void stepperRotate(int steps, int direction) {
	int delay = 20;
	int i;
	int stepnum =stepper_State;
	for(i=0;i<steps;i++){
		if(direction == 1)  stepnum = ((stepnum % 4) + 1);
		if(direction == -1)  stepnum = ((stepnum - 1) % 4);
		if(stepnum == 0){ stepnum = 4;}
		switch(stepnum){
			case(1):
				PORTA = STEP1;
				mTimer(delay);
                stepper_State=1;
				break;
			case(2):
				PORTA = STEP2;
				mTimer(delay);
				stepper_State=2;
				break;
			case(3):
				PORTA = STEP3;
				mTimer(delay);
				stepper_State=3;
				break;
			case(4):
				PORTA = STEP4;
				mTimer(delay);
				stepper_State=4;
				break;
			default: break;
		}//switch
		if((i>=15) && ((steps - i) >= 15)) delay = 12; //acceleration
	
	}
} 


//stepper_position
int stepperSorting(int CurrentPosition, int DesiredPosition){

	int diff = (DesiredPosition - CurrentPosition);
	if((diff == 1) || (diff == -3)){
	stepperRotate(50, 1);
	}
	else if((diff == -1) || (diff == 3)){ 
	stepperRotate(50, -1);
	}
	else if((diff == 2) || (diff == -2)){
	stepperRotate(100, 1);
	}
	else
	{
	stepperRotate(0, 1);
	}
	CurrentPosition = DesiredPosition;

	return CurrentPosition;
}




/* ----------- Des for interrupts --------- */
/* ---------------------------------------- */
// For ADC conversion, RL, PF1, INTADC, la Fini Mar 29, 3.25PM
ISR(ADC_vect){					// ADC interrupt for reflecness conversion , PF1
	

	ADC_resultH = ADCH;
	ADC_resultL = ADCL;

	ADC_min = ADC_resultH;
	ADC_min = ADC_min<<2;
	ADC_result_Flag = 1;
		    
}



// ISRs
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
	
	Count_OptOR=Count_OptOR+1;
	OR_Flag = 1;
	ADC_min =  0xff;			// reset ADC_min value
	ADCSRA|= _BV(ADSC); 		// rising on INT2, start ADC conversion

			if(ADC_min <= AL_MAX)
			{
				cylin[Count_OptOR-1].category = 0; // Alluminum
			}
			else if(ADC_min <= STL_MAX)
			{
				cylin[Count_OptOR-1].category = 0; // Steel
			}
			else if(ADC_min <= WPL_MAX){
				cylin[Count_OptOR-1].category = 2; // White plastic
			}
			else if(ADC_min <= BPL_MAX){
				cylin[Count_OptOR-1].category = 2; // Blk plastic
			}
			else
				cylin[Count_OptOR-1].category = 4; // Unknown


	}

// For optical 3, EX, PD3, INT3
ISR(INT3_vect){
	Count_OptEX=Count_OptEX+1;
	struct cylinderMATERIAL cylin[48];

	/*if (Count_OptEX <= 48)
	{
		DesiredPosition = cylin[Count_OptEX-1].category;
		stepperSorting(CurrentPosition, DesiredPosition);
	}*/
	Exit_Flag = 1;
	
}


// For press buttom low to stop/resume the belt, PE4, INT4
ISR(INT4_vect){
	
	//DCMotorCtrl(1);		// Vcc stop the dc motor, PE4 for falling edge press button
	//mTimer(5);
}


ISR(INT5_vect){
	Hall_Flag = 1;
}



// For press button high, PE6
ISR(INT6_vect){
	//DCMotorCtrl(0);
}



// the timer1
void mTimer(int count){
	int i;
	i = 0;
	TCCR1B|=_BV(WGM12)|_BV(CS10);;
	OCR1A = 0x03e8;
	TCNT1 = 0x0000;
	//TIMSK1 = TIMSK1|0b00000010;
	TIFR1 |=_BV(OCF1A);
	while(i<count){
		if ((TIFR1 & 0x02) == 0x02){	
		TIFR1|=_BV(OCF1A);
		i++;
		}
	}
	return;
}
