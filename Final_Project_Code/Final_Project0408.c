
// STAET MACHINE VERSION
// SAT APR 7, 12.50PM




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
volatile unsigned char dutyCycle = 0x80; 	// set PWM = 50%
volatile unsigned char DC_cw = 0x04;		// DC clockwise
volatile unsigned char DC_ccw = 0x02;		// DC counter-clockwise
volatile unsigned char DC_vccstop = 0x00;  	// vcc stop DC motot
unsigned char system_state=0;

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
volatile unsigned int ADC_min; // 16_bit to save 10-bit ADC reflectness

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

int pauseflag = 0;

typedef struct cylinderMATERIAL{
	int category;		// AL = 0; STL = 2; WPL = 3; BPL = 1; UNKnown = 4;
};
struct cylinderMATERIAL cylin[48];
/* Cylinder Info Storage */
// For reflective values of different materials

int AL_MAX = 67;

int STL_MIN = 423;

int WPL_MIN = 862;

int WPL_MAX = 870;

int BPL_MIN = 924;

// Cylinders count

unsigned char AL_SortedCount = 0b10000000;
unsigned char STL_SortedCount = 0b01000000;
unsigned char WPL_SortedCount = 0b00100000;
unsigned char BPL_SortedCount = 0b00010000;
unsigned char TotalSorted;

// state

char STATE;
//char POLLING_STAGE;

// Main function======================================================
void main() 
{	
	STATE = 0;
    
	cli();

	configIO();
	configInterrupts();
	configPWM(dutyCycle);
	configADC();

	sei();
	stepper_Home();
//while(1){
	goto POLLING_STAGE;

	POLLING_STAGE:
		DCMotorCtrl(0);			// start the motor
		switch(STATE){
			case (0):
				//DCMotorCtrl(0);	
				goto POLLING_STAGE;

				break;
			case (1):
				goto BUCKET_STAGE;
				break;
			case (2):
				goto PUASE_BUTTON;
				break;
			case (3):
				goto RAMPDOWN;
				break;
			case (4):
				goto END;
			default:
				goto POLLING_STAGE;

		}
//}

	BUCKET_STAGE:
		DCMotorCtrl(1);			// stop the belt

		DesiredPosition = cylin[Count_OptEX-1].category;

		CurrentPosition = stepperSorting(CurrentPosition, DesiredPosition);
		// Count Sorted Items	
		switch(CurrentPosition){
			case(0):
				STL_SortedCount++;
			case(1):
				BPL_SortedCount++;
			case(2):
				AL_SortedCount++;
			case(3):
				WPL_SortedCount++;
		}
		// PORTC = CurrentPosition;
		mTimer(400);
		//DCMotorCtrl(0);
		STATE = 0;	
		goto POLLING_STAGE;



	PUASE_BUTTON:

		

		DCMotorCtrl(1);			// 	stop the belt
		// Display sorted items
		PORTC = AL_SortedCount;
		mTimer(1000);
		PORTC = STL_SortedCount;
		mTimer(1000);
		PORTC = WPL_SortedCount;
		mTimer(1000);
		PORTC = BPL_SortedCount;
		mTimer(1000);
		//TotalSorted = AL_SortedCount + STL_SortedCount + WPL_SortedCount + BPL_SortedCount;
		//PORTC = (0xf0 + Count_OptOR - TotalSorted);
		STATE = 0;
		goto POLLING_STAGE;

		/*if (pauseflag == 0){
			goto POLLING_STAGE;
		}*/

	RAMPDOWN:
		//PORTC = 0xff;

	END:
		DCMotorCtrl(1);


	//DCMotorCtrl(system_state);

	// while(1){
	// 	if (Enter_Flag == 1){
	// 		Enter_Flag = 0;
	// 	}
	// 	if (Inductive_Flag == 1){
	// 		Inductive_Flag = 0;
	// 	if (Count_OptIO >= 1)
	// 		{
	// 			cylin[Count_OptIO-1].inductive = 1;
	// 		}
	// 	}
        
	// 	/*if (OR_Flag == 1){
	// 		OR_Flag = 0;
	// 	}*/
	// 	if (Exit_Flag == 1)
	// 	{	
	// 		DCMotorCtrl(1);		// belt stop for 0.5s
			
	// 		DesiredPosition = cylin[Count_OptEX-1].category;
			
	// 	    CurrentPosition = stepperSorting(CurrentPosition, DesiredPosition);
	// 		//PORTC = CurrentPosition;
	// 		mTimer(400);
			
	// 	    Exit_Flag = 0;
	// 	}
	// 	DCMotorCtrl(system_state);

	//}

}


/* ------------- Configurations ----------- */
/* ---------------------------------------- */

// Fini Mar 29, 3.25PM
void configIO(){
	/* IO Ports Definition */
	DDRA = 0xff;	// PORTA output, Stepper motor drive
	DDRB = 0xff;	// PORTB output, DC motor drive
	DDRC = 0xff;	// PORTC output, LEDs debugging
	DDRD = 0xF0;	// PORTD[0,3] input, interrupts
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
							// set 10bit ADC value structure, ADCH[7,0] = ADC[9,2], ADCL[7,6] = ADC[1,0]
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
		if((i>=4) && ((steps - i) >= 4)) delay = 8; //acceleration
	    //if((i<5) && (delay >= 6)) delay -= 2; //acceleration
		//if(((steps - i) <= 5) && (delay <=19)) delay += 2; //deceleration
	
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
	
	if (ADC < ADC_min){
		ADC_min = ADC;
	}

	if ((PIND & 0x04) == 4){
		ADCSRA |= _BV(ADSC);
	}
    else{
			if(ADC_min <= 75)
			{
				cylin[Count_OptOR-1].category = 2; // Alluminum
			}
			else if(ADC_min <= 700)
			{
				cylin[Count_OptOR-1].category = 0; // Steel
			}
			else if(ADC_min <= 920){
				cylin[Count_OptOR-1].category = 3; // White plastic
			}
			else if(ADC_min <= 1024){
				cylin[Count_OptOR-1].category = 1; // Blk plastic
			}
			else
				cylin[Count_OptOR-1].category = 4; // Unknown
		/*PORTC = ADC_min;
		PORTD = (ADC_min & 0xFF00) >> 3;
		mTimer(2000);*/

	}
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

	//DCMotorCtrl(1);
	//mTimer(1000);
	//PORTC = 0xf0;
}

// For optical 2, OR, PD2, INT2, la Fini
ISR(INT2_vect){					//  optical 2, PD2 
	
	Count_OptOR=Count_OptOR+1;
	ADC_min = 0xffff;
	//ADC_min =  0xff;			// reset ADC_min value
	ADCSRA|= _BV(ADSC); 		// rising on INT2, start ADC conversion
	
	}

// For optical 3, EX, PD3, INT3
ISR(INT3_vect){
	Count_OptEX = Count_OptEX +1 ;
	STATE = 1;		// goto BUTKET_STAGE
}


// For press buttom low to stop/resume the belt, PE4, INT4
ISR(INT4_vect){

			//STATE = 2;


			if (((PINA & 0x01) == 0) && pauseflag == 1){
			mTimer(20);
			if(){
			}
			STATE = 0;
			pauseflag = 0;
		}


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
