#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdlib.h>

//for stepper
#define STEP1 0b00110000;
#define STEP2 0b00000110;
#define STEP3 0b00101000;
#define STEP4 0b00000101;
#define CLOCKWISE 1;
#define WIDDERSHINS -1;
char init_stepper_flag;
char stepper_State;			// s1, s2, s3 ou s4


char CurrentPosition;		// AL0, Blk1, Whit2, STL3, Unknown4
char DesiredPosition;


// Function declarations==============================================
void mTimer(); //timer1
void stepper_Home();
void stepperRotate();
void stepperSorting(int CurrentPosition, int DesiredPosition);

// Main function======================================================
void main() 
{
    init_stepper_flag = 0;
	int stepnum = 1;
	cli();
	DDRA = 0xFF;// Stepper
    DDRE = 0x00;
	DDRC = 0xFF;
    EIMSK |= _BV(INT5); // enable INT5
    EICRB |= _BV(ISC51);
	sei();
	stepper_Home();
	PORTC = stepper_State;
	mTimer(1000);

	

/*
	stepperRotate(50, 1);
	PORTC = 0x0f;
	mTimer(1000);
	stepperRotate(50, -1);
	PORTC = 0x01;
	mTimer(1000);
	stepperRotate(100, 1);
	PORTC = 0x03;
	mTimer(1000);
	stepperRotate(100, -1);
	PORTC = 0x0f;
	mTimer(1000);
*/
 int iter = 0;
for (iter=0;iter<5;iter++)
{
	CurrentPosition=0;
	DesiredPosition=1;
	stepperSorting(CurrentPosition, DesiredPosition);
	mTimer(1000);

	DesiredPosition=2;
	stepperSorting(CurrentPosition, DesiredPosition);
	mTimer(1000);

	DesiredPosition=3;
	stepperSorting(CurrentPosition, DesiredPosition);
	mTimer(1000);

	DesiredPosition=3;
	stepperSorting(CurrentPosition, DesiredPosition);
	mTimer(1000);

	}


	
}

//stepper flag
ISR(INT5_vect){
	init_stepper_flag = 1;
	PORTC = 0xff;
}

// stepper go home
void stepper_Home(){
	

    while(init_stepper_flag == 0){
			
			if (init_stepper_flag == 0){
				PORTA = STEP1;
				mTimer(20);
				stepper_State = 1;
			}
			if (init_stepper_flag == 0){
				PORTA = STEP2;
				mTimer(20);
				stepper_State = 2;
			}
			if (init_stepper_flag == 0){
				PORTA = STEP3;
				mTimer(20);
				stepper_State = 3;
			}
			if (init_stepper_flag == 0){
				PORTA = STEP4;
				mTimer(20);
				stepper_State = 4;
			}
			//stepperRotate(1,1);

		  
	}
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
	
	}//for
} 


//stepper_position
void stepperSorting(int CurrentPosition, int DesiredPosition){

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
	CurrentPosition = DesiredPosition;
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
