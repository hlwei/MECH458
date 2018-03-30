
///2nd case 
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdlib.h>


// define the global variables that can be used in every function ==========
// For ADC
volatile unsigned char ADC_resultH; 
volatile unsigned char ADC_resultL; 
volatile unsigned int ADC_result_flag;

// pause and ramp down
volatile int pause_flag = 0;
volatile int ramp_flag = 0;

// DC motor
volatile int speed = 40;

//for stepper
#define STEP1 0b00000011;
#define STEP2 0b00011000;
#define STEP3 0b00000101;
#define STEP4 0b00101000;
#define CLOCKWISE 1
#define WIDDERSHINS -1
#define TURN_90 50
#define TURN_180 100
#define DELAY 20
volatile char init_stepper_flag = 0;


// Function declarations==============================================
void mTimer(); //timer1
void set0();   //set the stepper motot to initial position
void pwm();
void motor_forward();
void motor_brake();



// Main function======================================================
void main() {

	char d4, d7, pd;
	cli(); // disable all of the interrupt 
	
	DDRA = 0xFF;// Stepper
	DDRB = 0xFF;// PORTB<->DC motor
	DDRC = 0xFF;// set the PORTC and LEDs on board to display the ADC results
	DDRD = 0x00;// buttons
    DDRF = 0x00;// RL sensor->PORTF1

	// config the external interrupt 
	EIMSK |= (_BV(INT0)) | (_BV(INT5)); // enable INT0
	EICRA |= (_BV(ISC01)); // rising edge interrupt

    EICRB |=  _BV(ISC51);
    
	// config ADC
	ADCSRA |= _BV(ADEN);
	ADCSRA |= _BV(ADIE);
	ADMUX  |=  (_BV(REFS0))|(_BV(MUX0))|(_BV(ADLAR));

	// sets the Global Enable for all interrupts
	sei();

	//DC motor 
	TCCR0A|=_BV(WGM01)|_BV(WGM00)|_BV(COM0A1);
	TCCR0B|=_BV(CS01);
	TIMSK0|=_BV(OCIE0A);
	OCR0A = 0x80;// Control the PWM
	PORTB = 0x04;
    // initialize the ADC, start one conversion at the beginning ==========
	ADCSRA |= _BV(ADSC);
    
	//initilize the stepper
	stepper_Home();

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
//stepper flag
ISR(INT5_vect){
	init_stepper_flag =1;
}

void init_stepper(){
	PORTA = STEP1;
	stepnum = 1;
	stepper_position = 1;
	//stepper_on = 0;
}

//stepper home
void stepper_Home(){
	while(init_stepper_flag == 0){
		PORTA = stepperRotate(200, CLOCKWISE);
	}
}

//stepperRotate
void stepperRotate(int steps, int direction) {
	//stepper_on = 1;
	int delay = 20;
	int i;
	for(i=0;i<steps;i++){
		if(direction == CLOCKWISE)   stepnum = ((stepnum % 4) + 1);
		if(direction == WIDDERSHINS) stepnum = ((stepnum - 1) % 4);
		if(stepnum == 0) stepnum = 4;
		switch(stepnum){
			case(1):
				PORTA = STEP1;
				mTimer(delay);
				break;
			case(2):
				PORTA = STEP2;
				mTimer(delay);
				break;
			case(3):
				PORTA = STEP3;
				mTimer(delay);
				break;
			case(4):
				PORTA = STEP4;
				mTimer(delay);
				break;
			default: break;
		}//switch
		if((i<5) && (delay >= 10)) delay -= 2; //acceleration
		if(((steps - i) <= 5) && (delay <=20)) delay += 2; //deceleration
	}//for
	//stepper_on = 0;
} 

//stepper_position
void stepper_position(int new_position){
	int diff = (new_position - stepper_position);
	if((diff == 1) || (diff == -3)) stepperRotate(TURN_90, CLOCKWISE);
	else if((diff == -1) || (diff == 3)) stepperRotate(TURN_90, WIDDERSHINS);
	else if((diff == 2) || (diff == -2)) stepperRotate(TURN_180, CLOCKWISE);
	stepper_position = new_position;
}

//dc motor
void pwm(){
	OCR0A = speed;
}

// the timer1
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


// DC motor part＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝

#define FORWARD 0b00001000 //Commands for motor direction
#define REVERSE 0b00000100
#define BRAKE	0b00000000

void setMotorSpeed(char speed) {
	OCR0A = speed;
}

void setMotorFwd() {
	PORTC = (PORTC & 0xF0) | 0b00001000;
	PORTB = FORWARD;
}

void setMotorRev() {
	PORTC = (PORTC & 0xF0) | 0b00000001;
	PORTB = REVERSE;
}

void setMotorBrake() {
	PORTC = (PORTC & 0xF0) | 0b00000000;
	PORTB = BRAKE;
}

void setMotorSlide() {
	PORTC = (PORTC & 0xF0) | 0b00000110;
	PORTB = FORWARD;
	mTimer(100);
	PORTB = BRAKE;
}
