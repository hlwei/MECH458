/*
########################################################################
# MILESTONE : Final Project
# PROGRAM : 6: Automated Sorting
# PROJECT : Lab5:
# GROUP : X
# NAME 1 : Owen, Anderberg, V00862140
# NAME 2 : Ben, Duong, V00839087
# DESC : Automated sorting of cylindrical objects using sensors, DC and Stepper motors
# DATA
# REVISED 2018-FEB-9
########################################################################
*/
#include <stdlib.h> // the header of the general purpose standard library of C programming language
#include <avr/io.h>// the header of i/o port
#include <avr/interrupt.h>
#include <math.h>
//#include "LinkedQueue.h"
#include "interrupt.h"

/*Function Declarations*/
void stepperControl(int steps,int *stepperPos,int *stepperIt);
void stepperHome(int *stepperPos,int *stepperIt);
void setupPWM(int motorDuty);
void setupISR(void);
void setupADC(void);
void motorControl(int s, uint8_t d);//accepts speed and direction:speed range (0->100), direction possibilities {0b11,0b10,0b01,0b00}

/*User Defines*/
#define highNibMask 0xF0
#define lowNibMask 0x0F
#define DC_REVERSE 0x02 	//dc motor clock-wise
#define DC_FORWARD 0x01	//dc motor counter-clockwise
#define DC_BRAKE 0x00 //dc motor brake
#define CONVEYOR_SPEED 40 //50 is maximum for sustainability
#define AL_REFLECTIVITY 200 //minimum reflectivity of aluminum
#define FE_REFLECTIVITY 500 //minimum reflectivity of steel
#define WH_REFLECTIVITY 950 //minimum reflectivity of white plastic
#define BL_REFLECTIVITY 970 //minimum reflectivity of black plastic

/*Global Variables*/

volatile unsigned int ADCResult; //8 bits: 0 => (2^9-1); stores result of ADC conversion
//volatile unsigned int systemFlag; //bits(4:0) = {ADCResultFlag,optExitFlag,opt2Flag,inductiveFlag,opt1Flag}
volatile unsigned int stepEarlyCount;
volatile unsigned int lowADC;
volatile unsigned int ADCFilterCount;
volatile unsigned int lowADCArray[8];
volatile unsigned char ADCCompleteFlag; //allows the ADC conversions to stop if no object is in front of RL sensor
volatile unsigned int OI_Count; //count of objects that have hit optical sensor 1 (OI)
volatile unsigned int RL_Count; //count of objects that have had their reflectivities quantified
volatile unsigned int OR_Count; //count of objects that have hit optical sensor 2 (OR)
volatile unsigned int EX_Count; //count of objects that have hit optical sensor 3 (EX)
//volatile unsigned char opt1Flag; //set when 1st optical sensor is triggered (OI sensor)
volatile unsigned char inductiveFlag; //an inductive flag is picked up
//volatile unsigned int opt2Flag; //set when 2nd optical sensor is triggered (OR sensor)
volatile unsigned char optExitFlag; //object is at end of conveyor
volatile unsigned char ADCResultFlag; //8 bits: 0 => (2^9-1); thats that ADC conversion is complete
volatile unsigned char HallEffect; //becomes set during stepper homing to know position
unsigned int stepperSigOrd[4] = {0b00110110,0b00101110,0b00101101,0b00110101};

/* Main Routine */
int main(int argc, char *argv[]){
	CLKPR = _BV(CLKPCE);/*initialize clock to 8MHz*/
	CLKPR = 0;
	/*User Variables*/
	int i=0x00; //solely used in for loops
	int stepperPosition = 0x00; //stepper position w.r.t. 360 degrees (circle); steps 0-200 => degrees 0-360
	int stepperIteration = 0x00;
	int stepperMovement = 0x00;
	uint8_t stepEarlyFlag = 0x00;
	int stepEarlyMovement =0x00;
	int tempEarlyType = 0;
	int Direction = 1;
	int tempType = 0;
	//int tempOI_Count=0;
	//int tempInd =0;
	uint16_t ADCAverage = 0; //needs to be able to hold a maximum of 0x2000
	uint8_t BL_Count = 0x00;
	uint8_t WH_Count = 0x00;
	uint8_t ST_Count = 0x00;
	uint8_t AL_Count = 0x00;
	//int OIOR_Count = 0x00; //count of objects between optical sensors 1 and 2
	//int OIEX_Count = 0x00; //count of objects between optical sensors 1 and 3 (Exit sensor)
	//int OREX_Count = 0x00; //count of objects between optical sensors 2 and 3 (Exit sensor)
	int RLEX_Count = 0x00; //count of objects that have had their reflectivity measured, but not reached sensor 3 (EX)
	uint8_t inductiveArray[64]= {0};
	uint8_t tempFerrous=0;
	//uint8_t falseInductFlag=0x00;
	//initialize structure to store material characteristics
	//--ODA: eliminate entire struct in favor of an inductive and type array
	typedef struct material{
		uint16_t reflectance; //10 bit value; --ODA ELIMINATE AS UNNEEDED
		int type; //black=0;;aluminum=50;white=100;steel=150
		uint8_t inductive; //states whether object is ferrous or not (0=>non-ferrous;1=>ferrous)
		}material;
	//initialize array of structures for storage	
	struct material materialArray[64];
	/*initializations*/
	cli(); //disable interrupts
	setupPWM(CONVEYOR_SPEED); //DC Motor PWM setup;
	setupISR();
	setupADC();
	timer1Init();
	timer2Init();
	timer3Init();
	/*Port I/O Definitions*/
	DDRA = 0xFF; /* Sets all pins on Port A to output: stepper motor control */
		/*stepper motor connections to MCU: PA5:0 = EN0, L1, L2, EN1, L3, L4*/
	DDRB = 0xFF; /*controls dc motor: PB7=PWM signal PB3:0={INA,INB,ENA,ENB}*/
	DDRC = 0xFF; //LEDs Debugging
	DDRD = 0xF0; //upper nibble for on-board bi-color LEDs, interrupts on lower nibble	PORTD3:0=INT3:0
	DDRE = 0x00; /*PE4=HallEffect for stepper*/
	DDRF = 0x00; /*PF1=ADC1 pin*/	
	sei(); //enable interrupts
	// PORTB &= 0b1110000; //apply Vcc brake to motor
	//PORTB |=0b1000; //start motor in specified direction
	/*initialize flags and counters*/
	//systemFlag=0x0000;
	OR_Count=0;
	RL_Count=0;
	OI_Count=0;
	EX_Count=0;
	//opt1Flag=0x00;
	//opt2Flag=0x00;
	inductiveFlag=0x00;
	optExitFlag=0x00;
	ADCResultFlag=0x00;	
	HallEffect=0x00; 
	stepEarlyCount=0x00;
	stepperHome(&stepperPosition,&stepperIteration); //home stepper
	motorControl(CONVEYOR_SPEED,DC_FORWARD);//conveyor forward (counter-clock-wise)
	while(1){
		if (inductiveFlag){ //triggered on a falling edge when a ferrous material is in front of inductive sensor
			inductiveFlag=0;
			inductiveArray[((OI_Count-1)&0b00111111)]=0x01;
			/*
			if (falseInductFlag==0x00){
				if (OI_Count) tempInd=inductiveArray[OI_Count-1];
				else tempInd=inductiveArray[63];
				if (OI_Count) inductiveArray[OI_Count-1]=0x01; //set temporary inductive array equal to 1 for object based on OI_Count
				else inductiveArray[63]=0x01; //special case occurs on roll-over of counters when OI_Count==0; occurs as we are minusing 1 from count
				tempOI_Count=OI_Count;
				falseInductFlag=0x01;
				TCCR3B |= _BV(CS30); //clock pre-scalar (clk/1); initialize clock counting
				TCNT3=0x00; //set timer equal to zero
				if ((TIFR3 & 0x01) == 0x01)TIFR3|=0x01; //if TOV3 flag is set to 1, reset to 0 by setting bit to 1 (confused?)
			} //because of the closeness of interrupts OI and IN sensor, reliance on OI_Count w.r.t. inductive, delay given
			if ((TIFR3 & 0x01) == 0x01){ //if counter has overflowed ~>8ms; time to allow OI_Count to change
				systemFlag&=0xFD; //reset flag; allow flag to reset again after 8ms
				TCCR3B&=0b11111000; //disable timer 3
				falseInductFlag=0x00; //reset flag
				if (tempOI_Count!=OI_Count){
					inductiveArray[(OI_Count-2)&0b00111111]=tempInd; //send the value you stole back to the falsely set array object; modulus of 64
					inductiveArray[(OI_Count-1)&0b00111111]=0x01; //set the actual current object to inductive=1; modulus of 64
				}	
			}
			*/
		}
		/*
		if(systemFlag&0x04){ //optical sensor 2 (OR)
			systemFlag&=0xFB; //reset flag
						
			ADCSRA |= _BV(ADSC); //initialize an ADC conversion
			startMeasureFlag=0x01;//allow ADC conversions to continue
		}
		*/
		if(optExitFlag){ //object has hit sensor at end of conveyor
			optExitFlag=0; //reset flag
			//corresponding positions (black=0;aluminum=50;white=100;steel=150)
			//if object type matches stepper location; do nothing...
			tempType=materialArray[EX_Count].type;
			stepperMovement=stepperPosition-tempType;
			if (stepperMovement){//if object type doesn't match stepper location; stop motor, move stepper, start motor
				PORTB &=0xF0; //Apply Vcc brake to motor
				//stepper rotation logic
				//if (abs(stepperMovement)<=100) do nothing;
				if (abs(stepperMovement)>100){
					if (stepperMovement<0) stepperMovement+=200;
					else stepperMovement-=200;
				}
				stepperControl(stepperMovement, &stepperPosition, &stepperIteration);//rotate stepper to proper location
				PORTB |=0b00001000; //start motor forwards
			}
			if (tempType==0)BL_Count += 0x01;
			else if (tempType==50)ST_Count += 0x01;
			else if (tempType==100)WH_Count += 0x01;
			else if (tempType==150)AL_Count += 0x01;
			RLEX_Count-=1;
			EX_Count+=1;
		}
		if(ADCResultFlag){ //If the minimum reflectivity has been reached for an object
			ADCResultFlag=0; //reset flag 
			ADCAverage=0;
			for(i=0;i<8;i++){
				ADCAverage+=lowADCArray[ADCFilterCount];
				ADCFilterCount++;
				ADCFilterCount&=0x07; //modulus of 8 with positive incrementing variables
			}
			ADCAverage>>=3; //division by 8 with chopping, not rounding
			//PORTC=ADCAverage;
			//PORTD=((ADCAverage&0x0300)>>3); //0b01100000 if true //green green
			////////--ODA NOTE: There isn't a point to storing reflectance, just store object type/////////
			materialArray[RL_Count].reflectance=ADCAverage;//value of oldADCResult is now maximum possible reflectivity and is added to struct array
			tempFerrous=inductiveArray[RL_Count]; //store whether object was ferrous or non-ferrous
			inductiveArray[RL_Count]=0x00; //reset inductive array to zero; otherwise, array will produce errors if more than 64 objects are sorted
			materialArray[RL_Count].inductive=tempFerrous;//inductivity of material stored; 1 for inductive; 0 for non-ferrous
			if(tempFerrous){ //object is metal: aluminum (light), steel (dark)
				if (ADCAverage<AL_REFLECTIVITY) materialArray[RL_Count].type=150;//object is aluminium
				else materialArray[RL_Count].type=50;//object is steel
				} else { //object is plastic: white (light), black (dark)
				if (ADCAverage<WH_REFLECTIVITY) materialArray[RL_Count].type=100;//object is white plastic
				else materialArray[RL_Count].type=0;//object is black plastic
			}
			RL_Count+=1;//add one to amount of objects that have had their reflectivities measured
			RLEX_Count+=1;
			ADCCompleteFlag=0x01; //set flag to tell system there is no ADC conversions occurring
		}
		/*
		if (systemFlag&0x20){//if PAUSE Button is pressed
			//print Black, White, Aluminium, and Steel Counts to screen and display how many objects are between optical sensor 2 and 3 (EX)
		}*/
		//// -ODA, may add too much processing which could reduce ADC Conversion accuracy; therefore, may need to add additional conditioning
		////e.g. if ((OREX_Count) && (ADCCompleteFlag=0x00)) //only allow stepper to move in advance if no ADC is occurring
		////or maybe... if ((OREX_Count) && (OIOR_Count==0)) //when no objects are between the first and second optical sensors
		if((RLEX_Count) && (ADCCompleteFlag)){//if there are objects between the OR and EX sensor and no ADC conversions are occurring 
			if(stepEarlyFlag==0){
				TCCR1B |= _BV(CS10); //clock pre-scalar (clk/1); 8ms per overflow; Starts timer
				TCNT1=0x0000; //set timer equal to zero
				if ((TIFR1 & 0x01) == 0x01)TIFR1|=0x01; //if TOV1 flag is set to 1, reset to 0 by setting bit to 1 (confused?)
				stepEarlyFlag=1;
				stepEarlyCount=0;
			}
			if (stepEarlyCount>=2){ //takes >=16ms; Note that stepEarlyCount is updated in ISR
				stepEarlyCount=0;
				tempEarlyType=materialArray[EX_Count].type;
				stepEarlyMovement=stepperPosition-tempEarlyType;
				if (abs(stepEarlyMovement)>100){
					if (stepEarlyMovement<0) stepEarlyMovement+=200;
					else stepEarlyMovement-=200;
				}
				if (stepEarlyMovement<0)Direction=-1;
				else if (stepEarlyMovement>0)Direction=1;
				else Direction=0;
				stepperIteration+=Direction;
				if(stepperIteration==4)stepperIteration=0;
				if(stepperIteration==-1)stepperIteration=3;
				PORTA=stepperSigOrd[stepperIteration];
				stepperPosition+=Direction;
				stepperPosition%=200;		
			}
		} else {
			TCCR1B&=0b11111000; //disable timer 1 
			stepEarlyFlag=0; //re-initialize 
		}
		//efficient modulus for counters; forces them to stay within 0->63 as struct array only has 64 places
		OI_Count &= 0b00111111;//modulus of 64
		RL_Count &= 0b00111111;
		OR_Count &= 0b00111111;
		EX_Count &= 0b00111111;
	}
	return (0); //This line returns a 0 value to the calling program
	// generally means no error was returned
}

/*function allows control of direction and quantity of steps to */
void stepperControl(int steps,int *stepperPos, int *stepperIt){
	/*function variable declarations*/
	int i=0; //step quantity
	int k=0; //timer counter
	uint8_t maxDelay = 15; //20ms corresponds to 50 steps per second
	uint8_t minDelay = 7; //5ms corresponds to 200 steps per second; or 1 revolution per second
	uint8_t differential = maxDelay - minDelay;
	uint8_t delay = maxDelay;
	int PORTAREGSet = *stepperIt;
	int DIRECTION = 1;
	uint16_t absSteps = abs(steps); //compute absolute value now to save computations in "for" loop
	if(absSteps<(differential*2)){ //if there isn't enough time for stepper to fully ramp up to full speed
		minDelay=maxDelay-absSteps/2;
		differential = maxDelay - minDelay;
	}
	//determine direction 
	if (steps > 0) DIRECTION = 1;// positive or clock-wise
	else if (steps < 0) DIRECTION = -1; //negative or counter-clock-wise	
	else DIRECTION=0;
	/*perform one stepper cycle before "for" loop so there is no wasted delay at
	beginning or end of stepper motion*/
	PORTAREGSet+=DIRECTION;
	if(PORTAREGSet==4)PORTAREGSet=0;
	if(PORTAREGSet==-1)PORTAREGSet=3;
	TCCR1B &= 0b11111000; //disable timer1; needed due to automated counter in ISR that may cause missed steps
	TCCR2B |= _BV(CS20) | _BV(CS21); //clock pre-scalar (clk/32)
	TCNT2=0x00; //set timer equal to zero; note timer is already counting based on clock prescalar
	if ((TIFR2 & 0x01) == 0x01)TIFR2|=0x01; //if TOV2 flag is set to 1, reset it to zero
	PORTA = stepperSigOrd[PORTAREGSet];//initialize first step
	for(i=2;i<=absSteps;i++){	
		//ramp up
		if((absSteps-i) > (differential+1)){ //the "added" one causes it to slow down one step early
			if(delay>minDelay)delay -= 1;
			else delay = minDelay;
		} else { //ramp down if the amount of steps left are less than the differential between max and min delays
			if(delay<maxDelay)delay += 1;
			else delay = maxDelay;
		}
		/*determine direction and then iterate through stepper signals in correct direction*/
		PORTAREGSet+=DIRECTION;
		if(PORTAREGSet==4)PORTAREGSet=0;
		if(PORTAREGSet==-1)PORTAREGSet=3;
		k=0; //reset counter for timer
		while (k<delay){ //iterate through given count
			if ((TIFR2 & 0x01) == 0x01){ //if overflow has occurred in counter
				TIFR2|=0x01; //reset overflow flag by writing a 1 to TOV2 bit;equivalent => TIFR2 |= _BV(TOV2)
				k++;
			}
		}
		PORTA = stepperSigOrd[PORTAREGSet];//move stepper after first delay
	}
	TCCR2B&=0b11111000; //disable timer 2
	//re-enable timer 1 and re-initialize counter so the next early step doesn't occur until 16ms later, not instantly
	TCCR1B |= _BV(CS10); //clock pre-scalar (clk/1); 8ms per overflow; Starts timer1
	TCNT1=0x0000; //set timer equal to zero
	if ((TIFR1 & 0x01) == 0x01)TIFR1|=0x01; //if TOV1 flag is set to 1, reset to 0 by setting bit to 1 (confused?)
	stepEarlyCount =0; //reset counter for timer1
	*stepperIt=PORTAREGSet;
	//*stepperIt=stepperSigOrd[(CURRENT_ITERATION+DIRECTION*(i-1))%4]; //set value of current iteration to variable address
	*stepperPos += steps;
	*stepperPos %= 200; //represents 200 (0->199) steps of stepper positioning in a circle
	return; //returns nothing
}
////--ODA: CHANGE SO NO INTERRUPT IS USED FOR HALL EFFECT, simply check for voltage on an input pin
void stepperHome(int *stepperPos, int *stepperIt){
	uint8_t delay = 30; //20ms corresponds to 50 steps per second
	int i=0;
	int x=0;
	uint8_t offset=2; //arbitrary at this point
	uint8_t DIRECTION=1; //1 for clockwise, -1 for counter-clockwise
	PORTA=0x00;
	while (!HallEffect){
		PORTA = stepperSigOrd[i];
		mTimer2(delay);
		i++;
		if (i==4)i=0;
	}
	i--;
	HallEffect=0x00;
	EIMSK&=0b10111111;//disable hall effect sensor interrupt (INT6)
	/*Insert code here to compensate for offset --ODA CURRENTLY CAUSES MISSTEP... WHY?*/
	for (x=0;x<offset;x++){
		i+=DIRECTION;
		if (i==4)i=0;
		if (i==-1)i=3;
		PORTA = stepperSigOrd[i];
		mTimer2(delay);
	}
	*stepperIt = i;//modulus is heavy in terms of computation, but doesn't matter in this function
	//PORTA = stepperSigOrd[i];
	*stepperPos=0; //base stepper position (on black)
}
/*initializing the dc motor*/
void setupPWM(int motorDuty){
	uint8_t dutyCycle = 0;
	/*DC MOTOR PWM SETUP (runs conveyor)*/
	TCCR0A |= _BV(WGM00) | _BV(WGM01) | _BV(COM0A1); /*set to Fast PWM; OCRx updated at TOP; TOV set on MAX; Clear OC0A on Compare Match, set OC0A at TOP*/
	TCCR0B |= _BV(CS01) | _BV(CS00);//Set clock pre-scalar (8MHz*1/64): 488Hz measured on PB7*
	//TCCR0B &= 0b11111101;
	dutyCycle = motorDuty*2.55;
	OCR0A = dutyCycle;//set duty cycle/start motor
	PORTB &= 0xF0; //Apply Vcc brake to conveyor
}
void setupISR(void){
	/*INT(7:4) => PE(7:4); INT(3:0) => PD(3:0)*/
	//Ex: rising edge on INT2: EICRA |= _BV(ISC21) | _BV(ISC20);
	//Ex: falling edge on INT2: EICRA |= _BV(ISC21);
	//see ISR routines for 
	EIMSK |=0b01011111; //initialize INT6,4:0
	EICRA |= 0b11101110; //rising edge triggers for INT1 (OI) and INT3 (OR); falling edge detection on INT2 (IN) and INT4 (EX)
	EICRB |= 0b00100010; //active low for INT6 and INT4
}
void setupADC(void){
	ADCSRA |= _BV(ADEN) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS0); //adc scalar = 32;
	ADMUX |= _BV(REFS0) | _BV(MUX0); //AVcc reference (3.3V);read from ADC 1;output left-adjusted
	ADMUX &= 0b11100001; //reading from PF1 (ADC1); ADC0 works, but MCU has thermistor on pin...
}
void motorControl(int s, uint8_t d){//note that DC motor driver expects inverted bits
	uint8_t dutyCycle = 0;
	static uint8_t oldDirection;
	if((oldDirection & 0b00000011) != d){ //if current direction doesn't match new direction
		PORTB &= 0b11110000; //apply Vcc Brake
		PORTB |= ((~d & 0b11) << 2); //start motor in specified direction
		oldDirection=d;
	}
	dutyCycle = s*2.55;
	OCR0A = dutyCycle;//set duty cycle
}

/**********INTERRUPT SERVICE ROUTINES**********/
/*Button interrupt for emergency: shut-off dc motor, disable stepper, shut off, ensure nothing can be turned on*/
ISR(INT0_vect){ // on PD0; active low KILL SWITCH
	PORTB &= 0b11110000; //stop motor by applying Vcc break
}
/*sensor 1: OI: 1st Optical-Inductive-Near Inductive sensor*/
ISR(INT1_vect){ // on PD1; active low; triggered on rising-edge
	//systemFlag|=0x01;//opt1Flag=0x01;
	OI_Count+=1;
}
/*sensor 2: IN: Inductive sensor*/
ISR(INT2_vect){ //on PD3; active low; triggered on falling-edge
	inductiveFlag=0x01;
}
/*sensor 3: OR: 2nd Optical-Reflective-Near Reflective sensor*/
ISR(INT3_vect){ // on PD2; active high; triggered on rising-edge
	//systemFlag|=0x04;//opt2Flag=0x01;
	lowADC=0xFFFF;
	ADCSRA|= _BV(ADSC); //trigger ADC (i.e. begin ADC conversion)
	OR_Count+=1;	
	ADCCompleteFlag=0x00; // tell system ADC conversions are occurring
}
/*sensor 5: EX: 3rd Optical-Near exit of conveyor*/
ISR(INT4_vect){ //on PE4; active low; triggered on falling-edge
	optExitFlag=0x01;
}
/*ADC ISR: triggered when ADC is completed*/
ISR(ADC_vect){
	if (lowADC>ADC){ //if ADC result is still decreasing (i.e. if object's reflectivity is increasing)
		lowADC=ADC; //ADC holds the entire 10 bit value in a 16bit variable; lowADC set for future comparison
		lowADCArray[ADCFilterCount]=lowADC;
		ADCFilterCount++; //increment array location being set
		ADCFilterCount&=0b00000111; //modulus of 8
		//highByteADC=ADCH;
		//lowByteADC=ADCL;
	}
	if ((PIND&0b00000100)==0b00000100) ADCSRA|= _BV(ADSC); //if there is still an object keep initializing ADC conversions
	else ADCResultFlag = 1;
}
/*sensor 6: HE: Hall Effect sensor; used for homing stepper*/
ISR(INT6_vect){ //on PE6; Active low for hall effect sensor 
	HallEffect=0x01;
}
//timer 1 overflow flag; enabled through sei();
///Cant get to work yet
ISR(TIMER1_OVF_vect){
	stepEarlyCount+=1;
	TIFR1|=0x01;
}







