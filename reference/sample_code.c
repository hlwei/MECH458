
// Header Files
#include <avr/interrupt.h>
#include <avr/io.h>

// Global Variables
volatile char STATE;

//Function Declarations
void stepperRotation(int dir, int steps, int speed);
void setHome();
void mTimer();

//Stepper Variables
int stepper[4] = {48, 6, 40, 5};
enum sPosition { sBlack, sAluminum, sWhite, sSteel};

//Material Variables
enum material {Black, Aluminum, White, Steel};

//Main Function
int main(){

	STATE = 0;

	// Disables all interrupts
	cli();	

	// Pinout setup
	DDRA = 0xFF; //stepper(0-5)
	DDRB = 0xFF; //dc motor(0-3), PWM(7)
	DDRC = 0xFF; //LEDs
	DDRD = 0x00; //buttons(0,1), sensors(2-6)
	DDRF = 0x00; //reflector sensor(1)
	
	//set all outputs to 0
	PORTA = 0x00;
	PORTB = 0x00;
	PORTC = 0x00;
	
	//Set up interrupts

	// Set up the Interrupt 0,3 options
	//External Inturrupt Control Register A - EICRA (pg 94 and under the EXT_INT tab to the right
	// Set Interrupt sense control to catch a rising edge
	EICRA |= _BV(ISC01) | _BV(ISC00);
	EICRA |= _BV(ISC31) | _BV(ISC30);

//	EICRA &= ~_BV(ISC01) & ~_BV(ISC00); /* These lines would undo the above two lines */
//	EICRA &= ~_BV(ISC31) & ~_BV(ISC30); /* Nice little trick */


	// See page 96 - EIFR External Interrupt Flags...notice how they reset on their own in 'C'...not in assembly
	EIMSK |= 0x09;

	// Enable all interrupts
	sei();
	
	//Set stepper to home position
	setHome();

	//Current Stepper Position
	enum sPosition currentPosition = sBlack;

	goto POLLING_STAGE;

	// POLLING STATE
	POLLING_STAGE: 
		PORTC = 0x0F;	// Indicates this state is active
		switch(STATE){
			case (0) :
				goto POLLING_STAGE;
				break;	//not needed but syntax is correct
			case (1) :
				goto MAGNETIC_STAGE;
				break; 
			case (2) :
				goto REFLECTIVE_STAGE;
				break;
			case (4) :
				goto BUCKET_STAGE;
				break;
			case (5) :
				goto END;
			default :
				goto POLLING_STAGE;
		}//switch STATE
		

	MAGNETIC_STAGE:
		// Do whatever is necessary HERE
		PORTC = 0x01; // Just output pretty lights know you made it here
		//Reset the state variable
		STATE = 0;
		goto POLLING_STAGE;

	REFLECTIVE_STAGE: 
		// Do whatever is necessary HERE
		PORTC = 0x02; // Just output pretty lights know you made it here
		//Reset the state variable
		STATE = 0;
		goto POLLING_STAGE;
	
	BUCKET_STAGE:
		// Do whatever is necessary HERE
		PORTC = 0x04;
		//Reset the state variable
		STATE = 0;
		goto POLLING_STAGE;
		
	END: 
		// The closing STATE ... how would you get here?
		PORTC = 0xF0;	// Indicates this state is active
		// Stop everything here...'MAKE SAFE'
	return(0);

}

/* Set up the External Interrupt 0 Vector */
ISR(INT0_vect){
	/* Toggle PORTC bit 0 */
	STATE = 2;
}

ISR(INT1_vect){
	/* Toggle PORTC bit 0 */
	STATE = 2;
}

ISR(INT2_vect){
	/* Toggle PORTC bit 0 */
	STATE = 2;
}

ISR(INT3_vect){
	/* Toggle PORTC bit 3 */
	STATE = 4;
}

// If an unexpected interrupt occurs (interrupt is enabled and no handler is installed,
// which usually indicates a bug), then the default action is to reset the device by jumping 
// to the reset vector. You can override this by supplying a function named BADISR_vect which 
// should be defined with ISR() as such. (The name BADISR_vect is actually an alias	 for __vector_default.
// The latter must be used inside assembly code in case <avr/interrupt.h> is not included.
ISR(BADISR_vect)
{
    // user code here
}



void stepperRotation(int dir, int steps, int speed){
	int currentStep = 0;
	switch(dir){
		case 0: //clockwise
			for(int i=0; i<steps; i++){
				currentStep++;
				if(currentStep>3){
					currentStep = 0;
				}
				PORTA = stepper[currentStep];
				mTimer(speed);
			}
		break;
		case 1: //counter-clockwise
			for(int i=0; i<steps; i++){
				currentStep--;
				if(currentStep<0){
					currentStep = 3;
				}
				PORTA = stepper[currentStep];
				mTimer(speed);
			}
		break;
	}
	return;
}/*stepperRotation*/

void setHome(){
	while((PIND & 0b01000000) == 0b01000000){
		stepperRotation(0, 4, 15);
	}
	return;
}/*setHome*/

void mTimer(int count){

	int i;
	i = 0;
	
	
	TCCR1B |=_BV(CS10);
	// WGM12 Wave Form Generation Mode
	TCCR1B |=_BV(WGM12);

	OCR1A = 0x03e8;
	
	//sets the initial timer to 0x0000
	TCNT1 = 0x0000;
	
	// TIMSK1 - Timer/ counter1 interrupt mask register
	// enables the output compare interrupt 
    //TIMSK1 = TIMSK1|0b00000010;

	// clear flag and reset timer
	TIFR1 |= _BV(OCF1A);

	while(i<count){

		if((TIFR1 & 0x02) == 0x02){
	
		TIFR1 |= _BV(OCF1A);

		i++;

		}
	}

	return;
}/*mTimer*/
