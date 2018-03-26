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
