/* ------------- sub list ------------- */
void ADC_setup();
void PWM_setup(int duty_cyc);	// duty_cyc = 0x80;
void interrupts_setup();


void mTimer(int count);



void stepper_motor_init(); 		// stepper init 
void stepper_action(int stepper_state);		// stepper_state = 0,1,2,3 for AL, STL, BLAC_PL, WHIT_PL

void sys_state(int system_stat);		// conveyor belt start & pause controlled by press buttom interrupt
			/* void timer2Setup(); */ 



// to be done Sun morning 
void linked_queue();      // use the code given by patrick
ISR();    // interrupts define







/* -----------------  SUBROUTINES ----------------- */
void ADC_setup(){
	ADCSRA |= _BV(ADEN); 				// enable ADC
	ADMUX |= _BV(ADLAR) | _BV(REFS0);
	ADCSRA |= _BV(ADIE);				// enable interrupt of ADC
}

void PWM_setup(duty_cyc){
	TCCROA |= (_BV(COM0A1) | _BV(WGM01) | _BV(WGM00));
	TCCROB |= _BV(CS01);
	OCROA = duty_cyc;
}

void mTimer(int count){
	int i = 0;
	TCCR1B = (_BV(CS10) | _BV(WGM12));
	OCR1A = 0X03e8;
	TCNT1 = 0X00;
	TIFR1 = _BV(OCF1A);

	while(i < count){
		if ((TIFR1 & 0X02) == 0X02)
		{
			TIRF1 = _BV(OCF1A);
			i++;
		}
	}

}

void interrupts_setup(){
	EICRA |= _BV(ISC01);	// OI - init the linked-queue - falling edge
	EICRA |= _BV(ISC10);
	EICRA |= _BV(ISC11);	// OR - ADC conversion - rising edge
	EICRA |= _BV(ISC21);	// EX - stepper motor - falling edge
	EICRA |= _BV(ISC31);	// HE - reset the stepper count - falling edge
	EICRB |= _BV(ISC41);
	// need an interrupt for IN metals

}

