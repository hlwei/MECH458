#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include "LinkedQueue.h"
#include "Timing.h"
#include "StepperMotor.h"
#include "PWM.h"
#include "DCMotor.h"
#include "ADC.h"
//--------------------------------
//used to test the status of a pin
//--------------------------------
enum PushButtonStatus
{
}typedef buttonStatus;
STATUS_HIGH = 0x00,
STATUS_LOW = 0x01
enum SystemStatus
{
       SYSTEM_PAUSED = 0x00,
       SYSTEM_ACTIVE = 0x01
}typedef systemStatus;
//-----------------------------
//forward function declarations
//-----------------------------
void SetupIO(void);
void SetupInterrupts(void);
void SetHomePosition(void);
void ProcessObject(void);
void Reset(void);
//----------------------------
//global variable declarations
//----------------------------
volatile buttonStatus pauseButton;
volatile buttonStatus rampDownButton;
volatile systemStatus status;
volatile dcMotorMode  dcMotor;
volatile int
volatile int
volatile int
volatile int
volatile char
//----------------
//threshold values
//----------------
volatile int
volatile int
upperReg;
lowerReg;
currentOptimalVal;
counter;
STATE;
 minWhPlstVal;
 maxWhPlstVal;
volatile int
volatile int
volatile int
volatile int
volatile int
volatile int
minBlPlstVal;
maxBlPlstVal;
minStVal;
maxStVal;
minAlVal;
maxAlVal;
//------------
//object array
//------------
int objectList[48];
volatile int head;
volatile int tail;
int          listLength;
//-----------------------
//stepper motor variables
//-----------------------
volatile trayPosition currentTrayPosition;
volatile Direction    currentStepperDirection;
int main() {
STATE = 0;
       //------------------------
       //variable initializations
       //------------------------
pauseButton
rampDownButton
status
currentOptimalVal = 0xFFFF;
upperReg
lowerReg
counter
= 0; = 0; = 0;
= STATUS_HIGH;
= STATUS_HIGH;
= SYSTEM_ACTIVE;
//-----------------------
//setup the stepper motor
//-----------------------
step[0] = 48;
step[1] = 6;
step[2] = 40;
step[3] = 5;
halfTurn = 100;
quarterTurn = 50;
//---------------------------------------------
//initialize reflective values from calibration
//---------------------------------------------
minWhPlstVal = 750;//WHITE PLASTIC
maxWhPlstVal = 922;
minBlPlstVal = 923;//BLACK PLASTIC
maxBlPlstVal = 1100;
minStVal     = 300;//STEEL
maxStVal     = 749;

minAlVal     = 1;//ALUMINUM
maxAlVal
= 299;
//-----------
//object aray
//-----------
listLength = 48;
Reset();
//-------------------------
//setup the reference point
//-------------------------
currentTrayPosition     = POSITION_BLACK;
currentStepperDirection = COUNTERCLOCKWISE_ROTATION;
cli(); // Disables all interrupts
//------------------
//setup IO diretions
//------------------
SetupIO();
//----------------
//setup interrupts
//----------------
SetupInterrupts();
//---------
//setup ADC
//---------
SetupADC();
//---------
//setup PWM
//---------
SetupPWM();
// Enable all interrupts
sei(); // Note this sets the Global Enable for all interrupts
SetHomePosition();
OCR0A = 0x78;//initial duty cycle.Change as necessary
OperateDCMotor(DC_MOTOR_ROTATE_CCW);
goto POLLING_STAGE;
// POLLING STATE
POLLING_STAGE:
       switch(STATE){
              case (0) :
                     //-----------------------------
                     //poll for user to pause system
                     //-----------------------------
                     if(((PINA & 0x01) == 0) && pauseButton == STATUS_HIGH)
                     {

                            pauseButton = STATUS_LOW;//might be for testing
                            Timer(20);//waitout 'contact bounce'
                            if(status == SYSTEM_ACTIVE)//pause system
                            {
                                   status  = SYSTEM_PAUSED;
                                   dcMotor = DC_MOTOR_BRAKE_VCC;
                                   //------------------------------
                                   //display the current object count
                                   //------------------------------
                                   PORTC = head;
}
                            else//resume system
                            {
                                   status  = SYSTEM_ACTIVE;
                                   dcMotor = DC_MOTOR_ROTATE_CCW;
                                   //------------------------------
                                   //display the current object count
                                   //------------------------------
                                   PORTC = 0;
                            }
                            OperateDCMotor(dcMotor);
                     }
                     else if(((PINA & 0x01) == 1) && (pauseButton == STATUS_LOW))
                     {
                            Timer(20);//waitout 'contact bounce'
                            pauseButton = STATUS_HIGH;
}
goto POLLING_STAGE;
break; //not needed but syntax is correct
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
       //Reset the state variable
       STATE = 0;
       goto POLLING_STAGE;
REFLECTIVE_STAGE:
       //Reset the state variable
       STATE = 0;
       goto POLLING_STAGE;

}
BUCKET_STAGE:
       //------------------------------------------------------
       //determine the type of object currently being processed
       //------------------------------------------------------
       ProcessObject();
       OperateDCMotor(DC_MOTOR_ROTATE_CCW);
       //--------------------------------------------------------
       //IF THE OBJECT COUNT IS AT ITS LIMIT (48) END THE PROGRAM
       //--------------------------------------------------------
       if(head == listLength ||
((rampDownButton == STATUS_LOW) && ((tail - head) == 0))) {
STATE = 5;
}
else
END:
       Timer(500);
       OperateDCMotor(DC_MOTOR_BRAKE_VCC);
       //------------------------------
       //display the final object count
       //------------------------------
       PORTC = head;
       //----------------------------------
       //clear the list and other variables
       //----------------------------------
       Reset();
return(0);
{ }
       STATE = 0;
goto POLLING_STAGE;
//---------------------------------------------------------------------------------------
-
//---------------
//ISR definitions
//---------------
//---------------------------------------------------------------------------------------
-
/*
       Set up the External Interrupt 0 Vector
       used to detect FALLING EDGE from ramp down push button
*/
ISR(INT0_vect){
       //-----------------------
       //for debouncing purposes
       //-----------------------
       Timer(5);
       if(((PIND & 0x01) == 0x00) && (rampDownButton == STATUS_HIGH))

{
} }
rampDownButton = STATUS_LOW;
/* Set up the External Interrupt 1 Vector
   used to detect RISING EDGE from optical sensor
   from sensor station 2
*/
ISR(INT1_vect){
       //-----------------------
       //for debouncing purposes
       //-----------------------
       //Timer(5);
       if((PIND & 0x02) == 2)
       {
              ADCSRA |= _BV(ADSC);
}
else
{
} }
/*
*/
ISR(INT2_vect)
{
       Timer(5);
       if((PIND & 0x04) == 0)
       {
              OperateDCMotor(DC_MOTOR_BRAKE_VCC); //interrupt
              STATE = 4;//go to the bucket stage
} }
//---------------------------------------------------------------------------------------
---------------
// If an unexpected interrupt occurs (interrupt is enabled and no handler is installed,
// which usually indicates a bug), then the default action is to reset the device by
jumping
// to the reset vector. You can override this by supplying a function named BADISR_vect which
// should be defined with ISR() as such. (The name BADISR_vect is actually an alias for __vector_default.
// The latter must be used inside assembly code in case <avr/interrupt.h> is not
included.
//---------------------------------------------------------------------------------------
---------------
ISR(BADISR_vect)
{
       //stop motor and let user know that something is wrong
    OperateDCMotor(DC_MOTOR_BRAKE_VCC);
       for(int i = 0; i < 4; i++)
STATE = 0;
   Set up the External Interrupt 2 Vector
used to detect FALLING EDGE from optical sensor
from sensor station 3

}
{
PORTC  = 0xFF;
Timer(100);
PORTC  = 0x00;
Timer(100);
}
PORTC  = 0xFF;
//---------------------------------------------------------------------------------------
-------------
//--------------------END OF ISR DEFINITIONS---------------------------------------------
-------------
//---------------------------------------------------------------------------------------
-------------
//---------------
//ADC definitions
//---------------
//---------------------------------------------------------------------------------------
-------------
ISR(ADC_vect)
{
       if((ADC) < currentOptimalVal)
       {
              currentOptimalVal = (ADC);
              upperReg          = (ADCH << 6);
              lowerReg          = ADCL;
}
       if((PIND & 0x02) == 2)//the object is in front of sensor
       {
              ADCSRA |= _BV(ADSC);//start a new ADC measurements b/c the objet is still
              //infront of the sensor
       }
       else if((PIND & 0x02) == 0)//the object is not in front of sensor
       {
              PORTC = lowerReg;//display ADCH portion
              PORTD = upperReg;//display ADCL portion
              //-----------------------
              //store optimal ADC value
              //-----------------------
              if(tail < listLength)
{
                     objectList[tail++] = currentOptimalVal;
              currentOptimalVal = 0xFFFF;
} }
}
//---------------------------------------------------------------------------------------
-------------
//--------------------END OF ADC DEFINITIONS---------------------------------------------
-------------

//---------------------------------------------------------------------------------------
//--------------------
//function definitions
//--------------------
//---------------------------------------------------------------------------------------
void SetupIO(void)
{
//---------------------------------
//used for user input/ push buttons
//---------------------------------
DDRA = 0x00;
//--------------------
//pin 0 - 3 used for interrupts.
//--------------------
DDRD = 0xC0;
//---------------------------------
//used for ADC with optical sensors
//---------------------------------
DDRF = 0x00;
//----------------------------------------
//just use as a display. Light up the LEDs
//----------------------------------------
DDRC = 0xFF;
//------------------------------------
//PORTE is for output to stepper motor
//------------------------------------
DDRE = 0xFF;
//----------------------------------------------------------
//PORT B is output. Used for output PWM and control dc motor
//----------------------------------------------------------
DDRB = 0xFF;
PORTB = 0x00;
return; }
void Reset(void)
{
       //-------------------
       //erase list contents
       //-------------------
       for(int i = 0; i < listLength; i++){objectList[i] = 0;}
       head = 0;
tail = 0;
return; }
void ProcessObject(void)
{
int steps =0;
objectType object = OBJECT_UNDEFINED;
       //-------------------------
       //DETERMINE THE OBJECT TYPE
       //-------------------------
       int val = objectList[head];

   if(objectList[head] >= minBlPlstVal)
   {
          object = PLASTIC_BLACK;
   }
   else if((objectList[head] < maxWhPlstVal) && (objectList[head] > minWhPlstVal))
   {
          object = PLASTIC_WHITE;
}
else if ((objectList[head] <= maxAlVal) && (objectList[head] > minAlVal)) {
          object = METAL_ALUMINUM;
   }
   else if ((objectList[head] <= maxStVal) && (objectList[head] >= minStVal))
   {
      object = METAL_STEEL;
   }
   //-------------------------------------------------
   //DETERMINE WHERE TO POSITION THE SORTING TRAY
   //BASED ON THE CURRENT OBJECT CURRENT TRAY POSITION
   //-------------------------------------------------
if(object == METAL_STEEL)
   {
            if(currentTrayPosition == PLASTIC_BLACK)
            {
            steps = quarterTurn;
                  currentStepperDirection = COUNTERCLOCKWISE_ROTATION;
            }
            else if(currentTrayPosition == PLASTIC_WHITE)
            {
            steps = quarterTurn;
                  currentStepperDirection = CLOCKWISE_ROTATION;
            }
            else if(currentTrayPosition == METAL_ALUMINUM)
            {
            steps = halfTurn;
                  currentStepperDirection = COUNTERCLOCKWISE_ROTATION;
}
else
{
steps = 0; }
         currentTrayPosition = POSITION_STEEL;
   }
   else if (object == METAL_ALUMINUM)
   {
           if(currentTrayPosition == PLASTIC_BLACK)
            {
            steps = quarterTurn;
                  currentStepperDirection = CLOCKWISE_ROTATION;
            }
            else if(currentTrayPosition == PLASTIC_WHITE)
            {
            steps = quarterTurn;
                  currentStepperDirection = COUNTERCLOCKWISE_ROTATION;
            }
            else if(currentTrayPosition == METAL_STEEL)

{
         steps = halfTurn;
               currentStepperDirection = COUNTERCLOCKWISE_ROTATION;
}
else
{
steps = 0; }
      currentTrayPosition = POSIITON_ALUMINUM;
}
else if (object == PLASTIC_WHITE)
{
        if(currentTrayPosition == PLASTIC_BLACK)
         {
              steps = halfTurn;
               currentStepperDirection = CLOCKWISE_ROTATION;
         }
         else if(currentTrayPosition == METAL_STEEL)
         {
              steps = quarterTurn;
               currentStepperDirection = COUNTERCLOCKWISE_ROTATION;
         }
         else if(currentTrayPosition == METAL_ALUMINUM)
         {
               steps = quarterTurn;
               currentStepperDirection = CLOCKWISE_ROTATION;
         }
else
{
steps = 0;
         }
      currentTrayPosition = POSITION_WHITE;
}
else if(object == PLASTIC_BLACK)
{
        if(currentTrayPosition == METAL_STEEL)
         {
              steps = quarterTurn;
               currentStepperDirection = CLOCKWISE_ROTATION;
         }
         else if(currentTrayPosition == PLASTIC_WHITE)
         {
              steps = halfTurn;
               currentStepperDirection = COUNTERCLOCKWISE_ROTATION;
         }
         else if(currentTrayPosition == METAL_ALUMINUM)
         {
              steps = quarterTurn;
               currentStepperDirection = COUNTERCLOCKWISE_ROTATION;
}
else
{
steps = 0;
         }
         currentTrayPosition = POSITION_BLACK;

}
}
else
{
} }
//LET USER KNOW THAT THERE IS AN ERROR. THE OBJECT TYPE IS UNDEFINED
//THE CALIBRATED BOUNDARIES
//FLASH SOME LED LIGHTS
for(int i = 0; i < 5; i++)
{
PORTC = 0xF0;
Timer(50);
PORTC = 0x00;
Timer(50);
//---------------------------------------
//point to the next index in object array
//---------------------------------------
head++;
//-----------------------------------
//rotate to the next sorting position
//-----------------------------------
RotateStepperMotor(currentStepperDirection,steps,15);
return;
void SetHomePosition(void)
{
       while((PIND & 0b00010000) == 0b00010000)
       {
              RotateStepperMotor(CLOCKWISE_ROTATION,1,20);
}
return; }
void SetupInterrupts(void)
{
//------------------------------------------------- //This is to set up interrupt #1 - ramp down pin D0 //detects whether user pressed ramp down button //------------------------------------------------- EICRA |= _BV(ISC01);
//-------------------------------------------------- //This is to set up interrupt #1 - optical sensor #2 //detects a rising edge //-------------------------------------------------- EICRA |= _BV(ISC10) | _BV(ISC11);
//-------------------------------------------------- //This is to set up interrupt #2 - optical sensor #3 //detects a falling edge //-------------------------------------------------- EICRA |= _BV(ISC21);

       // See page 96 - EIFR External Interrupt Flags...notice how they reset on their
       EIMSK |= 0b00000111;
return; }
//---------------------------------------------------------------------------------------
-------------
//-----------------------END OF FUNCTION DEFINITIONS-------------------------------------
-------------
A.2 ADC Setup Source Code
#include <avr/interrupt.h>
#include "ADC.h"
void SetupADC()
{
        // config ADC =========================================================
        // by default, the ADC input (analog input is set to be ADC0 / PORTF0
        ADCSRA |= _BV(ADEN); // enable ADC
        ADCSRA |= _BV(ADIE); // enable interrupt of ADC
        //Setup three interrupts -> ADC2: activates pins PF0,PF1,PF2
        ADMUX |= _BV(REFS0) | _BV(MUX0);
       ADCSRB |= 0x80;
return; }
A.3 DC Motor Source Code
#include <avr/io.h>
#include "DCMotor.h"
//--------------------------------------------------
//Description: Operates the DC motor in three modes:
//
//
//
//--------------------------------------------------
void OperateDCMotor(dcMotorMode mode)
{
{
switch(mode)
1) Rotate shaft in CW direction
         2) Rotate shaft in CCW direction
3) Brake/Stop motor
case DC_MOTOR_ROTATE_CW:
       PORTB = 0x02;
break;
case DC_MOTOR_ROTATE_CCW:
       PORTB = 0x01;
break;

{
}
case DC_MOTOR_BRAKE_VCC:
default:
       PORTB = 0x00;
break;
return; }
A.4 PWM Setup Source Code
#include "PWM.h"
#include <avr/interrupt.h>
//---------------------------------------------------------------------------------------
-------------------------
//Description: Sets up and applies fast PWM.
//             The prescaler value is set to cs/8 as of now. This sets the frequency of
the signal to approx 500Hz
//             The duty cycle must be a fractional value between 0.00 and 1.00
//---------------------------------------------------------------------------------------
-------------------------
void SetupPWM(void)
//STEP 1
//----------------
//set the WGM bits
//----------------
TCCR0A |= _BV(WGM00);
TCCR0A |= _BV(WGM01);
//STEP 3
TCCR0A |= _BV(COM0A1);
//STEP 4
//-------------------------
//Set the Clock Select bits
//-------------------------
//NOTE THAT THIS VALUE SHOULD BE ADJUSTED DURING TESTING //the prescaler value is set to cs/64 for now
TCCR0B |= _BV(CS01);
//STEP 5
//-----------------------------------------------------------------
//set the value of the output compare register using the duty cycle
//-----------------------------------------------------------------
OCR0A = 0x00;//ADC_result
return; }
A.5 Stepper Motor Source Code
#include <avr/io.h>

#include "StepperMotor.h"
//--------------------------------------------------------------------------------------- ------
//Description: Rotates a stepper motor in the CW and CCW directions.
//             For a 90 degree rotation, pass 50 to numSteps.
//             For a 180 degree rotation, pass 100 to numSteps.
//             It is recommended to pass 20 to delay for a 20 ms rest between consecutive
steps
//---------------------------------------------------------------------------------------
------
void RotateStepperMotor(Direction direction,int numSteps,int delay)
{
       const int MAX_INDEX = 3;
       const int MIN_INDEX = 0;
       switch(direction)
       {
              case CLOCKWISE_ROTATION:
                     for(int i = 0; i < numSteps; i++)
                     {
} break;
return; }
A.6 Timer Source Code
#include "Timing.h"
#include <avr/interrupt.h>
void Timer(int count)
{
}
} break;
int currentCount = 0;
currentStepperMotorIndex++;
if (currentStepperMotorIndex > MAX_INDEX)
{
currentStepperMotorIndex = MIN_INDEX;
}
PORTE = step[currentStepperMotorIndex];
Timer(delay);
case COUNTERCLOCKWISE_ROTATION:
       for(int i = 0; i < numSteps; i++)
       {
currentStepperMotorIndex--;
if(currentStepperMotorIndex < MIN_INDEX)
{
       currentStepperMotorIndex = MAX_INDEX;
}
PORTE = step[currentStepperMotorIndex];
Timer(delay);

TCCR1B |= _BV(CS10);//setup timer
/*
This will set the WGM bits to 0100
*/
TCCR1B |= _BV(WGM12);
/*
Set the compare register for 1000 cycles = 1ms
Timer currently works at 1MHz
*/
OCR1A = 0x03E8;
/*
Set the timer counter register to 0
*/
TCNT1 = 0x0000;
/*
Enable the output compare interrupt to be enabled
*/
//TIMSK1 = TIMSK1 | 0b00000010;
/*
Clear the timer interrupt flag and begin timing
*/
TIFR1 |= _BV(OCF1A);
while(currentCount < count)
{
       /*
       Test to see whether the Output Compare Match Flag has been set
       Note: This occurs when the value in TCNT1 matches the value in OCR1A
       */
       if((TIFR1 & 0x02) == 0x02)
       {
} }
return; }
