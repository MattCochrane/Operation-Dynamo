/*
 *	Operation Dynamo - Rev 3 - In Progress - Update 1
 *
 *	Created: 22/03/2014 2:23:14 PM
 *  Author: Benjamin
 *	Group: 15
 *
 *	Change List:
 *	Added Comments to all current code that did not have them
 *	Changed formatting to a more uniform design
 *
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <stdlib.h>

//Define I/O Pins
#define CCW_LIM (PIND & (1<<PD0))	//Is 0 when PD0 is 0. Not 0 when PD0 is 1. used to check the CCW limit.
#define CW_LIM (PIND & (1<<PD1))	//Is 0 when PD1 is 0. Not 0 when PD1 is 1. Used to check the CW limit.
#define USER_ENABLE PD2				//When set to 0 control of the motor is allowed. When 1 control of the motor is disengaged.
#define THROW PD3					//When 1 the solenoid is released. Set to 0 to lift solenoid and launch a ball.
#define SPEED0 PD4					//First half of the binary speed selection.		ActualSpeed		Speed0		Speed1
#define SPEED1 PD5					//Second half of the binary speed selection.		Slowest			0			0
#define STEP PD6					//When transitioned high to low, steps the motor.	Faster			0			1
#define DIRECTION PD7				//Choses direction of the steps CW = 0, CCW = 1.	Fastest			1			0
									//													Off				1			1
//Define I/O Registers
#define CONTROL PORTD	//Control ribbon is on PORTD
#define LASERS PORTC	//Laser Ribbon is on PORTC
#define MOBILE PORTB	//Mobile receiver is on PORTB

// State Definitions
#define BUSY 1	//Allows for easy implementation of the TurnInProgress function
#define DONE 0	//Allows for easy implementation of the TurnInProgress function
#define CW  0	//Defines CW for easy use
#define CCW 1	//Defines CCW for easy use

//Global Variables
unsigned int InProgressStepCount;	// Turn step count
unsigned char TurnInProgress, StepDirection;	// Turn in progress indicator , Direction indicator.

void DelayTime(uint32_t Delay);
/*
*		Holds the process at this point of the given time in ms
*/

uint8_t Turn_Offense(uint16_t NumberOfSteps, uint8_t MotorDirection);
/*
*	Turns the given amount of steps in the given direction.
*	This function turns at a rate safe for the quarterback.
*/

uint8_t Turn_Defense(uint16_t NumberOfSteps, uint8_t MotorDirection);
/*
*	Turns the given amount of steps in the given direction.
*	This function turns at a rate safe for the defender.
*/

void AbortTurn();
/*
*	Cancels the current turn, if any.
*/

uint16_t Calibrate_Offense();
/*
*	Moves the quarterback between the two limit sensors.
*	Counts the steps between the two limits and returns it.
*/

uint16_t Calibrate_Defense();
/*
*	Moves the defender between the two limit sensors.
*	Counts the steps between the two limits and returns it.
*/

void LaunchBall();
/*
*	Lifts the solenoid and delays for launch of the balls.
*/

int main () {
	DDRD = (1<<DIRECTION) | (1<<STEP) | (1<<USER_ENABLE) | (1<<THROW) | (1<<SPEED0) | (1<<SPEED1);	// Enable output pins. The remaining pins are input.
	PORTD = (1<<THROW) | (1<<SPEED0) | (0<<SPEED1); //Default output for the device
	
	TurnInProgress = DONE;	//tells the system no turn is in progress
	
	sei();	//Enable global interrupts
	
	int TotalSteps = Calibrate_Offense();	//moves the quarterback between the two limits. Return the amount of steps between these limits.

	while (1==1);	//Loop forever

}

void DelayTime(uint32_t Delay) {			//  Delay = Approx 1mS * Delay
	for(uint32_t i = 0; i < Delay; i++) {	//Loops until it reaches delay, 1ms for each loop
		TCCR1A = 0;							//Normal Mode
		TCCR1B = (1<<CS12) | (1<<CS10);  	// CLK/1024, No Waveform Generation
		OCR1A = 7;							//Approx 1mS
		TCNT1 = 0;							//Clear counter
		TIFR1 = (1 << OCF1A);				//Clear output compare flag
		while ( !(TIFR1 & (1<<OCF1A)));		//Loop until output compare flag is set
	}
	TCCR1B = 0;		// Turn Timer Off
}

uint8_t Turn_Offense(uint16_t NumberOfSteps, uint8_t MotorDirection) {
	if (TurnInProgress > 0) {	//Fail out if already turning
		return 1;
	}
	
	StepDirection = MotorDirection;	//Save direction to global variable
	
	if (MotorDirection == 1) {	//Sends the motor direction to the CONTROL Ribbon
		CONTROL |= (1<<DIRECTION);
		} else {
		CONTROL &= ~(1<<DIRECTION);
	}
	
	InProgressStepCount = NumberOfSteps << 1; //Multiplies by two, function runs twice per step
	
	TurnInProgress = BUSY;	//Set the status to busy
	
	TCCR0A = (1<<WGM01) | (1<<COM0A0);	//Sets CTC mode, toggles PD6 when output compare is reached
	TCCR0B = (1<<CS02);	//CLK/256 prescaler
	OCR0A = 77;	//Sets pulse width to 2.5 ms
	TCNT0 = 0;	//Clears the counter
	CONTROL &= ~(1<<USER_ENABLE);	//Enables the motor
	TIMSK0 = (1<<OCIE0A);	//Sets output compare interrupts to true
	
	return 0;	//	Breaks out of while loop
}

uint8_t Turn_Defense(uint16_t NumberOfSteps, uint8_t MotorDirection) {
	if (TurnInProgress > 0) {	//Fail out if already turning
		return 1;
	}
	
	StepDirection = MotorDirection;	//Save direction to global variable
	
	if (MotorDirection == 1) {	//Sends the motor direction to the CONTROL Ribbon
		CONTROL |= (1<<DIRECTION);
		} else {
		CONTROL &= ~(1<<DIRECTION);
	}
	
	InProgressStepCount = NumberOfSteps << 1; //Multiplies by two, function runs twice per step
	
	TurnInProgress = BUSY;	//Set the status to busy
	
	TCCR0A = (1<<WGM01) | (1<<COM0A0);	//Sets CTC mode, toggles PD6 when output compare is reached
	TCCR0B = (1<<CS02);	//CLK/256 prescaler
	OCR0A = 4;	//Sets pulse width to 0.1 ms
	TCNT0 = 0;	//Clears the counter
	CONTROL &= ~(1<<USER_ENABLE);	//Enables the motor
	TIMSK0 = (1<<OCIE0A);	//Sets output compare interrupts to true
	
	return 0;	//	Breaks out of while loop
}

void AbortTurn(void) {
	TIMSK0 &= ~(1<<OCIE0A);			// Disable interrupts
	InProgressStepCount = 0;		// Clear Count
	TCCR0B = 0;						// Timer off until further notice.
	CONTROL |= (1<<USER_ENABLE);	// Disable Drive
	TurnInProgress = DONE;			//Clears the TurnInProgress flag
}

uint16_t Calibrate_Offense() {
	uint16_t Counter = 0;
	
	do {									// Rotate motor to CCW stop.
		while( Turn_Offense(1, CCW) == 1);	// Rotate motor 1 step.
	} while (CCW_LIM == 0);
	
	do {									// Rotate motor to CW stop.
		while( Turn_Offense(1, CW) == 1);	// Rotate motor 1 step.
		Counter++;							// Count Steps of span.
	} while (CW_LIM == 0);
	
	eeprom_write_word(0x00,Counter);
	
	return Counter;
}

uint16_t Calibrate_Defense() {
	uint16_t Counter = 0;
	
	do {									// Rotate motor to CCW stop.
		while( Turn_Defense(1, CCW) == 1);	// Rotate motor 1 step.
	} while (CCW_LIM == 0);
	
	do {									// Rotate motor to CW stop.
		while( Turn_Defense(1, CW) == 1);	// Rotate motor 1 step.
		Counter++;							// Count Steps of span.
	} while (CW_LIM == 0);
	
	eeprom_write_word(0x00,Counter);
	
	return Counter;
}

void LaunchBall() {
	DelayTime(100);			//Wait for system to steady itself
	PORTD &= ~(1<<THROW);	//Lift the solenoid
	DelayTime(500);			//Hold open for 500 ms
	PORTD |= (1<<THROW);	//Lower solenoid
	DelayTime(500);			//Wait for ball to leave the system
}

ISR (TIMER0_COMPA_vect) {		
	//  Continue to Toggle Clock until done
	if((CW_LIM != 0 && StepDirection == CW ) || (CCW_LIM != 0 && StepDirection == CCW ) || --InProgressStepCount == 0) { 		// Count transitions to zero
		TCCR0B = 0;				// Timer off until further notice.
		TIMSK0 &= ~(1<<OCIE0A);	// Disable interrupts
		CONTROL |= (1<<USER_ENABLE);	// Disable Drive
		TurnInProgress = DONE;
	}
}
