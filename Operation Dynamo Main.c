/*
 *	Full_Program_Revision_2.c
 *
 *	Created: 22/03/2014 2:23:14 PM
 *  Author: Benjamin
 *	Group: 15
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <stdlib.h>

#define CCW_LIM (PIND & (1<<PD0))
#define CW_LIM (PIND & (1<<PD1))
#define USER_ENABLE PD2
#define THROW PD3
#define SPEED0 PD4
#define SPEED1 PD5
#define STEP PD6
#define DIRECTION PD7

#define CONTROL PORTD

#define BUSY 1		// State Definitions
#define DONE 0
#define CW  0
#define CCW 1

//Global Variables

unsigned int InProgressStepCount;	// Turn step count
unsigned char TurnInProgress, StepDirection;	// Turn in progress indicator , Direction indicator.

int StepCount = 0;

ISR (TIMER0_COMPA_vect) {

	//  Continue to Toggle Clock until done
	if((CW_LIM != 0 && StepDirection == CW ) || (CCW_LIM != 0 && StepDirection == CCW ) || --InProgressStepCount == 0) { 		// Count transitions to zero
		TCCR0B = 0;				// Timer off until further notice.
		TIMSK0 &= ~(1<<OCIE0A);	// Disable interrupts
		CONTROL |= (1<<USER_ENABLE);	// Disable Drive
		TurnInProgress = DONE;
	}
}

void DelayTime(unsigned int Delay) { //  Delay = Approx 1mS * Delay
	int i;
	for(i = 0; i < Delay; i++) {
		TCCR1A = 0;
		TCCR1B = (1<<CS12) | (1<<CS10);  	// CLK/1024, No Waveform Generation
		OCR1A = 7;			//Approx 1mS
		TCNT1 = 0;
		TIFR1 = (1 << OCF1A);
		while ( !(TIFR1 & (1<<OCF1A)));
	}
	TCCR1B = 0;  	// Turn Timer Off
}


void AbortTurn_Defense(void) {
	
	TIMSK0 &= ~(1<<OCIE0A);	// Disable interrupts
	InProgressStepCount = 0;				// Clear Count
	TCCR0B = 0;				// Timer off until further notice.
	CONTROL |= (1<<USER_ENABLE);	// Disable Drive
	TurnInProgress = DONE;

}

void AbortTurn_Offense(void) {
	
	TIMSK0 &= ~(1<<OCIE0A);	// Disable interrupts
	InProgressStepCount = 0;				// Clear Count
	TCCR0B = 0;				// Timer off until further notice.
	CONTROL |= (1<<USER_ENABLE);	// Disable Drive
	TurnInProgress = DONE;

}

unsigned char Turn_Defense(short NoOfSteps, unsigned char MotorDirection) {

	if(TurnInProgress > 0)		// Fail out if already in progress
	return 1;
	
	StepDirection = MotorDirection;	// Save Direction
	
	if(MotorDirection == 1)			// Initial setup of Direction signals.

	CONTROL |= (1 << DIRECTION);
	else {
		CONTROL &= ~(1<<DIRECTION);
	}

	InProgressStepCount = NoOfSteps << 1;	// Initialize Counter
	
	TurnInProgress = BUSY;				// Make Busy

	TCCR0A = (1<<WGM01) | (1<<COM0A0);		// CTC Mode, Output Toggle

	TCCR0B =  (1<<CS02);	// Pre-scale 1/256

	OCR0A = 3; 				// Sets to 4mS pulse width : 8mS Period OCR0A = ( (period * Fio) / (2 * N) ) - 1
	//										OCR0A = ( (8mS * 8MHz) / (2 * 1024) ) - 1 = 30.25
	TCNT0 = 0; 					// Zero Counter
	CONTROL &= ~(1<<USER_ENABLE);	// Enable Drive

	TIMSK0 = (1<<OCIE0A);	// Interrupts take over from here.

	return 0;
}

unsigned char Turn_Offense(short NoOfSteps, unsigned char MotorDirection) {

	if(TurnInProgress > 0)		// Fail out if already in progress
	return 1;
	
	StepDirection = MotorDirection;	// Save Direction
	
	if(MotorDirection == 1)			// Initial setup of Direction signals.

	CONTROL |= (1 << DIRECTION);
	else {
		CONTROL &= ~(1<<DIRECTION);
	}

	InProgressStepCount = NoOfSteps << 1;	// Initialize Counter
	
	TurnInProgress = BUSY;				// Make Busy

	TCCR0A = (1<<WGM01) | (1<<COM0A0);		// CTC Mode, Output Toggle

	TCCR0B =  (1<<CS02);	// Pre-scale 1/256

	OCR0A = 78; 				// Sets to 4mS pulse width : 8mS Period OCR0A = ( (period * Fio) / (2 * N) ) - 1
	//										OCR0A = ( (8mS * 8MHz) / (2 * 1024) ) - 1 = 30.25
	TCNT0 = 0; 					// Zero Counter
	CONTROL &= ~(1<<USER_ENABLE);	// Enable Drive

	TIMSK0 = (1<<OCIE0A);	// Interrupts take over from here.

	return 0;
}

int Calibrate_Defense() {
	int Counter = 0;
	
	do {		// Rotate motor to CCW stop.
		while( Turn_Defense(1, CCW) == 1); // Rotate motor 1 step.
	} while (CCW_LIM == 0);
	do {		// Rotate motor to CW stop.
		while( Turn_Defense(1, CW) == 1); // Rotate motor 1 step.
		Counter++; // Count Steps of span.
	} while (CW_LIM == 0);
	
	eeprom_write_word(0x00,Counter);
	
	return Counter;
}

int Calibrate_Offense() {
	int Counter = 0;
	
	do {		// Rotate motor to CCW stop.
		while( Turn_Offense(1, CCW) == 1); // Rotate motor 1 step.
	} while (CCW_LIM == 0);
	do {		// Rotate motor to CW stop.
		while( Turn_Offense(1, CW) == 1); // Rotate motor 1 step.
		Counter++; // Count Steps of span.
	} while (CW_LIM == 0);
	
	eeprom_write_word(0x00,Counter);
	
	return Counter;
}

void LaunchBall()
{
	DelayTime(100);
	
	PORTD &= ~(1<<THROW);
	
	DelayTime(1000);
	
	PORTD |= (1<<THROW);
	
	DelayTime(1000);
}

void Offensive_Play(int TotalSteps, float ShotOrder[])
{

		for (int i = 0; i <= 0.61 * TotalSteps; i++)
		{
			while( Turn_Offense(1, CCW) == 1);
		}
		
		LaunchBall();
		
		for (int i = 0; i <= 0.16 * TotalSteps; i++)
		{
			while( Turn_Offense(1, CW) == 1);
		}
		
		LaunchBall();
			
		for (int i = 0; i <= .10 * TotalSteps; i++)
		{
			while( Turn_Offense(1, CCW) == 1);
		}
		
		LaunchBall();
}




int main () {


	DDRD = (1<<DIRECTION) | (1<<STEP) | (1<<USER_ENABLE) | (1<<THROW) | (1<<SPEED0) | (1<<SPEED1);	// Enable output pins. The remaining pins are input.
	
	PORTD = (1<<THROW) | (1<<SPEED0) | (0<<SPEED1);
	
	TurnInProgress = DONE;
	
	sei();

	float Shots[3] = {0.61,0.45,0.55};
	
	Offensive_Play(Calibrate_Offense(), Shots);
	

	while (1==1);

}


