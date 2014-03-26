/*
 *	Full_Program_Revision_1.c
 *
 *	Created: 20/03/2014 12:57:54 PM
 *  Author: Benjamin
 *	Group: 15
 */ 
 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#define CCW_LIM (PIND & (1<<PD0))
#define CW_LIM (PIND & (1<<PD1))
#define USER_ENABLE PD2
#define THROW PD3
#define SPEED0 PD4
#define SPEED1 PD5
#define STEP PD6
#define DIRECTION PD7

#define DIR 	PD7		// Direction Signal to Driver Board
#define CLK 	PD6		// Step Clock Signal to Driver Board
#define ENABLE 	PD2		// Enable Signal to Driver Board
#define CONTROL	PORTD

#define SenseCW 	(PIND & (1 << PD1))	 // Counter clockwise limit sensor from driver board.
#define SenseCCW 	(PIND & (1 << PD0))	 // Clockwise limit sensor from driver board.

#define BUSY 1		// State Definitions
#define DONE 0
#define CW  0
#define CCW 1

// Global Variables

int RCnt;	// Turn step count
unsigned char TIP, Direction;	// Turn in progress indicator , Direction indicator.

int stepcount = 0;

ISR (TIMER0_COMPA_vect) {

	//  Continue to Toggle Clock until done
	if((SenseCW != 0 && Direction == CW ) || (SenseCCW != 0 && Direction == CCW ) || --RCnt == 0) { 		// Count transitions to zero
		TCCR0B = 0;				// Timer off until further notice.
		TIMSK0 &= ~(1<<OCIE0A);	// Disable interrupts
		CONTROL |= (1<<ENABLE);	// Disable Drive
		TIP = DONE;
		}
}

void AbortTurn(void) {
		
	TIMSK0 &= ~(1<<OCIE0A);	// Disable interrupts	
	RCnt = 0;				// Clear Count
	TCCR0B = 0;				// Timer off until further notice.
	CONTROL |= (1<<ENABLE);	// Disable Drive
	TIP = DONE;

}

unsigned char Turn(short NoOfSteps, unsigned char Dir) {

	if(TIP > 0)		// Fail out if already in progress
		return 1;
		
	Direction = Dir;	// Save Direction
	
	if(Dir == 1)			// Initial setup of Direction signals.

		CONTROL |= (1 << DIR);
	else {
		CONTROL &= ~(1<<DIR);
		}

	RCnt = NoOfSteps << 1;	// Initialize Counter
	
	TIP = BUSY;				// Make Busy

	TCCR0A = (1<<WGM01) | (1<<COM0A0);		// CTC Mode, Output Toggle

	TCCR0B =  (1<<CS02);	// Pre-scale 1/256

	OCR0A = 3; 				// Sets to 4mS pulse width : 8mS Period OCR0A = ( (period * Fio) / (2 * N) ) - 1
								//										OCR0A = ( (8mS * 8MHz) / (2 * 1024) ) - 1 = 30.25
	TCNT0 = 0; 					// Zero Counter 
	CONTROL &= ~(1<<ENABLE);	// Enable Drive

	TIMSK0 = (1<<OCIE0A);	// Interrupts take over from here.					

	return 0;
}


int main () {
	int Cntr;

	DDRD = (1<<DIR) | (1<<CLK) | (1<<ENABLE);	// Enable output pins. The remaining pins are input.
	TIP = DONE;
	sei();
	Cntr = 0;
	do {		// Rotate motor to CCW stop.
		while( Turn(1, CCW) == 1); // Rotate motor 1 step.
		} while (SenseCCW == 0);
	do {		// Rotate motor to CW stop.
		while( Turn(1, CW) == 1); // Rotate motor 1 step.
		Cntr++; // Count Steps of span.
		} while (SenseCW == 0);
		
		for (int i = 0; i <= .75 * Cntr; i++)
		{
			while( Turn(1, CCW) == 1);
		}

	eeprom_write_dword(0x00,Cntr); 	// write steps / 100
	while (1==1)
	{
		for (int i = 0; i < 0.5 * Cntr; i++)
		{
			while( Turn(1,CW) == 1);
		}
		
		for (int i = 0; i < 0.5 * Cntr; i++)
		{
			while ( Turn(1,CCW) == 1);

		}
	}

}
