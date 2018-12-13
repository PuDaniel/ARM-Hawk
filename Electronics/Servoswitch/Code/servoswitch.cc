//----------------------------------------------------------------------
// Title     : servoswitch.cc
//----------------------------------------------------------------------
// Function  : Switch between two servo inputs
// Circuit	 : Servoswitch
//----------------------------------------------------------------------
// Processor : ATtiny13
// Speed	 : 4.8 MHz
// Language  : C
// Date	     : 10.07.2016
// Version   : 1.0
// Author    : Daniel Pusztai
//----------------------------------------------------------------------
#define 	F_CPU 4800000
#include	<avr\io.h>
#include	<avr\interrupt.h>
#include	<avr\wdt.h>

#define		LED				3
#define		MODE_OUT		2
#define		MODE_IN			1

#define		MODE_RECEIVER	0
#define		MODE_CONTROLLER	1

// Variables
volatile uint8_t mode = 0;		// 0 = receiver input; 1 = controller input
volatile uint16_t timeout = 0;	// Timeout counter for MODE input
//----------------------------------------------------------------------
main ()
{
	cli();
	
	// Init external interrupts
	PCMSK |= (1 << MODE_IN);	// Enable pin change interrupt on IN_MODE pin
	GIMSK |= (1 << PCIE);
	
	// Init Timer0
	TIMSK0 |= (1 << TOIE0); 	// Enable overflow interrupt
	
	// Init GPIO
	DDRB |= (1 << MODE_OUT) | (1 << LED);
	
	// Init watchdog
	wdt_enable(0);
	
	sei();
	
	while(1)
	{
		//Mux signals
		if(mode == MODE_CONTROLLER)
		{
			// Mux controller input to servo output
			PORTB |= (1 << MODE_OUT);
		}
		else
		{
			// Mux receiver input to servo output
			PORTB &= ~(1 << MODE_OUT);
			
			PORTB |= (1 << LED);
		}
		
		// Make sure that no timeout is blocking the MODE input
		if(timeout++ >= 10000)
		{
			mode = MODE_RECEIVER;
			timeout = 0;
		}
		
		// Reset watchdog
		wdt_reset();
	}
}
//----------------------------------------------------------------------
// INT0 external interrupt ISR
//----------------------------------------------------------------------
ISR(PCINT0_vect)
{
	if(PINB & (1 << MODE_IN))
	{
		// High level, start timer from 0
		TCNT0 = 0;
		TCCR0B |= (1 << CS01) | (1 << CS00);
	}
	else
	{
		// Low level, read timer value and stop
		TCCR0B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00));
		
		if(TCNT0 < 105)
			mode = MODE_RECEIVER;
		else
			mode = MODE_CONTROLLER;
			
		timeout = 0;
		PORTB ^= (1 << LED);
	}
}
//----------------------------------------------------------------------
// Overflow interrupt for Timer0
//----------------------------------------------------------------------
ISR(TIM0_OVF_vect)
{
	// Overflow occured => illegal state, switch to receiver mode
	TCCR0B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00));
	
	mode = MODE_RECEIVER;
}
//----------------------------------------------------------------------