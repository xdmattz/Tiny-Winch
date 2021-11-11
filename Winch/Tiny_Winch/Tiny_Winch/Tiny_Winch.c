/*
 * Tiny_Winch.c
 *
 * Created: 12/1/2014 11:19:27 AM
 *  Author: dmatthews
 */ 

/*
 * RC_Switch.c
 *
 * Created: 11/1/2014 2:51:07 PM
 *  Author: Dans i7
 */ 

#define F_CPU 1000000UL
#define LED1 3	// bit position of the LED
#define LED2 4	// bit position of the LED
#define SIG_IN 0 // Signal input of RC Receiver
#define TESTBIT 1
#define ADC_IN	2
#define MOTOR1	3
#define MOTOR2	4

// GOIOR0 flags - for fast testing of states
#define DIRECTION		0	// indicates the current direction of the motor
#define TIME_TICK_40mS	1	// a 40ms time tick for keeping track of things - set by Timer0
#define OVER_CURRENT	2	// indicates that over current has occurred.
#define SWITCH_POS		3
#define TIMEOUT			4	// timeout indicator


// these values are mostly determined empirically...
#define ADC_THRESHOLD 140	// reference is 1.1V so output is (ADC_THRESHOLD/1024) * 1.1V. 0.25 Ohm sense resistor trip at 600mA current
#define RX_TIMEOUT	11		// number of counts to determine if the RC Receiver is not receiving 40 ms per count
#define DELAY_TIMEOUT 8		// 1/2 sec delay count timeout value. 320ms really - 40ms per count
#define SWITCH_THRESHOLD 93 // RC servo timer position threshold - should be 62 counts/ms or 16us per tick
#define TURN_ON_DELAY 100	// 100 ms turn on delay

// a couple of useful macros for setting and clearing bits in an IO PORT
#define sbi(sfr,bit) sfr |= _BV(bit)
#define cbi(sfr,bit) sfr &= ~(_BV(bit))

// #define dir_FWD (sbi(GPIOR0,DIRECTION))
// #define dir_REV (cbi(GPIOR0,DIRECTION))

#define SetTestBit() (PORTB |= _BV(TESTBIT))
#define ClearTestBit() (PORTB &= ~(_BV(TESTBIT)))
#define ToggleTestBit() (PINB |= _BV(TESTBIT))

#define ADC_On()	(ADCSRA |= 0xC0)	// set the ADEN and ADSC bits - enable the ADC and start the first conversion
#define ADC_Off()	(ADCSRA &= 0x3f)	// clear the ADEN and ADSC bits - turn off the ADC

#include "Tiny_Winch.h"

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

volatile uint8_t S_count;
volatile uint8_t delay_count;
volatile uint8_t No_RX_Count;

void (*state_ptr)(void);


// the Pin change interrupt PCINT0 is used to measure
// the 1 - 2 ms pulse width of the servo control signal
ISR(PCINT0_vect)
{
	if(bit_is_set(PINB,SIG_IN))	// rising edge detect
	{
		// start the timer
		TCNT1 = 0x00;
		TCCR1 = 0x05;	// set the prescaler to 16 - this should give about 62 counts per ms 
		// set the test bit
		No_RX_Count = 0;	// reset the RX Counter
//		SetTestBit();
	}
	else
	{
		// stop the timer
		TCCR1 = 0;
		if(TCNT1 > SWITCH_THRESHOLD)
		{
			sbi(GPIOR0, SWITCH_POS);
		}
		else 
		{
			cbi(GPIOR0, SWITCH_POS);
		}
//		ClearTestBit();
	}
}


// TIMER 0 ISR is for the 100ms timer
ISR(TIM0_COMPA_vect)
{
	GPIOR0 |= _BV(TIME_TICK_40mS);	// set the time tick for 100ms timer
}

ISR(ADC_vect)
{
	uint16_t Result;
	
//	ToggleTestBit();
	
	Result = ADC;
	// get the ADC result
	if (Result > ADC_THRESHOLD)		// compare to the threshold
	{
		SetTestBit();
		sbi(GPIOR0,OVER_CURRENT);		// if greater than, set the OVER_CURRENT flag
	}
	else 
	{
		ClearTestBit();	
	}


}

int main(void)
{
//	uint8_t time_count1 = 0;
	setup();
	
    while(1)
    {
		if(bit_is_set(GPIOR0,TIME_TICK_40mS))
		{
			GPIOR0 &= ~(_BV(TIME_TICK_40mS));	// clear the time tick flag

			if(delay_count < DELAY_TIMEOUT)		// 1/2 second timer
			{
				delay_count++;	// increment the delay count
			}
			else
			{
				sbi(GPIOR0,TIMEOUT);	// set the timeout flag = 1/2 sec has occurred
			}
			if (No_RX_Count > RX_TIMEOUT)
			{
				Motor_Off();
				// if we get here the RC receiver must be off so change the state depending on the last known SWITCH_POS
				if (bit_is_set(GPIOR0,SWITCH_POS))
				{
					state_ptr = &(State1A);	// if forward
				} else
				{
					state_ptr = &(State1);	// if reverse
				}
			} 
			else
			{
				No_RX_Count++;
			}
		}
		// run the state machine
		(*state_ptr)();
		// I could actually sleep here!
		sleep_enable();
		sleep_cpu();
		sleep_disable();
		// _delay_ms(250);
    }
}

void setup(void)
{
	DDRB = _BV(MOTOR1) | _BV(MOTOR2) | _BV(TESTBIT);	// set the LED to output
//	PORTB = _BV(LED1);
	PORTB = 0;	// make sure everything is off
	
	// setup the pin change interrupt
	PCMSK = _BV(SIG_IN);
	
	// setup timer0
	GTCCR = 0x80;	// TSM set
	//	OCR0A = 0x61;	// Set the count to 97 (0x61) to give a 10 Hz tick (100ms)
	OCR0A = 0x27;	// Set the count to 39 (0x27) to give a 25 Hz tick (40ms)
	TCCR0A = 2;		// CTC mode
	TCCR0B = 0x05;	// Start timer with 1024 prescaler
	TIMSK = 0x10;	// set the OCIE0A interrupt enable
	
	
	// setup timer1 for timing the pulse widths
	TCCR1 = 0x00;	// set timer1 prescaler to 16 to start the counter.
	
	// enable the interrupts in the GMSK
	GIMSK = 0x20;	//
	
	set_sleep_mode(SLEEP_MODE_IDLE);
	
	// setup the ADC
	ADMUX = 0x81;
	DIDR0 = 0x04;	// disable PB2 input
	ADCSRA = 0x2e;	// set this register to start conversions when motor is on.
	ADCSRB = 0;
	
	ClearTestBit();
	
	// initialize the state machine
	state_ptr = &(State1);
	Motor_Off();
	GPIOR0 = 0;
	
	delay_count = 0;
	No_RX_Count = 0;
	
	sei();	// global interrupt enable
}

void Motor_On_FWD(void)
{
	cbi(GPIOR0,OVER_CURRENT);	// clear the current trip
	PORTB |= _BV(MOTOR1);
	// Do we need to pause here to blank the motor turn on current spike?
	_delay_ms(TURN_ON_DELAY);
	ADC_On();
}

void Motor_On_REV(void)
{
	cbi(GPIOR0,OVER_CURRENT);	// clear the current trip
	PORTB |= _BV(MOTOR2);
	// Do we need to pause here to blank the motor turn on current spike?
	_delay_ms(TURN_ON_DELAY);
	ADC_On();	
}

void Motor_Off(void)
{
	PORTB &= ~(_BV(MOTOR1) | _BV(MOTOR2));
	ADC_Off();
}

void State1(void)
{
	if (bit_is_set(GPIOR0, SWITCH_POS))	// switch went up
	{
		Motor_Off();
		// set state to State2
		state_ptr = &(State2);
		Start_Delay(); // Start 1/2 sec delay timer
	}
}

void State2(void)
{
	if (bit_is_clear(GPIOR0, SWITCH_POS))	// switch went low
	{
		Motor_Off();
		Start_Delay(); // restart 1/2 sec timer
		state_ptr = &(State2A);	// set state to State2A
	} else if (bit_is_set(GPIOR0,TIMEOUT))
	{
		cbi(GPIOR0,TIMEOUT);
		Motor_On_FWD();			// Direction Fwd
		state_ptr = &(State3);	// set state to State3
	}
}

void State3(void)
{
	if (bit_is_clear(GPIOR0,SWITCH_POS))	// switch went low
	{
		Motor_Off();
		state_ptr = &(State1);	// Set State to State1
	}
	else if (bit_is_set(GPIOR0,OVER_CURRENT))
	{
		Motor_Off();
		cbi(GPIOR0, OVER_CURRENT);	// clear the OVER_CURRENT;
		state_ptr = &(State1A);	// set State to State 1A
	}
}

void State1A(void)
{
	if (bit_is_clear(GPIOR0,SWITCH_POS))	// switch went low
	{
		Motor_Off();
		Start_Delay();			// start 1/2 sec timer
		state_ptr = &(State2A);	// set State to State2A
	}
}

void State2A(void)
{
	if(bit_is_set(GPIOR0,SWITCH_POS))	// switch went up
	{
		Motor_Off();
		Start_Delay();			// reset 1/2 sec timer
		state_ptr = &(State2);	// set State to State2
	}
	else if (bit_is_set(GPIOR0, TIMEOUT))
	{
		cbi(GPIOR0,TIMEOUT);
		Motor_On_REV();
		state_ptr = &(State3A);	// set State to State3A
	}
}

void State3A(void)
{
	if (bit_is_set(GPIOR0, SWITCH_POS)) // switch went up
	{
		Motor_Off();
		state_ptr = &(State1A);
	}
	else if (bit_is_set(GPIOR0, OVER_CURRENT))
	{
		Motor_Off();
		cbi(GPIOR0, OVER_CURRENT);	// clear the OVER_CURRENT;
		state_ptr = &(State1);
	}
}

void Start_Delay(void)
{
	delay_count = 0;		// reset timer
	cbi(GPIOR0,TIMEOUT);	// clear the timeout flag
}
