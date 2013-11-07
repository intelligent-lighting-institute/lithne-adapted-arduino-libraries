/**
 * DmxSimple - A simple interface to DMX.
 *
 * Copyright (c) 2008-2009 Peter Knight, Tinker.it! All rights reserved.
 * Additions for XMega Chips by Serge Offermans - Intellignet Lighting Institute Eindhoven
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "pins_arduino.h"

#include "DmxSimple.h"

/** dmxBuffer contains a software copy of all the DMX channels */
volatile uint8_t dmxBuffer[DMX_SIZE];
static uint16_t dmxMax = 16; /* Default to sending the first 16 channels */
static uint8_t dmxStarted = 0;
static uint16_t dmxState = 0;

/* The XMega uses a different method of handling ports */
#if defined(__AVR_XMEGA__)
	static volatile PORT_t *dmxPort;
#else
	static volatile uint8_t *dmxPort;	
#endif

static uint8_t dmxBit = 0;
static uint8_t dmxPin = 3; // Defaults to output on pin 3 to support Tinker.it! DMX shield

void dmxBegin();
void dmxEnd();
void dmxSendByte(volatile uint8_t);
void dmxWrite(int,uint8_t);
void dmxMaxChannel(int);

/* TIMER2 has a different register mapping on the ATmega8.
 * The modern chips (168, 328P, 1280) use identical mappings.
 */
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega168P__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	#define TIMER2_INTERRUPT_ENABLE() TIMSK2 |= _BV(TOIE2)
	#define TIMER2_INTERRUPT_DISABLE() TIMSK2 &= ~_BV(TOIE2)
#elif defined(__AVR_ATmega8__)
	#define TIMER2_INTERRUPT_ENABLE() TIMSK |= _BV(TOIE2)
	#define TIMER2_INTERRUPT_DISABLE() TIMSK &= ~_BV(TOIE2)
#elif defined(__AVR_XMEGA__)
	/* Use Port TCE0 - Disables port PE0 and PE1
	This is CPU pin 28/29 on the XMega32A4u > Used on the Akafuino board at board pin 14 / 15 
	This is CPU pin 36/37 on the XMega256A3u > Used on the Lithne board at board pin 32 (D8) / 33 (D9) 
	Alternatively you may use TCC0, which disables board pins 2/3 at the Akafuino and 16 / 17 at the Lithne board
	*/
	#define TIMER2 TCC0
	#define TIMER2_OVF_vect TCC0_OVF_vect
	#define TC_SetPeriod( _tc, _period ) ( (_tc)->PER = (_period) )
	#define TIMER2_INTERRUPT_DISABLE() TIMER2.INTCTRLA = TC_OVFINTLVL_OFF_gc 
#else
	#define TIMER2_INTERRUPT_ENABLE()
	#define TIMER2_INTERRUPT_DISABLE()
	/* Produce an appropriate message to aid error reporting on nonstandard
	 * platforms such as Teensy.
	 */
	#error "!!DmxSimple does not support this CPU!!"
#endif

#if defined(__AVR_XMEGA__)

uint8_t bitmask; 	// Used to address the correct ports for XMega
uint8_t getBitFromBitField(uint8_t input) // Function taken from WInterrupt.c; required for XMega
{
  switch(input)
  {
  case 0x1:
	return 0;
  case 0x2:
	return 1;
  case 0x4:
    return 2;
  case 0x8:
	return 3;
  case 0x10:
	return 4;
  case 0x20:
    return 5;
  case 0x40:
    return 6;
  case 0x80:
    return 7;
  default:
    return 0;
  }
}

/* Enable Timer Interrupt function for the XMega */
void TIMER2_INTERRUPT_ENABLE()
{	
	// Set Clock period 
	TC_SetPeriod( &TIMER2, 128 );   // 128 period with 64 prescaler results in a total period of 256us on a 32Mhz clock.
									// Formula: 	real_period = ( period * prescaler) / clockFrequency
	// Select clock source - Used clock A with 64 Prescaler 
	TIMER2.CTRLA = ( TIMER2.CTRLA & ~TC0_CLKSEL_gm ) | TC_CLKSEL_DIV64_gc; 
	//Actually Enable the Interrupt
	TIMER2.INTCTRLA = TC_OVFINTLVL_HI_gc;
}
#endif

/* Initialise the DMX engine */
void dmxBegin()
{
  dmxStarted = 1;
  // Set up port pointers for interrupt routine
  #if defined(__AVR_XMEGA__)  
	// Xmega config
	bitmask = digitalPinToBitMask(dmxPin);
    dmxBit = getBitFromBitField(bitmask);
	
	uint8_t portIndex = digitalPinToPort(dmxPin);
	dmxPort = portRegister(portIndex);
  #else
  // ATMega Config
   dmxPort = portOutputRegister(digitalPinToPort(dmxPin));
   dmxBit = digitalPinToBitMask(dmxPin);
  
  #endif	
  // Set DMX pin to output
  pinMode(dmxPin,OUTPUT);

  // Initialise DMX frame interrupt
  //
  // Presume Arduino has already set Timer2 to 64 prescaler,
  // Phase correct PWM mode
  // So the overflow triggers every 64*510 clock cycles
  // Which is 510 DMX bit periods at 16MHz,
  //          255 DMX bit periods at 8MHz,
  //          637 DMX bit periods at 20MHz
 
  TIMER2_INTERRUPT_ENABLE();
}

/** Stop the DMX engine
 * Turns off the DMX interrupt routine
 */
void dmxEnd()
{
  TIMER2_INTERRUPT_DISABLE();
  dmxStarted = 0;
  dmxMax = 0;
}

/** The following are the sendByte functions that transmit the DMX byte frames
 * These functions are called in teh interrupt routine
 * Transmit a complete DMX byte
 * We have no serial port for DMX, so everything is timed using an exact
 * number of instruction cycles.
 *
 * Really suggest you don't touch this function.
 */
 
// The XMega Function
#if defined(__AVR_XMEGA__)  
void dmxSendByte(volatile uint8_t value)
{
	// Disable other interrupts (e.g. the serial comm)
	__asm__ volatile ( "cli\n" );
	
	// Make pin low; DMX frame startbit
	dmxPort->OUTCLR = bitmask; 
	
	// Delay 4 microseconds in Assembly; 40 clock cycles is 4 us (41 for Lithne Boards is closer)
	uint8_t delCount;
	__asm__ volatile (
			"ldi %[delCount],41\n" 
			"delLoop%=:\n"
				"dec %[delCount]\n"
				"brne delLoop%=\n"
			:
			[delCount] "=&d" (delCount)
	);
	// Now go through the bits from in the order 1,2,4,8,16,32,64,128 
	for (int i = 0; i < 8; i++)
	{
		if ( (value & 1) != 0 ) // Determine whether the bit is high in the byte 'value'
		{
			dmxPort->OUTSET = bitmask; // Make pin High
		}
		else {
			dmxPort->OUTCLR = bitmask; // Make pin Low
		}
		value = value >> 1; // bitshift value
		
		// Delay 4 microseconds in Assembly; 35 clock cycles makes up for 3,5 us, the other 0,5 are taken by the code in the for loop above (36 for Lithne boards is closer)
		__asm__ volatile (
			"ldi %[delCount],36\n" 
			"delLoop%=:\n"
				"dec %[delCount]\n"
				"brne delLoop%=\n"
			:
			[delCount] "=&d" (delCount)
		);
	}
	
	dmxPort->OUTSET = bitmask; // Make pin high for the two DMX stopbits (2 * 4 us), this takes 80 clockcycles
	
	__asm__ volatile (
		"ldi %[delCount],80\n" 
		"delLoop%=:\n"
			"dec %[delCount]\n"
			"brne delLoop%=\n"	
		:
		[delCount] "=&d" (delCount)
	);
	
	// Re-enable all interrupts
	__asm__ volatile ("sei\n");
}

#else
/* The function for ATMega Boards */
void dmxSendByte(volatile uint8_t value)
{
  uint8_t bitCount, delCount;
  __asm__ volatile (
    "cli\n"
    "ld __tmp_reg__,%a[dmxPort]\n"
    "and __tmp_reg__,%[outMask]\n"
    "st %a[dmxPort],__tmp_reg__\n"
    "ldi %[bitCount],11\n" // 11 bit intervals per transmitted byte
    "rjmp bitLoop%=\n"     // Delay 2 clock cycles. 
  "bitLoop%=:\n"\
    "ldi %[delCount],%[delCountVal]\n"
  "delLoop%=:\n"
    "nop\n"
    "dec %[delCount]\n"
    "brne delLoop%=\n"
    "ld __tmp_reg__,%a[dmxPort]\n"
    "and __tmp_reg__,%[outMask]\n"
    "sec\n"
    "ror %[value]\n"
    "brcc sendzero%=\n"
    "or __tmp_reg__,%[outBit]\n"
  "sendzero%=:\n"
    "st %a[dmxPort],__tmp_reg__\n"
    "dec %[bitCount]\n"
    "brne bitLoop%=\n"
    "sei\n"
    :
      [bitCount] "=&d" (bitCount),
      [delCount] "=&d" (delCount)
    :
      [dmxPort] "e" (dmxPort),
      [outMask] "r" (~dmxBit),
      [outBit] "r" (dmxBit),
      [delCountVal] "M" (F_CPU/1000000-3),
      [value] "r" (value)
  );
}
#endif

/** DmxSimple interrupt routine
 * Transmit a chunk of DMX signal every timer overflow event.
 * 
 * The full DMX transmission takes too long, but some aspects of DMX timing
 * are flexible. This routine chunks the DMX signal, only sending as much as
 * it's time budget will allow.
 *
 * This interrupt routine runs with interrupts enabled most of the time.
 * With extremely heavy interrupt loads, it could conceivably interrupt its
 * own routine, so the TIMER2 interrupt is disabled for the duration of
 * the service routine.
 */
ISR(TIMER2_OVF_vect,ISR_NOBLOCK) 
{
  // Prevent this interrupt running recursively
  TIMER2_INTERRUPT_DISABLE();
  // Flag to check if we can leave the while loop; for the XMega this is after every byte
  bool bReady = false;
  
  // DMX Bit periods per timer tick (formula results in 512 for 16MHz and 1020 for 32Mhz) - every DMX bit lasts 4us
  uint16_t bitsLeft = F_CPU / 31372; 
  
  // Divide by 4, results in 25% CPU usage
  bitsLeft >>=2; 
  while (!bReady) 
  {
	// DMX state = 0 takes care of 88us initial dmx break and start byte which is always 0
    if (dmxState == 0) 
	{
      // Next thing to send is reset pulse (break) and start code
      // This takes 35 bit periods; 	35 * 4us = 140us = 88us(break) + 8us(required high) + 44us(startbyte=11*4us)
      if (bitsLeft < 35) bReady = true; // If we do not have enough time left for the break, we do it in the next interrupt and break the while loop
      // reduce the time we have left with 35 dmxBits (140 us)
	  bitsLeft-=35; 
	  // The way of writing ports is different for XMega and ATMega boards, but they do the same
	  #if defined(__AVR_XMEGA__)
		dmxPort->OUTCLR = bitmask; // Make Low
	  #else
		*dmxPort &= ~dmxBit; // Make Low
	  #endif
	  // Keep low for 88 us - DMX Packet Break
	  uint8_t i;
	  for (i=0; i<11; i++) _delay_us(8); 
	  #if defined(__AVR_XMEGA__)
		 dmxPort->OUTSET = bitmask; // Make High
	  #else
		*dmxPort |= dmxBit; // Make High
	  #endif
	  // Keep high for 8 us
      _delay_us(8); 
	  // Send the DMX Start Byte; always 0
      dmxSendByte(0); 
	  // Set dmxState to 1 so we start transmitting the actual bytes
	  dmxState = 1;
	  #if defined(__AVR_XMEGA__)
	    // We leave the while loop ending the interrupt routine; on the next loop (next interrupt in case of XMega) we will start sending DMX bytes
		bReady = true; 
	  #endif
    } 
	
	// After the initial startbyte we send the remaining DMX Channel Bytes
	// This is looped for all DMX channels, as long as we have 'bitsleft' (DMX bits; 4us per DMX bit), this is a measure for time.
	else // if dmxState != 0
	{
      // Now send a channel which takes 11 bit periods (1 startbit(low), the data byte (8 bits) and 2 stopbits (high)
	  // If we do not have enough time left for the byte, we do it in the next interrupt and break the while loop
      if (bitsLeft < 11) bReady = true; 
	  // Otherwise we reduce the time left
      bitsLeft-=11; 
	  // Send the data using this function defined above
      dmxSendByte(dmxBuffer[dmxState-1]); 
	  // Successfully completed that stage - move state machine forward	
	  dmxState++; 
	  // If we have sent all channels, end the loop and reset the state so it strats with a DMX Break the next routine
	  if (dmxState > dmxMax) 
	  {
	    dmxState = 0; // Reset for the next frame
	  }
	  #if defined(__AVR_XMEGA__)
		// We leave the while loop to end the routine
		bReady = true; 		
	  #endif
    }
  } // End while !bReady
  
  // Enable timer interrupt for the next transmission chunk
  TIMER2_INTERRUPT_ENABLE();
}

void dmxWrite(int channel, uint8_t value) {
  if (!dmxStarted) dmxBegin();
  if ((channel > 0) && (channel <= DMX_SIZE)) {
    if (value<0) value=0;
    if (value>255) value=255; 
    dmxMax = max((unsigned)channel, dmxMax);
    dmxBuffer[channel-1] = value;
  }
}

void dmxMaxChannel(int channel) {
  if (channel <=0) {
    // End DMX transmission
    dmxEnd();
    dmxMax = 0;
  } else {
    dmxMax = min(channel, DMX_SIZE);
    if (!dmxStarted) dmxBegin();
  }
}


/* C++ wrapper */


/** Set output pin
 * @param pin Output digital pin to use
 */
void DmxSimpleClass::usePin(uint8_t pin) {
  dmxPin = pin;
  if (dmxStarted && (pin != dmxPin)) {
    dmxEnd();
    dmxBegin();
  }
}

/** Set DMX maximum channel
 * @param channel The highest DMX channel to use
 */
void DmxSimpleClass::maxChannel(int channel) {
  dmxMaxChannel(channel);
}

/** Write to a DMX channel
 * @param address DMX address in the range 1 - 512
 */
void DmxSimpleClass::write(int address, uint8_t value)
{
	dmxWrite(address, value);
}
DmxSimpleClass DmxSimple;
