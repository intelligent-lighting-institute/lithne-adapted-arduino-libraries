/*
 * HarwareTouch.cpp
 * 
 * Serge Offermans
 * Intelligent Lighting Institute (ILI), TU/e.
 *
 * All rights reserved. LAST UPDATE: 13-11-2013
 * Code is based on Example from playground: http://playground.arduino.cc/Code/CapacitiveSensor
*/
#include "HardwareTouch.h"
  
/* CONSTRUCTORS */

/** Default Constructor, make sure to set the channel correctly later on! **/
TouchPin::TouchPin( )
{ 
  
}
TouchPin::TouchPin( uint16_t pin, bool calibrateNow )
{ 
  _pin = pin;
  if( calibrateNow )
  {
  	calibrate();
  	calibrate();	// The second calibration is required because the first reading is always faulty
  }
}
TouchPin::~TouchPin( )
{ 
  
}
uint16_t TouchPin::getPin(  )
{ 
  return _pin;
}

void TouchPin::setThreshold( uint8_t thresh )
{
	_threshold = thresh;
}

void TouchPin::calibrate()
{
	_calibratedValue = readRaw( getPin() );
}
uint8_t TouchPin::getCalibratedValue()
{
	return _calibratedValue;
}

bool TouchPin::newValue()
{
	bool newVal = false;
	int8_t newReading = read();
	if ( abs( newReading - _storedValue ) >= _threshold )
	{
		_storedValue = newReading;
		newVal = true;
	}
	return newVal;
}
int8_t TouchPin::getValue()
{
	return _storedValue;
}

bool TouchPin::touched()
{
	bool t = false;
	if( read() >= _threshold)
	{
		t = true;
	}
	return t;
}
int8_t TouchPin::read()
{
	return readRaw( getPin() ) - getCalibratedValue();
}

uint8_t TouchPin::readRaw() 
{
	readRaw( getPin() );
}

uint8_t TouchPin::readRaw( uint16_t pinToMeasure ) 
{
  uint8_t bitmask;  // Used to address the correct ports for XMega and ATmega

/* The XMega uses a different method of handling ports */
#if defined(__AVR_XMEGA__)
  static volatile PORT_t *port;
#else
  static volatile uint8_t *port; 
  volatile uint8_t* ddr;
  volatile uint8_t* pin;
#endif


#if defined(__AVR_XMEGA__)  
  // Xmega config
  bitmask = digitalPinToBitMask(pinToMeasure);
  uint8_t portIndex = digitalPinToPort(pinToMeasure);
  uint8_t bit = getBitFromBitField(bitmask);
  port = portRegister(portIndex);
  uint8_t* pinctrl = (uint8_t*)&port->PIN0CTRL;
#else
  // Variables used to translate from Arduino to AVR pin naming
  // ATMega Config
  port = portOutputRegister(digitalPinToPort(pinToMeasure));
  ddr = portModeRegister(digitalPinToPort(pinToMeasure));
  bitmask = digitalPinToBitMask(pinToMeasure);
  pin = portInputRegister(digitalPinToPort(pinToMeasure));
#endif  
  // Here we translate the input pin number from
  //  Arduino pin number to the AVR PORT, PIN, DDR,
  //  and which bit of those registers we care about.

#if defined(__AVR_XMEGA__)  
  // Discharge the pin first by setting it low and output
  port->DIRSET = bitmask; // set output
  port->OUTCLR = bitmask; // Make pin Low
#else
  // Discharge the pin first by setting it low and output
  *port &= ~(bitmask);
  *ddr  |= bitmask;
#endif
  delay(1);
  // Prevent the timer IRQ from disturbing our measurement
  noInterrupts();
#if defined(__AVR_XMEGA__)  
  // Xmega config
  port->DIRCLR = bitmask; // set INPUT
  pinctrl[bit] |= PORT_OPC_PULLUP_gc; // turn on pullup
#else
  // Make the pin an input with the internal pull-up on
  *ddr &= ~(bitmask);
  *port |= bitmask;
#endif

  // Now see how long the pin to get pulled up. This manual unrolling of the loop
  // decreases the number of hardware cycles between each read of the pin,
  // thus increasing sensitivity.
  uint8_t cycles = 17;

#if defined(__AVR_XMEGA__)  
  #define pinHigh port->IN & bitmask
#else
  #define pinHigh *pin & bitmask
#endif

  if (pinHigh) { 
    cycles =  0;
  }
  else if (pinHigh) { 
    cycles =  1;
  }
  else if (pinHigh) { 
    cycles =  2;
  }
  else if (pinHigh) { 
    cycles =  3;
  }
  else if (pinHigh) { 
    cycles =  4;
  }
  else if (pinHigh) { 
    cycles =  5;
  }
  else if (pinHigh) { 
    cycles =  6;
  }
  else if (pinHigh) { 
    cycles =  7;
  }
  else if (pinHigh) { 
    cycles =  8;
  }
  else if (pinHigh) { 
    cycles =  9;
  }
  else if (pinHigh) { 
    cycles = 10;
  }
  else if (pinHigh) { 
    cycles = 11;
  }
  else if (pinHigh) { 
    cycles = 12;
  }
  else if (pinHigh) { 
    cycles = 13;
  }
  else if (pinHigh) { 
    cycles = 14;
  }
  else if (pinHigh) { 
    cycles = 15;
  }
  else if (pinHigh) { 
    cycles = 16;
  }
  
  // End of timing-critical section
  interrupts();

  // Discharge the pin again by setting it low and output
  //  It's important to leave the pins low if you want to 
  //  be able to touch more than 1 sensor at a time - if
  //  the sensor is left pulled high, when you touch
  //  two sensors, your body will transfer the charge between
  //  sensors
#if defined(__AVR_XMEGA__)  
    // Discharge the pin first by setting it low and output
  port->DIRSET = bitmask; // set output
  port->OUTCLR = bitmask; // Make pin Low
  
#else
  // Discharge the pin first by setting it low and output
  *port &= ~(bitmask);
  *ddr  |= bitmask;
#endif

  return cycles;
}


#if defined(__AVR_XMEGA__)
uint8_t TouchPin::getBitFromBitField( uint8_t input ) // Function taken from WInterrupt.c; required for XMega
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
#endif