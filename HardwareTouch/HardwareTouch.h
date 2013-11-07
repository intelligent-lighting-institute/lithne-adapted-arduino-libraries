/*
 * HarwareTouch.h
 * 
 * Serge Offermans
 * Intelligent Lighting Institute (ILI), TU/e.
 *
 * All rights reserved. LAST UPDATE: 13-11-2013
 * Code is based on Example from playground: http://playground.arduino.cc/Code/CapacitiveSensor
*/

#ifndef HardwareTouch_h
#define HardwareTouch_h

/* The following code makes the Library compatible with Arduino 1.0 */
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
  #include "cppfix.h"
#endif

#include "HardwareTouch.h"

/*! \mainpage HarwareTouch Library
 *  This Arduino Library can be used to detect capacitive touch without using external hardware. It is compatible with both Arduino as well as XMega boards such as Akafuino and Lithne. <br /><br />
  <a href="functions_func.html">All available functions are documented here</a> 
 */

/** \brief The TouchPin class defines the TouchPins **/

class TouchPin
{
  public:
    TouchPin();
    TouchPin( uint16_t pin, bool calibrateNow = true );
    ~TouchPin();
    uint16_t getPin();

    void    calibrate();
    uint8_t getCalibratedValue();
    
    bool    touched();
    int8_t  read();
    uint8_t readRaw( );
    uint8_t readRaw( uint16_t pinToMeasure );

    bool    newValue();
    int8_t  getValue();

    void    setThreshold( uint8_t thresh );
    
  protected:
    uint16_t  _pin             = 0;
    uint8_t   _calibratedValue = 0; 
    int8_t    _storedValue     = 0;
    uint8_t   _threshold       = 1;

  #if defined(__AVR_XMEGA__)
    uint8_t getBitFromBitField( uint8_t input ); // Function taken from WInterrupt.c; required for XMega
  #endif

};

#endif