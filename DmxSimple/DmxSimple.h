/**
 * DmxSimple - A simple interface to DMX.
 *
 * Copyright (c) 2008-2009 Peter Knight, Tinker.it! All rights reserved.
 */

#ifndef DmxSimple_h
#define DmxSimple_h

/* The following code makes the Library compatible with Arduino 1.0 */
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "wiring.h"
#endif

#include <inttypes.h>

/*
#if RAMEND <= 0x4FF
#define DMX_SIZE 128
#else
#define DMX_SIZE 512
#endif
*/

#ifndef DMX_SIZE
	#define DMX_SIZE 64
#endif

class DmxSimpleClass
{
  public:
    void maxChannel(int);
    void write(int, uint8_t);
    void usePin(uint8_t);
};
extern DmxSimpleClass DmxSimple;

#endif
