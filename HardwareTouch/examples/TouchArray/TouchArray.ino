/*
  Library to read capacitive touch using just the Arduino hardware.
  Depending on your setup, values range between 0 (not touched) and 17, (very much touched)
*/

#include <HardwareTouch.h>
const static uint8_t NUM_PINS = 3; 

/* Define a pin array */
TouchPin pins[NUM_PINS];

void setup()
{
  Serial.begin( 115200 );
  /* Now create the pin objects, you may use any pin on your board. */
  pins[0] = TouchPin( A0 );
  pins[1] = TouchPin( A1 );
  pins[2] = TouchPin( A2 );
}

void loop()
{
  /* Use touched() to see if a pin is touched */
  Serial.print( millis() );
  Serial.print( "\t touched pins (place in array)): " );
  for(int i = 0; i < NUM_PINS; i++)
  {
    if( pins[i].touched() )
    {
      Serial.print( i );
      Serial.print( "\t" );
    }
  }
  Serial.println();
  delay( 100 );
  /* Now check the advanced example because it is much more useful!! =] */
}