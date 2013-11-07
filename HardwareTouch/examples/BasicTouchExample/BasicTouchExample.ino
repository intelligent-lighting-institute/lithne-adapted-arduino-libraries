/*
  Library to read capacitive touch using just the Arduino hardware.
  Depending on your setup, values range between 0 (not touched) and 17, (very much touched)
*/

#include <HardwareTouch.h>

/* Define a pin object */
TouchPin pin;

void setup()
{
  Serial.begin( 115200 );
  /* Now create the pin object, you may use any pin on your board. */
  pin = TouchPin( A0 );
}

void loop()
{

  /* Use touched() to see if a pin is touched */
  if( pin.touched() )
  {
    Serial.print( millis() );
    Serial.println("\t I'm touched!");
    delay( 10 );
  }

  /* Use the read() function to read the specific value from the Touch Pin. */  
  Serial.println( pin.read() );
  delay( 100 );

  /* Now check the advanced example because it is much more useful!! =] */
}
