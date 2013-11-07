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
  /* Now create the pin object, you may use any pin on your board. 

     By default, the pin is calibrated when it is defined, you can prevent this in the definition: TouchPin( A0, false );
  */
  pin = TouchPin( A0 );

  /* You may use calibrate() at any point to set the default value that is measured on a pin. Readings will return teh difference with the calibrated value */
  // pin.calibrate();
  
  Serial.print("Calibrated Touchpin ");
  Serial.print( pin.getPin() );
  Serial.print(" to value ");
  Serial.println( pin.getCalibratedValue() );

  /* Use setThreshold() to define the minimum change for a reading to be considered a newValue(). 
  The default value is 1, which will often result in a very unstable signal */
  pin.setThreshold( 2 );
}

void loop()
{
  /* Check for a newValue(); this reads the pin and compares it to the previous reading.
     Use getValue() to obtain this value.
  */
  if ( pin.newValue() )
  {
    Serial.println( pin.getValue() );  
  }

  /* You may also use the binary check touched() to see if a pin is touched
      This uses both the calibrated and threshold values defined in the setup. */
  /*
  if(pin.touched())
  {
    Serial.print(millis());
    Serial.println("\t I'm touched!");
    delay( 10 );
  }
  */

  /* Alternatively you may use the read() function to instantly read the value from the Touch Pin. 
     This takes will also take the calibration into consideration. If you wish, you can also neglect the calibration using the readRaw() function */
  /*  
  Serial.println( pin.read() );
  delay( 100 );
  */
}