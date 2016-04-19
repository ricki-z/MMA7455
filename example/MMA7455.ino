// MMA7455 Accelerometer
// ---------------------
//
// By arduino.cc user "Krodal".
// May 2012
// Open Source / Public Domain
//
// Fixes to union and type conversions by Arduino.cc user "Afroviking"
// August 2014
//
// Using Arduino 1.0.1
// It will not work with an older version, since Wire.endTransmission()
// uses a parameter to hold or release the I2C bus.
//
// Documentation:
//     - The Freescale MMA7455L datasheet
//     - The AN3468 Application Note (programming).
//     - The AN3728 Application Note (calibrating offset).
//
// The MMA7455 can be used by writing and reading a single byte,
// but it is also capable to read and write multiple bytes.
//
// The accuracy is 10-bits.
//

#include "MMA7455.h"

MMA7455 my_mma;

void setup()
{
  Serial.begin(9600);
  Serial.println("Freescale MMA7455 accelerometer");
  Serial.println("May 2012");
  my_mma.begin(D3,D4);
}


void loop()
{
  uint16_t x,y,z, error;
  double dX,dY,dZ;

  // The function MMA7455_xyz returns the 'g'-force
  // as an integer in 64 per 'g'.

  // set x,y,z to zero (they are not written in case of an error).
  x = y = z = 0;
  error = my_mma.xyz(&x, &y, &z); // get the accelerometer values.

  dX = (int16_t) x / 64.0;          // calculate the 'g' values.
  dY = (int16_t) y / 64.0;
  dZ = (int16_t) z / 64.0;

  if (error != 0 || abs(dX) > 0.07 || abs(dY) > 0.07 || abs(dZ) > 1.07) {
    Serial.print("error = ");
    Serial.print(error, DEC);
    Serial.print(", xyz g-forces = ");
    Serial.print(dX, 3);
    Serial.print(", ");
    Serial.print(dY, 3);
    Serial.print(", ");
    Serial.print(dZ, 3);
    Serial.println("");
  }
  
  delay(10);
}
