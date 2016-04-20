// MMA7455 Accelerometer example
// -----------------------------
//
// By R. Zschiegner (rz@madavi.de).
// April 2016

#include <MMA7455.h>

MMA7455 my_mma;

void setup()
{
  uint8_t c;

  Serial.begin(9600);
  Serial.println("Freescale MMA7455 accelerometer");
  Serial.println("May 2012");
  my_mma.begin(D3,D4);

  // Read the Status Register
  my_mma.read(MMA7455_STATUS, &c, 1);
  Serial.print("Status: "); Serial.println(c);

  // Read the "Who am I" value
  my_mma.read(MMA7455_WHOAMI, &c, 1);
  Serial.print("WhoAmI: "); Serial.println(c);

  // Read the optional temperature output value (I always read zero)
  my_mma.read(MMA7455_TOUT, &c, 1);
  Serial.print("Temp.: "); Serial.println(c);

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
