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

// --------------------------------------------------------
// MMA7455_read
//
// This is a common function to read multiple bytes
// from an I2C device.
//
// It uses the boolean parameter for Wire.endTransMission()
// to be able to hold or release the I2C-bus.
// This is implemented in Arduino 1.0.1.
//
// Only this function is used to read.
// There is no function for a single byte.
//

MMA7455::MMA7455(void) {
}

int MMA7455_read(int start, uint8_t *buffer, int size)
{
  int i, n, error;

  Wire.beginTransmission(MMA7455_I2C_ADDRESS);
  n = Wire.write(start);
  if (n != 1)
    return (-10);

  n = Wire.endTransmission(false); // hold the I2C-bus
  if (n != 0)
    return (n);

  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom((int)MMA7455_I2C_ADDRESS, size, (int)true);
  i = 0;
  while(Wire.available() && i<size)
  {
    buffer[i++]=Wire.read();
  }
  if ( i != size)
    return (-11);

  return (0);                  // return : no error
}

int MMA7455::read(int start, uint8_t *buffer, int size)
{
	return MMA7455_read(start, buffer, size);
}

// --------------------------------------------------------
// MMA7455_write
//
// This is a common function to write multiple bytes
// to an I2C device.
//
// Only this function is used to write.
// There is no function for a single byte.
//
int MMA7455_write(int start, const uint8_t *pData, int size)
{
  int n, error;

  Wire.beginTransmission(MMA7455_I2C_ADDRESS);
  n = Wire.write(start);        // write the start address
  if (n != 1)
    return (-20);

  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
    return (-21);

  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
    return (error);

  return (0);                   // return : no error
}

int MMA7455::write(int start, const uint8_t *pData, int size)
{
	return MMA7455_write(start, pData, size);
}

// --------------------------------------------------------
// MMA7455_xyz
//
// Get the 'g' forces.
// The values are with integers as 64 per 'g'.
//
int MMA7455_xyz( uint16_t *pX, uint16_t *pY, uint16_t *pZ)
{
  xyz_union xyz;
  int error;
  uint8_t c;

  // Wait for status bit DRDY to indicate that
  // all 3 axis are valid.
  do
  {
    error = MMA7455_read (MMA7455_STATUS, &c, 1);
  } while ( !bitRead(c, MMA7455_DRDY) && error == 0);
  if (error != 0)
    return (error);

  // Read 6 bytes, containing the X,Y,Z information
  // as 10-bit signed integers.
  error = MMA7455_read (MMA7455_XOUTL, (uint8_t *) &xyz, 6);
  if (error != 0)
    return (error);

  // The output is 10-bits and could be negative.
  // To use the output as a 16-bit signed integer,
  // the sign bit (bit 9) is extended for the 16 bits.
  if (xyz.reg.x_msb & 0x02)    // Bit 9 is sign bit.
    xyz.reg.x_msb |= 0xFC;     // Stretch bit 9 over other bits.
  else
    xyz.reg.x_msb &= 0x3;

  if (xyz.reg.y_msb & 0x02)
    xyz.reg.y_msb |= 0xFC;
  else
    xyz.reg.y_msb &= 0x3;

  if (xyz.reg.z_msb & 0x02)
    xyz.reg.z_msb |= 0xFC;
  else
    xyz.reg.z_msb &= 0x3;

  // The result is the g-force in units of 64 per 'g'.
  *pX = xyz.value.x;
  *pY = xyz.value.y;
  *pZ = xyz.value.z;

  return (0);                  // return : no error
}

int MMA7455::xyz( uint16_t *pX, uint16_t *pY, uint16_t *pZ)
{
	return MMA7455_xyz( pX, pY, pZ);
}

// --------------------------------------------------------
// MMA7455_init
//
// Initialize the MMA7455.
// Set also the offset, assuming that the accelerometer is
// in flat horizontal position.
//
// Important notes about the offset:
//    The sensor has internal registers to set an offset.
//    But the offset could also be calculated by software.
//    This function uses the internal offset registers
//    of the sensor.
//    That turned out to be bad idea, since setting the
//    offset alters the actual offset of the sensor.
//    A second offset calculation had to be implemented
//    to fine tune the offset.
//    Using software variables for the offset would be
//    much better.
//
//    The offset is influenced by the slightest vibration
//    (like a computer on the table).
//    
int MMA7455_init(void)
{
  uint16_t x, y, z;
  int error;
  xyz_union xyz;
  uint8_t c1, c2;

  // Initialize the sensor
  //
  // Sensitivity:
  //    2g : GLVL0
  //    4g : GLVL1
  //    8g : GLVL1 | GLVL0
  // Mode:
  //    Standby         : 0
  //    Measurement     : MODE0
  //    Level Detection : MODE1
  //    Pulse Detection : MODE1 | MODE0
  // There was no need to add functions to write and read
  // a single byte. So only the two functions to write
  // and read multiple bytes are used.

  // Set mode for "2g sensitivity" and "Measurement Mode".
  c1 = bit(MMA7455_GLVL0) | bit(MMA7455_MODE0);
  error = MMA7455_write(MMA7455_MCTL, &c1, 1);
  if (error != 0)
    return (error);

  // Read it back, to test the sensor and communication.
  error = MMA7455_read(MMA7455_MCTL, &c2, 1);
  if (error != 0)
    return (error);

  if (c1 != c2)
    return (-99);

  // Clear the offset registers.
  // If the Arduino was reset or with a warm-boot,
  // there still could be offset written in the sensor.
  // Only with power-up the offset values of the sensor
  // are zero.
  xyz.value.x = xyz.value.y = xyz.value.z = 0;
  error = MMA7455_write(MMA7455_XOFFL, (uint8_t *) &xyz, 6);
  if (error != 0)
    return (error);

  // The mode has just been set, and the sensor is activated.
  // To get a valid reading, wait some time.
  delay(100);

#define USE_INTERNAL_OFFSET_REGISTERS
#ifdef USE_INTERNAL_OFFSET_REGISTERS

  // Calcuate the offset.
  //
  // The values are 16-bits signed integers, but the sensor
  // uses offsets of 11-bits signed integers.
  // However that is not a problem,
  // as long as the value is within the range.

  // Assuming that the sensor is flat horizontal,
  // the 'z'-axis should be 1 'g'. And 1 'g' is
  // a value of 64 (if the 2g most sensitive setting
  // is used).  
  // Note that the actual written value should be doubled
  // for this sensor.

  error = MMA7455_xyz (&x, &y, &z); // get the x,y,z values
  if (error != 0)
    return (error);

  xyz.value.x = 2 * -x;        // The sensor wants double values.
  xyz.value.y = 2 * -y;
  xyz.value.z = 2 * -(z-64);   // 64 is for 1 'g' for z-axis.

  error = MMA7455_write(MMA7455_XOFFL, (uint8_t *) &xyz, 6);
  if (error != 0)
    return (error);

  // The offset has been set, and everything should be okay.
  // But by setting the offset, the offset of the sensor
  // changes.
  // A second offset calculation has to be done after
  // a short delay, to compensate for that.
  delay(200);

  error = MMA7455_xyz (&x, &y, &z);    // get te x,y,z values again
  if (error != 0)
    return (error);

  xyz.value.x += 2 * -x;       // add to previous value
  xyz.value.y += 2 * -y;
  xyz.value.z += 2 * -(z-64);  // 64 is for 1 'g' for z-axis.

  // Write the offset for a second time.
  // This time the offset is fine tuned.
  error = MMA7455_write(MMA7455_XOFFL, (uint8_t *) &xyz, 6);
  if (error != 0)
    return (error);

#endif

  return (0);          // return : no error
}

void MMA7455::begin(uint8_t pin_sda, uint8_t pin_scl) {
  int error;
  uint8_t c;

  _pin_sda = pin_sda;
  _pin_scl = pin_scl;

  // Initialize the 'Wire' class for I2C-bus communication.
  Wire.begin(_pin_sda,_pin_scl);

  // Initialize the MMA7455, and set the offset.
  error = MMA7455_init();
  if (error == 0)
    Serial.println("The MMA7455 is okay");
  else
    Serial.println("Check your wiring !");


  // Read the Status Register
  MMA7455_read(MMA7455_STATUS, &c, 1);

  // Read the "Who am I" value
  MMA7455_read(MMA7455_WHOAMI, &c, 1);

  // Read the optional temperature output value (I always read zero)
  MMA7455_read(MMA7455_TOUT, &c, 1);
}




