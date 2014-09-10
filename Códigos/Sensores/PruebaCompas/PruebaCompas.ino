#include <Wire.h>
int HMC6352Address = 0x42;
// This is calculated in the setup() function
int slaveAddress;
byte Direccion[2]; // Variable de dirección medida por el sensor.
int i, Orientacion; // Orientación del sensor.
int grados;
void setup()
{
// Shift the device's documented slave address (0x42) 1 bit right
// This compensates for how the TWI library only wants the
// 7 most significant bits (with the high bit padded with 0)
slaveAddress = HMC6352Address >> 1;   // This results in 0x21 as the address to pass to TWI
Serial.begin(9600);
Wire.begin();
}
void loop()
{
  grados = getCompas();
  
  Serial.print("Current heading: ");
  Serial.print(int (grados / 10));     // The whole number part of the heading
  Serial.print(".");
  Serial.print(int (grados % 10));     // The fractional part of the heading
  Serial.println(" degrees");
  delay(500);
}

int getCompas(){
  // Send a "A" command to the HMC6352
  // This requests the current heading data
  Wire.beginTransmission(slaveAddress);
  Wire.write("A");              // The "Get Data" command
  Wire.endTransmission();
  delay(10);                   // The HMC6352 needs at least a 70us (microsecond) delay
  // after this command.  Using 10ms just makes it safe
  // Read the 2 heading bytes, MSB first
  // The resulting 16bit word is the compass heading in 10th's of a degree
  // For example: a heading of 1345 would be 134.5 degrees
  Wire.requestFrom(slaveAddress, 2);        // Request the 2 byte heading (MSB comes first)
  i = 0;
  while(Wire.available() && i < 2)
  { 
    Direccion[i] = Wire.read();
    i++;
  }
  Orientacion = Direccion[0]*256 + Direccion[1];  // Put the MSB and LSB together
  return Orientacion;
}
