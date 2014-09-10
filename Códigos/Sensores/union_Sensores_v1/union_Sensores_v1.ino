//From the bildr article http://bildr.org/2012/11/hih4030-arduino/

#include <Wire.h>
#include <SPI.h>

// Variables
// Temperatura
int temperatura_pin = 0X48; // Direccion temperatura (I2C)

// Humedad
int humedad_pin = A0; // Pin A0 humedad

// Luz
int temt6000Pin = A1; // Pin A1 luz

//Compás
int HMC6352Address = 0x42; // Direccion compás (I2C). This is calculated in the setup() function
int slaveAddress;
byte Direccion[2]; // Bytes leidos por el compás
int i, Orientacion; // Orientación del sensor.
int Grados;

//Humedad
//Sensor's memory register addresses:
const int PRESSURE = 0x1F;      //3 most significant bits of pressure
const int PRESSURE_LSB = 0x20;  //16 least significant bits of pressure
const int TEMPERATURE = 0x21;   //16 bit temperature reading
const byte READ = 0b11111100;     // SCP1000's read command
const byte WRITE = 0b00000010;   // SCP1000's write command

const int dataReadyPin = 6;    
const int chipSelectPin = 7;
int Presion;

void setup(){
  // Configuraciones
  // General
  Serial.begin(9600);
  Wire.begin();
  SPI.begin();
  
  //Compas
  // Shift the device's documented slave address (0x42) 1 bit right
  // This compensates for how the TWI library only wants the
  // 7 most significant bits (with the high bit padded with 0)
  slaveAddress = HMC6352Address >> 1;   // This results in 0x21 as the address to pass to TWI.

  //Humedad
  // initalize the  data ready and chip select pins:
  pinMode(dataReadyPin, INPUT);
  pinMode(chipSelectPin, OUTPUT);
  
  //Configure SCP1000 for low noise configuration:
  writeRegister(0x02, 0x2D);
  writeRegister(0x01, 0x03);
  writeRegister(0x03, 0x02);
  // give the sensor time to set up:
  delay(100);
}

void loop(){
  
  // Variables medidas por los sensores.
  float temperature = getTemperature(); //Utilizamos la temperatura del termometro para obtener la lectura correcta de humedad
  float relativeHumidity  = getHumidity(temperature);
  int luz = analogRead(temt6000Pin);
  int Grados = getCompas();
  int Presion = getPresion();
  
  // Monitor serial.
  Serial.print("Humedad relativa:");
  Serial.println(relativeHumidity);
  
  Serial.print("Temperatura ");
  Serial.println(temperature);

  Serial.print("Luz: ");
  Serial.println(luz);
  
  Serial.print("Direccion: ");
  Serial.print(int (Grados / 10));     // The whole number part of the heading
  Serial.print(".");
  Serial.print(int (Grados % 10));     // The fractional part of the heading
  Serial.println(" grados");
  
  Serial.print("Presion [hPa]: ");
  Serial.println(Presion);  
  delay(1000); //velocidad lectura. Entre mas alto el valor, mas lenta la velocidad de impresion en el monitor serial
}

// Funciones

// Humedad
  float getHumidity(float degreesCelsius){
  //caculate relative humidity
  float supplyVolt = 5.0;

  // read the luz from the sensor:
  int HIH4030_luz = analogRead(humedad_pin);
  float voltage = HIH4030_luz/1023. * supplyVolt; // convert to voltage luz

  // convert the voltage to a relative humidity
  // - the equation is derived from the HIH-4030/31 datasheet
  // - it is not calibrated to your individual sensor
  //  Table 2 of the sheet shows the may deviate from this line
  float sensorRH = 161.0 * voltage / supplyVolt - 25.8;
  float trueRH = sensorRH / (1.0546 - 0.0026 * degreesCelsius); //temperature adjustment 

  return trueRH;
}

// Temperatura
float getTemperature(){
  Wire.requestFrom(temperatura_pin,2); 

  byte MSB = Wire.read();
  byte LSB = Wire.read();

  //it's a 12bit int, using two's compliment for negative
  int TemperatureSum = ((MSB << 8) | LSB) >> 4; 

  float celsius = TemperatureSum*0.0625;
  return celsius;
}

// Compas
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

// Presion  
int getPresion(){
  //Select High Resolution Mode
  writeRegister(0x03, 0x0A);

  // don't do anything until the data ready pin is high:
  if (digitalRead(dataReadyPin) == HIGH) {
    //Read the temperature data
    int tempData = readRegister(0x21, 2);

    // convert the temperature to celsius and display it:
    float realTemp = (float)tempData / 20.0;
   
    //Read the pressure data highest 3 bits:
    byte  pressure_data_high = readRegister(0x1F, 1);
    pressure_data_high &= 0b00000111; //you only needs bits 2 to 0

    //Read the pressure data lower 16 bits:
    unsigned int pressure_data_low = readRegister(0x20, 2);
    //combine the two parts into one 19-bit number:
    long pressure = (2.58*(((pressure_data_high << 16) | pressure_data_low)/4)+73160)/100;
    return pressure;
  }
}

//Read from or write to register from the SCP1000:
unsigned int readRegister(byte thisRegister, int bytesToRead ) {
  byte inByte = 0;           // incoming byte from the SPI
  unsigned int result = 0;   // result to return
  //Serial.print(thisRegister, BIN);
  //Serial.print("\t");
  // SCP1000 expects the register name in the upper 6 bits
  // of the byte. So shift the bits left by two bits:
  thisRegister = thisRegister << 2;
  // now combine the address and the command into one byte
  byte dataToSend = thisRegister & READ;
  //Serial.println(thisRegister, BIN);
  // take the chip select low to select the device:
  digitalWrite(chipSelectPin, LOW);
  // send the device the register you want to read:
  SPI.transfer(dataToSend);
  // send a value of 0 to read the first byte returned:
  result = SPI.transfer(0x00);
  // decrement the number of bytes left to read:
  bytesToRead--;
  // if you still have another byte to read:
  if (bytesToRead > 0) {
    // shift the first byte left, then get the second byte:
    result = result << 8;
    inByte = SPI.transfer(0x00);
    // combine the byte you just got with the previous one:
    result = result | inByte;
    // decrement the number of bytes left to read:
    bytesToRead--;
  }
  // take the chip select high to de-select:
  digitalWrite(chipSelectPin, HIGH);
  // return the result:
  return(result);
}

//Sends a write command to SCP1000
void writeRegister(byte thisRegister, byte thisValue) {

  // SCP1000 expects the register address in the upper 6 bits
  // of the byte. So shift the bits left by two bits:
  thisRegister = thisRegister << 2;
  // now combine the register address and the command into one byte:
  byte dataToSend = thisRegister | WRITE;

  // take the chip select low to select the device:
  digitalWrite(chipSelectPin, LOW);

  SPI.transfer(dataToSend); //Send register location
  SPI.transfer(thisValue);  //Send value to record into register

  // take the chip select high to de-select:
  digitalWrite(chipSelectPin, HIGH);
}



