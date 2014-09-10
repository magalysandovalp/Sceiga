#include <Arduino.h>
#include <SoftwareSerial.h>
#include "SM5100BGSMLite.h"
#include <Wire.h>
#include <SPI.h>
#include <ADXL345.h>
#include <TinyGPS.h>

// GSM
#define VERBOSE
//#define RXPINGSM     2
//#define TXPINGSM     3
#define RXPINGSM     10
#define TXPINGSM     11
#define PIN          "3400"  // Replace this number

SoftwareSerial GSMSerialport(RXPINGSM, TXPINGSM);
SM5100BGSMLite gsmModule(&GSMSerialport);
//SM5100BGSMLite gsmModule(&GSMSerialport,PIN); // use this line in case you have pin

// VARIABLES
// MENSAJE TWITTER
char mensajeTwitter[140] = "p8kg4d-nvydfv@twittermail.com  Temperatura: ";
char* numeroC;
char auxTwitter[140];
char numeroFloat[10];
boolean sent_SMS = true;

// TEMPERATURA
int temperatura_pin = 0X48; // Direccion temperatura (I2C)

// HUMEDAD
int humedad_pin = A10; // Pin A10 humedad

// LUZ
int temt6000Pin = A11; // Pin A11 luz

// PRESION
//Sensor's memory register addresses:
const int PRESSURE = 0x1F;      //3 most significant bits of pressure
const int PRESSURE_LSB = 0x20;  //16 least significant bits of pressure
const int TEMPERATURE = 0x21;   //16 bit temperature reading
const byte READ = 0b11111100;     // SCP1000's read command
const byte WRITE = 0b00000010;   // SCP1000's write command

const int dataReadyPin = 6;    
const int chipSelectPin = 7;
int Presion;

// ACELEROMETRO
// Declare a global instance of the accelerometer.
ADXL345 accel;
float aceleracionX;
float aceleracionY;
float aceleracionZ;

void setup() {                
  // CONFIGURACIONES
  // GENERAL
  Serial.begin(9600);
  Wire.begin();
  SPI.begin();
  
  // PRESION
  // initalize the  data ready and chip select pins:
  pinMode(dataReadyPin, INPUT);
  pinMode(chipSelectPin, OUTPUT);
  
  //Configure SCP1000 for low noise configuration:
  writeRegister(0x02, 0x2D);
  writeRegister(0x01, 0x03);
  writeRegister(0x03, 0x02);
  // give the sensor time to set up:
  delay(100);
  
  // ACELEROMETRO
  // Create an instance of the accelerometer on the default address (0x1D)
  accel = ADXL345();
  // Set the range of the accelerometer to a maximum of 2G.
  accel.SetRange(8, true);
  // Tell the accelerometer to start taking measurements.
  accel.EnableMeasurements();
  
    // GSM
  GSMSerialport.begin(9600);
  gsmModule.init();
  
  while (!gsmModule.isRegistered()) {
    Serial.println("searching for provider...");
    gsmModule.checkNetwork();
    delay(3000);
  }
}

void loop() {
  float temperature = getTemperature(); //Utilizamos la temperatura del termometro para obtener la lectura correcta de humedad
  float relativeHumidity  = getHumidity(temperature);
  int luz = analogRead(temt6000Pin);
  int Presion = getPresion();
  getAcelerometro();
  if (sent_SMS){
    //Convierte la temperatura a char y lo agrega al mensaje
    numeroC = itoa(temperature,auxTwitter,10); 
    strcat(mensajeTwitter,numeroC);
    //strcat(mensajeTwitter," *C"); // Unidades de temperatura °C
    
    //Convierte la humedad a char y lo agrega al mensaje
    numeroC = itoa(relativeHumidity,auxTwitter,10);
    strcat(mensajeTwitter,"\nHumedad: ");
    strcat(mensajeTwitter,numeroC);
    
    //Convierte la luz a char y lo agrega al mensaje
    numeroC = itoa(luz,auxTwitter,10);
    strcat(mensajeTwitter,"\nLuz: ");
    strcat(mensajeTwitter,numeroC);
    
    //Convierte la presion a char y lo agrega al mensaje
    numeroC = itoa(Presion,auxTwitter,10);
    strcat(mensajeTwitter,"\nPresion: ");
    strcat(mensajeTwitter,numeroC);
    
    //Convierte la aceleracion a char y lo agrega al mensaje
    strcat(mensajeTwitter,"\nAceleracion: ");
    dtostrf(aceleracionX,1,1,numeroFloat);
    strcat(mensajeTwitter,numeroFloat);
    strcat(mensajeTwitter,"G, ");
    dtostrf(aceleracionY,1,1,numeroFloat);
    strcat(mensajeTwitter,numeroFloat);
    strcat(mensajeTwitter,"G, ");
    dtostrf(aceleracionZ,1,1,numeroFloat);
    strcat(mensajeTwitter,numeroFloat);
    strcat(mensajeTwitter,"G");
    
    //Envia el mensaje a Twitter
    gsmModule.sendSMS("1001", mensajeTwitter);
    sent_SMS = false;
  }
  
  Serial.print("Temperatura: ");
  Serial.println(temperature);
  Serial.print("Humedad: ");
  Serial.println(relativeHumidity);
  Serial.print("Luz: ");
  Serial.println(luz);
  Serial.print("Presion ");
  Serial.println(Presion);
  delay(2000);
}

// FUNCIONES
// TEMPERATURA
float getTemperature(){
  Wire.requestFrom(temperatura_pin,2); 
  byte MSB = Wire.read();
  byte LSB = Wire.read();

  //it's a 12bit int, using two's compliment for negative
  int TemperatureSum = ((MSB << 8) | LSB) >> 4; 
  float celsius = TemperatureSum*0.0625;
  return celsius;
}

// HUMEDAD
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

// PRESION
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

// Acelerometro
void getAcelerometro(){
  if(accel.IsConnected) // If we are connected to the accelerometer.
  {
    // Read the raw data from the accelerometer.
    AccelerometerRaw raw = accel.ReadRawAxis();
    //This data can be accessed like so:
    int xAxisRawData = raw.XAxis;
    
    // Read the *scaled* data from the accelerometer (this does it's own read from the accelerometer
    // so you don't have to ReadRawAxis before you use this method).
    // This useful method gives you the value in G thanks to the Love Electronics library.
    AccelerometerScaled scaled = accel.ReadScaledAxis();
    // This data can be accessed like so:
    float xAxisGs = scaled.XAxis;
    
    // We output our received data.
    Output(raw, scaled);
  }
}

// Output the data down the serial port.
void Output(AccelerometerRaw raw, AccelerometerScaled scaled)
{
   // Tell us about the this data, but scale it into useful units (G).
   aceleracionX = scaled.XAxis;
   aceleracionY = scaled.YAxis;
   aceleracionZ = scaled.ZAxis;
   
   Serial.print("Aceleracion (x,y,z): ");
   
   Serial.print(aceleracionX);
   Serial.print("G   ");   
   Serial.print(aceleracionY);
   Serial.print("G   ");   
   Serial.print(aceleracionZ);
   Serial.println("G");
   
   //Serial3.print(scaled.XAxis);
   //Serial3.print(",");     
   //Serial3.print(scaled.YAxis);
   //Serial3.print(",");      
   //Serial3.print(scaled.ZAxis);
   //Serial3.println("");
}
