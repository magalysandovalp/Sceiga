/*
Titulo: SCEIGA 2.0
Descripcion del programa: Aquisicion de datos y escritura en la SD.
Autor: Felipe Arce
       Magaly Sandoval
       Fabián Solano
Fecha de creación: 3/18/2013
*/

#include <Wire.h>
#include <SPI.h>
#include <ADXL345.h>
#include <TinyGPS.h>



// Variables
// Temperatura
int temperatura_pin = 0X48; // Direccion temperatura (I2C)

// Humedad
int humedad_pin = A0; // Pin A0 humedad

// Luz
int temt6000Pin = A1; // Pin A1 luz

// Compás
int HMC6352Address = 0x42; // Direccion compás (I2C). This is calculated in the setup() function
int slaveAddress;
byte Direccion[2]; // Bytes leidos por el compás
int i, Orientacion; // Orientación del sensor.
int Grados;

// Presion
//Sensor's memory register addresses:
const int PRESSURE = 0x1F;      //3 most significant bits of pressure
const int PRESSURE_LSB = 0x20;  //16 least significant bits of pressure
const int TEMPERATURE = 0x21;   //16 bit temperature reading
const byte READ = 0b11111100;     // SCP1000's read command
const byte WRITE = 0b00000010;   // SCP1000's write command

const int dataReadyPin = 6;    
const int chipSelectPin = 7;
int Presion;

// Acelerometro
// Declare a global instance of the accelerometer.
ADXL345 accel;

// SD
boolean contdoc = true;

//GPS
TinyGPS gps;

float flat, flon;
float falt;
unsigned long fix_age, time, date, vel, direccion;
int year;
byte mes, dia, hora, minuto, segundo, milisegundos;
boolean datocorrecto;


void setup(){
  // Configuraciones
  // General
  Serial3.begin(9600);//openlog
  Serial1.begin(57600);//gps
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
  
  //Acelerometro
  // Create an instance of the accelerometer on the default address (0x1D)
  accel = ADXL345();
  // Set the range of the accelerometer to a maximum of 2G.
  accel.SetRange(2, true);
  // Tell the accelerometer to start taking measurements.
  accel.EnableMeasurements();
}

void loop(){
  
  //GPS
  
feedgps();
  
  // SD
  modSD();
  
  // Variables medidas por los sensores.
  float temperature = getTemperature(); //Utilizamos la temperatura del termometro para obtener la lectura correcta de humedad
  float relativeHumidity  = getHumidity(temperature);
  int luz = analogRead(temt6000Pin);
  int Grados = getCompas();
  int Presion = getPresion();
    
  // Monitor serial.
  feedgps();
  gps.crack_datetime(&year,&mes,&dia,&hora,&minuto,&segundo,&milisegundos);
  Serial3.print(dia,DEC);
  Serial3.print("/");
  Serial3.print(mes,DEC);
  Serial3.print("/");
  Serial3.print(year);
  Serial3.print(",");
  
  Serial3.print(hora,DEC);
  Serial3.print(":");
  Serial3.print(minuto,DEC);
  Serial3.print(":");
  Serial3.print(segundo,DEC);
  Serial3.print(",");
 
  Serial.print("Fecha: ");
  Serial.print(dia,DEC);
  Serial.print("/");
  Serial.print(mes,DEC);
  Serial.print("/");
  Serial.println(year);

  
  Serial.print("Hora(UTC): ");
  Serial.print(hora,DEC);
  Serial.print(":");
  Serial.print(minuto,DEC);
  Serial.print(":");
  Serial.println(segundo,DEC);
  
 

  
  feedgps();
  gps.f_get_position(&flat, &flon);  

  Serial3.print(flat,5);
  Serial3.print(",");
  Serial3.print(flon,5);
  Serial3.print(",");
  Serial3.print("http://maps.google.com/maps?q=");
  Serial3.print(flat,5);
  Serial3.print(" ");
  Serial3.print(flon,5);
  Serial3.print(",");
  
  Serial.print("Latitud: ");
  Serial.println(flat,5);
  Serial.print("Longitud: ");
  Serial.println(flon,5);
  Serial.print("http://maps.google.com/maps?q=");
  Serial.print(flat,5);
  Serial.print(",");
  Serial.println(flon,5);
      

  
  feedgps();
  falt = gps.f_altitude(); // +/- altitude in meters
  
  Serial3.print(falt,2);
  Serial3.print(",");
   
  Serial.print("Altura: ");
  Serial.print(falt,2);
  Serial.println("m.");    

  feedgps();
  direccion = gps.f_course();
  
  Serial3.print(direccion);
  Serial3.print(",");
  
  Serial.print("Direccion: ");
  Serial.print(direccion);
  Serial.println(" grados.");


  
  feedgps();
  vel = gps.f_speed_kmph();

  Serial3.print(vel);
  Serial3.print(",");

  Serial.print("Velocidad: ");
  Serial.print(vel);
  Serial.println("Km/h");
  Serial.println("");

  Serial.println("");
  Serial.print("");
  Serial.print("Humedad relativa:");
  Serial.println(relativeHumidity);
  Serial3.print(relativeHumidity);
  Serial3.print(",");
  
  Serial.print("Temperatura ");
  Serial.println(temperature);
  Serial3.print(temperature);
  Serial3.print(",");

  Serial.print("Luz: ");
  Serial.println(luz);
  Serial3.print(luz);
  Serial3.print(",");
  
  Serial.print("Direccion: ");
  Serial.print(int (Grados / 10));     // The whole number part of the heading
  Serial.print(".");
  Serial.print(int (Grados % 10));     // The fractional part of the heading
  Serial.println(" grados");
  Serial3.print(int (Grados / 10));
  Serial3.print(".");
  Serial3.print(int (Grados % 10));
  Serial3.print(",");
  
  Serial.print("Velocidad: ");
  Serial.print(vel);
  Serial.println("Km/h");
  Serial.println("");
  
  Serial.print("Presion [hPa]: ");
  Serial.println(Presion);
  Serial3.print(Presion);
  Serial3.print(",");  
  
  getAcelerometro();
  
  delay(10000); //velocidad lectura. Entre mas alto el valor, mas lenta la velocidad de impresion en el monitor serial
}



// Funciones

//GPS
void feedgps(){
   datocorrecto=false;
  while(datocorrecto==false){
    if(Serial1.available() > 0){
      int c =Serial1.read();
      if(gps.encode(c)){
        datocorrecto=true;
      }
    }
  }
}

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
   // Tell us about the raw values coming from the accelerometer.
   //Serial.print("Raw:\t");
   //Serial.print(raw.XAxis);
   //Serial.print("   ");   
   //Serial.print(raw.YAxis);
   //Serial.print("   ");   
   //Serial.print(raw.ZAxis);
   
   // Tell us about the this data, but scale it into useful units (G).
   Serial.print("Aceleracion (x,y,z): ");
   Serial.print(scaled.XAxis);
   Serial.print("G   ");   
   Serial.print(scaled.YAxis);
   Serial.print("G   ");   
   Serial.print(scaled.ZAxis);
   Serial.println("G");
   
   Serial3.print(scaled.XAxis);
   Serial3.print(",");     
   Serial3.print(scaled.YAxis);
   Serial3.print(",");      
   Serial3.print(scaled.ZAxis);
   Serial3.println("");
}

// Memoria SD
void modSD(){        // Crea un nuevo archivo al inicio, cuando la condicion de contdoc es true.
 if (contdoc){
    nuevoArchivo();
    contdoc = false;
 }
}

void nuevoArchivo(){                     //Función encargada de crear un archivo nuevo para escribir los valores del juego de mediciones actuales 
  
randomSeed(analogRead(3));              //Lee la variable analógica del pin 3 para crear el numero random del archivo
long randomaleat=random(1000);          //Guarda en la variable randomaleat el numero random leido en el pin analógico

delay(500); 
Serial3.print("new Rec" + String(randomaleat) +  ".csv");    //Crea el archivo llamado Rec + el numero random leido en el pin analógico + .csv esto con el fin de poder abrirlo en exel
delay(500);  
Serial3.println("");
delay(500);  
 
Serial3.print("append Rec"+ String(randomaleat) + ".csv");  //Comando para indicar que guarde cualquier dato en el archivo creado anteriormente
delay(200);  
Serial3.println("");
delay(170);   
Serial3.println("Sistema de adquicion de datos");
delay(170);  
Serial3.println("");
delay(170);  

Serial3.print("Fecha");
Serial3.print(",");
Serial3.print("Hora");
Serial3.print(",");
Serial3.print("Latitud");
Serial3.print(",");
Serial3.print("Longitud");
Serial3.print(",");
Serial3.print("Googlemaps");
Serial3.print(",");
Serial3.print("Altura (m)");
Serial3.print(",");
Serial3.print("Direccion (grados)");
Serial3.print(",");
Serial3.print("Velocidad (Km/h)");
Serial3.print(",");
Serial3.print("Humedad");
Serial3.print(",");
Serial3.print("Temperatura");
Serial3.print(",");
Serial3.print("Luz");
Serial3.print(",");
Serial3.print("Direccion");
Serial3.print(",");
Serial3.print("Presion");
Serial3.print(",");
Serial3.print("AceleracionX");
Serial3.print(",");
Serial3.print("AceleracionY");
Serial3.print(",");
Serial3.println("AceleracionZ");
}






