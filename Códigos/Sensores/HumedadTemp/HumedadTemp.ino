//From the bildr article http://bildr.org/2012/11/hih4030-arduino/

#include <Wire.h>
int temperatura_pin = 0X48; //direccion temperatura
int humedad_pin = A0; //analog pin 0

void setup(){
  Serial.begin(9600);
  Wire.begin();
}

void loop(){
  
  float temperature = getTemperature(); //Utilizamos la temperatura del termometro para obtener la lectura correcta de humedad
  float relativeHumidity  = getHumidity(temperature);

  Serial.print("Humedad relativa:");
  Serial.println(relativeHumidity);
  float celsius = getTemperature();
  Serial.print("Celsius: ");
  Serial.println(celsius);

  delay(1000); //velocidad lectura. Entre mas alto el valor, mas lenta la velocidad de impresion en el monitor serial
}


float getHumidity(float degreesCelsius){
  //caculate relative humidity
  float supplyVolt = 5.0;

  // read the value from the sensor:
  int HIH4030_Value = analogRead(humedad_pin);
  float voltage = HIH4030_Value/1023. * supplyVolt; // convert to voltage value

  // convert the voltage to a relative humidity
  // - the equation is derived from the HIH-4030/31 datasheet
  // - it is not calibrated to your individual sensor
  //  Table 2 of the sheet shows the may deviate from this line
  float sensorRH = 161.0 * voltage / supplyVolt - 25.8;
  float trueRH = sensorRH / (1.0546 - 0.0026 * degreesCelsius); //temperature adjustment 

  return trueRH;
}

float getTemperature(){
  Wire.requestFrom(temperatura_pin,2); 

  byte MSB = Wire.read();
  byte LSB = Wire.read();

  //it's a 12bit int, using two's compliment for negative
  int TemperatureSum = ((MSB << 8) | LSB) >> 4; 

  float celsius = TemperatureSum*0.0625;
  return celsius;
}
