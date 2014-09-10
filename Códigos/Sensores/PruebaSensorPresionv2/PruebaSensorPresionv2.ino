/*
SCP1000 (parallax board on 5v arduino mega)
code from Conor with fixes by BLP from thread:
( http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1213217330/5#5 )
uses pins 10, 11, 12 ,13  for Duemilanove or 53, 51, 50, 52 for Arduino Mega
Must use resistors with a 5 volt Arduino talking to this 3.3 volt SPI sensor. 
See: http://www.sparkfun.com/commerce/tutorial_info.php?tutorials_id=65
Parallax SCP1000      ArduinoD/ArduinoMega      
1   VSS/GND           gnd      
2   AVDD              3.3 v
3   TRIG              Connect to gnd if not used      
4   DRDY (Int)        n/a      
5   DVDD              3.3 V      
6   PD                connect to gnd if not used      
7   SCLK              pin 13/52 using 10K resistor
8   MOSI              pin 11/51 using 10K resistor
9   MISO              pin 12/50 straight through (no resistor needed)
10  CSB               pin 10/53 using 1K with 10K 3.3v pull up from
                        sensor side of 1K 
*/

// define spi bus pins
// set for mega change pins for duemilanove
#define SLAVESELECT 53   // CS/SS pin 10 or 53
#define SPICLOCK 52      // clk pin 13 or 52
#define DATAOUT 51  //MOSI pin 11 or 51
#define DATAIN 50  //MISO pin 12 or 50
#define UBLB(a,b)  ( ( (a) << 8) | (b) )
#define UBLB19(a,b) ( ( (a) << 16 ) | (b) )

//Addresses
#define REVID 0x00 //ASIC Revision Number
#define OPSTATUS 0x04   //Operation Status
#define STATUS 0x07     //ASIC Status
#define START 0x0A      //Constant Readings
#define PRESSURE 0x1F   //Pressure 3 MSB
#define PRESSURE_LSB 0x20 //Pressure 16 LSB
#define TEMP 0x21       //16 bit temp

char rev_in_byte;     
int temp_in;
unsigned long pressure_lsb;
unsigned long pressure_msb;
unsigned long temp_pressure;
unsigned long pressure;

void setup()
{
  byte clr;
  pinMode(DATAOUT, OUTPUT);
  pinMode(DATAIN, INPUT);
  pinMode(SPICLOCK,OUTPUT);
  pinMode(SLAVESELECT,OUTPUT);
  digitalWrite(SLAVESELECT,HIGH); //disable device  
  
  SPCR = B01010011; //MPIE=0, SPE=1 (on), DORD=0 (MSB first), MSTR=1 (master), CPOL=0 (clock idle when low), CPHA=0 (samples MOSI on rising edge), SPR1=0 & SPR0=0 (500kHz)
  clr=SPSR;
  clr=SPDR;
  delay(10);
  Serial.begin(9600);
  delay(500);

  Serial.println("Initialize High Speed Constant Reading Mode");
  write_register(0x03,0x09);
}

void loop()
{
  
  rev_in_byte = read_register(REVID);
  
  pressure_msb = read_register(PRESSURE);
  pressure_msb &= B00000111;
  pressure_lsb = read_register16(PRESSURE_LSB);
  pressure_lsb &= 0x0000FFFF;  // this is BLP's fix
  pressure = UBLB19(pressure_msb, pressure_lsb);
  pressure /= 4;
  
  Serial.print("PRESSURE (Pa)[");
  Serial.print(pressure, DEC);
  Serial.println("]");
  
  Serial.print("PRESSURE (Atm)[");
  float pAtm=float(pressure)/101325.0;
  Serial.print(pAtm, 3);
  Serial.println("]");
  
  temp_in = read_register16(TEMP);
  float tempC = float(temp_in)/19.0; // use 20 by the spec sheet - my board seems closer to 19
  Serial.print("TEMP C [");
  Serial.print(tempC , 0);
  Serial.println("]");
  float tempF = tempC*1.8 + 32;
  Serial.print("TEMP F [");
  Serial.print(tempF , 0);
  Serial.println("]");
  Serial.println("--------------------------");
  delay(2500);
  
}

char spi_transfer(volatile char data)
{
  SPDR = data;   // Start the transmission
  while (!(SPSR & (1<<SPIF)))     // Wait for the end of the transmission
  {
  };
  return SPDR;   // return the received byte
}


char read_register(char register_name)
{
    char in_byte;
    register_name <<= 2;
    register_name &= B11111100; //Read command
  
    digitalWrite(SLAVESELECT,LOW); //Select SPI Device
    spi_transfer(register_name); //Write byte to device
    in_byte = spi_transfer(0x00); //Send nothing, but we should get back the register value
    digitalWrite(SLAVESELECT,HIGH);
    delay(10);
    return(in_byte);
  
}

unsigned long read_register16(char register_name)
{
    byte in_byte1;
    byte in_byte2;
    float in_word;
    
    register_name <<= 2;
    register_name &= B11111100; //Read command

    digitalWrite(SLAVESELECT,LOW); //Select SPI Device
    spi_transfer(register_name); //Write byte to device
    in_byte1 = spi_transfer(0x00);    
    in_byte2 = spi_transfer(0x00);
    digitalWrite(SLAVESELECT,HIGH);
    in_word = UBLB(in_byte1,in_byte2);
    return(in_word);
}

void write_register(char register_name, char register_value)
{
    register_name <<= 2;
    register_name |= B00000010; //Write command

    digitalWrite(SLAVESELECT,LOW); //Select SPI device
    spi_transfer(register_name); //Send register location
    spi_transfer(register_value); //Send value to record into register
    digitalWrite(SLAVESELECT,HIGH);
} 

