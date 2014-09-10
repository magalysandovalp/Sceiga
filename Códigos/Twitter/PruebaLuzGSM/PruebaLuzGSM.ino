#include <Arduino.h>
#include <SoftwareSerial.h>
#include "SM5100BGSMLite.h"

#define VERBOSE

//#define RXPINGSM     2
//#define TXPINGSM     3
#define RXPINGSM     10
#define TXPINGSM     11
#define PIN          "3400"  // Replace this number

// Spreadtrum GSM Shield
SoftwareSerial GSMSerialport(RXPINGSM, TXPINGSM);
SM5100BGSMLite gsmModule(&GSMSerialport);
//SM5100BGSMLite gsmModule(&GSMSerialport,PIN); // use this line in case you have pin
int temt6000Pin = A0;
char luz[40] = "p8kg4d-nvydfv@twittermail.com  Luz: ";
char* luzC;
char auxLuz[40];


void setup() {                
  // initialize softserial gsm
  Serial.begin(9600);
  GSMSerialport.begin(9600);
  
  gsmModule.init();
  
  while (!gsmModule.isRegistered()) {
    Serial.println("searching for provider...");
    gsmModule.checkNetwork();
    delay(3000);
  }
  
  int value = analogRead(temt6000Pin);
  luzC = itoa(value,auxLuz,10);
  strcat(luz,luzC);
  gsmModule.sendSMS("1001", luz);
  
  //at this point the module is registered
  //then we can send a sms
  
  //gsmModule.sendSMS("<destination number here>", "hola");
}

void loop() {
  // your program
}
