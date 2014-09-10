#include <Arduino.h>
#include <SoftwareSerial.h>
#include "SM5100BGSMLite.h"

//#define VERBOSE
//
//#define RXPINGSM     10 //Para Duemilanove es 
//#define TXPINGSM     11 //RX 2, TX 3
//#define PIN          "3400"  // Replace this number
//
//// Spreadtrum GSM Shield
//SoftwareSerial GSMSerialport(RXPINGSM, TXPINGSM);
//SM5100BGSMLite gsmModule(&GSMSerialport);

char texto[140] = "Numero 1: ";
float numero = 200.457546451;
char numeroC[10];
//char auxTexto[40];

void setup() {
    Serial.begin(9600);  
//  GSMSerialport.begin(9600); 
//  gsmModule.init();
//  
//  while (!gsmModule.isRegistered()) {
//    Serial.println("searching for provider...");
//    gsmModule.checkNetwork();
//    delay(3000);
//  }
//
  dtostrf(numero,1,4,numeroC);
  //numeroC = itoa(numero,auxTexto,10);
  strcat(texto,numeroC); 
  //numeroC = itoa(numero2,auxTexto,10);
  //strcat(texto,texto2);
  //strcat(texto,numeroC);
  //Serial.print(texto);
  //gsmModule.sendSMS("1001", texto);
  Serial.print(texto);
  }

void loop() {
  
}
