
#include <TinyGPS.h>


TinyGPS gps;
float flat, flon;
float falt;
unsigned long fix_age, time, date, vel, direccion;
int year;
byte mes, dia, hora, minuto, segundo, milisegundos;
boolean datocorrecto;

void setup()
{

Serial.begin(57600);
Serial3.begin(57600);

}

void loop(){
datocorrecto=false;
while(datocorrecto==false){
  if(Serial3.available() > 0){
    int c =Serial3.read();
    if(gps.encode(c)){
      datocorrecto=true;
    }
  }
}
    gps.crack_datetime(&year,&mes,&dia,&hora,&minuto,&segundo,&milisegundos);
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
    
     gps.f_get_position(&flat, &flon);    
     Serial.print("Latitud: ");
     Serial.println(flat,5);
     Serial.print("Longitud: ");
     Serial.println(flon,5);
     Serial.print("http://maps.google.com/maps?q=");
     Serial.print(flat,5);
     Serial.print(",");
     Serial.println(flon,5);
     
     falt = gps.f_altitude(); // +/- altitude in meters
     Serial.print("Altura: ");
     Serial.print(falt,2);
     Serial.println("m.");  
     
     direccion = gps.f_course();
     Serial.print("Direccion: ");
     Serial.print(direccion);
     Serial.println(" grados.");
     
     vel = gps.f_speed_kmph();
     Serial.print("Velocidad: ");
     Serial.print(vel);
     Serial.println("Km/h");
     Serial.println("");
     delay(3000);
  }
