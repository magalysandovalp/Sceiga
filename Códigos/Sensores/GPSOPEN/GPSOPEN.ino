
#include <TinyGPS.h>


TinyGPS gps;

void diatiempo(TinyGPS &gps);
void posicion(TinyGPS &gps);
void altitud(TinyGPS &gps);
void dir(TinyGPS &gps);
void velocidad(TinyGPS &gps);

float flat, flon;
float falt;

unsigned long fix_age, time, date, vel, direccion;
int year;
byte mes, dia, hora, minuto, segundo, milisegundos;


void setup(){

Serial.begin(57600);
Serial3.begin(57600);


}

void loop(){
feedgps();
diatiempo(gps);
posicion(gps);
altitud(gps);
dir(gps);
velocidad(gps);
delay(3000);
}

void dir(TinyGPS &gps){
  if(feedgps()){
  direccion = gps.f_course();
  Serial.print("Direccion: ");
  Serial.print(direccion);
  Serial.println(" grados."); 
  }
}

void velocidad(TinyGPS &gps){
  if(feedgps()){
  vel = gps.f_speed_kmph();
  Serial.print("Velocidad: ");
  Serial.print(vel);
  Serial.println("Km/h");
  Serial.println("");
  }
}

void altitud(TinyGPS &gps){
  if(feedgps()){
  falt = gps.f_altitude(); // +/- altitude in meters
  Serial.print("Altura: ");
  Serial.print(falt,2);
  Serial.println("m.");  
  }
}

void posicion(TinyGPS &gps){
  if(feedgps()){
  gps.f_get_position(&flat, &flon);    
  Serial.print("Latitud: ");
  Serial.println(flat,5);
  Serial.print("Longitud: ");
  Serial.println(flon,5);
  Serial.print("http://maps.google.com/maps?q=");
  Serial.print(flat,5);
  Serial.print(",");
  Serial.println(flon,5); 
  } 
}


void diatiempo(TinyGPS &gps){
    if(feedgps()){
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
    }
}
  

static bool feedgps()
{
  if (Serial3.available())
  {
    if (gps.encode(Serial3.read()))
      return true;
  }
  return false;
}
