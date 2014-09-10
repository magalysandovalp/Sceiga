
int temt6000Pin = A0;
boolean contdoc = true;

void setup() {
  Serial.begin(9600);
}

void loop() {
  if (contdoc){
    nuevoArchivo();
    contdoc = false;
    delay (2000);
  }
  else{
    GrabarSD();
  }
  delay (2000);
 
}

void SensorLuz(){
  int value = analogRead(temt6000Pin);
  Serial.print(value);
//  Serial.print(";");  
}

void GrabarSD(){                          //Función encargada de llamar a las funciones encargadas de leer los datos de los sensores para guardar los datos en la memoria SD.
  SensorLuz();                            //Función encargada de tomar los datos del sensor de Luz
  Serial.println("");                    //Comando encargado de hacer un cambio de fila
}

void nuevoArchivo(){                     //Función encargada de crear un archivo nuevo para escribir los valores del juego de mediciones actuales 
  
randomSeed(analogRead(3));              //Lee la variable analógica del pin 3 para crear el numero random del archivo
long randomaleat=random(1000);          //Guarda en la variable randomaleat el numero random leido en el pin analógico

delay(500); 
Serial.print("new Rec" + String(randomaleat) +  ".csv");    //Crea el archivo llamado Rec + el numero random leido en el pin analógico + .csv esto con el fin de poder abrirlo en exel
delay(500);  
Serial.println("");
delay(500);  
 
Serial.print("append Rec"+ String(randomaleat) + ".csv");  //Comando para indicar que guarde cualquier dato en el archivo creado anteriormente
delay(200);  
Serial.println("");
delay(170);   
Serial.println("Sistema de adquicion de datos");
delay(170);  
Serial.println("");
delay(170);  

Serial.println("C.Luz");

}


