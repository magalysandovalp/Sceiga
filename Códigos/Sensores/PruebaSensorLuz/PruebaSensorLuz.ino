int temt6000Pin = A0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  int value = analogRead(temt6000Pin);
  Serial.println(value);

  delay(200); //only here to slow down the output so it is easier to read
}
