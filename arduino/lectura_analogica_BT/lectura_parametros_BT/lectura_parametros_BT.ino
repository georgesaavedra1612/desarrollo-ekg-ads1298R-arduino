
void setup() {
  Serial.begin(9600);
}

void loop() {
  //Serial.print("|");
  int lectura = analogRead(A0);
  Serial.print("65");
  Serial.print("bpm");
  Serial.print("|");
  Serial.print("15");
  Serial.print("Bpm");
  Serial.print("|");
  Serial.print("37");
  Serial.print("ºC");
  Serial.print("|");
  Serial.print("99");
  Serial.print("%");
  delay(2000);    //ATENCION: Para la lectura de los parametros, el Clock de la app debe estar configurado a un delay mayor, tal como de 2 seg. mas o menos..
}
