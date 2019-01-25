
void setup() {
  Serial.begin(9600);
}

void loop() {
  //Serial.print("|");
  //int lectura = analogRead(A1);
  //Serial.println(lectura);
  //delay(10);   //ATENCION: Para la lectura del voltaje instantaneo de la señal, el delay debe ser de 2 ms o menos y sebe ser lo mismo en la app..
                //para este ejemplo, se envía la señal de un potenciometro muestreada cada 100 ms, el clock de la app está configurado a 100 ms también.
                //probé con tasa de transmision de 115200 bps, pero no hay cambios significativos en la velocidad.
  byte Data[5];
  int i = analogRead(A1);
  int bateria = 99;
  int frec = 65;
  Data[0]='a';
  Data[1]=i/256;
  Data[2]=i%256;
  Data[3]=bateria;
  Data[4]=frec;
  //Serial.println(i);
  //delay(10);
  //Serial.print("  ");
  //Serial.print(Data[0]);
  //Serial.print("  ");
  delay(10);
  Serial.write(Data[1]);
  Serial.write(Data[2]);
  Serial.write(Data[3]);
  Serial.write(Data[4]);
  delay(10);
}
