/*
 * Text-mode driver for ADS129x 
 * for Arduino Due
 *
 * Copyright (c) 2013 by Adam Feuer <adam@adamfeuer.com>
 * Copyright (c) 2012 by Chris Rorden
 * Copyright (c) 2012 by Steven Cogswell and Stefan Rado 
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <SPI.h>
#include <SoftwareSerial.h>
#include <SD.h>

#include <stdlib.h>
#include "adsCommand.h"
#include "ads129x.h"
#include "SerialCommand.h"
#include "Base64.h"
#include "SpiDma.h"

#define BAUD_RATE  115200     // WiredSerial ignores this and uses the maximum rate
#define txActiveChannelsOnly  // reduce bandwidth: only send data for active data channels
#define PinInt1 17 // Pin 17 como interruptor en teensy. N

//#define WiredSerial SerialUSB 
#define WiredSerial Serial
//SoftwareSerial myBT(10, 11); // RX, TX myBT es el dispositivo o modulo que estoy conectando al arduino.
SoftwareSerial myBT(5, 6); // RX, TX myBT es el dispositivo o modulo que estoy conectando al teensy. N

//Chip Select (CS) para micro SD para arduino mega
//const int chipSelect = 4; 
//Chip Select (CS) para micro SD para teensy 3.1 N
const int chipSelect = 9;                                      // XXXXXXXXXXXXXXXXXXXX

int maxChannels = 0;
int numActiveChannels = 0;
int standard; // para la transmision de la conversion..
boolean gActiveChan[9]; // reports whether channels 1..9 are active
boolean isRdatac = false;
boolean base64Mode = true;
const int PIN_LED = 8;

char hexDigits[] = "0123456789ABCDEF";
uint8_t serialBytes[200];
char sampleBuffer[1000];
int32_t codigo1;  // guarda los 24 bits del canal 1
int32_t codigo2;  // guarda los 24 bits del canal 2
int32_t codigo3;  // guarda los 24 bits del canal 3
int32_t codigo4;  // guarda los 24 bits del canal 4
int32_t codigo5;  // guarda los 24 bits del canal 5
int32_t codigo6;  // guarda los 24 bits del canal 6
int32_t codigo7;  // guarda los 24 bits del canal 7
int32_t codigo8;  // guarda los 24 bits del canal 8

const char *hardwareType = "unknown";
const char *boardName = "HackEEG";
const char *makerName = "StarCat, LLC";
const char *driverVersion = "ADS129x driver v0.2.1";

SPISettings settingsFE(4000000, MSBFIRST, SPI_MODE1);                                                                               //XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
SPISettings settingsSD(6000000, MSBFIRST, SPI_MODE3);                                                                               //XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

SerialCommand serialCommand;  

void setup() {  
  WiredSerial.begin(115200); 
  myBT.begin(19200);
  pinMode(PinInt1,INPUT);   //N
  //pinMode(22,OUTPUT);   //En este pin habrá una señal cuadrada sincronizada con la onda R, esta irá al pin del interruptor 4 arduino.
  pinMode(16,OUTPUT);   //En este pin habrá una señal cuadrada sincronizada con la onda R, esta irá al pin del interruptor 1 teensy. N
  //attachInterrupt(4,frec,RISING);  //interruptor 4 en pin 19  , recibe señal cuadrada y llama a función frec para calcular FC cuando ocurre un RISING.
  attachInterrupt(PinInt1,frec,RISING);  //interruptor 1 en pin 17  , recibe señal cuadrada y llama a función frec para calcular FC cuando ocurre un RISING. N
  pinMode(chipSelect,OUTPUT);

  arduinoSetup();
  adsSetup();
  SD.begin(chipSelect);
  SPI.beginTransaction(settingsSD);
  
  // Setup callbacks for SerialCommand commands 
  serialCommand.addCommand("version",version_command);        // Echos the driver version number
  serialCommand.addCommand("status",status_command);        // Echos the driver version number
  serialCommand.addCommand("serialnumber",serialNumber_command);        // Echos the driver version number
  serialCommand.addCommand("ledon",ledOn_command);            // Turns Due onboad LED on
  serialCommand.addCommand("ledoff", ledOff_command);         // Turns Due onboard LED off
  serialCommand.addCommand("boardledoff", boardLedOff_command);  // Turns HackEEG GPIO LED off
  serialCommand.addCommand("boardledon", boardLedOn_command);    // Turns HackEEG GPIO LED on
  serialCommand.addCommand("wakeup", wakeup_command);         // Enter read data continuous mode
  serialCommand.addCommand("standby", standby_command);         // Enter read data continuous mode
  serialCommand.addCommand("reset", reset_command);         // Enter read data continuous mode
  serialCommand.addCommand("start", start_command);         // Enter read data continuous mode
  serialCommand.addCommand("stop", stop_command);         // Enter read data continuous mode
  serialCommand.addCommand("rdatac", rdatac_command);         // Enter read data continuous mode
  serialCommand.addCommand("sdatac", sdatac_command);         // Stop read data continuous mode
  serialCommand.addCommand("rdata", rdata_command);           // Read one sample of data from each channel
  serialCommand.addCommand("rreg", readRegister_command);     // Read ADS129x register, argument in hex, print contents in hex
  serialCommand.addCommand("wreg", writeRegister_command);    // Write ADS129x register, arguments in hex
  serialCommand.addCommand("base64", base64ModeOn_command);   // rdata commands send base64 encoded data - default
  serialCommand.addCommand("hex", hexModeOn_command);         // rdata commands send hex encoded data
  serialCommand.addCommand("help", help_command);             // Print list of commands
  serialCommand.setDefaultHandler(unrecognized);      // Handler for any command that isn't matched 
  
  rdatac_command();
  serialCommand.readSerial();  
  
  //Para muestrear durante 10 segundos:
  /*
  for (int prueba = 1; prueba <=10000; prueba++){ //prueba <=5000
    delayMicroseconds(1440);
    sendSamples();
   }
  */
}
 
//PARA TIEMPO QUE DURA EL MUESTREO:
unsigned long tiempo0=0;
unsigned long tiempo1=0;
unsigned long delta;

int y=1;

void loop() {  
  
  sendSamples();           
  delayMicroseconds(1440);
  
  // para conocer el tiempo que demora en ejecutar una muestra:
  /*delay(5000);
  tiempo0 = micros();
  sendSamples();
  tiempo1 = micros();
  delta = (tiempo1 - tiempo0);
  Serial.println(delta); //Tiempo que demora en ejecutar función sendSamples(): 564 us, tolerancia de +-4 us.    
  */
}

long hexToLong(char *digits) {
  using namespace std;
  char *error;
  long n = strtol(digits, &error, 16);
  if ( *error != 0 ) { 
    return -1; // error
  } 
  else {
    return n;
  }
}

void outputHexByte(int value) {
  int clipped = value & 0xff;
  char charValue[3];
  sprintf(charValue, "%02X", clipped);
  WiredSerial.print(charValue);
}

void encodeHex(char* output, char* input, int inputLen) {
  register int count = 0;
  for (register int i=0; i < inputLen; i++) {
    register uint8_t lowNybble = input[i] & 0x0f;
    register uint8_t highNybble = input[i] >> 4;
    output[count++] = hexDigits[highNybble];
    output[count++] = hexDigits[lowNybble];
  }
  output[count] = 0;
}

void version_command() {
  WiredSerial.println("200 Ok");
  WiredSerial.println(driverVersion);
  WiredSerial.println(); 
}

void status_command() {
  WiredSerial.println("200 Ok");
  WiredSerial.print("Board name: ");   
  WiredSerial.println(boardName);   
  WiredSerial.print("Board maker: ");   
  WiredSerial.println(makerName);   
  WiredSerial.print("Hardware type: ");   
  WiredSerial.println(hardwareType);   
  WiredSerial.print("Max channels: "); 
  WiredSerial.println(maxChannels); 
  detectActiveChannels();
  WiredSerial.print("Number of active channels: "); 
  WiredSerial.println(numActiveChannels); 
  WiredSerial.print("Driver version: "); 
  WiredSerial.println(driverVersion); 
  WiredSerial.println(); 
}

void serialNumber_command() {
  WiredSerial.println("200 Ok");
  WiredSerial.println("Not implemented yet. "); 
  WiredSerial.println(); 
}

void ledOn_command() {
  digitalWrite(PIN_LED,HIGH);  
  WiredSerial.println("200 Ok");
  WiredSerial.println("LED on"); 
  WiredSerial.println(); 
}

void ledOff_command() {
  digitalWrite(PIN_LED,LOW);
  WiredSerial.println("200 Ok");
  WiredSerial.println("LED off"); 
  WiredSerial.println(); 
}

void boardLedOn_command() {
  int state = adc_rreg(ADS129x::GPIO);
  state = state & 0xF7;
  state = state | 0x80;
  adc_wreg(ADS129x::GPIO, state);
  WiredSerial.println("200 Ok");
  WiredSerial.println("Board GPIO LED on"); 
  WiredSerial.println(); 
}

void boardLedOff_command() {
  int state = adc_rreg(ADS129x::GPIO);
  state = state & 0x77;
  adc_wreg(ADS129x::GPIO, state);
  WiredSerial.println("200 Ok");
  WiredSerial.println("Board GPIO LED off"); 
  WiredSerial.println(); 
}

void base64ModeOn_command() {
  base64Mode = true;
  WiredSerial.println("200 Ok");
  WiredSerial.println("Base64 mode on - rdata commands will send bas64 encoded data."); 
  WiredSerial.println(); 
}

void hexModeOn_command() {
  base64Mode = false;
  WiredSerial.println("200 Ok");
  WiredSerial.println("Hex mode on - rdata commands will send hex encoded data"); 
  WiredSerial.println(); 
}

void help_command() {
  WiredSerial.println("200 Ok");
  WiredSerial.println("Available commands: "); 
  serialCommand.printCommands();
  WiredSerial.println();
}

void readRegister_command() {
  using namespace ADS129x; 
  char *arg1; 
  arg1 = serialCommand.next();   
  if (arg1 != NULL) {
    long registerNumber = hexToLong(arg1);
    if (registerNumber >= 0) {
      int result = adc_rreg(registerNumber);
      WiredSerial.print("200 Ok");
      WiredSerial.print(" (Read Register "); 
      outputHexByte(registerNumber); 
      WiredSerial.print(") "); 
      WiredSerial.println();               
      outputHexByte(result);      
      WiredSerial.println();      
    } 
    else {
      WiredSerial.println("402 Error: expected hexidecimal digits."); 
    }
  } 
  else {
    WiredSerial.println("403 Error: register argument missing."); 
  }
  WiredSerial.println();      
}

void writeRegister_command() {
  char *arg1, *arg2; 
  arg1 = serialCommand.next();   
  arg2 = serialCommand.next();  
  if (arg1 != NULL) {
    if (arg2 != NULL) { 
      long registerNumber = hexToLong(arg1);
      long registerValue = hexToLong(arg2);
      if (registerNumber >= 0 && registerValue >= 0) {
        adc_wreg(registerNumber, registerValue);        
        WiredSerial.print("200 Ok"); 
        WiredSerial.print(" (Write Register "); 
        outputHexByte(registerNumber); 
        WiredSerial.print(" "); 
        outputHexByte(registerValue); 
        WiredSerial.print(") "); 
        WiredSerial.println();
      } 
      else {
        WiredSerial.println("402 Error: expected hexidecimal digits."); 
      }
    } 
    else {
      WiredSerial.println("404 Error: value argument missing."); 
    }
  } 
  else {
    WiredSerial.println("403 Error: register argument missing."); 
  }
  WiredSerial.println();      
}


void wakeup_command() {
  using namespace ADS129x; 
  adc_send_command(WAKEUP);
  WiredSerial.println("200 Ok ");
  WiredSerial.println("Wakeup command sent.");
  WiredSerial.println(); 
}

void standby_command() {
  using namespace ADS129x; 
  adc_send_command(STANDBY);
  WiredSerial.println("200 Ok ");
  WiredSerial.println("Standby command sent.");
  WiredSerial.println(); 
}

void reset_command() {
  using namespace ADS129x; 
  adc_send_command(RESET);
  WiredSerial.println("200 Ok ");
  WiredSerial.println("Reset command sent.");
  WiredSerial.println(); 
}

void start_command() {
  using namespace ADS129x; 
  adc_send_command(START);
  WiredSerial.println("200 Ok ");
  WiredSerial.println("Start command sent.");
  WiredSerial.println(); 
}

void stop_command() {
  using namespace ADS129x; 
  adc_send_command(STOP);
  WiredSerial.println("200 Ok ");
  WiredSerial.println("Stop command sent.");
  WiredSerial.println(); 
}

void rdata_command() {
  using namespace ADS129x; 
  while (digitalRead(IPIN_DRDY) == HIGH);
  adc_send_command_leave_cs_active(RDATA);
  WiredSerial.println("200 Ok ");
  sendSample();
  WiredSerial.println(); 
}

void rdatac_command() {
  using namespace ADS129x; 
  detectActiveChannels();
  if (numActiveChannels > 0) { 
    isRdatac = true;
    adc_send_command(RDATAC);
    //WiredSerial.println("200 Ok");         
    //WiredSerial.println("RDATAC mode on."); 
  } else {
    WiredSerial.println("405 Error: no active channels.");
  }
  WiredSerial.println(); 
}

void sdatac_command() {
  using namespace ADS129x; 
  isRdatac = false;
  adc_send_command(SDATAC);
  using namespace ADS129x; 
  WiredSerial.println("200 Ok");
  WiredSerial.println("RDATAC mode off."); 
  WiredSerial.println(); 
}

// This gets set as the default handler, and gets called when no other command matches. 
void unrecognized(const char *command) {
  WiredSerial.println("406 Error: Unrecognized command."); 
  WiredSerial.println();   
}


void detectActiveChannels() {  //set device into RDATAC (continous) mode -it will stream data
  if ((isRdatac) ||  (maxChannels < 1)) return; //we can not read registers when in RDATAC mode
  //Serial.println("Detect active channels: ");
  using namespace ADS129x; 
  numActiveChannels = 0;
  for (int i = 1; i <= maxChannels; i++) {
    delayMicroseconds(1); 
    int chSet = adc_rreg(CHnSET + i);
    gActiveChan[i] = ((chSet & 7) != SHORTED);
    if ( (chSet & 7) != SHORTED) numActiveChannels ++;   
  }
  //WiredSerial.println(numActiveChannels);   //AHORA IMPRIMIRA LA CANTIDAD DE CANALES ACTIVOS EN EL SETUP.
}


int32_t convert_two_complement(uint8_t adc_2, uint8_t adc_1, uint8_t adc_0)
{
  int32_t adc;  
  adc = (int32_t)(adc_0) | ((int32_t)(adc_1) << 8) | ((int32_t)(adc_2) << 16);
  return -(adc & 0x800000L) + (adc & ~0x800000L);
}

//PARA CALCULO DE FRECUENCIA:
int time0=0;
int time1=0;
int diferencia;
float periodo;
float frecu;

void frec(){
  time1=millis();
  diferencia = (time1 - time0);
  periodo = (diferencia/1000.0);
  frecu = ((1.0/periodo)*60.0);
  time0 = time1;
  }
  
//#define testSignal //use this to determine if your software is accurately measuring full range 24-bit signed data -8388608..8388607
#ifdef testSignal
int testInc = 1;
int testPeriod = 100;
byte testMSB, testLSB; 
#endif 

inline void sendSamples(void) { 
  if ((!isRdatac) || (numActiveChannels < 1) )  {
    WiredSerial.println("nada"); 
    return;
    }
  if (digitalRead(IPIN_DRDY) == HIGH) {
    //WiredSerial.println("Pin DRDY pasó a HIGH");
    return; 
    }
  //WiredSerial.println("Va a segunda funcion");    
  sendSample(); 
}

// Use SAM3X DMA
inline void sendSample(void) { 
  //WiredSerial.println("A");  // Para detectar posibles fallas
  digitalWrite(PIN_CS, LOW);
  register int numSerialBytes = (3 * (maxChannels+1)); //24-bits header plus 24-bits per channel
  uint8_t returnCode = spiRec(serialBytes, numSerialBytes);
  //WiredSerial.println("B");  // Para detectar posibles fallas
  digitalWrite(PIN_CS, HIGH);
  //WiredSerial.println("C");  // Para detectar posibles fallas
  register unsigned int count = 0;
  
  //WiredSerial.print(convert_two_complement(serialBytes[3], serialBytes[4], serialBytes[5]));  // ch1 midiendo V6 OK
  //WiredSerial.print(",");
  //WiredSerial.println(convert_two_complement(serialBytes[6], serialBytes[7], serialBytes[8]));  // ch2 OK
  //WiredSerial.print(",");
  //WiredSerial.println(convert_two_complement(serialBytes[9], serialBytes[10], serialBytes[11]));  //ch3 OK 
  //WiredSerial.print(",");
  //WiredSerial.print(convert_two_complement(serialBytes[12], serialBytes[13], serialBytes[14])); //ch4 OK
  //WiredSerial.print(",");
  //WiredSerial.print(convert_two_complement(serialBytes[15], serialBytes[16], serialBytes[17])); //ch5 OK 
  //WiredSerial.print(",");
  //WiredSerial.print(convert_two_complement(serialBytes[18], serialBytes[19], serialBytes[20])); //ch6 OK
  //WiredSerial.print(",");
  //WiredSerial.print(convert_two_complement(serialBytes[21], serialBytes[22], serialBytes[23])); //ch7 OK
  //WiredSerial.print(",");
  //WiredSerial.println(convert_two_complement(serialBytes[24], serialBytes[25], serialBytes[26])); //ch8 OK
  
  //Lo siguiente sirve para hacer equivalencia entre el rango de magnitud de la señal a 0-1023, ojo! amplitud de señal simulada debe ser de 2 mV:
  //Se enviará la señal correspondiente al canal 2:                                                                          
  //standard = map((convert_two_complement(serialBytes[6], serialBytes[7], serialBytes[8])), -3000, 3000, 0, 1023);  
  standard = map((convert_two_complement(serialBytes[9], serialBytes[10], serialBytes[11])), -3000, 3000, 0, 1023);  
  //Ahora se fija un umbral para detectar onda R de la señal standard, el umbral usado es 1000, que tiene equivalencia de 682.
  if(standard>750){
    digitalWrite(16,HIGH); //22 para mega
    }
  if(standard<750){
    digitalWrite(16,LOW);
    }
      
//RESULTADOS DE LA ESTANDARIZACIÓN Y FRECUENCIA DE LA SEÑAL:
  WiredSerial.println(standard);   //Ver primero en monitor serial y en el plotter la señal standard, nada mas, solo esa, si esta bien, continuar. 
  //WiredSerial.print("  ");       //Imprimir las tres lineas, ver en monitor serial, debe ir cambiando la frec desde el simulador y tendría que cambiar frecu.
  //WiredSerial.println(frecu);    //Si responde bien, comentar estas tres lineas y ver la transmisión a la app con los serialwrite.


//LO SIGUIENTE FUNCIONA BIEN PARA TRANSMITIR A BLUETOOTH CON PUERTO SERIAL PROPIO:
  byte Data[4];
  int bateria = 99;   // En caso de alimentar el uC con una bateria, medir su voltaje con un analogRead y hacer equivalencia a % de carga.
  Data[0]=standard/256;
  Data[1]=standard%256;
  Data[2]=bateria;
  Data[3]=frecu;
  myBT.write(Data[0]);
  myBT.write(Data[1]);
  myBT.write(Data[2]);
  myBT.write(Data[3]);

/*
  if (y==3){
    byte Data[4];
    int bateria = 99;   // En caso de alimentar el uC con una bateria, medir su voltaje con un analogRead y hacer equivalencia a % de carga.
    Data[0]=standard/256;
    Data[1]=standard%256;
    Data[2]=bateria;
    Data[3]=frecu;
    myBT.write(Data[0]);
    myBT.write(Data[1]);
    myBT.write(Data[2]);
    myBT.write(Data[3]);
    //WiredSerial.println("*************");
    y=1;
    }
  y++;  
*/  
//PROBAR PARA MICRO SD:
  String dataString = "";
  dataString += String(convert_two_complement(serialBytes[3], serialBytes[4], serialBytes[5]));
  dataString += ",";
  dataString += String(convert_two_complement(serialBytes[6], serialBytes[7], serialBytes[8]));
  dataString += ",";
  dataString += String(convert_two_complement(serialBytes[9], serialBytes[10], serialBytes[11]));
  dataString += ",";
  dataString += String(convert_two_complement(serialBytes[12], serialBytes[13], serialBytes[14]));
  dataString += ",";
  dataString += String(convert_two_complement(serialBytes[15], serialBytes[16], serialBytes[17]));
  dataString += ",";
  dataString += String(convert_two_complement(serialBytes[18], serialBytes[19], serialBytes[20]));
  dataString += ",";
  dataString += String(convert_two_complement(serialBytes[21], serialBytes[22], serialBytes[23]));
  dataString += ",";
  dataString += String(convert_two_complement(serialBytes[24], serialBytes[25], serialBytes[26]));
  digitalWrite(chipSelect,LOW);
  File dataFile = SD.open("registroECG.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    //WiredSerial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }                                                                                                                                  
  digitalWrite(chipSelect,HIGH);
}

void adsSetup() { //default settings for ADS1298 and compatible chips
  //SPI.begin();
  //SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
  //SPI.beginTransaction(settingsFE);
  using namespace ADS129x;
  // Send SDATAC Command (Stop Read Data Continuously mode)
  delay(1000); //pause to provide ads129n enough time to boot up...
  adc_send_command(SDATAC);
  // delayMicroseconds(2);
  delay(100); 
  int val = adc_rreg(ID) ;
  switch (val & B00011111 ) { //least significant bits reports channels
  case  B10000: //16
    hardwareType = "ADS1294";
    maxChannels = 4;
    break;
  case B10001: //17
    hardwareType = "ADS1296";
    maxChannels = 6; 
    break;
  case B10010: //18
    hardwareType = "ADS1298";
    maxChannels = 8; 
    break;
  case B11110: //30
    hardwareType = "ADS1299";
    maxChannels = 8; 
    break;
  default: 
    maxChannels = 0;
  }
  //Para confirmar comunicación con front end usado se imprimen los siguientes registros: 
  //WiredSerial.print("Device Type (ID Control Register): "); WiredSerial.println(val); WiredSerial.print("Channels: "); WiredSerial.println(maxChannels); 
  //WiredSerial.print("Tarjeta Front end: "); WiredSerial.println(hardwareType);         
  if (maxChannels == 0) { //error mode.  Si hay un error con el numero de canales leidos, hace el blink led por siempre.
    while(1) { //loop forever 
      digitalWrite(PIN_LED, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(500);               // wait for half second`
      digitalWrite(PIN_LED, LOW);    // turn the LED off by making the voltage LOW
      delay(500); 
    } //while forever
  } //error mode
  // All GPIO set to output 0x0000: (floating CMOS inputs can flicker on and off, creating noise)
  adc_wreg(GPIO, 0);
  //adc_wreg(CONFIG3,PD_REFBUF | CONFIG3_const); 
  adc_wreg(CONFIG3, 0xCD);    
  adc_wreg(RLD_SENSP, 0xFF);  
  adc_wreg(RLD_SENSN, 0xFF); 
  adc_wreg(CONFIG1,HIGH_RES_500_SPS);
  adc_wreg(CONFIG2, 0x11 );  
  adc_wreg(WCT1, 0x0B); // Para terminal central de wilson, captura de precordiales.
  adc_wreg(WCT2, 0xD4);
  
  for (int i = 1; i <= 8; ++i) {
    adc_wreg(CHnSET + i, ELECTRODE_INPUT | GAIN_1X); //report this channel with x12 gain              //XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
    //adc_wreg(CHnSET + i, ELECTRODE_INPUT | GAIN_12X); //report this channel with x12 gain
    //adc_wreg(CHnSET + i, TEST_SIGNAL | GAIN_12X); //create square wave
    //adc_wreg(CHnSET + i,SHORTED); //disable this channel
  }
  digitalWrite(PIN_START, HIGH);
  int conf1 = adc_rreg(CONFIG1);
  int conf2 = adc_rreg(CONFIG2);
  int conf3 = adc_rreg(CONFIG3);
  int sensp = adc_rreg(RLD_SENSP);
  int sensn = adc_rreg(RLD_SENSN);
  int ch1   = adc_rreg(CH1SET); 
  int ch2   = adc_rreg(CH2SET); 
  int ch3   = adc_rreg(CH3SET); 
  int ch4   = adc_rreg(CH4SET); 
  int ch5   = adc_rreg(CH5SET); 
  int ch6   = adc_rreg(CH6SET); 
  int ch7   = adc_rreg(CH7SET); 
  int ch8   = adc_rreg(CH8SET); 
  int wct1 = adc_rreg(WCT1);
  int wct2 = adc_rreg(WCT2);
  /*
  WiredSerial.print("valor de config 1: "); WiredSerial.println(conf1); // imprime 134: 10000110
  WiredSerial.print("valor de config 2: "); WiredSerial.println(conf2); // imprime 16:  00010000
  WiredSerial.print("valor de config 3: "); WiredSerial.println(conf3); // imprime 192: 11000000
  WiredSerial.print("valor de sensp: "); WiredSerial.println(sensp); // imprime : 
  WiredSerial.print("valor de sensn: "); WiredSerial.println(sensn); // imprime :
  WiredSerial.print("valor de channel 1: "); WiredSerial.println(ch1); // imprime :   
  WiredSerial.print("valor de channel 2: "); WiredSerial.println(ch2); // imprime : 
  WiredSerial.print("valor de channel 3: "); WiredSerial.println(ch3); // imprime : 
  WiredSerial.print("valor de channel 4: "); WiredSerial.println(ch4); // imprime : 
  WiredSerial.print("valor de channel 5: "); WiredSerial.println(ch5); // imprime : 
  WiredSerial.print("valor de channel 6: "); WiredSerial.println(ch6); // imprime : 
  WiredSerial.print("valor de channel 7: "); WiredSerial.println(ch7); // imprime : 
  WiredSerial.print("valor de channel 8: "); WiredSerial.println(ch8); // imprime : 
  WiredSerial.print("valor de WCT1: "); WiredSerial.println(wct1); // imprime : 
  WiredSerial.print("valor de WCT2: "); WiredSerial.println(wct2); // imprime : 
  */
  //WiredSerial.println("B");
}

void arduinoSetup(){
  pinMode(PIN_LED, OUTPUT);
  using namespace ADS129x;
  //prepare pins to be outputs or inputs
  pinMode(PIN_SCLK, OUTPUT); //optional - SPI library will do this for us
  pinMode(PIN_DIN, OUTPUT); //optional - SPI library will do this for us
  pinMode(PIN_DOUT, INPUT); //optional - SPI library will do this for us
  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_START, OUTPUT);
  pinMode(IPIN_DRDY, INPUT);
  pinMode(IPIN_RESET, OUTPUT);// *optional
  //start Serial Peripheral Interface
  //spiBegin(PIN_CS);//Bandido                                  //XXXXXXXXXXXXXXXXXXXXXXXXX
  //spiInit(MSBFIRST, SPI_MODE1, SPI_CLOCK_DIVIDER);            //XXXXXXXXXXXXXXXXXXXXXXXXX
  
  SPI.begin();

  //SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
  SPI.beginTransaction(settingsFE);
  //Start ADS1298
  delay(500); //wait for the ads129n to be ready - it can take a while to charge caps
  delay(10); // wait for oscillator to wake up  
  delay(1);
  digitalWrite(IPIN_RESET, HIGH);
  delay(1000);// *optional
  digitalWrite(IPIN_RESET, LOW);  // pin RESET va negado al chip, cuando escribimos LOW resetea el chip.
  delay(1);// *optional
  digitalWrite(IPIN_RESET, HIGH);
  delay(1);  // *optional Wait for 18 tCLKs AKA 9 microseconds, we use 1 millisecond
  //WiredSerial.println("A");
} //setup()   
