//int valor = 0;
String str;
int tamanho;
const int Sensor_corrente_controle = A3;
int mVperAmp = 66; 
int RawValue = 2;
int ACSoffset = 2500;
double Voltage = 0;
double Amps = 0;
float vetCorrente[300];

float Load_Cell();
float RPM_3144E();
float pitot();
float sensor_corrente_controle();
float sensor_corrente_potencia();
float tensao_bateria_controle();
float tensao_bateria_potencia();
float temperatura_bateria();

const int Sensor_tensao_controle = A2;
const int Sensor_tensao_potencia = A1;//PINO ANALÓGICO EM QUE O SENSOR ESTÁ CONECTADO
 
float tensaoEntrada = 0.0; //VARIÁVEL PARA ARMAZENAR O VALOR DE TENSÃO DE ENTRADA DO SENSOR
float tensaoMedida = 0.0;//VARIÁVEL PARA ARMAZENAR O VALOR DA TENSÃO MEDIDA PELO SENSOR
float offset_pot=0.6;
float offset_cont=0.6;//VARIAVEL DE OFFSET DE TENSAO DO SENSOR DE TENSAO DA BATERIA DE POT
float valorR1 = 30000.0; //VALOR DO RESISTOR 1 DO DIVISOR DE TENSÃO
float valorR2 = 7500.0; // VALOR DO RESISTOR 2 DO DIVISOR DE TENSÃO
int leituraSensor = 0; //VARIÁVEL PARA ARMAZENAR A LEITURA DO PINO ANALÓGICO


int massa;
int rotacao;
int velocidade;
float tensao_bateria_control;
float tensao_bateria_pot;
float corrente_bateria_control;


//Biblioteca Pitot
float V_0 = 5.0;
float rho = 1.274;

int offset = 0;
int offset_size = 10;
int veloc_mean_size = 20;
int zero_span = 2;

//Variavel para Sensor de Corrente de Controle
int corrente_controle;

//Variavel para Sensor de Tensão de Controle
int tensao_controle = 2;


//Variavel para Sensor de Tensão de Potência
int tensao_potencia;

//Biblioteca para RPM
#include <Wire.h> 
#define TEST_DELAY   999
volatile int counter = 2;
int RPM;

//Biblioteca para Célula de Carga
#include <HX711_ADC.h>
#include <EEPROM.h>

//Pinos para Célula de Carga
const int HX711_dout = 4; //mcu > HX711 dout pin
const int HX711_sck = 5; //mcu > HX711 sck pin

//Amplificador HX711
HX711_ADC LoadCell(HX711_dout, HX711_sck);

const int calVal_eepromAdress = 0;
long t;
float peso;

//Sensor de temperatura LM35
const int LM35 = A5; // Define o pino que lera a saída do LM35
float temperatura; // Variável que armazenará a temperatura medida

void setup() {
  pinMode(Sensor_tensao_controle, INPUT); //DEFINE O PINO COMO ENTRADA
  pinMode(Sensor_tensao_potencia, INPUT);
  pinMode(Sensor_corrente_controle, INPUT);
  Serial.begin(9600);
  delay(10);
  attachInterrupt(0,count,RISING);
  for (int ii=0; ii<offset_size; ii++){
    offset += analogRead(A0)-(1023/2);
  }
  offset /= offset_size;
  
  LoadCell.begin();
  float calibrationValue; // calibration value (see example file "Calibration.ino")
  calibrationValue = 696.0; // uncomment this if you want to set the calibration value in the sketch
#if defined(ESP8266)|| defined(ESP32)
  //EEPROM.begin(512); // uncomment this if you use ESP8266/ESP32 and want to fetch the calibration value from eeprom
#endif
  //EEPROM.get(calVal_eepromAdress, calibrationValue); // uncomment this if you want to fetch the calibration value from eeprom

  long stabilizingtime = 2000; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell.setCalFactor(calibrationValue); // set calibration value (float)
    //Serial.println("Startup is complete");
  }
}

  
void loop() {
  // put your main code here, to run repeatedly:
  massa = Load_Cell();
  rotacao = RPM_3144E();
  velocidade = pitot();
  tensao_bateria_control = tensao_bateria_controle();
  tensao_bateria_pot = tensao_bateria_potencia();
  corrente_bateria_control = sensor_corrente_controle();
  temperatura = temperatura_bateria();
  Serial.print(massa);
  Serial.print(",");
  Serial.print(rotacao);
  Serial.print(",");
  Serial.print(velocidade);
  Serial.print(",");
  Serial.print(tensao_bateria_control);
  Serial.print(",");
  Serial.print(tensao_bateria_pot);
  Serial.print(",");
  Serial.print(corrente_bateria_control);
  Serial.print(",");
  Serial.print(temperatura);
  Serial.print("\n");
  delay(100);
}
float Load_Cell(){

  static boolean newDataReady = 0;
  const int serialPrintInterval = 0; //increase value to slow down serial print activity

  // check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;

  // get smoothed value from the dataset:
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      float i = LoadCell.getData()*-1;
      //Serial.print("Load_cell output val: ");
      //Serial.println(i);
      newDataReady = 0;
      t = millis();
      return i;
    }
  }

  // receive command from serial terminal, send 't' to initiate tare operation:
  if (Serial.available() > 0) {
    float i;
    char inByte = Serial.read();
    if (inByte == 't') LoadCell.tareNoDelay();
  }

  // check if last tare operation is complete:
  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete");
  }


}

float RPM_3144E(){
  //delay(1000);  //Delay almost 1 second.  
  //Serial.print(counter * 60); // Counter * 60 seconds.
  //Serial.println("rpm.");
  
  RPM = (counter * 60);
   
  //lcd.print("Name");
  counter = 0;
  return RPM;
  
}

void count()
{
 counter++;
}

float pitot(){
  float adc_avg = 0; float veloc = 0.0;
  
// average a few ADC readings for stability
  for (int ii=0;ii<veloc_mean_size;ii++){
    adc_avg+= analogRead(A0)-offset;
  }
  adc_avg/=veloc_mean_size;
  
  // make sure if the ADC reads below 512, then we equate it to a negative velocity
  if (adc_avg>512-zero_span and adc_avg<512+zero_span){
  } else{
    if (adc_avg<512){
      veloc = -sqrt((-10000.0*((adc_avg/1023.0)-0.5))/rho);
    } else{
      veloc = sqrt((10000.0*((adc_avg/1023.0)-0.5))/rho);
    }
  }
  return veloc; // print velocity
}

float tensao_bateria_controle(){
   leituraSensor = analogRead(Sensor_tensao_controle); //FAZ A LEITURA DO PINO ANALÓGICO E ARMAZENA NA VARIÁVEL O VALOR LIDO
   tensaoEntrada = (leituraSensor * 5.0) / 1024.0; //VARIÁVEL RECEBE O RESULTADO DO CÁLCULO
   tensaoMedida = (tensaoEntrada / (valorR2/(valorR1+valorR2))-offset_cont); //VARIÁVEL RECEBE O VALOR DE TENSÃO DC MEDIDA PELO SENSOR
   return tensaoMedida;
}

float tensao_bateria_potencia(){ 
   leituraSensor = analogRead(Sensor_tensao_potencia); //FAZ A LEITURA DO PINO ANALÓGICO E ARMAZENA NA VARIÁVEL O VALOR LIDO
   tensaoEntrada = (leituraSensor * 5.0) / 1024.0; //VARIÁVEL RECEBE O RESULTADO DO CÁLCULO
   tensaoMedida = (tensaoEntrada / (valorR2/(valorR1+valorR2)))-offset_pot; //VARIÁVEL RECEBE O VALOR DE TENSÃO DC MEDIDA PELO SENSOR
   return tensaoMedida;

}

float sensor_corrente_controle(){
  RawValue = analogRead(Sensor_corrente_controle);
  Voltage = (RawValue / 1024.0) * 5000; // Gets you mV
  Amps = ((Voltage - ACSoffset) / mVperAmp);
  return Amps;
}

float temperatura_bateria(){
  temperatura = (float(analogRead(LM35))*5/(1023))/0.01;
  return temperatura;
}
