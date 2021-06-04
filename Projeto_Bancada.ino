//BIBLIOTECAS

#include <Servo.h>
#include <Wire.h> 
#include <HX711_ADC.h>
#include <EEPROM.h>

//DEFINIÇÃO DOS PINOS ANALóGICOS

const int pitot_tube_pin = A0; //PINO UTILIZADO PELO TUBO DE PITOT
const int Sensor_tensao_controle = A1; //PINO UTILIZADO PELO SENSOR DE TENSÃO DO SISTEMA DE CONTROLE
const int Sensor_tensao_potencia = A2;//PINO UTILIZADO PELO SENSOR DE TENSÃO DO SISTEMA DE POTENCIA
const int Sensor_corrente_controle = A3; //PINO UTILIZADO PELO SENSOR DE CORRENTE DO SISTEMA DE CONTROLE
const int c_sens = A4; //PINO UTILIZADO PELO SENSOR DE CORRENTE DO SISTEMA DE POTENCIA
const int LM35 = A5; //PINO UTILIZADO PELO SENSOR DE TEMPERATURA LM35


//PARÂMETROS INICIAIS PARA SENSOR DE CORRENTE DO MOTOR ACS758

float volt; //TENSÃO LIDA PELO SENSOR
float c_value; //VARIáVEL PARA VALOR PARAMETRO DO SENSOR
float offset_corrente_pot; //VARIÁVEL PARA OFFSET DO SENSOR DE CORRENTE
float cutOffLimit = 1.00; //VALOR DE CUTOFF (MíNIMO LIDO)
float quiescent_Output_voltage =0.5; 
float QOV; //VARIÁVEL QOV
float cutoff;//VARIÁVEL CUTOFF
int mVperAmp = 66; //VARIÁVEL QUE DEFINE A CORRENTE PELA TENSÃO
int RawValue = 2;
int ACSoffset = 2500; //OFFSET PADRÃO DATASHEET
double Voltage = 0; //DEFININDO VALOR INICIAL PARA TENSÃO
double Amps = 0; //DEFININDO VALOR INICIAL PARA CORRENTE
float vetCorrente[300];

//PARAMETROS PARA SENSORES DE TENSÃO DO SISTEMA DE POTENCIA E DE CONTROLE

float tensaoEntrada = 0.0; //VARIÁVEL PARA ARMAZENAR O VALOR DE TENSÃO DE ENTRADA DO SENSOR
float tensaoMedida = 0.0;//VARIÁVEL PARA ARMAZENAR O VALOR DA TENSÃO DE SAÍDA 
float offset_pot=0.95;//VARIAVEL DE OFFSET DE TENSAO DO SENSOR DE TENSAO DA BATERIA DE POTENCIA
float offset_cont=0.7;//VARIAVEL DE OFFSET DE TENSAO DO SENSOR DE TENSAO DA BATERIA DE CONTROLE
float valorR1 = 30000.0; //VALOR DO RESISTOR 1 DO DIVISOR DE TENSÃO
float valorR2 = 7500.0; // VALOR DO RESISTOR 2 DO DIVISOR DE TENSÃO
int leituraSensor = 0; //VARIÁVEL PARA ARMAZENAR A LEITURA DO PINO ANALÓGICO

//PARAMETROS PARA O SENSOR DE VELOCIDADE DO AR

float pitot();//FUNÇÃO DO SENSOR DE VELOCIDADE
float V_0 = 5.0; //TENSÃO DE ALIMENTAÇÃO
float rho = 1.274; //RHO
int offset = 0; //VALOR DE OFFSET
int offset_size = 10;
int veloc_mean_size = 20;
int zero_span = 2;
int velocidade;

//PARÂMETROS PARA O SENSOR DE ROTAÇÃO 3144E

#define TEST_DELAY   999
volatile int counter = 2;
int RPM; //VARIÁVEL RPM

//PARÂMETROS PARA O SENSOR DE MEDIÇÃO DE TORQUE (CÉLULA DE CARGA)

const int HX711_dout = 4; //mcu > HX711 dout pin
const int HX711_sck = 5; //mcu > HX711 sck pin
HX711_ADC LoadCell(HX711_dout, HX711_sck);//Amplificador HX711
const int calVal_eepromAdress = 0;
long t;
float peso;

//PARÂMETROS PARA O SENSOR DE TEMPERATURA LM35
float temperatura; // Variável que armazenará a temperatura medida


//VARIÁVEIS CRIADAS PARA A SAÍDA DA PORTA SERIAL

float massa;
float rotacao;
float corrente_bateria_control;
float corrente_bateria_potencia;
float tensao_bateria_control; 
float tensao_bateria_pot; 

//FUNÇõES CRIADAS PARA CADA UM DOS SENSORES
  //PARA VERIFICAR CADA TENSÃO VÁ ATé O FINAL DO CODIGO

float tensao_bateria_controle();//FUNÇÃO PARA A TENSÃO DA BATERIA DE CONTROLE
float tensao_bateria_potencia();///FUNÇÃO PARA TENSÃO DA BATERIA DE POTENCIA
float sensor_corrente_potencia(); 
float Load_Cell();
float RPM_3144E();
float sensor_corrente_controle();
float temperatura_bateria();

void setup() {
  //DEFINIÇÃO DO MODO DE CADA UM DOS PINOS DE INPUT
  pinMode(c_sens, INPUT);
  pinMode(Sensor_tensao_controle, INPUT); 
  pinMode(Sensor_tensao_potencia, INPUT);
  pinMode(Sensor_corrente_controle, INPUT);
  
  //PORTA SERIAL DEFINIDA COMO 9600 BPS
  Serial.begin(9600);
  delay(10);

  //INTERRUPÇÃO SETADA PARA O SETOR DE ROTAÇÃO
  attachInterrupt(0,count,RISING);
  for (int ii=0; ii<offset_size; ii++){
    offset += analogRead(A0)-(1023/2);
  }
  offset /= offset_size;

  //SETUP PARA A CÉLULA DE CARGA
  LoadCell.begin();
  float calibrationValue; // calibration value (see example file "Calibration.ino")
  calibrationValue = -450.15; // uncomment this if you want to set the calibration value in the sketch
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

  //PUXANDO VALORES DAS FUNÇÕES E COLOCANDO EM VARIÁVEIS
  
  massa = Load_Cell(); //EXTRAÇÃO DO VALOR DE SAÍDA DA FUNÇÃO DA CÉLULA DE CARGA
  rotacao = RPM_3144E(); //EXTRAÇÃO DO VALOR DE SAÍDA DA FUNÇÃO DE RPM
  velocidade = pitot(); //EXTRAÇÃO DO VALOR DE SAÍDA DO SENSOR TUBO DE PITOT
  tensao_bateria_control = tensao_bateria_controle(); //EXTRAÇÃO DO VALOR DE SAÍDA DO SENSOR DE TENSÃO
  tensao_bateria_pot = tensao_bateria_potencia(); //EXTRAÇÃO DO VALOR DE SAÍDA DO SENSOR DE TENSÃO
  corrente_bateria_control = sensor_corrente_controle(); //EXTRAÇÃO DO VALOR DE SAÍDA DO SENSOR DE CORRENTE
  temperatura = temperatura_bateria(); //EXTRAÇÃO DO VALOR DE SAÍDA DO SENSOR DE TENSÃO
  corrente_bateria_potencia = sensor_corrente_potencia(); //EXTRAÇÃO DO VALOR DE SAÍDA DO SENSOR DE TEMPERATURA

  //COLOCANDO VARIÁVEIS NA PORTA SERIAL
  
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
  Serial.print(",");
  Serial.print(corrente_bateria_potencia);
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
      return -i;
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
    adc_avg+= analogRead(pitot_tube_pin)-offset;
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

float sensor_corrente_potencia(){
  QOV = quiescent_Output_voltage*5.0;
  cutoff = 0.04/cutOffLimit;
  volt = (5.0 / 1023.0)*analogRead(c_sens);
  volt = volt - QOV + 0.007;
  offset_corrente_pot = -2.00;
  c_value = (volt/0.04 - (offset_corrente_pot))*-1;
  return c_value;
}