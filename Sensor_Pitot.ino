  
float V_0 = 5.0;
float rho = 1.274;

int offset = 0;
int offset_size = 10;
int veloc_mean_size = 20;
int zero_span = 2;

void setup() {
  Serial.begin(9600);
  for (int ii=0; ii<offset_size; ii++){
    offset += analogRead(A0)-(1023/2);
  }
  offset /= offset_size;
  
}

void loop() {
  float adc_avg = 0; 
  float veloc = 0;

  for (int ii=0; ii<veloc_mean_size; ii++){
    adc_avg+=analogRead(A0)-offset;
  }
  adc_avg/=veloc_mean_size;
   if ((adc_avg>512-zero_span) and (adc_avg<512+zero_span)){
  } else{
    if (adc_avg<512){
      veloc = -sqrt((10000.0*((adc_avg/1023.0)-0.5))/rho);
    } else{
      veloc = sqrt((10000.0*((adc_avg/1023.0)-0.5))/rho);
    }
  }
  Serial.println(veloc);
  delay(10);
}
