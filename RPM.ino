// Author: Silverjoda
// Edited By CWKshop.com
// 1/25/2020
// Target: Arduino
#include <Wire.h> 

// Code uses external interrupt to count the incoming pulses.


// The amount of time (in milliseconds) between tests
#define TEST_DELAY   999


volatile int counter = 2;
int RPM;

void setup() {

  Serial.begin(9600);
  
  attachInterrupt(0,count,RISING);

}
void loop() {
  delay(1000);  //Delay almost 1 second.  
  Serial.print(counter * 60); // Counter * 60 seconds.
  Serial.println("rpm.");
  
  RPM = (counter * 60);
   
   //lcd.print("Name");
   delay(100);

  counter = 0;
  
 
  
}
 
void count()
{
 counter++;
}
