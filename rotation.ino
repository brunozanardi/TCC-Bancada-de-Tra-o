int sensorPin = 3;
int counter = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(sensorPin, INPUT_PULLUP);
}

void loop()
{
  counter++;
  int sensorValue = digitalRead(sensorPin);
  Serial.print(counter);
  Serial.print(" ");
  Serial.println(sensorValue);
  delay(1000);
}
