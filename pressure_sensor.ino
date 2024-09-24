const int analogPin = A0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  int pressureValue = analogRead(analogPin); 
  Serial.println(pressureValue);
  delay(100);
}
