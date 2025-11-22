#include <Arduino.h>

void setup() {
  Serial.begin(9600); 
  Serial.println("=== Setup started ===");
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("=== Setup finished ===");
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("LED ON");
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  Serial.println("LED OFF");
  digitalWrite(LED_BUILTIN, LOW);

}
