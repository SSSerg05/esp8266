#include <Arduino.h>

// put function declarations here:


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  analogReference(EXTERNAL);
}

void loop() {
  // put your main code here, to run repeatedly:
  int signal = analogRead(0);
  Serial.println(signal);
}

