#include <Servo.h>

Servo ESC;     // create servo object to control the ESC

int STOP = 87; // ranges from 85 to 88
int CW = 89;
int CCW = 84;

void setup() {
  // Attach the ESC on pin 9
  Serial.begin(9600);
  ESC.attach(9,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
}

void loop() {
  // Test multidirectional ESC
  ESC.write(STOP);
  Serial.println(STOP);
  delay(2000);

  ESC.write(CCW);
  Serial.println(CCW);
  delay(2000);

  ESC.write(STOP);
  Serial.println(STOP);
  delay(2000);



  // ESC.write(CCW);
  // Serial.println(CCW);
  // delay(2000);



}