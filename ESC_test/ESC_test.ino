#include <Servo.h>

Servo ESC;     // create servo object to control the ESC

int STOP_CCW = 87; // ranges from 85 to 88
int STOP_CW = 86;
int CCW = 84;
int CW = 89;

void setup() {
  // Attach the ESC on pin 9
  Serial.begin(9600);
  ESC.attach(9,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
}

void loop() {
  ESC.write(CCW);
  Serial.println(CCW);
  delay(2000);

  ESC.write(STOP_CCW);
  Serial.println(STOP_CCW);
  delay(2000);

  ESC.write(CW);
  Serial.println(CW);
  delay(2000);

  ESC.write(STOP_CW);
  Serial.println(STOP_CW);
  delay(2000);


}