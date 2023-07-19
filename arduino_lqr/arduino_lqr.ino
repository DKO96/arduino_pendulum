#include <Arduino_LSM9DS1.h>

float x, y, z;
float degreesY = 0;

void setup() {
  // initialize serial monitor
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  // initialize imu
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  // initialize built-in led
  pinMode(LED_BUILTIN, OUTPUT);

  // print accelerometer sample rate
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println("Hz");
}

void loop() {
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
  }

  // left tilt
  if (y > 0.0) {
    y = 100 * y;
    degreesY = map(y, 0, 97, 0, 90);
    degreesY = constrain(degreesY, 0, 90);
    Serial.print("Tilting left ");
    Serial.print(degreesY);
    Serial.println("  degrees");
  }
  // right tilt
  if (y < -0.0) {
    y = 100 * y;
    degreesY = map(y, 0, -100, 0, 90);
    degreesY = constrain(degreesY, 0, 90);
    Serial.print("Tilting right ");
    Serial.print(degreesY);
    Serial.println("  degrees");
  }
  
  // led output
  int blink_delay = map(abs(degreesY), 0, 90, 75, 10);
  blink_delay = constrain(blink_delay, 10, 75);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(blink_delay);
  digitalWrite(LED_BUILTIN, LOW);
  delay(blink_delay);

}