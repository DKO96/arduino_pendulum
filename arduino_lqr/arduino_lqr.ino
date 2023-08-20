#include <MPU6050.h>
#include <Wire.h>
#include <Servo.h>

#define ACCELE_RANGE 4
#define GYROSC_RANGE 500

Servo ESC;

int STOP_CCW = 88; 
int STOP_CW = 85;
int CCW = 83;
int CW = 90;

const int MPU_addr = 0x68; // I2C address of the MPU-6050
float AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
double g = 9.81;
double yawFiltered = 0;
const double alpha = 0.05;

double yawOffset = 0.0;
bool isCalibrated = false;








void setup() {
  Serial.begin(9600);
  ESC.attach(9, 1000, 2000);

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  calibrateYaw();
}


void calibrateYaw() {
  int calibrationSamples = 100;
  double yawSum = 0;

    for (int i = 0; i < calibrationSamples; i++) {
      Wire.beginTransmission(MPU_addr);
      Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
      AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
      AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)

      yawSum += atan2(AcY, AcX) * (180.0 / PI);

      delay(10);  // Short delay between readings
    }

    yawOffset = yawSum / calibrationSamples;
    isCalibrated = true;
    Serial.print("Calibration Offset: ");
    Serial.println(yawOffset);
}


void loop() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  float AcX_g = AcX / 65536 * ACCELE_RANGE;
  float AcY_g = AcY / 65536 * ACCELE_RANGE;
  // double yaw = atan2(AcY_g, AcX_g) * (180.0 / PI);
  double yaw = atan2(AcY_g, AcX_g) * (180.0 / PI) - yawOffset;

  yawFiltered = alpha * yaw + (1.0 - alpha) * yawFiltered;
  // yawFiltered = yawFiltered - 0.2;

  // Serial.print("Filtered Yaw (degs): ");
  Serial.print(-60);
  Serial.print(" ");
  Serial.print(60);
  Serial.print(" ");
  Serial.println(yawFiltered);

  // For CW direction of pendulum
  if (yawFiltered > 5.0) {
    ESC.write(CW);
  } 
  else if (yawFiltered < 5.0 && yawFiltered > 0.0) {
    ESC.write(STOP_CW);
  }

  // For CCW direction of pendulum
  if (yawFiltered < -5.0) {
    ESC.write(CCW);
  }
  else if (yawFiltered > -5.0 && yawFiltered < 0.0) {
    ESC.write(STOP_CCW);
  }

  delay(1);

}