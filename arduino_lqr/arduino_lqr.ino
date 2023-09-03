#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

#define ACCELE_RANGE 4
#define GYROSC_RANGE 500
#define BIT_RANGE 65536
#define TIMESTEP 0.1

unsigned long lastTime = 0;
unsigned long currentTime = 0;

// SENSOR VARIABLES 
const int MPU_addr = 0x68; // I2C address of the MPU-6050
float AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float yaw = 0;
float KalmanYaw = 0, KalmanUncertaintyYaw = 4;
float KalmanOutput[] = {0,0};
float processNoise = 16;
float measurementNoise = 9;

// MOTOR VARIABLES
Servo ESC;
const int CW = 90;
const int STOP_CW = 85;
const int CCW = 83;
const int STOP_CCW = 88;


void MPU_signal();
void KalmanFilter(float &KalmanState, float &KalmanUncertainty,
                  float KalmanInput, float KalmanMeasurement);

void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  ESC.attach(9, 1000, 2000);  
  Serial.begin(115200);
}

void loop() {
  currentTime = millis();
  if (currentTime - lastTime >= (TIMESTEP*1000)) {
    MPU_signal();
    KalmanFilter(KalmanYaw, KalmanUncertaintyYaw, GyZ, yaw);
    Serial.print(60);
    Serial.print(" ");
    Serial.print(-60);
    Serial.print(" ");
    Serial.println(KalmanYaw);
  }
}

///////////////////////////////////////////////////////////
///// FUNCTIONS 
//////////////////////////////////////////////////////////
void MPU_signal(){
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

  AcX = AcX/BIT_RANGE * ACCELE_RANGE;
  AcY = AcY/BIT_RANGE * ACCELE_RANGE;
  AcZ = AcZ/BIT_RANGE * ACCELE_RANGE;
  GyX = GyX/BIT_RANGE * GYROSC_RANGE;
  GyY = GyY/BIT_RANGE * GYROSC_RANGE;
  GyZ = GyZ/BIT_RANGE * GYROSC_RANGE;

  yaw = atan2(AcY, AcX) * (180.0/PI); 
  // Serial.print(" AcX = "); Serial.print(AcX); Serial.print("g ");
  // Serial.print(" | AcY = "); Serial.print(AcY); Serial.print("g ");
  // Serial.print(" | AcZ = "); Serial.print(AcZ); Serial.println("g ");
  // Serial.print(" GyX = "); Serial.print(GyX); Serial.print("d/s ");
  // Serial.print(" | GyY = "); Serial.print(GyY); Serial.print("d/s ");
  // Serial.print("|GyZ= "); Serial.print(GyZ); Serial.println("d/s \n");
}

void KalmanFilter(float &KalmanState, float &KalmanUncertainty, 
                  float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + TIMESTEP*KalmanInput;
  KalmanUncertainty = KalmanUncertainty + TIMESTEP*TIMESTEP*processNoise;

  float KalmanGain = KalmanUncertainty / (KalmanUncertainty + measurementNoise);
  KalmanState = KalmanState + KalmanGain*(KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
}