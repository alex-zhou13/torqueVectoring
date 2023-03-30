#include <Wire.h>
#include <Adafruit_ISM330DHCX.h>

// Source: https://learn.adafruit.com/lsm6dsox-and-ism330dhc-6-dof-imu/arduino

Adafruit_ISM330DHCX ism330dhcx;

const int steeringPin = A0; // analog input for steering potentiometer
const int throttlePin = A1; // analog input for throttle potentiometer

int steeringValue; // analog value for steering
int throttleValue; // analog value for throttle

void setup() {
  Serial.begin(115200);

  if (!ism330dhcx.begin_I2C()) {
    Serial.println("Failed to find ISM330DHCX chip");
  }

  ism330dhcx.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  // Serial.print("Accelerometer range set to: ");
  // switch (ism330dhcx.getAccelRange()) {
  // case LSM6DS_ACCEL_RANGE_2_G:
  //   Serial.println("+-2G");
  //   break;
  // case LSM6DS_ACCEL_RANGE_4_G:
  //   Serial.println("+-4G");
  //   break;
  // case LSM6DS_ACCEL_RANGE_8_G:
  //   Serial.println("+-8G");
  //   break;
  // case LSM6DS_ACCEL_RANGE_16_G:
  //   Serial.println("+-16G");
  //   break;
  // }

  ism330dhcx.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  // Serial.print("Gyro range set to: ");
  // switch (ism330dhcx.getGyroRange()) {
  // case LSM6DS_GYRO_RANGE_125_DPS:
  //   Serial.println("125 degrees/s");
  //   break;
  // case LSM6DS_GYRO_RANGE_250_DPS:
  //   Serial.println("250 degrees/s");
  //   break;
  // case LSM6DS_GYRO_RANGE_500_DPS:
  //   Serial.println("500 degrees/s");
  //   break;
  // case LSM6DS_GYRO_RANGE_1000_DPS:
  //   Serial.println("1000 degrees/s");
  //   break;
  // case LSM6DS_GYRO_RANGE_2000_DPS:
  //   Serial.println("2000 degrees/s");
  //   break;
  // case ISM330DHCX_GYRO_RANGE_4000_DPS:
  //   Serial.println("4000 degrees/s");
  //   break;
  // }

  pinMode(2,OUTPUT); //ADDED!!

  pinMode(steeringPin, INPUT);
  pinMode(throttlePin, INPUT);

  digitalWrite(12, HIGH);
}

void loop() {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  ism330dhcx.getEvent(&accel, &gyro, &temp);

  Serial.print(accel.acceleration.x);
  Serial.print(",");
  Serial.print(accel.acceleration.y);
  Serial.print(",");
  Serial.print(accel.acceleration.z);

  // Serial.print("Gyro (deg/s): ");
  // Serial.print(gyro.gyro.x);
  // Serial.print(", ");
  // Serial.print(gyro.gyro.y);
  // Serial.print(", ");
  // Serial.print(gyro.gyro.z);
  // Serial.println();

  steeringValue = analogRead(steeringPin);
  throttleValue = analogRead(throttlePin);
  Serial.print(",");
  Serial.print(steeringValue);
  Serial.print(",");
  Serial.print(throttleValue);
  Serial.println();
  
  delay(100);
}