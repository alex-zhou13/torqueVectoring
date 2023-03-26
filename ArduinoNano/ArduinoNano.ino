#include <Wire.h>
#include <Adafruit_ISM330DHCX.h>

Adafruit_ISM330DHCX ism330dhcx;

void setup() {
  Serial.begin(115200);

  if (!ism330dhcx.begin_I2C()) {
    Serial.println("Failed to find ISM330DHCX chip");
  }

  ism330dhcx.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  ism330dhcx.setGyroRange(ISM330DHCX_GYRO_RANGE_4000_DPS);

  pinMode (2,OUTPUT); //ADDED!!
}

void loop() {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  ism330dhcx.getEvent(&accel, &gyro, &temp);

  Serial.print("Acceleration (m/s^2): ");
  Serial.print(accel.acceleration.x);
  Serial.print(", ");
  Serial.print(accel.acceleration.y);
  Serial.print(", ");
  Serial.print(accel.acceleration.z);
  Serial.println();

  Serial.print("Gyro (deg/s): ");
  Serial.print(gyro.gyro.x);
  Serial.print(", ");
  Serial.print(gyro.gyro.y);
  Serial.print(", ");
  Serial.print(gyro.gyro.z);
  Serial.println();
  
  delay(500);
}