#include <Wire.h>
#include <MPU6050_tockn.h>
#include <Kalman.h>

MPU6050 mpu6050(Wire);
Kalman kalmanX;
Kalman kalmanY;

double roll, pitch;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  Serial.print("\n");
  Serial.print("roll");
  Serial.print("\t");
  Serial.println("pitch");
}

void loop() {
  mpu6050.update();

  roll = mpu6050.getAngleX();
  pitch = mpu6050.getAngleY();

  double rollKalman = kalmanX.getAngle(roll, mpu6050.getGyroX(), mpu6050.getAccZ());
  double pitchKalman = kalmanY.getAngle(pitch, mpu6050.getGyroY(), mpu6050.getAccZ());

  Serial.print(rollKalman);
  Serial.print("\t");
  Serial.println(pitchKalman);

  delay(10);
}
