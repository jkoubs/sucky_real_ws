#include <Arduino.h>
#include <Wire.h>
#include <MPU9250.h>

MPU9250 mpu;

// === Calibrated offsets ===
float gx_offset =  -3.00;
float gy_offset = -1.64;
float gz_offset = 0.59;

float ax_offset =  -0.020781;
float ay_offset = 0.002566;
float az_offset = -0.016774;

float mag_offset_x = 542.15;
float mag_offset_y = -1557.22;
float mag_offset_z = -415.03;;

// === Soft iron correction matrix ===
const float softIronMatrix[3][3] = {
  { 0.003483, 0.000446, -0.000444 },
  { 0.000446, 0.003383, -0.000160 },
  { -0.000444, -0.000160, 0.002453 },
};
unsigned long lastPrint = 0;
const unsigned long printInterval = 1000;  // Rate: 1 Hz (1000 ms)

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!mpu.setup(0x68)) {
    Serial.println("MPU9250 not connected");
    while (1);
  }

  delay(1000);  // Let sensor stabilize
  Serial.println("Starting IMU data stream...");
}

void loop() {
  if (mpu.update()) {
    unsigned long now = millis();
    if (now - lastPrint >= printInterval) {
      lastPrint = now;

      // === Gyroscope (bias corrected)
      float gx = mpu.getGyroX() - gx_offset;
      float gy = mpu.getGyroY() - gy_offset;
      float gz = mpu.getGyroZ() - gz_offset;

      // === Accelerometer
      float ax = mpu.getAccX();
      float ay = mpu.getAccY();
      float az = mpu.getAccZ();

      // === Magnetometer ===
      // Step 1: Hard iron offset
      float raw_mx = mpu.getMagX() - mag_offset_x;
      float raw_my = mpu.getMagY() - mag_offset_y;
      float raw_mz = mpu.getMagZ() - mag_offset_z;

      // Step 2: Soft iron correction (apply 3x3 matrix)
      float mx = softIronMatrix[0][0] * raw_mx + softIronMatrix[0][1] * raw_my + softIronMatrix[0][2] * raw_mz;
      float my = softIronMatrix[1][0] * raw_mx + softIronMatrix[1][1] * raw_my + softIronMatrix[1][2] * raw_mz;
      float mz = softIronMatrix[2][0] * raw_mx + softIronMatrix[2][1] * raw_my + softIronMatrix[2][2] * raw_mz;

      // === Output ===
      // Serial.print("Gyro [°/s]: X="); Serial.print(gx, 2);
      // Serial.print("  Y="); Serial.print(gy, 2);
      // Serial.print("  Z="); Serial.println(gz, 2);

      // Serial.print("Accel [g]:  X="); Serial.print(ax, 3);
      // Serial.print("  Y="); Serial.print(ay, 3);
      // Serial.print("  Z="); Serial.println(az, 3);

      // Serial.print("Mag [µT]:   X="); Serial.print(mx, 1);
      // Serial.print("  Y="); Serial.print(my, 1);
      // Serial.print("  Z="); Serial.println(mz, 1);

      // Serial.println();
      Serial.print("Gyro: ");
      Serial.print(gx, 2); Serial.print(", ");
      Serial.print(gy, 2); Serial.print(", ");
      Serial.print(gz, 2); Serial.print(" | Accel: ");
      Serial.print(ax, 3); Serial.print(", ");
      Serial.print(ay, 3); Serial.print(", ");
      Serial.println(az, 3);
    }
  }
}
