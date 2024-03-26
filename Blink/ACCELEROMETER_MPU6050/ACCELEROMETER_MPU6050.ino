#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize MPU6050
  mpu.initialize();
  // Optionally set gyroscope and accelerometer offsets here, if needed.
}

void loop() {
  // Read accelerometer and gyroscope data
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Print the sensor data to the Serial Monitor
  Serial.print("Accelerometer: ");
  Serial.print("X = "); Serial.print(ax);
  Serial.print(" | Y = "); Serial.print(ay);
  Serial.print(" | Z = "); Serial.println(az);

  Serial.print("Gyroscope: ");
  Serial.print("X = "); Serial.print(gx);
  Serial.print(" | Y = "); Serial.print(gy);
  Serial.print(" | Z = "); Serial.println(gz);

  Serial.println();

  delay(1000); // Adjust the delay as needed for your application
}