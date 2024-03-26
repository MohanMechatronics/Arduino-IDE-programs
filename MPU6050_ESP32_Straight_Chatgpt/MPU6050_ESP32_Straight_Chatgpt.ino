#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);

  pinMode(14,OUTPUT);   //left motors  forward
  pinMode(27,OUTPUT);   //left motors reverse
  pinMode(26,OUTPUT);   //right  motors forward
  pinMode(25,OUTPUT);

  while (!Serial) {
    delay(10);
  }

  Serial.println("Adafruit MPU6050 Yaw Angle Test");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {O
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(1000);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate yaw angle using gyroscope data
  float yawAngle = atan2(g.gyro.x, g.gyro.z) * RAD_TO_DEG;

  // Print the yaw angle
  Serial.print("Yaw Angle: ");
  Serial.println(yawAngle);

  if ( yawAngle <= -25 && yawAngle >= -170 ) {   // you can increse the range to suit your sensor's accuracy
    digitalWrite(14,HIGH);
    digitalWrite(27,LOW);
    digitalWrite(26,HIGH);
    digitalWrite(25,LOW);
  }
  else{
    digitalWrite(14,LOW);
    digitalWrite(27,LOW);
    digitalWrite(26,LOW);
    digitalWrite(25,LOW);
  }

  delay(500);  // Adjust the delay based on your application requirements
}
