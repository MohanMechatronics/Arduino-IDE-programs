
#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
unsigned long timer = 0;

const int motor1A = 33;
const int motor1B = 27;
const int motor2A = 26;
const int motor2B = 25;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");

  // configure LED PWM functionalitites
  ledcSetup(0,2000,8); //ledcSetup(ledChannel, freq, resolution);
  ledcSetup(1,2000,8);
  ledcSetup(2,2000,8);
  ledcSetup(3,2000,8);


  // attach the channel to the GPIO to be controlled
  ledcAttachPin(33, 0); //ledcAttachPin(ledPin, ledChannel);
  ledcAttachPin(27, 1);
  ledcAttachPin(26, 2);
  ledcAttachPin(25, 3);

  
}

void loop() {
  mpu.update();
  
  if((millis()-timer)>10){ // print data every 10ms

	
  int yawValue = map(mpu.getAngleZ(), -90, 90, 0, 180);
	Serial.print("Z : ");
	Serial.println(yawValue);
  if(yawValue>= 89 && yawValue<= 91){
    ledcWrite(0, 255);
    ledcWrite(1, 0);
    ledcWrite(2, 255);
    ledcWrite(3, 0);
  }
  else if(yawValue<= 89){
    ledcWrite(0, 0);
    ledcWrite(1, 150);
    ledcWrite(2, 255);
    ledcWrite(3, 0);
  }
  else if(yawValue>= 91){
    ledcWrite(0, 255);
    ledcWrite(1, 0);
    ledcWrite(2, 0);
    ledcWrite(3, 150);
  }
  else{

  }

  timer = millis();  
  }
}
