#include <Arduino.h>
#include "BluetoothSerial.h"
#include <vector>

BluetoothSerial SerialBT;

#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define FORWARD_LEFT 5
#define FORWARD_RIGHT 6
#define BACKWARD_LEFT 7
#define BACKWARD_RIGHT 8
#define TURN_LEFT 9
#define TURN_RIGHT 10
#define STOP 0

#define BACK_RIGHT_MOTOR 0
#define BACK_LEFT_MOTOR 1
#define FRONT_RIGHT_MOTOR 2
#define FRONT_LEFT_MOTOR 3

#define FORWARD 1
#define BACKWARD -1

struct MOTOR_PINS
{
  int pinIN1;
  int pinIN2;
};

std::vector<MOTOR_PINS> motorPins = 
{
  {4, 5},  //FRONT_RIGHT_MOTOR
  {18, 19},  //BACK_RIGHT_MOTOR
  {14, 27},  //FRONT_LEFT_MOTOR
  {26, 25},  //BACK_LEFT_MOTOR  
};

int xAxisValue;
int yAxisValue;
int zAxisValue;

// callback function that will be executed when data is received
void logic() 
{
  if (  xAxisValue < 75 &&  yAxisValue < 75)
  {
    processCarMovement(FORWARD_LEFT);    
  }
  else if (  xAxisValue > 175 &&  yAxisValue < 75)
  {
    processCarMovement(FORWARD_RIGHT);    
  } 
  else if (  xAxisValue < 75 &&  yAxisValue > 175)
  {
    processCarMovement(BACKWARD_LEFT);    
  }
  else if (  xAxisValue > 175 &&  yAxisValue > 175)
  {
    processCarMovement(BACKWARD_RIGHT);    
  }  
  else if ( zAxisValue > 175)
  {
    processCarMovement(TURN_RIGHT);
  }
  else if ( zAxisValue < 75)
  {
    processCarMovement(TURN_LEFT);
  }
  else if ( yAxisValue < 75)
  {
    processCarMovement(FORWARD);  
  }
  else if ( yAxisValue > 175)
  {
    processCarMovement(BACKWARD);     
  }
  else if ( xAxisValue > 175)
  {
    processCarMovement(RIGHT);   
  }
  else if ( xAxisValue < 75)
  {
    processCarMovement(LEFT);    
  } 
  else
  {
    processCarMovement(STOP);     
  }
   
}

void rotateMotor(int motorNumber, int motorDirection)
{
  if (motorDirection == FORWARD)
  {
    digitalWrite(motorPins[motorNumber].pinIN1, HIGH);
    digitalWrite(motorPins[motorNumber].pinIN2, LOW);    
  }
  else if (motorDirection == BACKWARD)
  {
    digitalWrite(motorPins[motorNumber].pinIN1, LOW);
    digitalWrite(motorPins[motorNumber].pinIN2, HIGH);     
  }
  else
  {
    digitalWrite(motorPins[motorNumber].pinIN1, LOW);
    digitalWrite(motorPins[motorNumber].pinIN2, LOW);       
  }
}

void processCarMovement(int inputValue)
{
  switch(inputValue)
  {
    case FORWARD:
      rotateMotor(FRONT_RIGHT_MOTOR, FORWARD);
      rotateMotor(BACK_RIGHT_MOTOR, FORWARD);
      rotateMotor(FRONT_LEFT_MOTOR, FORWARD);
      rotateMotor(BACK_LEFT_MOTOR, FORWARD);                  
      break;
  
    case BACKWARD:
      rotateMotor(FRONT_RIGHT_MOTOR, BACKWARD);
      rotateMotor(BACK_RIGHT_MOTOR, BACKWARD);
      rotateMotor(FRONT_LEFT_MOTOR, BACKWARD);
      rotateMotor(BACK_LEFT_MOTOR, BACKWARD);   
      break;
  
    case LEFT:
      rotateMotor(FRONT_RIGHT_MOTOR, FORWARD);
      rotateMotor(BACK_RIGHT_MOTOR, BACKWARD);
      rotateMotor(FRONT_LEFT_MOTOR, BACKWARD);
      rotateMotor(BACK_LEFT_MOTOR, FORWARD);   
      break;
  
    case RIGHT:
      rotateMotor(FRONT_RIGHT_MOTOR, BACKWARD);
      rotateMotor(BACK_RIGHT_MOTOR, FORWARD);
      rotateMotor(FRONT_LEFT_MOTOR, FORWARD);
      rotateMotor(BACK_LEFT_MOTOR, BACKWARD);  
      break;
  
    case FORWARD_LEFT:
      rotateMotor(FRONT_RIGHT_MOTOR, FORWARD);
      rotateMotor(BACK_RIGHT_MOTOR, STOP);
      rotateMotor(FRONT_LEFT_MOTOR, STOP);
      rotateMotor(BACK_LEFT_MOTOR, FORWARD);  
      break;
  
    case FORWARD_RIGHT:
      rotateMotor(FRONT_RIGHT_MOTOR, STOP);
      rotateMotor(BACK_RIGHT_MOTOR, FORWARD);
      rotateMotor(FRONT_LEFT_MOTOR, FORWARD);
      rotateMotor(BACK_LEFT_MOTOR, STOP);  
      break;
  
    case BACKWARD_LEFT:
      rotateMotor(FRONT_RIGHT_MOTOR, STOP);
      rotateMotor(BACK_RIGHT_MOTOR, BACKWARD);
      rotateMotor(FRONT_LEFT_MOTOR, BACKWARD);
      rotateMotor(BACK_LEFT_MOTOR, STOP);   
      break;
  
    case BACKWARD_RIGHT:
      rotateMotor(FRONT_RIGHT_MOTOR, BACKWARD);
      rotateMotor(BACK_RIGHT_MOTOR, STOP);
      rotateMotor(FRONT_LEFT_MOTOR, STOP);
      rotateMotor(BACK_LEFT_MOTOR, BACKWARD);   
      break;
  
    case TURN_LEFT:
      rotateMotor(FRONT_RIGHT_MOTOR, FORWARD);
      rotateMotor(BACK_RIGHT_MOTOR, FORWARD);
      rotateMotor(FRONT_LEFT_MOTOR, BACKWARD);
      rotateMotor(BACK_LEFT_MOTOR, BACKWARD);  
      break;
  
    case TURN_RIGHT:
      rotateMotor(FRONT_RIGHT_MOTOR, BACKWARD);
      rotateMotor(BACK_RIGHT_MOTOR, BACKWARD);
      rotateMotor(FRONT_LEFT_MOTOR, FORWARD);
      rotateMotor(BACK_LEFT_MOTOR, FORWARD);   
      break;
  
    case STOP:
      rotateMotor(FRONT_RIGHT_MOTOR, STOP);
      rotateMotor(BACK_RIGHT_MOTOR, STOP);
      rotateMotor(FRONT_LEFT_MOTOR, STOP);
      rotateMotor(BACK_LEFT_MOTOR, STOP);    
      break;
  
    default:
      rotateMotor(FRONT_RIGHT_MOTOR, STOP);
      rotateMotor(BACK_RIGHT_MOTOR, STOP);
      rotateMotor(FRONT_LEFT_MOTOR, STOP);
      rotateMotor(BACK_LEFT_MOTOR, STOP);    
      break;
  }
}

void setUpPinModes()
{
  for (int i = 0; i < motorPins.size(); i++)
  {
    pinMode(motorPins[i].pinIN1, OUTPUT);
    pinMode(motorPins[i].pinIN2, OUTPUT);  
    rotateMotor(i, STOP);  
  }
}

void setup(void) 
{
  setUpPinModes();
  
  Serial.begin(115200);
  SerialBT.begin("ESP32test"); // Bluetooth device name
  Serial.println("Bluetooth initialized.");

}
 
void loop() 
{
  uint8_t recv_data[3];
  if (SerialBT.available()) {
    SerialBT.readBytes(recv_data, 3);

    xAxisValue = recv_data[0];
    yAxisValue = recv_data[1]; // Joystick X-axis value
    zAxisValue = recv_data[2]; // Joystick Y-axis value
    Serial.printf("xAxisValue: %d, yAxisValue: %d, zAxisValue: %d\n", recv_data[0], recv_data[1], recv_data[2]);
    void logic();
  }
  delay(20);
}
