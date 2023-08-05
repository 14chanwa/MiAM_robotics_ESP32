#include <Arduino.h>
#include <parameters.hpp>
#include <tasks.hpp>
#include <RobotWheel.hpp>


// RobotWheels
RobotWheel* leftRobotWheel;
RobotWheel* rightRobotWheel;

void IRAM_ATTR encoderInterruptLeft()
{
  leftRobotWheel->handleEncoderInterrupt();
}

void IRAM_ATTR encoderInterruptRight()
{
  rightRobotWheel->handleEncoderInterrupt();
}

void printToSerial(void* parameters)
{
  for(;;)
  {
    print_battery();
    leftRobotWheel->printToSerial();
    rightRobotWheel->printToSerial();
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

double speed = 0;

void setup()
{
  // initially motors should be stopped
  digitalWrite(EN_A, LOW);
  digitalWrite(EN_B, LOW);

  Serial.begin(115200);

  leftRobotWheel = new RobotWheel(EN_A, IN1_A, IN2_A, ENCODER_A1, ENCODER_B1, "left_");
  rightRobotWheel = new RobotWheel(EN_B, IN2_B, IN1_B, ENCODER_A2, ENCODER_B2, "right_");
  
  run_monitor_battery();

  // interupts have to be handled outside low level loop
  pinMode(leftRobotWheel->pinEncoderA_, INPUT_PULLUP);
  pinMode(leftRobotWheel->pinEncoderB_, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(leftRobotWheel->pinEncoderA_), encoderInterruptLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftRobotWheel->pinEncoderB_), encoderInterruptLeft, CHANGE);

  pinMode(rightRobotWheel->pinEncoderA_, INPUT_PULLUP);
  pinMode(rightRobotWheel->pinEncoderB_, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rightRobotWheel->pinEncoderA_), encoderInterruptRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightRobotWheel->pinEncoderB_), encoderInterruptRight, CHANGE);

  leftRobotWheel->startLowLevelLoop();
  vTaskDelay(2 / portTICK_PERIOD_MS);
  rightRobotWheel->startLowLevelLoop();

  xTaskCreate(
      printToSerial, 
      "printToSerial",
      10000,
      NULL,
      1,
      NULL
  ); 
}

int increment = 10;

void loop()
{
  leftRobotWheel->setWheelSpeed(RPM_TO_RAD_S(speed));
  rightRobotWheel->setWheelSpeed(RPM_TO_RAD_S(speed));
  speed += increment;

  if (speed > 150)
  {
    speed = 150;
    increment = -10;
  }
  if (speed < -150)
  {
    speed = -150;
    increment = 10;
  }

  vTaskDelay(250 / portTICK_PERIOD_MS);
}
