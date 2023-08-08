#include <Arduino.h>
#include <parameters.hpp>
#include <tasks.hpp>
#include <RobotWheel.hpp>
#include <DrivetrainKinematics.h>
#include <RobotPosition.h>
#include <Trajectory.h>
#include <StraightLine.h>

using namespace miam;
using namespace miam::trajectory;

// RobotWheels
RobotWheel* leftRobotWheel;
RobotWheel* rightRobotWheel;

// Kinematics
DrivetrainKinematics kinematics = DrivetrainKinematics(WHEEL_RADIUS_MM,
                                       WHEEL_SPACING_MM,
                                       WHEEL_RADIUS_MM,
                                       WHEEL_SPACING_MM);

RobotPosition currentPosition(0.0, 0.0, 0.0);
BaseSpeed targetSpeed(0.0, 0);
WheelSpeed wheelSpeed;

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
    Serial.print(">targetSpeed.linear:");
    Serial.println(targetSpeed.linear);
    Serial.print(">targetSpeed.angular:");
    Serial.println(targetSpeed.angular);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void performLowLevel(void* parameters)
{
  for(;;)
  {
    // update sensors
    leftRobotWheel->updateEncoderSpeed();
    rightRobotWheel->updateEncoderSpeed();
    // perform motion control
    wheelSpeed = kinematics.inverseKinematics(targetSpeed);
    leftRobotWheel->setWheelSpeed(wheelSpeed.left);
    rightRobotWheel->setWheelSpeed(wheelSpeed.right);
    // update motor control
    leftRobotWheel->updateMotorControl();
    rightRobotWheel->updateMotorControl();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

TrajectoryConfig tc;

void setup()
{
  // initially motors should be stopped
  digitalWrite(EN_A, LOW);
  digitalWrite(EN_B, LOW);

  Serial.begin(115200);

  leftRobotWheel = new RobotWheel(EN_A, IN1_A, IN2_A, ENCODER_A1, ENCODER_B1, "left_");
  rightRobotWheel = new RobotWheel(EN_B, IN1_B, IN2_B, ENCODER_B2, ENCODER_A2, "right_");
  
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

  xTaskCreatePinnedToCore(
      performLowLevel, 
      "performLowLevel",
      10000,
      NULL,
      1,
      NULL,
      0
  ); 

  xTaskCreatePinnedToCore(
      printToSerial, 
      "printToSerial",
      10000,
      NULL,
      1,
      NULL,
      1
  ); 

  tc.maxWheelVelocity = 200.0;
  tc.maxWheelAcceleration = 200.0;
  tc.robotWheelSpacing = WHEEL_SPACING_MM;

  vTaskDelay(1000 / portTICK_PERIOD_MS);
}


void loop()
{

  RobotPosition currentPosition(0.0, 0.0, 0.0);
  RobotPosition targetPosition(300.0, 0.0, 0.0);

  StraightLine sl(
    tc,
    currentPosition,
    targetPosition
  );

  double start_time = millis() / 1000.0;
  double current_time = start_time;
  TrajectoryPoint tp;
  while(current_time - start_time < sl.getDuration() + 2)
  {
    tp = sl.getCurrentPoint(current_time-start_time);
    Serial.print(">trajectory.getDuration:");
    Serial.println(sl.getDuration());
    Serial.print(">trajectory.linear:");
    Serial.println(tp.linearVelocity);
    Serial.print(">trajectory.angular:");
    Serial.println(tp.angularVelocity);
    targetSpeed.linear = tp.linearVelocity;
    targetSpeed.angular = tp.angularVelocity;
    current_time = millis() / 1000.0;
    Serial.print("Tick at time ");
    Serial.println(current_time);
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }

  vTaskDelay(1000 / portTICK_PERIOD_MS);

  sl = StraightLine(
    tc,
    targetPosition,
    currentPosition,
    0.0,
    0.0,
    true
  );
  start_time = millis() / 1000.0;
  current_time = start_time;
  while(current_time - start_time < sl.getDuration() + 2)
  {
    tp = sl.getCurrentPoint(current_time-start_time);
    targetSpeed.linear = tp.linearVelocity;
    targetSpeed.angular = tp.angularVelocity;
    current_time = millis() / 1000.0;
    Serial.print("Tick at time ");
    Serial.println(current_time);
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }

  vTaskDelay(1000 / portTICK_PERIOD_MS);

  // // Forward
  // targetSpeed.linear = 100.0;
  // targetSpeed.angular = 0.0;
  // vTaskDelay(10000 / portTICK_PERIOD_MS);
  // // Backwards
  // targetSpeed.linear = -100.0;
  // targetSpeed.angular = 0.0;
  // vTaskDelay(10000 / portTICK_PERIOD_MS);
  // // Rotation to left
  // targetSpeed.linear = 0.0;
  // targetSpeed.angular = M_PI_2;
  // vTaskDelay(10000 / portTICK_PERIOD_MS);
  // // Rotation to right
  // targetSpeed.linear = 0.0;
  // targetSpeed.angular = -M_PI_2;
  // vTaskDelay(10000 / portTICK_PERIOD_MS);

  

}
