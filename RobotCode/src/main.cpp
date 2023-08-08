#include <Arduino.h>
#include <parameters.hpp>
#include <tasks.hpp>
#include <RobotWheel.hpp>
#include <DrivetrainKinematics.h>
#include <RobotPosition.h>
#include <Trajectory.h>
#include <StraightLine.h>
#include <PointTurn.h>
#include <Utilities.h>
#include <MotionController.hpp>

using namespace miam;
using namespace miam::trajectory;

// RobotWheels
RobotWheel* leftRobotWheel;
RobotWheel* rightRobotWheel;

// Motion controller
MotionController* motionController;

// Kinematics
DrivetrainKinematics kinematics = DrivetrainKinematics(WHEEL_RADIUS_MM,
                                       WHEEL_SPACING_MM,
                                       WHEEL_RADIUS_MM,
                                       WHEEL_SPACING_MM);

bool hasMatchStarted_ = true;

DrivetrainMeasurements measurements;
DrivetrainTarget target;

// Serial semaphore
SemaphoreHandle_t xMutex_Serial = NULL;

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
    if (xSemaphoreTake(xMutex_Serial, portMAX_DELAY))
    {
      print_battery();
      leftRobotWheel->printToSerial();
      rightRobotWheel->printToSerial();
      Serial.print(">targetSpeed.linear:");
      Serial.println(motionController->targetSpeed_.linear);
      Serial.print(">targetSpeed.angular:");
      Serial.println(motionController->targetSpeed_.angular);
      Serial.print(">currentPosition_:");
      Serial.print(motionController->currentPosition_.x);
      Serial.print(":");
      Serial.print(motionController->currentPosition_.y);
      Serial.println("|xy");
      Serial.print(">currentPosition.theta:");
      Serial.println(motionController->currentPosition_.theta);
      Serial.print(">targetPoint.position:");
      Serial.print(motionController->targetPoint.position.x);
      Serial.print(":");
      Serial.print(motionController->targetPoint.position.y);
      Serial.println("|xy");
      Serial.print(">targetPoint.position.theta:");
      Serial.println(motionController->targetPoint.position.theta);
      Serial.print(">targetPoint.linearVelocity:");
      Serial.println(motionController->targetPoint.linearVelocity);
      Serial.print(">targetPoint.angularVelocity:");
      Serial.println(motionController->targetPoint.angularVelocity);
      xSemaphoreGive(xMutex_Serial);  // release the mutex
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  }
}

void performLowLevel(void* parameters)
{
  for(;;)
  {
    // update sensors
    leftRobotWheel->updateEncoderSpeed();
    rightRobotWheel->updateEncoderSpeed();
    measurements.motorSpeed[side::LEFT] = leftRobotWheel->getWheelSpeed();
    measurements.motorSpeed[side::RIGHT] = rightRobotWheel->getWheelSpeed();

    // If playing side::RIGHT side: invert side::RIGHT/side::LEFT encoders.
    if (motionController->isPlayingRightSide_)
    {
        double temp = measurements.motorSpeed[side::RIGHT];
        measurements.motorSpeed[side::RIGHT] = measurements.motorSpeed[side::LEFT];
        measurements.motorSpeed[side::LEFT] = temp;
    }

    target = motionController->computeDrivetrainMotion(
      measurements, 
      LOW_LEVEL_LOOP_TIME_MS / 1000.0, 
      hasMatchStarted_
    );

    // invert kinematics
    if (motionController->isPlayingRightSide_)
    {
      leftRobotWheel->setWheelSpeed(target.motorSpeed[side::RIGHT]);
      rightRobotWheel->setWheelSpeed(target.motorSpeed[side::LEFT]);
    }
    else
    {
      leftRobotWheel->setWheelSpeed(target.motorSpeed[side::LEFT]);
      rightRobotWheel->setWheelSpeed(target.motorSpeed[side::RIGHT]);
    }

    // update motor control
    leftRobotWheel->updateMotorControl();
    rightRobotWheel->updateMotorControl();
    vTaskDelay(LOW_LEVEL_LOOP_TIME_MS / portTICK_PERIOD_MS);
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
  
  xMutex_Serial = xSemaphoreCreateMutex();  // crete a mutex object

  motionController = new MotionController(&xMutex_Serial);
  motionController->init(RobotPosition(0.0, 0.0, 0.0));

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

TrajectoryVector traj;

void loop()
{

  RobotPosition currentPosition(0.0, 0.0, 0.0);
  RobotPosition targetPosition(300.0, 0.0, 0.0);

  std::shared_ptr<Trajectory> sl(new StraightLine(
    tc,
    currentPosition,
    targetPosition
  ));

  traj.clear();
  traj.push_back(sl);

  motionController->setTrajectoryToFollow(traj);
  motionController->waitForTrajectoryFinished();

  vTaskDelay(1000 / portTICK_PERIOD_MS);

  std::shared_ptr<Trajectory> sl2(new StraightLine(
    tc,
    targetPosition,
    currentPosition,
    0.0,
    0.0,
    true
  ));

  traj.clear();
  traj.push_back(sl2);

  motionController->setTrajectoryToFollow(traj);
  motionController->waitForTrajectoryFinished();
  
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
