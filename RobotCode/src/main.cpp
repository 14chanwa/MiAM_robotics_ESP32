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
#include <WiFi.h>

#define SEND_TELEPLOT_UDP

#ifdef SEND_TELEPLOT_UDP
  #include <WiFiUdp.h>
  WiFiUDP udp;
  char buffer[30];

  void sendTelemetry(const char* variableName, const float value)
  {
    udp.beginPacket("192.168.0.255",47269);
    size_t size = snprintf(buffer, 30, "%s:%f", variableName, value);
    udp.write((uint8_t*)&buffer, size);
    udp.endPacket();
  }
#endif

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin("ssid", "password");
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}

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

// low level loop timing
double dt_lowLevel_ms = 0.0;
double dt_period_ms = 0.0;
long timeStartLoop = 0;
long timeEndLoop = 0;

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

    #ifdef SEND_TELEPLOT_UDP

      sendTelemetry("targetSpeed.linear", motionController->targetSpeed_.linear);
      sendTelemetry("targetSpeed.angular", motionController->targetSpeed_.angular);
      sendTelemetry("dt_period_ms", dt_period_ms);
      sendTelemetry("dt_lowLevel_ms", dt_lowLevel_ms);
      sendTelemetry("battery_reading", get_current_battery_reading());
    #else

    if (xSemaphoreTake(xMutex_Serial, portMAX_DELAY))
    {
      print_battery();
      leftRobotWheel->printToSerial();
      rightRobotWheel->printToSerial();
      Serial.print(">targetSpeed.linear:");
      Serial.println(motionController->targetSpeed_.linear);
      Serial.print(">targetSpeed.angular:");
      Serial.println(motionController->targetSpeed_.angular);
      // Serial.print(">currentPosition_:");
      // Serial.print(motionController->currentPosition_.x);
      // Serial.print(":");
      // Serial.print(motionController->currentPosition_.y);
      // Serial.println("|xy");
      Serial.print(">currentPosition.x:");
      Serial.println(motionController->currentPosition_.x);
      Serial.print(">currentPosition.y:");
      Serial.println(motionController->currentPosition_.y);
      Serial.print(">currentPosition.theta:");
      Serial.println(motionController->currentPosition_.theta);
      // Serial.print(">targetPoint.position:");
      // Serial.print(motionController->targetPoint.position.x);
      // Serial.print(":");
      // Serial.print(motionController->targetPoint.position.y);
      // Serial.println("|xy");
      Serial.print(">targetPoint.position.x:");
      Serial.println(motionController->targetPoint.position.x);
      Serial.print(">targetPoint.position.y:");
      Serial.println(motionController->targetPoint.position.y);
      Serial.print(">targetPoint.position.theta:");
      Serial.println(motionController->targetPoint.position.theta);
      Serial.print(">targetPoint.linearVelocity:");
      Serial.println(motionController->targetPoint.linearVelocity);
      Serial.print(">targetPoint.angularVelocity:");
      Serial.println(motionController->targetPoint.angularVelocity);
      xSemaphoreGive(xMutex_Serial);  // release the mutex
    }
    #endif
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void performLowLevel(void* parameters)
{
  for(;;)
  {
    timeEndLoop = micros();
    dt_period_ms = (timeEndLoop - timeStartLoop) / 1000.0;
    timeStartLoop = timeEndLoop;
    
    // update sensors
    monitor_battery();
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
      dt_period_ms / 1000.0, 
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

    timeEndLoop = micros();
    dt_lowLevel_ms = (timeEndLoop - timeStartLoop) / 1000.0;

    // wait till next tick (integer division rounds toward 0)
    vTaskDelay( std::max(1L, (timeStartLoop + LOW_LEVEL_LOOP_TIME_MS * 1000 - timeEndLoop) / 1000) / portTICK_PERIOD_MS);
  }
}

TrajectoryConfig tc;

void setup()
{
  // initially motors should be stopped
  digitalWrite(EN_A, LOW);
  digitalWrite(EN_B, LOW);

  vTaskDelay(3000 / portTICK_PERIOD_MS);

  Serial.begin(115200);
  Serial.println("Attempt connect WiFi");
  
  // connect wifi
  initWiFi();

  leftRobotWheel = new RobotWheel(EN_A, IN1_A, IN2_A, ENCODER_A1, ENCODER_B1, "left_");
  rightRobotWheel = new RobotWheel(EN_B, IN1_B, IN2_B, ENCODER_B2, ENCODER_A2, "right_");
  
  xMutex_Serial = xSemaphoreCreateMutex();  // crete a mutex object

  motionController = new MotionController(&xMutex_Serial);
  motionController->init(RobotPosition(0.0, 0.0, 0.0));

  // monitor battery
  pinMode(BAT_READING, INPUT_PULLDOWN);
  analogReadResolution(12);
  analogSetAttenuation(ADC_6db);

  // interupts have to be handled outside low level loop
  // since they are declared in setup, they are attached to core1
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
      2,  // high priority
      NULL, 
      1 // pin to core 1
  ); 

  xTaskCreatePinnedToCore(
      printToSerial, 
      "printToSerial",
      10000,
      NULL,
      1,
      NULL,
      0 // pin to core 0
  ); 

  tc.maxWheelVelocity = MAX_WHEEL_SPEED_MM_S * 0.7;
  tc.maxWheelAcceleration = MAX_WHEEL_ACCELERATION_MM_S * 0.7;
  tc.robotWheelSpacing = WHEEL_SPACING_MM;

  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

TrajectoryVector traj;

void loop()
{

  RobotPosition currentPosition(0.0, 0.0, 0.0);
  RobotPosition targetPosition(currentPosition);
  targetPosition.x += 500;

  traj.clear();
  std::shared_ptr<Trajectory> sl(new StraightLine(tc, currentPosition, targetPosition));
  traj.push_back(sl);
  std::shared_ptr<Trajectory> pt(new PointTurn(tc, sl->getEndPoint().position, -M_PI));
  traj.push_back(pt);
    std::shared_ptr<Trajectory> sl2(new StraightLine(tc, pt->getEndPoint().position, currentPosition));
  traj.push_back(sl2);
  std::shared_ptr<Trajectory> pt2(new PointTurn(tc, sl2->getEndPoint().position, M_PI_2));
  traj.push_back(pt2);
  std::shared_ptr<Trajectory> pt3(new PointTurn(tc, pt2->getEndPoint().position, 0));
  traj.push_back(pt3);

  motionController->setTrajectoryToFollow(traj);
  motionController->waitForTrajectoryFinished();

  vTaskDelay(1000 / portTICK_PERIOD_MS);
}