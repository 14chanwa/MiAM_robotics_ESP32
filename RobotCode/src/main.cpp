#include <Arduino.h>
#include <parameters.hpp>
#include <DrivetrainKinematics.h>
#include <RobotPosition.h>
#include <Trajectory.h>
#include <StraightLine.h>
#include <PointTurn.h>
#include <Utilities.h>
#include <SampledTrajectory.h>

#include <Robot.hpp>

#include <Strategy.hpp>
#include <AnalogReadings.hpp>
#include <ServoHandler.hpp>
#include <I2CHandler.hpp>
#include <WiFiHandler.hpp>
#include <HeartbeatHandler.hpp>
#include <MessageHandler.hpp>

#include <RobotBaseDC.hpp>
#include <RobotBaseStepper.hpp>


#ifdef SEND_TELEPLOT_UDP

  #include <TeleplotArduino.hpp>
  // Teleplot teleplot("192.168.0.255", 47269);
  Teleplot teleplot(MIAM_BROADCAST_ADDRESS, 47269);

#endif



/////////////////////////////////////////////////////////////////////
// Tasks
/////////////////////////////////////////////////////////////////////

using namespace miam;
using namespace miam::trajectory;


void logTelemetry(void* parameters)
{
  Robot* robot = Robot::getInstance();
  for(;;)
  {

    #ifdef SEND_TELEPLOT_UDP

      // teleplot.update("currentPosition.x", motionController->currentPosition_.x);
      // teleplot.update("currentPosition.y", motionController->currentPosition_.y);
      RobotPosition curPos = robot->motionController->getCurrentPosition();
      teleplot.update2D("currentPosition", curPos.x, curPos.y);
      teleplot.update("currentPosition.theta", curPos.theta);
      // teleplot.update("targetPosition.x", motionController->targetPoint.position.x);
      // teleplot.update("targetPosition.y", motionController->targetPoint.position.y);
      teleplot.update2D("targetPosition", robot->motionController->targetPoint.position.x, robot->motionController->targetPoint.position.y);
      teleplot.update("targetPosition.theta", robot->motionController->targetPoint.position.theta);
      teleplot.update("targetPoint.linear", robot->motionController->targetPoint.linearVelocity);
      teleplot.update("targetPoint.angular", robot->motionController->targetPoint.angularVelocity);
      teleplot.update("clampedSlowDownCoeff_", robot->motionController->clampedSlowDownCoeff_);
      // teleplot.update("dt_period_ms", dt_period_ms, "", 0, TELEPLOT_FLAG_NOPLOT);
      // teleplot.update("dt_lowLevel_ms", dt_lowLevel_ms, "", 0, TELEPLOT_FLAG_NOPLOT);
      teleplot.update("battery_reading", AnalogReadings::get_current_battery_reading(), "", 0, TELEPLOT_FLAG_NOPLOT);
      // teleplot.update("tcrt0", AnalogReadings::get_current_tcrt0_reading(), "", 0);
      // teleplot.update("tcrt1", AnalogReadings::get_current_tcrt1_reading(), "", 0);
      // teleplot.update("tcrt2", AnalogReadings::get_current_tcrt2_reading(), "", 0);
      // teleplot.update("touchSensor", get_current_touch_sensor_reading(), "", 0);
      teleplot.update("leftSwitchPin", AnalogReadings::get_left_switch_value(), "", 0);
      teleplot.update("rightSwitchPin", AnalogReadings::get_right_switch_value(), "", 0);


      #ifdef USE_DC_MOTORS
      teleplot.update("rightWheelCurrentSpeed", robotBase->getRightWheel()->currentSpeed_);
      teleplot.update("rightWheelTargetSpeed", robotBase->getRightWheel()->targetSpeed_);
      teleplot.update("leftWheelCurrentSpeed", robotBase->getLeftWheel()->currentSpeed_);
      teleplot.update("leftWheelTargetSpeed", robotBase->getLeftWheel()->targetSpeed_);
      teleplot.update("rightBasePWM", static_cast<RobotWheelDC*>(robotBase->getRightWheel())->basePWMTarget_);
      teleplot.update("leftBasePWM", static_cast<RobotWheelDC*>(robotBase->getLeftWheel())->basePWMTarget_);
      teleplot.update("rightNewPWM", static_cast<RobotWheelDC*>(robotBase->getRightWheel())->newPWMTarget_);
      teleplot.update("leftNewPWM", static_cast<RobotWheelDC*>(robotBase->getLeftWheel())->newPWMTarget_);
      #endif


      // sendTelemetry("leftBasePWMTarget", leftRobotWheel->basePWMTarget_);
      // sendTelemetry("leftNewPWMTarget", leftRobotWheel->newPWMTarget_);
      // sendTelemetry("rightBasePWMTarget", rightRobotWheel->basePWMTarget_);
      // sendTelemetry("rightNewPWMTarget", rightRobotWheel->newPWMTarget_);

      #ifdef USE_STEPPER_MOTORS
      // teleplot.update("baseTarget_left_", (static_cast<RobotBaseStepper* >(robotBase))->baseTarget_left_, "", 0, TELEPLOT_FLAG_NOPLOT);
      teleplot.update("targetSpeed_left_", (static_cast<RobotBaseStepper* >(robot->robotBase))->targetSpeed_left_);
      // teleplot.update("targetSpeedDriver_left_", (static_cast<RobotBaseStepper* >(robotBase))->targetSpeedDriver_left_, "", 0, TELEPLOT_FLAG_NOPLOT);
      teleplot.update("currentSpeed_left_", (static_cast<RobotBaseStepper* >(robot->robotBase))->currentSpeed_left_);
      // teleplot.update("baseTarget_right_", (static_cast<RobotBaseStepper* >(robotBase))->baseTarget_right_, "", 0, TELEPLOT_FLAG_NOPLOT);
      teleplot.update("targetSpeed_right_", (static_cast<RobotBaseStepper* >(robot->robotBase))->targetSpeed_right_);
      // teleplot.update("targetSpeedDriver_right_", (static_cast<RobotBaseStepper* >(robotBase))->targetSpeedDriver_right_, "", 0, TELEPLOT_FLAG_NOPLOT);
      teleplot.update("currentSpeed_right_", (static_cast<RobotBaseStepper* >(robot->robotBase))->currentSpeed_right_);
      // teleplot.update("isQueueRunningLeft_", (static_cast<RobotBaseStepper* >(robotBase))->isRunningLeft, "", 0, TELEPLOT_FLAG_NOPLOT);
      // teleplot.update("isQueueRunningRight_", (static_cast<RobotBaseStepper* >(robotBase))->isRunningRight, "", 0, TELEPLOT_FLAG_NOPLOT);
      // teleplot.update("isQueueEmptyLeft_", (static_cast<RobotBaseStepper* >(robotBase))->isEmptyLeft, "", 0, TELEPLOT_FLAG_NOPLOT);
      // teleplot.update("isQueueEmptyRight_", (static_cast<RobotBaseStepper* >(robotBase))->isEmptyRight, "", 0, TELEPLOT_FLAG_NOPLOT);
      // teleplot.update("isQueueFullLeft_", (static_cast<RobotBaseStepper* >(robotBase))->isEmptyLeft, "", 0, TELEPLOT_FLAG_NOPLOT);
      // teleplot.update("isQueueFullRight_", (static_cast<RobotBaseStepper* >(robotBase))->isEmptyRight, "", 0, TELEPLOT_FLAG_NOPLOT);
      // teleplot.update("isGenActiveLeft_", (static_cast<RobotBaseStepper* >(robotBase))->isGenActiveLeft, "", 0, TELEPLOT_FLAG_NOPLOT);
      // teleplot.update("isGenActiveRight_", (static_cast<RobotBaseStepper* >(robotBase))->isGenActiveRight, "", 0, TELEPLOT_FLAG_NOPLOT);
      // teleplot.update("desyncDetectedLeft_", (static_cast<RobotBaseStepper* >(robotBase))->desyncDetectedLeft, "", 0, TELEPLOT_FLAG_NOPLOT);
      // teleplot.update("desyncDetectedRight_", (static_cast<RobotBaseStepper* >(robotBase))->desyncDetectedRight, "", 0, TELEPLOT_FLAG_NOPLOT);
      #endif

      teleplot.update("vlx_ranging_data_mm", I2CHandler::get_current_vl53l0x());

    #else
    #ifdef SEND_SERIAL

    if (xSemaphoreTake(xMutex_Serial, portMAX_DELAY))
    {
      // print_battery();
      leftRobotWheel->logTelemetry();
      rightRobotWheel->logTelemetry();
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
    #endif
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

/////////////////////////////////////////////////////////////////////
// Setup
/////////////////////////////////////////////////////////////////////


void setup()
{
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  Robot::init();

  // start heartbeat
  HeartbeatHandler::start_heartbeat();


  Serial.begin(115200);
  Serial.println("Attempt connect WiFi");
  
  // connect wifi
  WiFiHandler::initWiFi();

  // Init i2c peripherals
  I2CHandler::init();

  // analog readings: monitor battery and infrared captors
  Serial.println("Init analog readings");
  AnalogReadings::init();

  // init the ServoHandler
  ServoHandler::init();
  ServoHandler::servoUp();

  Robot::startLowLevelLoop();

#if defined(SEND_SERIAL) || defined(SEND_TELEPLOT_UDP)
  Serial.println("Launch print to serial");
  xTaskCreatePinnedToCore(
      logTelemetry, 
      "logTelemetry",
      10000,
      NULL,
      1,
      NULL,
      0 // pin to core 0
  ); 
#endif

// #ifdef ENABLE_OTA_UPDATE
//   Serial.println("Launch OTA");
//   xTaskCreatePinnedToCore(
//       handleOTATask, 
//       "handleOTATask",
//       10000,
//       NULL,
//       1,
//       NULL,
//       1 // pin to core 0
//   ); 
// #endif

  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

/////////////////////////////////////////////////////////////////////
// Receiver loop
/////////////////////////////////////////////////////////////////////




void loop()
{
#ifdef DEBUG_MODE_MATCH
  match_started = true;
  match_current_time_s = 85.0f;  
#endif
#ifdef DEBUG_MODE_SERVO
  taskYIELD();
  for(;;)
  {
    // for(int posDegrees = 0; posDegrees <= 180; posDegrees++) {
    //   ServoHandler::servoWrite(posDegrees);
    //   // Serial.println(posDegrees);
    //   delay(20);
    // }
    Serial.println("Up");
    ServoHandler::servoUp();
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    Serial.println("Down");
    ServoHandler::servoDown();
    vTaskDelay(3000 / portTICK_PERIOD_MS);
  }
#endif 
#ifdef DEBUG_MODE_SIMPLE_TRAJECTORY
  taskYIELD();
  for (;;)
  {
    Serial.println("Moving...");
    movement_override = true;
    strategy::make_a_square(motionController);
    // strategy::go_to_zone_3(motionController);

    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
#endif

  MessageHandler::start();

  for(;;)
  {
    taskYIELD();
    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
  
}
