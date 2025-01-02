#include <TelemetryHandler.hpp>
#include <Robot.hpp>
#include <Arduino.h>
#include <parameters.hpp>

#include <AnalogReadings.hpp>
#include <I2CHandler.hpp>
#include <RobotBaseDC.hpp>
#include <RobotBaseStepper.hpp>

#ifdef SEND_TELEPLOT_UDP

#include <TeleplotArduino.hpp>
// Teleplot teleplot("192.168.0.255", 47269);
Teleplot teleplot(MIAM_BROADCAST_ADDRESS, 47269);

#endif

void logTelemetry(void* parameters)
{
  Robot* robot = Robot::getInstance();
  for(;;)
  {

    #ifdef SEND_TELEPLOT_UDP

      teleplot.update("currentPosition.x", robot->motionController->getCurrentPosition().x);
      teleplot.update("currentPosition.y", robot->motionController->getCurrentPosition().y);
      // RobotPosition curPos = robot->motionController->getCurrentPosition();
      // teleplot.update2D("currentPosition", curPos.x, curPos.y);
      // teleplot.update("currentPosition.theta", curPos.theta);
      teleplot.update("targetPosition.x", robot->motionController->targetPoint.position.x);
      teleplot.update("targetPosition.y", robot->motionController->targetPoint.position.y);
      // teleplot.update2D("targetPosition", robot->motionController->targetPoint.position.x, robot->motionController->targetPoint.position.y);
      // teleplot.update("targetPosition.theta", robot->motionController->targetPoint.position.theta);
      // teleplot.update("targetPoint.linear", robot->motionController->targetPoint.linearVelocity);
      // teleplot.update("targetPoint.angular", robot->motionController->targetPoint.angularVelocity);
      // teleplot.update("clampedSlowDownCoeff_", robot->motionController->clampedSlowDownCoeff_);
      // // teleplot.update("dt_period_ms", dt_period_ms, "", 0, TELEPLOT_FLAG_NOPLOT);
      // // teleplot.update("dt_lowLevel_ms", dt_lowLevel_ms, "", 0, TELEPLOT_FLAG_NOPLOT);
      // teleplot.update("battery_reading", AnalogReadings::get_current_battery_reading(), "", 0, TELEPLOT_FLAG_NOPLOT);
      // // teleplot.update("tcrt0", AnalogReadings::get_current_tcrt0_reading(), "", 0);
      // // teleplot.update("tcrt1", AnalogReadings::get_current_tcrt1_reading(), "", 0);
      // // teleplot.update("tcrt2", AnalogReadings::get_current_tcrt2_reading(), "", 0);
      // // teleplot.update("touchSensor", get_current_touch_sensor_reading(), "", 0);
      // teleplot.update("leftSwitchPin", AnalogReadings::get_left_switch_value(), "", 0);
      // teleplot.update("rightSwitchPin", AnalogReadings::get_right_switch_value(), "", 0);
      // teleplot.update("currentRobotState", (int)robot->get_current_robot_state(), "", 0);


      #ifdef USE_DC_MOTORS
      // teleplot.update("rightWheelCurrentSpeed", robot->robotBase->getRightWheel()->currentSpeed_);
      // teleplot.update("rightWheelTargetSpeed", robot->robotBase->getRightWheel()->targetSpeed_);
      // teleplot.update("leftWheelCurrentSpeed", robot->robotBase->getLeftWheel()->currentSpeed_);
      // teleplot.update("leftWheelTargetSpeed", robot->robotBase->getLeftWheel()->targetSpeed_);
      // teleplot.update("rightBasePWM", static_cast<RobotWheelDC*>(robot->robotBase->getRightWheel())->basePWMTarget_);
      // teleplot.update("leftBasePWM", static_cast<RobotWheelDC*>(robot->robotBase->getLeftWheel())->basePWMTarget_);
      // teleplot.update("rightNewPWM", static_cast<RobotWheelDC*>(robot->robotBase->getRightWheel())->newPWMTarget_);
      // teleplot.update("leftNewPWM", static_cast<RobotWheelDC*>(robot->robotBase->getLeftWheel())->newPWMTarget_);
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
      teleplot.update("vlx_ranging_data_smoothed_mm", I2CHandler::get_smoothed_vl53l0x());

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

namespace TelemetryHandler
{
    void begin()
    {
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
    }
}