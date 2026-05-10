// #include <Arduino.h>

// // Basic example of making a STS3215 servo move.
// //
// // This code will simply make the servo move in circle from (0, 180, 360)deg.

// #include "STSServoDriver.h"

// STSServoDriver servos;

// // ID of the servo currently being tested.
// byte SERVO_ID = 1;

// #define RXD2 16
// #define TXD2 17

// HardwareSerial serial2(2);

// void setup() {

//   serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

//   pinMode(2, OUTPUT);
//   digitalWrite(2, LOW);
//   // Since the serial port is taken by the servo, we can't easily send debug messages, so
//   // we use the on-board led instead.
//   // Try to connect with the servos, using pin 2 as direction pin and the default (only) serial
//   // interface of an Arduino Uno.
//   if (!servos.init(5, &serial2))
//   {
//     // Failed to get a ping reply, turn on the led.
//     digitalWrite(2, HIGH);
//   }

//   // Reset all servos to position mode: servos have three modes (position, velocity, step position).
//   // Position is the default mode so this shouldn't be needed but it's here just to make sure
//   // (depending on what you've run before, the servos could be in a different mode)
//   servos.setMode(0xFE, STSMode::POSITION); // 0xFE is broadcast address and applies to all servos.
// }

// void loop()
// {
//   // Move servo to 0.
//   servos.setTargetPosition(SERVO_ID, 0);
//   // Wait for servo to start moving, then wait for end of motion
//   delay(100);
//   while (servos.isMoving(SERVO_ID))
//     delay(50);
//   // Wait a bit more to see it stop
//   delay(500);

//   // Move to 180deg.
//   servos.setTargetPosition(SERVO_ID, 2048);
//   delay(100);
//   while (servos.isMoving(SERVO_ID))
//     delay(50);
//   delay(500);

//   // Move to 360deg, at a slower speed (second argument is steps/s).
//   servos.setTargetPosition(SERVO_ID, 4095, 500);
//   delay(100);
//   while (servos.isMoving(SERVO_ID))
//     delay(50);
//   delay(500);
// }

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
#include <TelemetryHandler.hpp>

//#define DEBUG_MODE_SIMPLE_TRAJECTORY


/////////////////////////////////////////////////////////////////////
// Tasks
/////////////////////////////////////////////////////////////////////

using namespace miam;
using namespace miam::trajectory;

/////////////////////////////////////////////////////////////////////
// Setup
/////////////////////////////////////////////////////////////////////


#define DEBUG_MODE_SIMPLE_TRAJECTORY

void setup()
{
  // Switch pump off
  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);

  Serial.begin(115200);
  Serial.println("Setup begin");
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  Robot::init();
  Serial.println("Robot::init() OK");

  // start heartbeat
  HeartbeatHandler::start_heartbeat();
  Serial.println("HeartbeatHandler::start_heartbeat() OK");

  // Serial.println("Attempt connect WiFi");
  
  // // connect wifi
  // WiFiHandler::initWiFi();

  // Init i2c peripherals
  // I2CHandler::init();
  // Serial.println("I2CHandler::init OK");

  // analog readings: monitor battery and infrared captors
  AnalogReadings::init();
  Serial.println("AnalogReadings::init() OK");

  // init the ServoHandler
  ServoHandler::init();
  Serial.println("ServoHandler::init() OK");
  ServoHandler::servoUp();

  Serial.println("Low Level Loop");
  Robot::startLowLevelLoop();

  TelemetryHandler::begin();

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
  // taskYIELD();
  Robot* robot = Robot::getInstance();

  for (;;)
  {
    Serial.println("Moving...");
    strategy::go_forward(robot->motionController, 1500);
    robot->currentRobotState_ = RobotState::MOVING_SETUP_TRAJECTORY;
    robot->motionController->waitForTrajectoryFinished();
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // strategy::go_to_zone_3(motionController);
    strategy::go_forward(robot->motionController, -1500);
    robot->currentRobotState_ = RobotState::MOVING_SETUP_TRAJECTORY;
    robot->motionController->waitForTrajectoryFinished();
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    // strategy::make_a_square(robot->motionController);
    // robot->currentRobotState_ = RobotState::MOVING_SETUP_TRAJECTORY;
    // robot->motionController->waitForTrajectoryFinished();
    // vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
#endif

  // MessageHandler::startListening();
  MessageHandler::startReportBroadcast();
}

void loop()
{
  for(;;)
  {
    taskYIELD();
    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
  
}
