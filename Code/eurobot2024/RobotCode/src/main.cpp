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
#include <TelemetryHandler.hpp>


/////////////////////////////////////////////////////////////////////
// Tasks
/////////////////////////////////////////////////////////////////////

using namespace miam;
using namespace miam::trajectory;

#define FASTLED_PIN 0
#include <FastLED.h>

// How many leds in your strip?
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

void task_fastled(void* parameters)
{
  FastLED.addLeds<WS2812, FASTLED_PIN, GRB>(leds, NUM_LEDS);  // GRB ordering is typical
  for(;;)
  {
    // Turn the LED on, then pause
    leds[0] = CRGB::Red;
    FastLED.show();
    delay(500);
    // Now turn the LED off, then pause
    leds[0] = CRGB::Black;
    FastLED.show();
    delay(500);
  }
}

/////////////////////////////////////////////////////////////////////
// Setup
/////////////////////////////////////////////////////////////////////


// #define DEBUG_MODE_SIMPLE_TRAJECTORY

void setup()
{
  Serial.begin(115200);
  Serial.println("Setup begin");
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  // xTaskCreatePinnedToCore(
  //   task_fastled,
  //   "task_fastled",
  //   1000,
  //   NULL,
  //   30,
  //   NULL,
  //   1
  // );

  Robot::init();

  // start heartbeat
  HeartbeatHandler::start_heartbeat();


  // Serial.println("Attempt connect WiFi");
  
  // // connect wifi
  // WiFiHandler::initWiFi();

  // Init i2c peripherals
  I2CHandler::init();

  // analog readings: monitor battery and infrared captors
  Serial.println("Init analog readings");
  AnalogReadings::init();

  // init the ServoHandler
  ServoHandler::init();
  ServoHandler::servoUp();

  Serial.println("Low Level Loop");
  Robot::startLowLevelLoop();

  // TelemetryHandler::begin();

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
    robot->currentRobotState_ = RobotState::MOVING_SETUP_TRAJECTORY;
    strategy::make_a_square(robot->motionController);
    // strategy::go_to_zone_3(motionController);

    vTaskDelay(5000 / portTICK_PERIOD_MS);
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
