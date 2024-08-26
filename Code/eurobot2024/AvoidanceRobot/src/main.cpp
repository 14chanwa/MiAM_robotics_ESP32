#include <Arduino.h>

#include <WiFiHandler.hpp>
#include <EncoderHandler.hpp>
#include <StepperHandler.hpp>
#include <MessageHandler.hpp>
#include <BluetoothReceiverHandler.hpp>

#include <vector>

#include <parameters.hpp>
#include <secret.hpp>

#include <SoftwareButton.hpp>

/*
  Tasks
*/

void task_print_encoders(void *parameters)
{
  for (;;)
  {
    log_d("stepper init: %d - right encoder: %d - left encoder: %d", stepper_handler::is_inited(), encoder_handler::getRightValue(), encoder_handler::getLeftValue());
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

std::vector<float> targetController({0, 0});

/*
  Functions
*/

// Compute max stepper motor speed.
int maxSpeed = robotdimensions::maxWheelSpeed / robotdimensions::wheelRadius / robotdimensions::stepSize;
int maxAcceleration = robotdimensions::maxWheelAcceleration / robotdimensions::wheelRadius / robotdimensions::stepSize;

void task_async_connect_wifi(void* parameters)
{
  // WiFi
  const uint8_t newMacAddress[] = SECRET_AVOIDANCE_ROBOT_WIFI_MAC;
  // wifi_handler::setBaseMACAddress(newMacAddress);
  // wifi_handler::printCurrentMACAddress();
  wifi_handler::connectSTA(SECRET_SSID, SECRET_PASSWORD);
  // wifi_handler::printCurrentMACAddress();

  // Message broadcast
  message_handler::init();

  // When wifi is connected, this task will end
  vTaskDelete(NULL);
}

void setup()
{
  vTaskDelay(100 / portTICK_PERIOD_MS);

  Serial.begin(115200);

  // Encoders
  encoder_handler::init();

  // Async init WiFi, message and OTA
  xTaskCreate(task_async_connect_wifi, "task_connect_wifi", 10000, NULL, 30, NULL);

  // Bluetooth serial receiver
  bluetooth_receiver_handler::init();

  // Print encoder
  xTaskCreate(task_print_encoders, "task_print_encoders", 10000, NULL, 20, NULL);

  // Steppers
  while (!stepper_handler::is_inited())
  {
    if (!stepper_handler::init(maxSpeed, maxAcceleration, robotdimensions::stepMode))
    {
      Serial.print(".");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }
}

std::vector<float> motorSpeed_({0, 0});
#define DEAD_ZONE 15

long lastControllerMillis = 0;
#define CONTROLLER_TIMEOUT 200
bool motorHighZ_ = false;

SoftwareButton buttonX(LOW);

void loop()
{

  if (bluetooth_receiver_handler::newMessageReceived())
  {

    const ps3_data_type::ps3_t *data = bluetooth_receiver_handler::getData();
    buttonX.update(data->button.cross);

    bool buttonWasPressed = buttonX.getEvent() == ButtonEvent::NEW_STATE_HIGH;

    if (buttonWasPressed)
    {
      if (!motorHighZ_)
      {
        stepper_handler::getStepperMotors()->highZ();
        log_i("Stepper to highZ");
      }
      else
      {
        stepper_handler::getStepperMotors()->hardStop();
        log_i("Stepper to hardStop");
      }
      motorHighZ_ = !motorHighZ_;
    }
    else
    {

      targetController[0] = -data->analog.stick.ly;
      targetController[1] = -data->analog.stick.ly;

      if (abs(targetController[0]) > DEAD_ZONE)
      {
        motorSpeed_[RIGHT_ENCODER_INDEX] = maxSpeed * targetController[0] / 128.0;
        motorSpeed_[LEFT_ENCODER_INDEX] = maxSpeed * targetController[0] / 128.0;
      }
      else
      {
        motorSpeed_[RIGHT_ENCODER_INDEX] = 0;
        motorSpeed_[LEFT_ENCODER_INDEX] = 0;
      }

      if (!motorHighZ_)
      {
        stepper_handler::setSpeed(motorSpeed_[RIGHT_ENCODER_INDEX], motorSpeed_[LEFT_ENCODER_INDEX]);
      }
    }
  }
  else if (millis() - lastControllerMillis > CONTROLLER_TIMEOUT)
  {
    // Stop the robot
    motorSpeed_[RIGHT_ENCODER_INDEX] = 0;
    motorSpeed_[LEFT_ENCODER_INDEX] = 0;
  }

  vTaskDelay(10 / portTICK_PERIOD_MS);
}
