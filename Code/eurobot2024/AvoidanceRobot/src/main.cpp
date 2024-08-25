#include <Arduino.h>

#include <WiFiHandler.hpp>
#include <EncoderHandler.hpp>
#include <StepperHandler.hpp>
#include <MessageHandler.hpp>
#include <BluetoothReceiverHandler.hpp>

#include <vector>

#include <parameters.hpp>
#include <secret.hpp>

/*
  Tasks
*/

void task_print_encoders(void* parameters)
{
  bool tock = false; 
  for(;;)
  {
    if (tock)
    {
      Serial.print("tock ");
    }
    else
    {
      Serial.print("tick ");
    }
    Serial.print(stepper_handler::is_inited());
    Serial.print(", ");
    Serial.print(encoder_handler::getRightValue());
    Serial.print(", ");
    Serial.print(encoder_handler::getLeftValue());
    Serial.println();
    tock = !tock;
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

std::vector<float > targetController({0, 0});


/*
  Functions
*/


// Compute max stepper motor speed.
int maxSpeed = robotdimensions::maxWheelSpeed / robotdimensions::wheelRadius / robotdimensions::stepSize;
int maxAcceleration = robotdimensions::maxWheelAcceleration / robotdimensions::wheelRadius / robotdimensions::stepSize;

void setup()
{
  vTaskDelay(100 / portTICK_PERIOD_MS);

  Serial.begin(115200);

  // Encoders
  encoder_handler::init();

  // WiFi
  const uint8_t newMacAddress[] = SECRET_AVOIDANCE_ROBOT_WIFI_MAC;
  // wifi_handler::setBaseMACAddress(newMacAddress);
  // wifi_handler::printCurrentMACAddress();
  wifi_handler::connectSTA(SECRET_SSID, SECRET_PASSWORD);
  // wifi_handler::printCurrentMACAddress();

  // // Message broadcast
  // message_handler::init();

  // Bluetooth serial receiver
  bluetooth_receiver_handler::init();

  // Print encoder
  xTaskCreate(task_print_encoders, "task_print_encoders", 1000, NULL, 10, NULL);

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

void loop()
{

  if (bluetooth_receiver_handler::newMessageReceived())
  {
    const ps3_data_type::ps3_t* data = bluetooth_receiver_handler::getData();
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

    stepper_handler::setSpeed(motorSpeed_[RIGHT_ENCODER_INDEX], motorSpeed_[LEFT_ENCODER_INDEX]);
    
  }

  vTaskDelay(10 / portTICK_PERIOD_MS);

}
