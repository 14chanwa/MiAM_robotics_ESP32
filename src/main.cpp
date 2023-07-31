#include <Arduino.h>
#include <parameters.hpp>
#include <tasks.hpp>
#include <RobotWheel.hpp>

float speed = 0;

// printing
char buffer[40];

// RobotWheels
RobotWheel* leftRobotWheel;
RobotWheel* rightRobotWheel;

void IRAM_ATTR encoderInterruptLeft()
{
  leftRobotWheel->handleEncoderInterrupt();
}

void setup()
{
  leftRobotWheel = new RobotWheel(EN_A, IN1_A, IN2_A, ENCODER_A1, ENCODER_B1);
  rightRobotWheel = new RobotWheel(EN_B, IN1_B, IN2_B, ENCODER_A2, ENCODER_B2);
  Serial.begin(115200);

  run_monitor_battery();

  // interupts have to be handled outside low level loop
  pinMode(leftRobotWheel->pinEncoderA_, INPUT_PULLUP);
  pinMode(leftRobotWheel->pinEncoderB_, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(leftRobotWheel->pinEncoderA_), encoderInterruptLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftRobotWheel->pinEncoderB_), encoderInterruptLeft, CHANGE);

  leftRobotWheel->startLowLevelLoop();
}

int increment = 10;

void loop()
{
  leftRobotWheel->setWheelSpeed(RPM_TO_RAD_S(speed));
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

  vTaskDelay(250.0 / portTICK_PERIOD_MS);


  // // put your main code here, to run repeatedly:

  // // Serial.print("speed: ");
  // // Serial.println(speed);

  // // forward
  // setSpeed(speed);
  // vTaskDelay(100.0 / portTICK_PERIOD_MS);
  // // Serial.print("run bat reading: ");
  // // Serial.println(get_current_battery_reading());
  // vTaskDelay(2000.0 / portTICK_PERIOD_MS);

  // // Serial.print("encoder speed forwward: ");
  // // values = get_encoder_speed();
  // // sprintf(buffer, "left: %f - right: %f", values.left_value, values.right_value);
  // // Serial.println(buffer);

  // setSpeed(0.0);
  // vTaskDelay(100.0 / portTICK_PERIOD_MS);
  // // Serial.print("stop bat reading: ");
  // // Serial.println(get_current_battery_reading());
  // // Serial.print("min bat reading: ");
  // // Serial.println(get_min_battery_reading());
  // vTaskDelay(2000.0 / portTICK_PERIOD_MS);

  // // Serial.print("encoder count after forwward: ");
  // // values = get_encoder_values();
  // // sprintf(buffer, "left: %d - right: %d", values.left_value, values.right_value);
  // // Serial.println(buffer);

  // // // backward
  // // myRobotWheels.setSpeed(speed);
  // // myRobotWheels.backward();
  // // vTaskDelay(100.0 / portTICK_PERIOD_MS);
  // // Serial.print("run bat reading: ");
  // // Serial.println(get_current_battery_reading());
  // // vTaskDelay(1000.0 / portTICK_PERIOD_MS);

  // // Serial.print("encoder speed backward: ");
  // // values = get_encoder_speed();
  // // sprintf(buffer, "left: %d - right: %d", values.left_value, values.right_value);
  // // Serial.println(buffer);

  // // myRobotWheels.stop();
  // // vTaskDelay(100.0 / portTICK_PERIOD_MS);
  // // Serial.print("stop bat reading: ");
  // // Serial.println(get_current_battery_reading());
  // // Serial.print("min bat reading: ");
  // // Serial.println(get_min_battery_reading());
  // // vTaskDelay(1000.0 / portTICK_PERIOD_MS);

  // // Serial.print("encoder count after back: ");
  // // values = get_encoder_values();
  // // sprintf(buffer, "left: %d - right: %d", values.left_value, values.right_value);
  // // Serial.println(buffer);

  // speed += 10;
  // if (speed > 150)
  // {
  //   speed = 0;
  // }
}
