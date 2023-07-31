#include <Arduino.h>
#include <parameters.hpp>
#include <tasks.hpp>
#include <Motor.hpp>

float speed = 0;

// printing
char buffer[40];

// Motors
Motor* leftMotor;
Motor* rightMotor;

void setup()
{
  leftMotor = new Motor(EN_A, IN1_A, IN2_A, 0);
  rightMotor = new Motor(EN_B, IN1_B, IN2_B, 1);
  Serial.begin(115200);

  // run_blink_led();
  run_monitor_battery();
  run_handle_encoders();
  vTaskDelay(100.0 / portTICK_PERIOD_MS);
  run_low_level_loop(leftMotor);
  // run_low_level_loop(&rightMotor);
}

int increment = 10;

void loop()
{
  leftMotor->setSpeed(speed);
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
  // // myMotors.setSpeed(speed);
  // // myMotors.backward();
  // // vTaskDelay(100.0 / portTICK_PERIOD_MS);
  // // Serial.print("run bat reading: ");
  // // Serial.println(get_current_battery_reading());
  // // vTaskDelay(1000.0 / portTICK_PERIOD_MS);

  // // Serial.print("encoder speed backward: ");
  // // values = get_encoder_speed();
  // // sprintf(buffer, "left: %d - right: %d", values.left_value, values.right_value);
  // // Serial.println(buffer);

  // // myMotors.stop();
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
