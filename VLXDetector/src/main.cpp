#include <Arduino.h>
#include <I2CHandler.hpp>

#define OUTPUT_BOOL 25
#define OUTPUT_FLOAT 26

#define AVOIDANCE_THRESHOLD 250
#define AVOIDANCE_FAR 400
#define AVOIDANCE_NEAR 100
bool avoidance_bool = false;
float avoidance_float = 0;

void setup() {

  Serial.begin(115200);
  Serial.print("SDA: ");
  Serial.println(SDA);
  Serial.print("SCL: ");
  Serial.println(SCL);

  pinMode(16, OUTPUT);
  pinMode(17, OUTPUT);

  // Init i2c peripherals
  I2CHandler::init();

  pinMode(OUTPUT_BOOL, OUTPUT);

}

uint16_t vlx1_measure;
uint16_t vlx2_measure;

void loop() {
  vlx1_measure = I2CHandler::get_smoothed_vl53l0x();
  vlx2_measure = I2CHandler::get_smoothed_vl53l0x2();

  Serial.print(">vlx1:");
  Serial.println(vlx1_measure);
  Serial.print(">vlx2:");
  Serial.println(vlx2_measure);

  // minimum of the two
  uint16_t vlx_min = std::min(vlx1_measure, vlx2_measure);

  // write digital value
  bool output_bool = vlx_min < AVOIDANCE_THRESHOLD;
  digitalWrite(OUTPUT_BOOL, output_bool);

  // // write analog value
  // char output_float = std::round(std::max(std::min((float)AVOIDANCE_FAR, (float)vlx_min), (float)AVOIDANCE_NEAR) * 255.0f / (AVOIDANCE_FAR - AVOIDANCE_NEAR));
  // dacWrite(OUTPUT_FLOAT, output_float);

  Serial.print(">output_bool:");
  Serial.println(output_bool);
  // Serial.print(">output_float:");
  // Serial.println(output_float);

  vTaskDelay(20 / portTICK_PERIOD_MS);
}
