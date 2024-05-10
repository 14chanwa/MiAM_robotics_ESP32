#include <Arduino.h>
// #include <L298N.h>
#include <I2CHandler.hpp>
#include <Button.hpp>

#define STBY 12

#define IN1_A 4
#define IN2_A 17
#define EN_A 27

#define IN1_B 23
#define IN2_B 15
#define EN_B 13

#define CHANNEL_A 2
#define CHANNEL_B 4

#define AVOIDANCE_THRESHOLD 70

#define START_SWITCH 26
#define EMERGENCY_STOP 25

// LED CONFIGURATION
#define LED_SLOW_BLINK_MS 1000
#define LED_FAST_BLINK_MS 150

#define LED_PIN 14
#define LED_PWM_CHANNEL 0

#define LED_PWM_FREQUENCY 1000
#define LED_PWM_RESOLUTION 8
#define LED_PWM_HIGH_LEVEL 255

// Read buttons with hysteresis
Button start_switch_button(START_SWITCH);
Button emergency_stop_button(EMERGENCY_STOP);

void task_read_pins(void* parameters)
{
  
  for(;;)
  {
    start_switch_button.update();
    emergency_stop_button.update();
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}


bool matchStarted = false;
long millisMatchStarted;

enum MatchState
{
  WAITING_FOR_START_SWITCH = 0,
  WAITING_FOR_START = 1,
  MATCH_STARTED_WAITING = 2,
  MATCH_STARTED_ACTION = 3,
  MATCH_ENDED = 4
};

MatchState matchState = WAITING_FOR_START_SWITCH;

void taskEvolveMatchState(void* parameters)
{
  
  for (;;)
  {

    // Check buttons
    ButtonEvent startSwitchButtonEvent = start_switch_button.getEvent();
    ButtonEvent emergencyButtonEvent = emergency_stop_button.getEvent();

    if (matchState == MatchState::WAITING_FOR_START_SWITCH)
    {
      // WAITING_FOR_START_SWITCH -> WAITING_FOR_START
      if (startSwitchButtonEvent == ButtonEvent::NEW_STATE_LOW)
      {
        Serial.println("WAITING_FOR_START_SWITCH -> WAITING_FOR_START");
        matchState = MatchState::WAITING_FOR_START;
      }

      // For debug : if click once on emg stop, start in 2 sec
      // MATCH_STARTED_WAITING
      if (emergencyButtonEvent == ButtonEvent::NEW_STATE_LOW)
      {
        Serial.println("WAITING_FOR_START_SWITCH -> MATCH_STARTED_WAITING");
        matchStarted = true;
        millisMatchStarted = millis() - 88000;
        matchState = MatchState::MATCH_STARTED_WAITING;
      }
    }
    else if (matchState == MatchState::WAITING_FOR_START)
    {
      // WAITING_FOR_START_SWITCH -> MATCH_STARTED_WAITING
      if (startSwitchButtonEvent == ButtonEvent::NEW_STATE_HIGH)
      {
        Serial.println("WAITING_FOR_START -> MATCH_STARTED_WAITING");
        matchStarted = true;
        millisMatchStarted = millis();
        matchState = MatchState::MATCH_STARTED_WAITING;
      }
    }
    else if (matchState == MatchState::MATCH_STARTED_WAITING)
    {
      // Cancel match
      // MATCH_STARTED_WAITING -> WAITING_FOR_START
      if (startSwitchButtonEvent == ButtonEvent::NEW_STATE_LOW)
      {
        Serial.println("MATCH_STARTED_WAITING -> WAITING_FOR_START");
        matchState = MatchState::WAITING_FOR_START;
      }
      // MATCH_STARTED_WAITING -> MATCH_STARTED_ACTION
      else if (millis() - millisMatchStarted >= 90000)
      {
        Serial.println("MATCH_STARTED_WAITING -> MATCH_STARTED_ACTION");
        matchState = MatchState::MATCH_STARTED_ACTION;
      }
    }
    else if (matchState == MatchState::MATCH_STARTED_ACTION)
    {
      // Emergency stop
      // MATCH_STARTED_ACTION -> MATCH_ENDED
      if (emergencyButtonEvent == ButtonEvent::NEW_STATE_LOW)
      {
        Serial.println("MATCH_STARTED_ACTION -> MATCH_ENDED");
        matchState = MatchState::MATCH_ENDED;
      }
      // MATCH_STARTED_ACTION -> MATCH_ENDED
      else if (millis() - millisMatchStarted >= 100000)
      {
        Serial.println("MATCH_STARTED_ACTION -> MATCH_ENDED");
        matchState = MatchState::MATCH_ENDED;
      }
    }
    else if (matchState == MatchState::MATCH_ENDED)
    {
      // MATCH_ENDED -> WAITING_FOR_START
      if (startSwitchButtonEvent == ButtonEvent::NEW_STATE_LOW)
      {
        Serial.println("MATCH_ENDED -> WAITING_FOR_START");
        matchState = MatchState::WAITING_FOR_START;
      }
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
};

void task_blink_led(void* parameters)
{
  for (;;)
  {
      if (matchState == MatchState::WAITING_FOR_START_SWITCH)
      {
          // constant ON
          ledcWrite(LED_PWM_CHANNEL, LED_PWM_HIGH_LEVEL);
          vTaskDelay(LED_FAST_BLINK_MS / portTICK_PERIOD_MS);
      }
      else if (matchState == MatchState::MATCH_STARTED_WAITING || matchState == MatchState::MATCH_STARTED_ACTION)
      {
          // blink fast
          ledcWrite(LED_PWM_CHANNEL, LED_PWM_HIGH_LEVEL);
          vTaskDelay(LED_FAST_BLINK_MS / portTICK_PERIOD_MS);
          ledcWrite(LED_PWM_CHANNEL, 0);
          vTaskDelay(LED_FAST_BLINK_MS / portTICK_PERIOD_MS);
      }
      else
      {
          // blink slow
          ledcWrite(LED_PWM_CHANNEL, LED_PWM_HIGH_LEVEL);
          vTaskDelay(LED_SLOW_BLINK_MS / portTICK_PERIOD_MS);
          ledcWrite(LED_PWM_CHANNEL, 0);
          vTaskDelay(LED_SLOW_BLINK_MS / portTICK_PERIOD_MS);
      }
  }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  pinMode(STBY, OUTPUT);
  pinMode(IN1_A, OUTPUT);
  pinMode(IN2_A, OUTPUT);
  pinMode(IN1_B, OUTPUT);
  pinMode(IN2_B, OUTPUT);

  // pinMode(EN_A, OUTPUT);
  // pinMode(EN_B, OUTPUT);
  ledcAttachPin(EN_A, CHANNEL_A);
  ledcSetup(CHANNEL_A, 800, 8);
  ledcAttachPin(EN_B, CHANNEL_B);
  ledcSetup(CHANNEL_B, 800, 8);

  ledcWrite(CHANNEL_A, 0);
  ledcWrite(CHANNEL_B, 0);

  digitalWrite(STBY, HIGH);

  digitalWrite(IN1_A, HIGH);
  digitalWrite(IN2_A, LOW);

  digitalWrite(IN1_B, LOW);
  digitalWrite(IN2_B, HIGH);

  I2CHandler::init();
  start_switch_button.init();
  emergency_stop_button.init();

  xTaskCreatePinnedToCore(
    task_read_pins,
    "task_read_pins",
    10000,
    NULL,
    90,
    NULL,
    0
  );


  xTaskCreatePinnedToCore(
    taskEvolveMatchState,
    "taskEvolveMatchState",
    10000,
    NULL,
    50,
    NULL,
    0
  );

  ledcAttachPin(LED_PIN, LED_PWM_CHANNEL);
  ledcSetup(LED_PWM_CHANNEL, LED_PWM_FREQUENCY, LED_PWM_RESOLUTION);
  ledcWrite(LED_PWM_CHANNEL, LED_PWM_HIGH_LEVEL);
  xTaskCreatePinnedToCore(
    task_blink_led,
    "task_blink_led",
    10000,
    NULL,
    30,
    NULL,
    0
  );

}

uint16_t vlx1_measure;

void loop() {
  // put your main code here, to run repeatedly:
  vlx1_measure = I2CHandler::get_smoothed_vl53l0x();
  // vlx2_measure = I2CHandler::get_smoothed_vl53l0x2();

  Serial.print(">vlx1:");
  Serial.println(vlx1_measure);
  // Serial.print(">vlx2:");
  // Serial.println(vlx2_measure);

  // minimum of the two
  uint16_t vlx_min = vlx1_measure; //std::min(vlx1_measure, vlx2_measure);

  // write digital value
  bool output_bool = vlx_min < AVOIDANCE_THRESHOLD;
  // digitalWrite(OUTPUT_BOOL, output_bool);

  // // write analog value
  // char output_float = std::round(std::max(std::min((float)AVOIDANCE_FAR, (float)vlx_min), (float)AVOIDANCE_NEAR) * 255.0f / (AVOIDANCE_FAR - AVOIDANCE_NEAR));
  // dacWrite(OUTPUT_FLOAT, output_float);

  Serial.print(">output_bool:");
  Serial.println(output_bool);
  // Serial.print(">output_float:");
  // Serial.println(output_float);

  Serial.print(">start_switch:");
  Serial.println(digitalRead(START_SWITCH));
  Serial.print(">engstop:");
  Serial.println(digitalRead(EMERGENCY_STOP));


  Serial.print(">matchState:");
  Serial.println(matchState);

  // go forward if match started
  // ignore vlx for the last 1 sec
  if (matchState == MatchState::MATCH_STARTED_ACTION)
  {
    if (vlx_min >= AVOIDANCE_THRESHOLD)
    {
      ledcWrite(CHANNEL_A, 255);
      ledcWrite(CHANNEL_B, 255); 
    }
    else if (vlx_min >= 10 && (millis()-millisMatchStarted >= 99000))
    {
      ledcWrite(CHANNEL_A, 75);
      ledcWrite(CHANNEL_B, 75);
    }
    else
    {
      ledcWrite(CHANNEL_A, 0);
      ledcWrite(CHANNEL_B, 0);
    }
  }
  else
  {
    ledcWrite(CHANNEL_A, 0);
    ledcWrite(CHANNEL_B, 0);
  }

  vTaskDelay(20 / portTICK_PERIOD_MS);
}
