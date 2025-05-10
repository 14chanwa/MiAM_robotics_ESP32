#include <Arduino.h>

#include <WiFiHandler.hpp>
#include <EncoderHandler.hpp>
#include <StepperHandler.hpp>
#include <MessageHandler.hpp>
// #include <BluetoothReceiverHandler.hpp>

#include <vector>

#include <parameters.hpp>
#include <secret.hpp>

#include <SoftwareButton.hpp>

#include <DrivetrainKinematics.h>


#include <esp_now.h>
typedef struct struct_message {
  uint16_t X;
  uint16_t Y;
} struct_message;
// Create a structured object
struct_message myData;

bool newMessageReceived = false;

// Callback function executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  newMessageReceived = true;
  // Serial.print("Received ");
  // Serial.print(myData.X);
  // Serial.print(", ");
  // Serial.println(myData.Y);
}

Vector2 wheelSpeed_rad_s_({0, 0});
Vector2 remoteStickReading({0, 0});
#define REMOTE_DEAD_ZONE 15
#define REMOTE_MAX_VALUE 128.0

long lastRemoteControllerMillis = 0;
#define REMOTE_CONTROLLER_TIMEOUT 200

SoftwareButton buttonX(LOW);
SoftwareButton buttonO(LOW);

enum ControlState
{
  CONTROLLED_BY_REMOTE,
  CONTROLLED_BY_MESSAGE
};

enum MotorState
{
  HIGHZ,
  POWERED
};

enum StepperCommunicationState
{
  IDLE,
  COMMAND_REQUESTED
};

ControlState currentControlState = CONTROLLED_BY_REMOTE;
MotorState currentMotorState = POWERED;
StepperCommunicationState stepperCommunicationState = IDLE;




/*
  Tasks
*/

void task_print_encoders(void *parameters)
{
  for (;;)
  {
    log_d("stepper init: %d - right encoder: %d - left encoder: %d", stepper_handler::is_inited(), encoder_handler::getRightValue(), encoder_handler::getLeftValue());
    log_d(
      "motorSpeed[RIGHT] %f - motorSpeed[LEFT] %f", 
      wheelSpeed_rad_s_[RIGHT_ENCODER_INDEX], 
      wheelSpeed_rad_s_[LEFT_ENCODER_INDEX]
    );

    log_d(
      "remotestick %f -  %f", 
      remoteStickReading[0],
      remoteStickReading[1]
    );
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

/*
  Functions
*/

DrivetrainKinematics driveTrainKinematics(
    robotdimensions::wheelRadius,
    robotdimensions::wheelSpacing,
    robotdimensions::encoderWheelRadius,
    robotdimensions::encoderWheelSpacing);

// Compute max stepper motor speed.
const int maxStepperSpeed = robotdimensions::maxWheelSpeed / robotdimensions::wheelRadius / robotdimensions::stepSize;
const int maxStepperAcceleration = robotdimensions::maxWheelAcceleration / robotdimensions::wheelRadius / robotdimensions::stepSize;

const float maxWheelSpeedRadS = robotdimensions::maxWheelSpeed / robotdimensions::wheelRadius;

// Max linear speed is max speed achieved when wheel spin forward
const float maxLinearBaseSpeed = driveTrainKinematics.forwardKinematics(
                                                         WheelSpeed(maxWheelSpeedRadS, maxWheelSpeedRadS))
                                     .linear; // mm/s
// Max angular speed ; to tweak
const float maxAngularBaseSpeed = M_PI; // rad/s

// void task_async_connect_wifi(void *parameters)
// {
//   // WiFi
//   const uint8_t newMacAddress[] = SECRET_AVOIDANCE_ROBOT_WIFI_MAC;
//   // wifi_handler::setBaseMACAddress(newMacAddress);
//   // wifi_handler::printCurrentMACAddress();
//   wifi_handler::connectSTA(SECRET_SSID, SECRET_PASSWORD);
//   // wifi_handler::printCurrentMACAddress();

//   // Message broadcast
//   message_handler::init();

//   // When wifi is connected, this task will end
//   vTaskDelete(NULL);
// }

void setup()
{
  vTaskDelay(100 / portTICK_PERIOD_MS);

  Serial.begin(115200);

  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
 
  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Register callback function
  esp_now_register_recv_cb(OnDataRecv);

  // Encoders
  encoder_handler::init();

  // // Async init WiFi, message and OTA
  // xTaskCreate(task_async_connect_wifi, "task_connect_wifi", 10000, NULL, 30, NULL);

  // // Bluetooth serial receiver
  // bluetooth_receiver_handler::init();

  // Print encoder
  xTaskCreate(task_print_encoders, "task_print_encoders", 10000, NULL, 20, NULL);

  // Steppers
  while (!stepper_handler::is_inited())
  {
    if (!stepper_handler::init(maxStepperSpeed, maxStepperAcceleration, robotdimensions::stepMode))
    {
      Serial.print(".");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }
}



void loop()
{
  // newMessageReceived = bluetooth_receiver_handler::newMessageReceived();
  bool needSendCommand = false;
  if (newMessageReceived)
  {
    lastRemoteControllerMillis = millis();
    remoteStickReading[0] =  -(myData.Y - 2048) * 256.0 / 4096.0;
    remoteStickReading[1] =  (myData.X - 2048) * 256.0 / 4096.0;
    // Serial.print("New message received");
    newMessageReceived = false;
    needSendCommand = true;

  //   const ps3_data_type::ps3_t *data = bluetooth_receiver_handler::getData();
  //   buttonX.update(data->button.cross);
  //   buttonO.update(data->button.circle);
  //   // full left = -127, full right = 128
  //   remoteStickReading[0] = data->analog.stick.lx;
  //   // full back = -127, full forward = 128
  //   remoteStickReading[1] = -data->analog.stick.ly;

  //   // Handle X was pressed
  //   if (buttonX.getEvent() == ButtonEvent::NEW_STATE_HIGH)
  //   {
  //     switch (currentMotorState)
  //     {
  //     case MotorState::POWERED:
  //       stepper_handler::getStepperMotors()->highZ();
  //       currentMotorState = MotorState::HIGHZ;
  //       log_i("Motor -> HIGHZ");
  //       break;
  //     case MotorState::HIGHZ:
  //       stepper_handler::getStepperMotors()->hardStop();
  //       currentMotorState = MotorState::POWERED;
  //       log_i("Motor -> POWERED");
  //       break;
  //     }
  //   }

  //   // Handle O was pressed
  //   if (buttonO.getEvent() == ButtonEvent::NEW_STATE_HIGH)
  //   {
  //     switch (currentControlState)
  //     {
  //     case ControlState::CONTROLLED_BY_MESSAGE:
  //       currentControlState = ControlState::CONTROLLED_BY_REMOTE;
  //       log_i("Control -> CONTROLLED_BY_REMOTE");
  //       break;
  //     case ControlState::CONTROLLED_BY_REMOTE:
  //       currentControlState = ControlState::CONTROLLED_BY_MESSAGE;
  //       log_i("Control -> CONTROLLED_BY_MESSAGE");
  //       break;
  //     }
  //   }
  //   // Stop the robot
  //   wheelSpeed_rad_s_[RIGHT_ENCODER_INDEX] = 0;
  //   wheelSpeed_rad_s_[LEFT_ENCODER_INDEX] = 0;
  //   stepperCommunicationState = StepperCommunicationState::COMMAND_REQUESTED;
  }

  if (currentControlState == ControlState::CONTROLLED_BY_REMOTE)
  {
    if (millis() - lastRemoteControllerMillis > REMOTE_CONTROLLER_TIMEOUT)
    {
      log_e("Remote controller timeout");
      // Stop the robot
      wheelSpeed_rad_s_[RIGHT_ENCODER_INDEX] = 0;
      wheelSpeed_rad_s_[LEFT_ENCODER_INDEX] = 0;
      stepperCommunicationState = StepperCommunicationState::COMMAND_REQUESTED;
    }
    else
    {
      if (needSendCommand)
      {
        // Get the angle of the joystick vector
        float x = remoteStickReading[0] / REMOTE_MAX_VALUE;
        float y = remoteStickReading[1] / REMOTE_MAX_VALUE;
        float joystickAngle = atan2f(y, x);

        // Get the norm of the joystick vector
        float joystickNorm = remoteStickReading.norm();

        // log_v("x: %f - y: %f - joystickAngle: %f - joystickNorm: %f",
        //   x, y, joystickAngle, joystickNorm);

        if (joystickNorm > REMOTE_DEAD_ZONE)
        {

          WheelSpeed newWheelSpeed = driveTrainKinematics.inverseKinematics(
              BaseSpeed(
                  maxLinearBaseSpeed * x,
                  maxAngularBaseSpeed * y // if joystick pushed left, angle should be direct
                ) 
              );

          // Trim the wheel speed if necessary
          float maxSpeedCalculated = max(abs(newWheelSpeed.right), abs(newWheelSpeed.left));
          
          if (abs(maxSpeedCalculated) > maxWheelSpeedRadS)
          {
            newWheelSpeed.right = newWheelSpeed.right * maxWheelSpeedRadS / maxSpeedCalculated;
            newWheelSpeed.left = newWheelSpeed.left * maxWheelSpeedRadS / maxSpeedCalculated;
          }

          // Set speed
          wheelSpeed_rad_s_[RIGHT_ENCODER_INDEX] = newWheelSpeed.right;
          wheelSpeed_rad_s_[LEFT_ENCODER_INDEX] = newWheelSpeed.left;
          stepperCommunicationState = StepperCommunicationState::COMMAND_REQUESTED;

          log_d(
            "newWheelSpeed.right %f - newWheelSpeed.left %f - motorSpeed[RIGHT] %f - motorSpeed[LEFT] %f", 
            newWheelSpeed.right, 
            newWheelSpeed.left, 
            wheelSpeed_rad_s_[RIGHT_ENCODER_INDEX], 
            wheelSpeed_rad_s_[LEFT_ENCODER_INDEX]
          );
        }
        else
        {
          wheelSpeed_rad_s_[RIGHT_ENCODER_INDEX] = 0;
          wheelSpeed_rad_s_[LEFT_ENCODER_INDEX] = 0;
          stepperCommunicationState = StepperCommunicationState::COMMAND_REQUESTED;
        }
        needSendCommand = false;
      }
    }
  }

  if (stepperCommunicationState == StepperCommunicationState::COMMAND_REQUESTED)
  {
    if (currentMotorState == MotorState::POWERED)
    {
      stepper_handler::setSpeed(
        wheelSpeed_rad_s_[RIGHT_ENCODER_INDEX] / robotdimensions::stepSize, 
        wheelSpeed_rad_s_[LEFT_ENCODER_INDEX] / robotdimensions::stepSize
      );
    }
    stepperCommunicationState = StepperCommunicationState::IDLE;
  }

  vTaskDelay(10 / portTICK_PERIOD_MS);
}
