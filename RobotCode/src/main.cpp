#include <Arduino.h>
#include <parameters.hpp>
#include <tasks.hpp>
#include <DrivetrainKinematics.h>
#include <RobotPosition.h>
#include <Trajectory.h>
#include <StraightLine.h>
#include <PointTurn.h>
#include <Utilities.h>
#include <MotionController.hpp>
#include <WiFi.h>
#include <secret.hpp>

#include <RobotBaseDC.hpp>
#include <RobotBaseStepper.hpp>

#include <Preferences.h>
#include <SampledTrajectory.h>

#include <Servo.hpp>

#include <Strategy.hpp>
#include <AnalogReadings.hpp>
#include <I2CHandler.hpp>

#define MATCH_DURATION_S 100.0f
#define MATCH_PAMI_START_TIME_S 90.0f
#define LED_SLOW_BLINK_MS 1000
#define LED_FAST_BLINK_MS 150

// RobotBase
AbstractRobotBase* robotBase;

// Robot ID
int robotID = 0;
int length_of_saved_traj_float = 0;
float duration_of_saved_traj = 0.0f;
float* saved_trajectory;

bool match_started = false;
bool trajectory_was_read = false;
bool movement_override = false;
float match_current_time_s = 0.0f;

TrajectoryVector saved_trajectory_vector;

// Preferences
Preferences preferences;

// Display
DisplayInformations display_informations;

// Semaphores
SemaphoreHandle_t xMutex_Serial = NULL;

#ifdef SEND_TELEPLOT_UDP

  #include <TeleplotArduino.hpp>
  // Teleplot teleplot("192.168.0.255", 47269);
  Teleplot teleplot("10.42.0.255", 47269);

#endif

#ifdef ENABLE_OTA_UPDATE
#include <ArduinoOTA.h>

void task_handle_ota(void* parameters)
{
  for(;;)
  {
    ArduinoOTA.handle();
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
#endif

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(SECRET_SSID, SECRET_PASSWORD);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
  delay(1000);

#ifdef ENABLE_OTA_UPDATE
  ArduinoOTA.begin();

  xTaskCreate(
    task_handle_ota,
    "task_handle_ota",
    10000,
    NULL,
    30,
    NULL
  );
#endif
}

/////////////////////////////////////////////////////////////////////
// Tasks
/////////////////////////////////////////////////////////////////////

using namespace miam;
using namespace miam::trajectory;

// Motion controller
MotionController* motionController;

// low level loop timing
float dt_lowLevel_ms = 0.0;
float dt_period_ms = 0.0;

DrivetrainMeasurements measurements;
DrivetrainTarget target;

void logTelemetry(void* parameters)
{
  for(;;)
  {

    #ifdef SEND_TELEPLOT_UDP

      // teleplot.update("currentPosition.x", motionController->currentPosition_.x);
      // teleplot.update("currentPosition.y", motionController->currentPosition_.y);
      RobotPosition curPos = motionController->getCurrentPosition();
      teleplot.update2D("currentPosition", curPos.x, curPos.y);
      teleplot.update("currentPosition.theta", curPos.theta);
      // teleplot.update("targetPosition.x", motionController->targetPoint.position.x);
      // teleplot.update("targetPosition.y", motionController->targetPoint.position.y);
      teleplot.update2D("targetPosition", motionController->targetPoint.position.x, motionController->targetPoint.position.y);
      teleplot.update("targetPosition.theta", motionController->targetPoint.position.theta);
      teleplot.update("targetPoint.linear", motionController->targetPoint.linearVelocity);
      teleplot.update("targetPoint.angular", motionController->targetPoint.angularVelocity);
      teleplot.update("clampedSlowDownCoeff_", motionController->clampedSlowDownCoeff_);
      // teleplot.update("dt_period_ms", dt_period_ms, "", 0, TELEPLOT_FLAG_NOPLOT);
      // teleplot.update("dt_lowLevel_ms", dt_lowLevel_ms, "", 0, TELEPLOT_FLAG_NOPLOT);
      teleplot.update("battery_reading", AnalogReadings::get_current_battery_reading(), "", 0, TELEPLOT_FLAG_NOPLOT);
      teleplot.update("tcrt0", AnalogReadings::get_current_tcrt0_reading(), "", 0);
      teleplot.update("tcrt1", AnalogReadings::get_current_tcrt1_reading(), "", 0);
      teleplot.update("tcrt2", AnalogReadings::get_current_tcrt2_reading(), "", 0);
      // teleplot.update("touchSensor", get_current_touch_sensor_reading(), "", 0);


      #ifdef USE_DC_MOTORS
      teleplot.update("rightWheelCurrentSpeed", robotBase->getRightWheel()->currentSpeed_);
      teleplot.update("rightWheelTargetSpeed", robotBase->getRightWheel()->targetSpeed_);
      teleplot.update("leftWheelCurrentSpeed", robotBase->getLeftWheel()->currentSpeed_);
      teleplot.update("leftWheelTargetSpeed", robotBase->getLeftWheel()->targetSpeed_);
      teleplot.update("rightBasePWM", static_cast<RobotWheelDC*>(robotBase->getRightWheel())->basePWMTarget_);
      teleplot.update("leftBasePWM", static_cast<RobotWheelDC*>(robotBase->getLeftWheel())->basePWMTarget_);
      teleplot.update("rightNewPWM", static_cast<RobotWheelDC*>(robotBase->getRightWheel())->newPWMTarget_);
      teleplot.update("leftNewPWM", static_cast<RobotWheelDC*>(robotBase->getLeftWheel())->newPWMTarget_);
      #endif


      // sendTelemetry("leftBasePWMTarget", leftRobotWheel->basePWMTarget_);
      // sendTelemetry("leftNewPWMTarget", leftRobotWheel->newPWMTarget_);
      // sendTelemetry("rightBasePWMTarget", rightRobotWheel->basePWMTarget_);
      // sendTelemetry("rightNewPWMTarget", rightRobotWheel->newPWMTarget_);

      #ifdef USE_STEPPER_MOTORS
      // teleplot.update("baseTarget_left_", (static_cast<RobotBaseStepper* >(robotBase))->baseTarget_left_, "", 0, TELEPLOT_FLAG_NOPLOT);
      teleplot.update("targetSpeed_left_", (static_cast<RobotBaseStepper* >(robotBase))->targetSpeed_left_);
      // teleplot.update("targetSpeedDriver_left_", (static_cast<RobotBaseStepper* >(robotBase))->targetSpeedDriver_left_, "", 0, TELEPLOT_FLAG_NOPLOT);
      teleplot.update("currentSpeed_left_", (static_cast<RobotBaseStepper* >(robotBase))->currentSpeed_left_);
      // teleplot.update("baseTarget_right_", (static_cast<RobotBaseStepper* >(robotBase))->baseTarget_right_, "", 0, TELEPLOT_FLAG_NOPLOT);
      teleplot.update("targetSpeed_right_", (static_cast<RobotBaseStepper* >(robotBase))->targetSpeed_right_);
      // teleplot.update("targetSpeedDriver_right_", (static_cast<RobotBaseStepper* >(robotBase))->targetSpeedDriver_right_, "", 0, TELEPLOT_FLAG_NOPLOT);
      teleplot.update("currentSpeed_right_", (static_cast<RobotBaseStepper* >(robotBase))->currentSpeed_right_);
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

void performStrategy(void* parameters)
{
  strategy::perform_strategy(
      motionController,
      strategy::get_waiting_time_s(), 
      saved_trajectory_vector
  );
  Serial.println("Strategy ended");
  vTaskDelete( NULL );
}

void performLowLevel(void* parameters)
{
  long timeStartLoop = 0;
  long timeEndLoop = 0;
  for(;;)
  {
    timeEndLoop = micros();
    dt_period_ms = (timeEndLoop - timeStartLoop) / 1000.0;
    timeStartLoop = timeEndLoop;

    // update match time
    if (match_started)
    {
      match_current_time_s += dt_period_ms / 1000;

      // if match started and trajectory was not read, start
      if (
        !trajectory_was_read && 
        match_current_time_s > MATCH_PAMI_START_TIME_S && 
        match_current_time_s < MATCH_DURATION_S && 
        motionController->isTrajectoryFinished()
      )
      {
        if (saved_trajectory_vector.size() > 0)
        {
          trajectory_was_read = true;
          Serial.println("Beginning match");
          // Start the strategy
          xTaskCreate(
            performStrategy,
            "performStrategy",
            10000,
            NULL,
            10,
            NULL
          );
        }
        else
        {
          Serial.println("No trajectory to perform");
        }
      }

      if (match_current_time_s > MATCH_DURATION_S)
      {
        match_started = false;
        trajectory_was_read = false;
        motionController->clearTrajectories();
      }
    }
    
    // Serial.println("Update sensors");
    robotBase->updateSensors();
    // Serial.println("Get measurements");
    measurements = robotBase->getMeasurements();
    measurements.vlx_range_detection_mm = I2CHandler::get_current_vl53l0x();

    // If playing side::RIGHT side: invert side::RIGHT/side::LEFT encoders.
    if (motionController->isPlayingRightSide_)
    {
        float temp = measurements.motorSpeed[side::RIGHT];
        measurements.motorSpeed[side::RIGHT] = measurements.motorSpeed[side::LEFT];
        measurements.motorSpeed[side::LEFT] = temp;
    }

    // Serial.println("Compute drivetrain motion");
    // Motion occurs only if match started
    target = motionController->computeDrivetrainMotion(
      measurements, 
      dt_period_ms / 1000.0, 
      match_started || movement_override
    );

    // invert kinematics
    if (motionController->isPlayingRightSide_)
    {
      float leftSpeed = target.motorSpeed[side::LEFT];
      target.motorSpeed[side::LEFT] = target.motorSpeed[side::RIGHT];
      target.motorSpeed[side::RIGHT] = leftSpeed;
    }

    // Serial.println("Set base speed");
    robotBase->setBaseSpeed(target);

    // update motor control
    // Serial.println("Update motor control");
    robotBase->updateControl();

    // update sensors
    AnalogReadings::update();

    // Serial.println("Register time");
    timeEndLoop = micros();
    dt_lowLevel_ms = (timeEndLoop - timeStartLoop) / 1000.0;

    // wait till next tick (integer division rounds toward 0)
    vTaskDelay( std::max(1L, (timeStartLoop + LOW_LEVEL_LOOP_TIME_MS * 1000 - timeEndLoop) / 1000) / portTICK_PERIOD_MS);
  }
}

void task_blink_led(void* parameters)
{
  for(;;)
  {
    if (match_started) {
      ledcWrite(0, 32);
      vTaskDelay(LED_FAST_BLINK_MS / portTICK_PERIOD_MS);
      ledcWrite(0, 0);
      vTaskDelay(LED_FAST_BLINK_MS / portTICK_PERIOD_MS);
    }
    else
    {
      ledcWrite(0, 32);
      vTaskDelay(LED_SLOW_BLINK_MS / portTICK_PERIOD_MS);
      ledcWrite(0, 0);
      vTaskDelay(LED_SLOW_BLINK_MS / portTICK_PERIOD_MS);
    }

  }
}

void task_update_vl53l0x(void* parameters)
{
    for (;;)
    {
      I2CHandler::update_vl53l0x();
      vTaskDelay(25 / portTICK_PERIOD_MS);
    }
}

void task_update_ssd1306(void* parameters)
{
  IPAddress ip;
  for (;;)
  {
    // Update IP
    ip = WiFi.localIP();
    sprintf(display_informations.ip_address,"%d:%d:%d:%d", ip[0],ip[1],ip[2],ip[3]);

    // Update ID
    display_informations.id = robotID;

    // Update match state
    display_informations.match_started = match_started;
    display_informations.current_time_s = std::round(match_current_time_s);

    // Update display
    I2CHandler::update_ssd1306(&display_informations);

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

/////////////////////////////////////////////////////////////////////
// Setup
/////////////////////////////////////////////////////////////////////

TrajectoryConfig tc;

void setup()
{
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  #ifdef USE_DC_MOTORS
  robotBase = RobotBaseDC::getInstance();
  #else 
  #ifdef USE_STEPPER_MOTORS
  robotBase = RobotBaseStepper::getInstance();
  #endif
  #endif

  // Init preferences
  preferences.begin("miam-pami", false); 
  // Load existing preferences
  robotID = preferences.getInt("id", -1);
  if (robotID == -1)
  {
    robotID = PAMI_ID;
  }

  // Init motionController
  motionController = new MotionController(&xMutex_Serial, robotBase->getParameters());
  motionController->init(RobotPosition(0.0, 0.0, 0.0));

  length_of_saved_traj_float = preferences.getInt("traj_len_float", -1);
  duration_of_saved_traj = preferences.getFloat("traj_duration", -1);
  // if (length_of_saved_traj_float > 0)
  // {
  //   Serial.print("Reading saved traj of length ");
  //   Serial.print(length_of_saved_traj_float);
  //   Serial.print(", duration ");
  //   Serial.println(duration_of_saved_traj);
  //   saved_trajectory = new float[length_of_saved_traj_float]();
  //   preferences.getBytes("traj_coord", saved_trajectory, length_of_saved_traj_float*4);
  //   Serial.print("First float ");
  //   Serial.println(saved_trajectory[0]);
  //   Serial.print("Last float ");
  //   Serial.println(saved_trajectory[length_of_saved_traj_float-1]);

  //   std::vector<TrajectoryPoint > tp_vec;
  //   TrajectoryPoint tp;
  //   for (int i = 0; i < length_of_saved_traj_float / 5; i++)
  //   {
  //     tp.position.x = saved_trajectory[5*i];
  //     tp.position.y = saved_trajectory[5*i+1];
  //     tp.position.theta = saved_trajectory[5*i+2];
  //     tp.linearVelocity = saved_trajectory[5*i+3];
  //     tp.angularVelocity = saved_trajectory[5*i+4];
  //     tp_vec.push_back(tp);
  //   }

  //   saved_trajectory_vector.clear();
  //   std::shared_ptr<Trajectory > traj(new SampledTrajectory(tc, tp_vec, duration_of_saved_traj));
  //   saved_trajectory_vector.push_back(traj);
  // }
  // else
  // {
    Serial.println("Load default trajectory");
    saved_trajectory_vector = strategy::get_default_trajectory(motionController);
  // }

  xMutex_Serial = xSemaphoreCreateMutex();  // crete a mutex object

  Serial.println("Create robot base");
  robotBase->setup();

  Serial.begin(115200);
  Serial.println("Attempt connect WiFi");

  // blue LED constant ON until wifi is enabled
  ledcAttachPin(LED_BUILTIN, 0);
  ledcSetup(0, 1000, 8);
  ledcWrite(0, 32);
  
  // connect wifi
  initWiFi();

  // start heartbeat
  Serial.println("Launch heartbeat");
  xTaskCreatePinnedToCore(
      task_blink_led, 
      "task_blink_led",
      1000,
      NULL,
      1,
      NULL,
      0 // pin to core 0
  ); 

  // Init i2c peripherals
  I2CHandler::init();

  xTaskCreatePinnedToCore(
    task_update_vl53l0x, 
    "task_update_vl53l0x",
    2000,
    NULL,
    10,  // mid priority
    NULL, 
    1 // pin to core 1
  ); 

  xTaskCreatePinnedToCore(
    task_update_ssd1306, 
    "task_update_ssd1306",
    2000,
    NULL,
    10,  // mid priority
    NULL, 
    1 // pin to core 1
  ); 

  // analog readings: monitor battery and infrared captors
  Serial.println("Init analog readings");
  AnalogReadings::init();

  // init the servo
  Servo::init();
  Servo::servoUp();

  // begin low level loop
  Serial.println("Launch low level loop");
  xTaskCreatePinnedToCore(
      performLowLevel, 
      "performLowLevel",
      10000,
      NULL,
      19, // high priority
      NULL, 
      0 // pin to core 0
  ); 

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

// #ifdef ENABLE_OTA_UPDATE
//   Serial.println("Launch OTA");
//   xTaskCreatePinnedToCore(
//       handleOTATask, 
//       "handleOTATask",
//       10000,
//       NULL,
//       1,
//       NULL,
//       1 // pin to core 0
//   ); 
// #endif

  tc.maxWheelVelocity = motionController->getParameters().maxWheelSpeed * 0.9;
  tc.maxWheelAcceleration = motionController->getParameters().maxWheelAcceleration * 0.9;
  tc.robotWheelSpacing = motionController->getParameters().wheelSpacing;

  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

/////////////////////////////////////////////////////////////////////
// Receiver loop
/////////////////////////////////////////////////////////////////////

#include <MessageReceiver.hpp>
MessageReceiver messageReceiver;
MessageReceiverUDP messageReceiverUDP;


void task_messageReceiver(void* parameters)
{
  for (;;)
  {
    Serial.println("Standby...");
    MessageType mt = messageReceiver.receive();

    if (mt == MessageType::NEW_TRAJECTORY)
    {
      Serial.println("Received trajectory, following...");
      motionController->resetPosition(messageReceiver.targetTrajectory.getCurrentPoint(0).position, true, true, true);
      movement_override = true;
      motionController->setTrajectoryToFollow(messageReceiver.targetTrajectory);
      motionController->waitForTrajectoryFinished();
      movement_override = false;
    }
    else if (mt == MessageType::SET_ID)
    {
      Serial.print("Received new id: ");
      Serial.println(messageReceiver.newID);
      robotID = messageReceiver.newID;
      preferences.putInt("id", robotID);
    }
    else if (mt == MessageType::NEW_TRAJECTORY_SAVE)
    {
      Serial.println("Received trajectory, saving...");

      length_of_saved_traj_float = (int)messageReceiver.receivedTrajectory.at(1) * 5;
      duration_of_saved_traj = (float)messageReceiver.receivedTrajectory.at(2);

      if ((messageReceiver.receivedTrajectory.size() - 3) == length_of_saved_traj_float)
      {
        preferences.putInt("traj_len_float", length_of_saved_traj_float);
        preferences.putFloat("traj_duration", duration_of_saved_traj);

        // Ignore first 3 bytes
        float* tmp = new float[length_of_saved_traj_float]();
        for (int i = 0; i < length_of_saved_traj_float; i++)
        {
          tmp[i] = messageReceiver.receivedTrajectory.at(i+3);
        }
        preferences.putBytes("traj_coord", tmp, length_of_saved_traj_float * 4);
        Serial.print("Saved traj of length ");
        Serial.println(length_of_saved_traj_float);

        delete tmp;
      }
      else
      {
        Serial.println("Not saving: decrepency in sizes");
        Serial.print("Expected ");
        Serial.print(length_of_saved_traj_float);
        Serial.print(" received ");
        Serial.println(messageReceiver.receivedTrajectory.size() - 3);
      }
    }
    else if (mt == MessageType::MATCH_STATE)
    {
      Serial.println("Received match state");
      if (messageReceiver.matchStarted)
      {
        Serial.print("Match started, current time ");
        Serial.println(messageReceiver.matchCurrentTime);
        match_started = true;
        trajectory_was_read = false;
        match_current_time_s = messageReceiver.matchCurrentTime;
      }
      else
      {
        Serial.println("Match not started");
        match_started = false;
        match_current_time_s = 0.0f;
        trajectory_was_read = false;
        motionController->clearTrajectories();
      }
    }
    else
    {
      Serial.println("Received error");
    }
  }
}

void task_messageReceiverUDP(void* parameters)
{
  for (;;)
  {
    Serial.println("Standby...");
    MessageType mt = messageReceiverUDP.receive();

    if (mt == MessageType::MATCH_STATE)
    {
      Serial.println("Received match state");
      if (messageReceiverUDP.matchStarted)
      {
        Serial.print("Match started, current time ");
        Serial.println(messageReceiverUDP.matchCurrentTime);
        match_started = true;
        trajectory_was_read = false;
        match_current_time_s = messageReceiverUDP.matchCurrentTime;
      }
      else
      {
        Serial.println("Match not started");
        match_started = false;
        match_current_time_s = 0.0f;
        trajectory_was_read = false;
        motionController->clearTrajectories();
      }
    }
    else
    {
      Serial.println("Received error");
    }
  }
}


void loop()
{
#ifdef DEBUG_MODE_MATCH
  match_started = true;
  match_current_time_s = 85.0f;  
#endif
#ifdef DEBUG_MODE_SERVO
  taskYIELD();
  for(;;)
  {
    // for(int posDegrees = 0; posDegrees <= 180; posDegrees++) {
    //   Servo::servoWrite(posDegrees);
    //   // Serial.println(posDegrees);
    //   delay(20);
    // }
    Serial.println("Up");
    Servo::servoUp();
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    Serial.println("Down");
    Servo::servoDown();
    vTaskDelay(3000 / portTICK_PERIOD_MS);
  }
#endif 
#ifdef DEBUG_MODE_SIMPLE_TRAJECTORY
  taskYIELD();
  for (;;)
  {
    Serial.println("Moving...");
    movement_override = true;
    strategy::make_a_square(motionController);
    // strategy::go_to_zone_3(motionController);

    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
#endif

  messageReceiver.begin();
  messageReceiverUDP.begin();

  xTaskCreate(
    task_messageReceiver,
    "task_messageReceiver",
    20000,
    NULL,
    40,
    NULL
  );

  xTaskCreate(
    task_messageReceiverUDP,
    "task_messageReceiverUDP",
    20000,
    NULL,
    40,
    NULL
  );

  for(;;)
  {
    taskYIELD();
    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
  
}
