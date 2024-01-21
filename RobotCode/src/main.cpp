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

// RobotBase
AbstractRobotBase* robotBase;

#ifdef SEND_TELEPLOT_UDP
  // #include <WiFiUdp.h>
  // WiFiUDP udp;
  // char buffer[30];

  // void sendTelemetry(const char* variableName, const float value)
  // {
  //   udp.beginPacket("192.168.0.255",47269);
  //   size_t size = snprintf(buffer, 30, "%s:%f", variableName, value);
  //   udp.write((uint8_t*)&buffer, size);
  //   udp.endPacket();
  // }
  #include <TeleplotArduino.hpp>
  Teleplot teleplot("192.168.0.255", 47269);

  // void sendTelemetry(const char* variableName, const float value, bool plot = true, unsigned int maxFrequencyHz = 0U)
  // {
  //   if (plot)
  //     teleplot.update(variableName, value, "", maxFrequencyHz);
  //   else
  //     teleplot.update(variableName, value, "", maxFrequencyHz, TELEPLOT_FLAG_NOPLOT);
  // }

  // void sendTelemetry(const char* variableName, const float value, bool plot = true, unsigned int maxFrequencyHz = 0U)
  // {
  //   if (plot)
  //     teleplot.update(variableName, value, "", maxFrequencyHz);
  //   else
  //     teleplot.update(variableName, value, "", maxFrequencyHz, TELEPLOT_FLAG_NOPLOT);
  // }
#endif

#ifdef ENABLE_OTA_UPDATE
// // #include <ArduinoOTA.h>
// // char mdnsName[] = "miam-pami";
// // char otaPassword[] = "";
// // String critERR = "";

// // #include <AsyncTCP.h>
// // #include <ESPAsyncWebServer.h>
// #include <ElegantOTA.h>
// AsyncWebServer server(80);


// unsigned long ota_progress_millis = 0;

// void onOTAStart() {
//   // Log when OTA has started
//   Serial.println("OTA update started!");
//   // <Add your own code here>
// }

// void onOTAProgress(size_t current, size_t final) {
//   // Log every 1 second
//   if (millis() - ota_progress_millis > 1000) {
//     ota_progress_millis = millis();
//     Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
//   }
// }

// void onOTAEnd(bool success) {
//   // Log when OTA has finished
//   if (success) {
//     Serial.println("OTA update finished successfully!");
//   } else {
//     Serial.println("There was an error during OTA update!");
//   }
//   // <Add your own code here>
// }
  #include <ESPAsyncWebServer.h>
  #include <AsyncElegantOTA.h>

  AsyncWebServer server(80);
  const char* PARAM_MESSAGE = "message";

  void notFound(AsyncWebServerRequest *request) {
      request->send(404, "text/plain", "Not found");
  }

// void handleOTATask(void* parameters)
// {
//   for (;;)
//   {
//     ElegantOTA.loop();
//     vTaskDelay(100 / portTICK_PERIOD_MS);
//   }
// }

#endif

void initWiFi() {
#ifdef USE_WIFI
  WiFi.mode(WIFI_STA);
  WiFi.begin(SECRET_SSID, SECRET_PASSWORD);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
  delay(1000);
#endif

#ifdef ENABLE_OTA_UPDATE
  // // byte mac[6];
  // // WiFi.macAddress(mac);
  // // // Start OTA once connected
  // // Serial.println("Setting up OTA");
  // // // Port defaults to 3232
  // // ArduinoOTA.setPort(3232);
  // // // Hostname defaults to esp3232-[MAC]
  // // char value[80];
  // // sprintf(value, "%s-%02x%02x%02x", mdnsName, mac[2], mac[1], mac[0]);
  // // ArduinoOTA.setHostname(value);
  // // Serial.print("Hostname: ");
  // // Serial.println(ArduinoOTA.getHostname());
  // // // Partition label
  // // Serial.print("Partition label: ");
  // // Serial.println(ArduinoOTA.getPartitionLabel());
  // // // No authentication by default
  // // if (strlen(otaPassword) != 0) {
  // //     ArduinoOTA.setPassword(otaPassword);
  // //     Serial.printf("OTA Password: %s\n\r", otaPassword);
  // // } else {
  // //     Serial.printf("\r\nNo OTA password has been set! (insecure)\r\n\r\n");
  // // }
  // // ArduinoOTA
  // //     .onStart([]() {
  // //         String type;
  // //         if (ArduinoOTA.getCommand() == U_FLASH)
  // //             type = "sketch";
  // //         else // U_SPIFFS
  // //             // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
  // //             type = "filesystem";
  // //         Serial.println("Start updating " + type);
  // //         robotBase->forceStop();
  // //         critERR = "<h1>OTA Has been started</h1>";
  // //         critERR += "<p>Wait for OTA to finish and reboot, or <a href=\"control?var=reboot&val=0\" title=\"Reboot Now (may interrupt OTA)\">reboot manually</a> to recover</p>";
  // //     })
  // //     .onEnd([]() {
  // //         Serial.println("\r\nEnd");
  // //     })
  // //     .onProgress([](unsigned int progress, unsigned int total) {
  // //         Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  // //     })
  // //     .onError([](ota_error_t error) {
  // //         Serial.printf("Error[%u]: ", error);
  // //         if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
  // //         else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
  // //         else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
  // //         else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
  // //         else if (error == OTA_END_ERROR) Serial.println("End Failed");
  // //     });
  // // ArduinoOTA.begin();
  //   server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
  //     request->send(200, "text/plain", "Hi! This is ElegantOTA AsyncDemo.");
  //   });

  //   ElegantOTA.begin(&server);    // Start ElegantOTA
  //   // ElegantOTA callbacks
  //   ElegantOTA.onStart(onOTAStart);
  //   ElegantOTA.onProgress(onOTAProgress);
  //   ElegantOTA.onEnd(onOTAEnd);

  //   server.begin();
  //   Serial.println("HTTP server started");

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/plain", "Hello, world");
    });

    // Send a GET request to <IP>/get?message=<message>
    server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
        String message;
        if (request->hasParam(PARAM_MESSAGE)) {
            message = request->getParam(PARAM_MESSAGE)->value();
        } else {
            message = "No message sent";
        }
        request->send(200, "text/plain", "Hello, GET: " + message);
    });

    // Send a POST request to <IP>/post with a form field message set to <message>
    server.on("/post", HTTP_POST, [](AsyncWebServerRequest *request){
        String message;
        if (request->hasParam(PARAM_MESSAGE, true)) {
            message = request->getParam(PARAM_MESSAGE, true)->value();
        } else {
            message = "No message sent";
        }
        request->send(200, "text/plain", "Hello, POST: " + message);
    });

    server.onNotFound(notFound);

    AsyncElegantOTA.begin(&server);
    server.begin();

    delay(1000);
#endif
}

using namespace miam;
using namespace miam::trajectory;

// Motion controller
MotionController* motionController;

// low level loop timing
float dt_lowLevel_ms = 0.0;
float dt_period_ms = 0.0;
long timeStartLoop = 0;
long timeEndLoop = 0;

bool hasMatchStarted_ = true;

DrivetrainMeasurements measurements;
DrivetrainTarget target;

// Serial semaphore
SemaphoreHandle_t xMutex_Serial = NULL;

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
      teleplot.update("dt_period_ms", dt_period_ms, "", 0, TELEPLOT_FLAG_NOPLOT);
      teleplot.update("dt_lowLevel_ms", dt_lowLevel_ms, "", 0, TELEPLOT_FLAG_NOPLOT);
      teleplot.update("battery_reading", get_current_battery_reading(), "", 0, TELEPLOT_FLAG_NOPLOT);
      teleplot.update("tcrt0", get_current_tcrt0_reading(), "", 0);
      teleplot.update("tcrt1", get_current_tcrt1_reading(), "", 0);
      teleplot.update("tcrt2", get_current_tcrt2_reading(), "", 0);
      teleplot.update("touchSensor", get_current_touch_sensor_reading(), "", 0);


      #ifdef USE_DC_MOTORS
      teleplot.update("rightWheelCurrentSpeed", robotBase->getRightWheel()->currentSpeed_);
      teleplot.update("rightWheelTargetSpeed", robotBase->getRightWheel()->targetSpeed_);
      teleplot.update("leftWheelCurrentSpeed", robotBase->getLeftWheel()->currentSpeed_);
      teleplot.update("leftWheelTargetSpeed", robotBase->getLeftWheel()->targetSpeed_);
      #endif


      // sendTelemetry("leftBasePWMTarget", leftRobotWheel->basePWMTarget_);
      // sendTelemetry("leftNewPWMTarget", leftRobotWheel->newPWMTarget_);
      // sendTelemetry("rightBasePWMTarget", rightRobotWheel->basePWMTarget_);
      // sendTelemetry("rightNewPWMTarget", rightRobotWheel->newPWMTarget_);

      #ifdef USE_STEPPER_MOTORS
      // sendTelemetry("encoderValue_left_", (static_cast<RobotBaseStepper* >(robotBase))->encoderValue_left_);
      // sendTelemetry("encoderValue_right_", (static_cast<RobotBaseStepper* >(robotBase))->encoderValue_right_);
      teleplot.update("baseTarget_left_", (static_cast<RobotBaseStepper* >(robotBase))->baseTarget_left_, "", 0, TELEPLOT_FLAG_NOPLOT);
      teleplot.update("targetSpeed_left_", (static_cast<RobotBaseStepper* >(robotBase))->targetSpeed_left_);
      teleplot.update("targetSpeedDriver_left_", (static_cast<RobotBaseStepper* >(robotBase))->targetSpeedDriver_left_, "", 0, TELEPLOT_FLAG_NOPLOT);
      teleplot.update("currentSpeed_left_", (static_cast<RobotBaseStepper* >(robotBase))->currentSpeed_left_);
      // sendTelemetry("newTarget_left_", (static_cast<RobotBaseStepper* >(robotBase))->newTarget_left_);
      teleplot.update("baseTarget_right_", (static_cast<RobotBaseStepper* >(robotBase))->baseTarget_right_, "", 0, TELEPLOT_FLAG_NOPLOT);
      teleplot.update("targetSpeed_right_", (static_cast<RobotBaseStepper* >(robotBase))->targetSpeed_right_);
      teleplot.update("targetSpeedDriver_right_", (static_cast<RobotBaseStepper* >(robotBase))->targetSpeedDriver_right_, "", 0, TELEPLOT_FLAG_NOPLOT);
      teleplot.update("currentSpeed_right_", (static_cast<RobotBaseStepper* >(robotBase))->currentSpeed_right_);
      // sendTelemetry("newTarget_right_", (static_cast<RobotBaseStepper* >(robotBase))->newTarget_right_);
      teleplot.update("leftStepperResult_", (int)(static_cast<RobotBaseStepper* >(robotBase))->leftStepperResult_, "", 0, TELEPLOT_FLAG_NOPLOT);
      teleplot.update("rightStepperResult_", (int)(static_cast<RobotBaseStepper* >(robotBase))->rightStepperResult_, "", 0, TELEPLOT_FLAG_NOPLOT);
      // sendTelemetry("leftStepInterval", (static_cast<RobotBaseStepper* >(robotBase))->getStepIntervalLeft());
      // sendTelemetry("rightStepInterval", (static_cast<RobotBaseStepper* >(robotBase))->getStepIntervalRight());
      teleplot.update("isQueueRunningLeft_", (static_cast<RobotBaseStepper* >(robotBase))->isRunningLeft, "", 0, TELEPLOT_FLAG_NOPLOT);
      teleplot.update("isQueueRunningRight_", (static_cast<RobotBaseStepper* >(robotBase))->isRunningRight, "", 0, TELEPLOT_FLAG_NOPLOT);
      teleplot.update("isQueueEmptyLeft_", (static_cast<RobotBaseStepper* >(robotBase))->isEmptyLeft, "", 0, TELEPLOT_FLAG_NOPLOT);
      teleplot.update("isQueueEmptyRight_", (static_cast<RobotBaseStepper* >(robotBase))->isEmptyRight, "", 0, TELEPLOT_FLAG_NOPLOT);
      teleplot.update("isQueueFullLeft_", (static_cast<RobotBaseStepper* >(robotBase))->isEmptyLeft, "", 0, TELEPLOT_FLAG_NOPLOT);
      teleplot.update("isQueueFullRight_", (static_cast<RobotBaseStepper* >(robotBase))->isEmptyRight, "", 0, TELEPLOT_FLAG_NOPLOT);
      teleplot.update("isGenActiveLeft_", (static_cast<RobotBaseStepper* >(robotBase))->isGenActiveLeft, "", 0, TELEPLOT_FLAG_NOPLOT);
      teleplot.update("isGenActiveRight_", (static_cast<RobotBaseStepper* >(robotBase))->isGenActiveRight, "", 0, TELEPLOT_FLAG_NOPLOT);
      teleplot.update("desyncDetectedLeft_", (static_cast<RobotBaseStepper* >(robotBase))->desyncDetectedLeft, "", 0, TELEPLOT_FLAG_NOPLOT);
      teleplot.update("desyncDetectedRight_", (static_cast<RobotBaseStepper* >(robotBase))->desyncDetectedRight, "", 0, TELEPLOT_FLAG_NOPLOT);
      #endif

      teleplot.update("vlx_ranging_data_mm", get_current_vl53l0x());

    #else
    #ifdef SEND_SERIAL

    if (xSemaphoreTake(xMutex_Serial, portMAX_DELAY))
    {
      print_battery();
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

void performLowLevel(void* parameters)
{
  for(;;)
  {
    timeEndLoop = micros();
    dt_period_ms = (timeEndLoop - timeStartLoop) / 1000.0;
    timeStartLoop = timeEndLoop;
    
    // Serial.println("Update sensors");
    robotBase->updateSensors();
    // Serial.println("Get measurements");
    measurements = robotBase->getMeasurements();

    // If playing side::RIGHT side: invert side::RIGHT/side::LEFT encoders.
    if (motionController->isPlayingRightSide_)
    {
        float temp = measurements.motorSpeed[side::RIGHT];
        measurements.motorSpeed[side::RIGHT] = measurements.motorSpeed[side::LEFT];
        measurements.motorSpeed[side::LEFT] = temp;
    }

    // Serial.println("Compute drivetrain motion");
    target = motionController->computeDrivetrainMotion(
      measurements, 
      dt_period_ms / 1000.0, 
      hasMatchStarted_
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

    // get_current_vl53l0x();
#ifdef USE_MONITOR_BATTERY
    // update sensors
    monitor_battery();
#endif

    // Serial.println("Register time");
    timeEndLoop = micros();
    dt_lowLevel_ms = (timeEndLoop - timeStartLoop) / 1000.0;

    // wait till next tick (integer division rounds toward 0)
    vTaskDelay( std::max(1L, (timeStartLoop + LOW_LEVEL_LOOP_TIME_MS * 1000 - timeEndLoop) / 1000) / portTICK_PERIOD_MS);
  }
}


// I2C Semaphore
SemaphoreHandle_t xMutex_I2C = NULL;

void task_update_vl53l0x(void* parameters)
{
    for (;;)
    {
      if (xSemaphoreTake(xMutex_I2C, portMAX_DELAY))
      {
        update_vl53l0x();
        xSemaphoreGive(xMutex_I2C);  // release the mutex
      }
      vTaskDelay(25 / portTICK_PERIOD_MS);
    }
}


void task_update_ssd1306(void* parameters)
{
    for (;;)
    {
      if (xSemaphoreTake(xMutex_I2C, portMAX_DELAY))
      {
        update_ssd1306();
        xSemaphoreGive(xMutex_I2C);  // release the mutex
      }
      vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}


TrajectoryConfig tc;

void setup()
{
  vTaskDelay(1000 / portTICK_PERIOD_MS);

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

  #ifdef USE_DC_MOTORS
  robotBase = RobotBaseDC::getInstance();
  #else 
  #ifdef USE_STEPPER_MOTORS
  robotBase = RobotBaseStepper::getInstance();
  #endif
  #endif

  xMutex_Serial = xSemaphoreCreateMutex();  // crete a mutex object
  xMutex_I2C = xSemaphoreCreateMutex();  // crete a mutex object

  motionController = new MotionController(&xMutex_Serial, robotBase->getParameters());
  motionController->init(RobotPosition(0.0, 0.0, 0.0));


  Serial.println("Create robot base");
  robotBase->setup();

  Wire.begin(SDA, SCL, 400000);

#ifdef USE_VL53L0X
  Serial.println("Init VL53L0X");
  init_vl53l0x(&Wire);

  xTaskCreatePinnedToCore(
    task_update_vl53l0x, 
    "task_update_vl53l0x",
    2000,
    NULL,
    10,  // mid priority
    NULL, 
    1 // pin to core 1
  ); 
#endif

  Serial.println("Init SSD1306");
  initOLEDScreen(&Wire);

  xTaskCreatePinnedToCore(
    task_update_ssd1306, 
    "task_update_ssd1306",
    2000,
    NULL,
    10,  // mid priority
    NULL, 
    1 // pin to core 1
  ); 

#ifdef USE_MONITOR_BATTERY
  // monitor battery
  Serial.println("Init monitor battery");
  pinMode(BAT_READING, INPUT_PULLDOWN);
  pinMode(TCRT_0, INPUT_PULLDOWN);
  pinMode(TCRT_1, INPUT_PULLDOWN);
  pinMode(TCRT_2, INPUT_PULLDOWN);
  analogReadResolution(12);
  // analogSetAttenuation(ADC_6db);
  analogSetAttenuation(ADC_11db);

  // xTaskCreatePinnedToCore(
  //   task_update_analog_readings, 
  //   "task_update_analog_readings",
  //   2000,
  //   NULL,
  //   10,  // mid priority
  //   NULL, 
  //   1 // pin to core 1
  // ); 
#endif

  Serial.println("Launch low level loop");
  xTaskCreatePinnedToCore(
      performLowLevel, 
      "performLowLevel",
      10000,
      NULL,
      19,  // high priority
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

  // xTaskCreatePinnedToCore(
  //   loop_task_planning, 
  //   "loop_task_planning",
  //   30000,
  //   NULL,
  //   1,
  //   NULL,
  //   0 // pin to core 0
  // ); 
}




/////////////////////////////////////////////////////////////////////
// Strategy
/////////////////////////////////////////////////////////////////////

TrajectoryVector traj;

void go_forward(float distance)
{
  RobotPosition curPos(motionController->getCurrentPosition());
  TrajectoryVector tv(computeTrajectoryStraightLine(tc, curPos, distance));
  motionController->setTrajectoryToFollow(tv);
  motionController->waitForTrajectoryFinished();

}

void turn_around(float angle)
{
  traj.clear();
  RobotPosition curPos(motionController->getCurrentPosition());
  std::shared_ptr<Trajectory> pt(new PointTurn(tc, curPos, curPos.theta + angle));
  traj.push_back(pt);
  motionController->setTrajectoryToFollow(traj);
  motionController->waitForTrajectoryFinished();
}

void go_to_point(RobotPosition targetPoint)
{
  RobotPosition curPos(motionController->getCurrentPosition());
  TrajectoryVector tv(computeTrajectoryStraightLineToPoint(tc, curPos, targetPoint));
  motionController->setTrajectoryToFollow(tv);
  motionController->waitForTrajectoryFinished();
}

void make_a_square()
{
  RobotPosition startPosition(motionController->getCurrentPosition());
  RobotPosition targetPosition(startPosition);

  targetPosition.x += 1000;
  go_to_point(targetPosition);
  turn_around(M_PI_2);
  targetPosition.y += 1000;
  go_to_point(targetPosition);
  turn_around(M_PI_2);
  targetPosition.x -= 1000;
  go_to_point(targetPosition);
  turn_around(M_PI_2);
  targetPosition.y -= 1000;
  go_to_point(targetPosition);
  turn_around(M_PI_2);
}

void go_to_zone_3()
{
  motionController->resetPosition(RobotPosition(1275, 1925, -M_PI_2), true, true, true);
  RobotPosition startPosition(motionController->getCurrentPosition());
  RobotPosition targetPosition(startPosition);
  targetPosition.x = 2815;
  targetPosition.y = 980;

  go_to_point(targetPosition);
}

void loop_task_planning(void* parameters)
{
  
  for (;;)
  {
    // make_a_square();
    go_to_zone_3();

    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }

}



void loop()
{
  // taskYIELD();
  loop_task_planning(NULL);
}
