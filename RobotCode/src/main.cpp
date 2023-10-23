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

#define ENABLE_OTA_UPDATE
#define SEND_TELEPLOT_UDP

#ifdef SEND_TELEPLOT_UDP
  #include <WiFiUdp.h>
  WiFiUDP udp;
  char buffer[30];

  void sendTelemetry(const char* variableName, const float value)
  {
    udp.beginPacket("192.168.0.255",47269);
    size_t size = snprintf(buffer, 30, "%s:%f", variableName, value);
    udp.write((uint8_t*)&buffer, size);
    udp.endPacket();
  }
#endif

#ifdef ENABLE_OTA_UPDATE
  #include <ESPAsyncWebServer.h>
  #include <AsyncElegantOTA.h>

  AsyncWebServer server(80);
  const char* PARAM_MESSAGE = "message";

  void notFound(AsyncWebServerRequest *request) {
      request->send(404, "text/plain", "Not found");
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

// RobotWheels
AbstractRobotBase* robotBase;

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

void printToSerial(void* parameters)
{
  for(;;)
  {

    #ifdef SEND_TELEPLOT_UDP

      sendTelemetry("targetPoint.linear", motionController->targetPoint.linearVelocity);
      sendTelemetry("targetPoint.angular", motionController->targetPoint.angularVelocity);
      sendTelemetry("currentPosition.x", motionController->currentPosition_.x);
      sendTelemetry("currentPosition.y", motionController->currentPosition_.y);
      sendTelemetry("currentPosition.theta", motionController->currentPosition_.theta);
      sendTelemetry("targetPosition.x", motionController->targetPoint.position.x);
      sendTelemetry("targetPosition.y", motionController->targetPoint.position.y);
      sendTelemetry("targetPosition.theta", motionController->targetPoint.position.theta);
      sendTelemetry("dt_period_ms", dt_period_ms);
      sendTelemetry("dt_lowLevel_ms", dt_lowLevel_ms);
      sendTelemetry("battery_reading", get_current_battery_reading());
      sendTelemetry("rightWheelCurrentSpeed", robotBase->getRightWheel()->currentSpeed_);
      sendTelemetry("rightWheelTargetSpeed", robotBase->getRightWheel()->targetSpeed_);
      sendTelemetry("leftWheelCurrentSpeed", robotBase->getLeftWheel()->currentSpeed_);
      sendTelemetry("leftWheelTargetSpeed", robotBase->getLeftWheel()->targetSpeed_);
      // sendTelemetry("leftBasePWMTarget", leftRobotWheel->basePWMTarget_);
      // sendTelemetry("leftNewPWMTarget", leftRobotWheel->newPWMTarget_);
      // sendTelemetry("rightBasePWMTarget", rightRobotWheel->basePWMTarget_);
      // sendTelemetry("rightNewPWMTarget", rightRobotWheel->newPWMTarget_);

    #else

    if (xSemaphoreTake(xMutex_Serial, portMAX_DELAY))
    {
      print_battery();
      leftRobotWheel->printToSerial();
      rightRobotWheel->printToSerial();
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
    
    // update sensors
    monitor_battery();
    robotBase->updateSensors();
    measurements = robotBase->getMeasurements();

    // If playing side::RIGHT side: invert side::RIGHT/side::LEFT encoders.
    if (motionController->isPlayingRightSide_)
    {
        float temp = measurements.motorSpeed[side::RIGHT];
        measurements.motorSpeed[side::RIGHT] = measurements.motorSpeed[side::LEFT];
        measurements.motorSpeed[side::LEFT] = temp;
    }

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
    robotBase->setBaseSpeed(target);

    // update motor control
    robotBase->updateControl();

    timeEndLoop = micros();
    dt_lowLevel_ms = (timeEndLoop - timeStartLoop) / 1000.0;

    // wait till next tick (integer division rounds toward 0)
    vTaskDelay( std::max(1L, (timeStartLoop + LOW_LEVEL_LOOP_TIME_MS * 1000 - timeEndLoop) / 1000) / portTICK_PERIOD_MS);
  }
}

TrajectoryConfig tc;

void setup()
{
  // initially motors should be stopped
  digitalWrite(EN_A, LOW);
  digitalWrite(EN_B, LOW);

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
  xTaskCreatePinnedToCore(
      task_blink_led, 
      "task_blink_led",
      1000,
      NULL,
      1,
      NULL,
      1 // pin to core 1
  ); 

  robotBase = RobotBaseDC::getInstance();
  
  xMutex_Serial = xSemaphoreCreateMutex();  // crete a mutex object

  motionController = new MotionController(&xMutex_Serial, robotBase->getParameters());
  motionController->init(RobotPosition(0.0, 0.0, 0.0));

  // monitor battery
  pinMode(BAT_READING, INPUT_PULLDOWN);
  analogReadResolution(12);
  analogSetAttenuation(ADC_6db);

  // interupts have to be handled outside low level loop
  // since they are declared in setup, they are attached to core1
  robotBase->setup();

  xTaskCreatePinnedToCore(
      performLowLevel, 
      "performLowLevel",
      10000,
      NULL,
      2,  // high priority
      NULL, 
      1 // pin to core 1
  ); 

  xTaskCreatePinnedToCore(
      printToSerial, 
      "printToSerial",
      10000,
      NULL,
      1,
      NULL,
      0 // pin to core 0
  ); 

  tc.maxWheelVelocity = MAX_WHEEL_SPEED_MM_S * 0.9;
  tc.maxWheelAcceleration = MAX_WHEEL_ACCELERATION_MM_S * 0.9;
  tc.robotWheelSpacing = WHEEL_SPACING_MM;

  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

TrajectoryVector traj;

void loop()
{

  RobotPosition currentPosition(0.0, 0.0, 0.0);
  RobotPosition targetPosition(currentPosition);
  targetPosition.x += 500;

  traj.clear();
  std::shared_ptr<Trajectory> sl(new StraightLine(tc, currentPosition, targetPosition, 0.0, 0.0, false));
  traj.push_back(sl);
  std::shared_ptr<Trajectory> pt(new PointTurn(tc, sl->getEndPoint().position, -M_PI));
  traj.push_back(pt);
  std::shared_ptr<Trajectory> sl2(new StraightLine(tc, pt->getEndPoint().position, currentPosition, 0.0, 0.0, false));
  traj.push_back(sl2);
  std::shared_ptr<Trajectory> pt2(new PointTurn(tc, sl2->getEndPoint().position, M_PI_2));
  traj.push_back(pt2);
  std::shared_ptr<Trajectory> pt3(new PointTurn(tc, pt2->getEndPoint().position, 0));
  traj.push_back(pt3);

  motionController->setTrajectoryToFollow(traj);
  motionController->waitForTrajectoryFinished();

  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
