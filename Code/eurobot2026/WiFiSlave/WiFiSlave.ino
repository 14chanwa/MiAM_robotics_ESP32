#include <WiFi.h>
#include "secret.hpp"
#include <FreeRTOS.h>
#include <PacketSerial.h>
#include <PubSubClient.h>

#define MAX_SEND_SIZE 512

char ssid[] = WIFI_SSID;      // your network SSID (name)
char pass[] = WIFI_PASS;      // your network password
int status = WL_IDLE_STATUS;  // Indicator of Wifi status

WiFiClient wifiClient;

char mqttServer[] = MQTT_SERVER_IP;
char clientId[] = "amebaClient";
char publishTopic[] = "pami-state";
char subscribeTopic[] = "match-state";

// Access to data wifi tx is protected using a mutex
// It is accessed in taskSerialMonitor and taskSendDataToServer
SemaphoreHandle_t xSemaphore = NULL;
bool need_data_sent = false;  // whether wifi needs to send data
uint data_wifi_tx_size = 0;
char data_wifi_tx[MAX_SEND_SIZE];

PacketSerial myPacketSerial;
PubSubClient client(wifiClient);

void reconnect() {
  // Loop until we're reconnected
  while (!(client.connected())) {
    Serial.print("\r\nAttempting MQTT connection...");
    // Attempt to connect
    byte bssid[6];
    WiFi.BSSID(bssid);
    char buffer[22];
    int n;
    n = sprintf(buffer, "bw16-%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx", bssid[5], bssid[4], bssid[3], bssid[2], bssid[1], bssid[0]);
    printf("[%s] is a string %d chars long\n", buffer, n);
    if (client.connect(buffer)) {
      Serial.println("connected");
      client.subscribe(subscribeTopic);
    } else {
      Serial.println("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      //Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

#define LED_PIN PA30

void task_blink_led(void* parameters) {
  pinMode(LED_PIN, OUTPUT);

  for (;;) {
    digitalWrite(LED_PIN, HIGH);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    digitalWrite(LED_PIN, LOW);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}


void setup() {

  xSemaphore = xSemaphoreCreateMutex();

  xTaskCreate(
    task_blink_led,
    "task_blink_led",
    10000,
    NULL,
    1,
    NULL);

  Serial.begin(115200);
  Serial1.begin(115200);
  myPacketSerial.setStream(&Serial1);
  myPacketSerial.setPacketHandler(&onPacketReceived);

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);
  }

  wifiClient.setBlockingMode();
  wifiClient.setTimeout(1);
  client.setServer(mqttServer, 1883);
  client.setCallback(callback);

  delay(1500);

  // xTaskCreate(taskPrintCurrentNet, "taskPrintCurrentNet", 1000, NULL, 70, NULL);
  xTaskCreate(taskSendDataToServer, "taskSendDataToServer", 10000, NULL, 30, NULL);
  vTaskDelay(2 / portTICK_PERIOD_MS);
  xTaskCreate(taskSerialMonitor, "taskSerialMonitor", 10000, NULL, 30, NULL);
}

// On serial packet received, trigger transfer over wifi
void onPacketReceived(const uint8_t* buffer, size_t size) {
  Serial.print(">>> Received serial message: size ");
  Serial.println(size);

  if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
    for (uint i = 0; i < size; i++) {
      data_wifi_tx[i] = buffer[i];
    }
    data_wifi_tx_size = size;
    need_data_sent = true;
    xSemaphoreGive(xSemaphore);
  } else {
    Serial.println("onPacketReceived: Semaphore not taken");
  }
}

// Periodically print net info over serial debug
void taskPrintCurrentNet(void* parameters) {
  for (;;) {
    printCurrentNet();
    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
}

// Periodically connect to SCD, send and receive messages
void taskSendDataToServer(void* parameters) {
  for (;;) {

    if (!client.connected()) {
      reconnect();
    }
    client.loop();

    if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
      if (need_data_sent) {
        Serial.print(">> need_data_sent:true, size: ");
        Serial.println(data_wifi_tx_size);
        client.publish((const char*)publishTopic, (const uint8_t*)data_wifi_tx, (unsigned int)data_wifi_tx_size);
        need_data_sent = false;
      }
      xSemaphoreGive(xSemaphore);
    } else {
      Serial.println("taskSendDataToServer: Semaphore not taken");
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)(payload[i]));
  }
  Serial.println();

  if (length < 0) {
    Serial.println("[ERROR] Receive data failed");
  } else if (length > 0) {
    Serial.print("[CLIENT] Receive data from server: ");
    Serial.println((const char*)payload);
    Serial.print("Size: ");
    Serial.println(length);

    Serial.print("Received message: ");
    for (uint i = 0; i < length; i++) {
      Serial.print((uint8_t)payload[i]);
      Serial.print(" ");
    }
    Serial.println();

    // Transfer data over serial
    myPacketSerial.send((uint8_t*)payload, (size_t)length);
  }
}


// Periodically check serial
void taskSerialMonitor(void* parameters) {
  for (;;) {
    myPacketSerial.update();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void loop() {
  taskYIELD();
}

// Print net info
void printCurrentNet() {
  // print your WiFi IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print the MAC address of the router you're attached to:
  byte bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("BSSID: ");
  Serial.print(bssid[5], HEX);
  Serial.print(":");
  Serial.print(bssid[4], HEX);
  Serial.print(":");
  Serial.print(bssid[3], HEX);
  Serial.print(":");
  Serial.print(bssid[2], HEX);
  Serial.print(":");
  Serial.print(bssid[1], HEX);
  Serial.print(":");
  Serial.println(bssid[0], HEX);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.println(rssi);

  // print the encryption type:
  byte encryption = WiFi.encryptionType();
  Serial.print("Encryption Type:");
  Serial.println(encryption, HEX);
  Serial.println();
}
