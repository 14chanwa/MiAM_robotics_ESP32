/*
  ESP-NOW Demo - Transmit
  esp-now-demo-xmit.ino
  Sends data to Responder
  
  DroneBot Workshop 2022
  https://dronebotworkshop.com
*/

// Include Libraries
#include <esp_now.h>
#include <WiFi.h>
#include "secret.hpp"

#define DEADZONE_SIZE 250

// Variables for test data
int int_value;
float float_value;
bool bool_value = true;

// MAC Address of responder - edit as required
uint8_t broadcastAddress[] = {ESP32_SECRET_MAC_ADDRESS};

// Define a data structure
typedef struct struct_message {
  uint16_t X;
  uint16_t Y;
} struct_message;

// Create a structured object
struct_message myData;

// Peer info
esp_now_peer_info_t peerInfo;

// Callback function called when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

bool sent_stopped = false;

void setup() {
  
  // Set up Serial Monitor
  Serial.begin(115200);
 
  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the send callback
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {

  // Create test data

  myData.X = analogRead(1);
  myData.Y = analogRead(2);

  Serial.print("myData.X ");
  Serial.print(myData.X);
  Serial.print(", myData.Y ");
  Serial.println(myData.Y);

  bool joystick_triggered = max(abs(myData.X-2048), abs(myData.Y-2048)) > DEADZONE_SIZE;

  if (joystick_triggered || !sent_stopped)
  {
    if (!joystick_triggered)
    {
      myData.X = 2048;
      myData.Y = 2048;
    }
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    
    if (result == ESP_OK) {
      Serial.println("Sending confirmed");
    }
    else {
      Serial.println("Sending error");
    }
    if (!sent_stopped)
    {
      sent_stopped = true;
    }
    if (joystick_triggered)
    {
      sent_stopped = false;
    }
  }
  
  delay(10);
}