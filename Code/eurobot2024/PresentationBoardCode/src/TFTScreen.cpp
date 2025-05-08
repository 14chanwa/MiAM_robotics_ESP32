#include <TFTScreen.hpp>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <vector>

#include <ButtonDrawable.hpp>
#include <XPT2046_Touchscreen.h>

SPIClass mySpi = SPIClass(VSPI);
XPT2046_Touchscreen ts(XPT2046_CS, XPT2046_IRQ);
uint16_t tX = 0, tY = 0;

TFT_eSPI tft = TFT_eSPI();       // Invoke custom library

#define DRAW_MARGIN 10
#define DRAW_DIMENSIONS 100

// cycle colors
std::shared_ptr<ButtonDrawable > button_change_color;
uint color_idx = 0;
uint16_t colors[] = { TFT_WHITE, TFT_RED, TFT_GREEN, TFT_BLUE, TFT_PURPLE };
char* color_names[] = { "WHITE", "RED", "GREEN", "BLUE", "PURPLE" };
const uint number_of_colors = 5;

// brightness control
std::shared_ptr<ButtonDrawable > button_change_brightness;
uint brightness_level = 150;
#define BRIGHTNESS_OFFSET 30;

#include <WiFi.h>
#include <esp_now.h>
// Communication protocol
// Define a data structure
#define COM_NUM_LED 6
typedef struct struct_message {
  uint8_t red[COM_NUM_LED];
  uint8_t green[COM_NUM_LED];
  uint8_t blue[COM_NUM_LED];
  uint8_t brightness;
} struct_message;

// Create a structured object
struct_message myData;

// Peer info
esp_now_peer_info_t peerInfo;

// MAC Address of responder - edit as required
//uint8_t broadcastAddress[] = {0xA8, 0x42, 0xE3, 0x57, 0xBF, 0x28};
uint8_t broadcastAddress[] = {0x24, 0xEC, 0x4A, 0x30, 0x83, 0x8C };

// Callback function called when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void send_message()
{
    for (uint i=0; i<COM_NUM_LED; i++)
    {
        myData.red[i] = (colors[color_idx] & 0xF800) >> 8;
        myData.green[i] = (colors[color_idx] & 0x7E0) >> 3;
        myData.blue[i] = (colors[color_idx] & 0x1F) << 3;
    }
    myData.brightness = (char)brightness_level;

    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    
    if (result == ESP_OK) {
        Serial.println("Sending confirmed");
    }
    else {
        Serial.println("Sending error");
    }
}


void TFTScreen::init()
{
    
    Vector2 top_left_corner = Vector2(DRAW_MARGIN, DRAW_MARGIN);
    Vector2 dimensions = Vector2(DRAW_DIMENSIONS, DRAW_DIMENSIONS);
    button_change_color = std::make_shared<ButtonDrawable >(
        top_left_corner,
        dimensions
    );

    top_left_corner[0] += DRAW_DIMENSIONS + DRAW_MARGIN;
    button_change_brightness = std::make_shared<ButtonDrawable >(
        top_left_corner,
        dimensions
    );

    // Start the SPI for the touch screen and init the TS library
    mySpi.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
    ts.begin(mySpi);

    tft.init(); 
    tft.setRotation(1); 
    tft.invertDisplay(true);

    tft.setTextWrap(true);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1);

    tft.setCursor(10, 230);
    tft.println("Sketch has been running for");

    
    
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

  // Send first message
  send_message();
}

void TFTScreen::update()
{
    // Update match time
    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    button_change_color->update(
        color_names[color_idx],
        TFT_BLACK,
        TFT_BLACK,
        2,
        colors[color_idx]
    );
    button_change_color->draw(tft);

    std::string s = std::to_string(brightness_level);
    button_change_brightness->update(
        s,
        TFT_BLACK,
        TFT_BLACK,
        2,
        TFT_DARKGREY
    );
    button_change_brightness->draw(tft);

    // Update seconds count
    tft.setTextSize(1);
    tft.setCursor(180, 230);
    tft.setTextColor(TFT_MAGENTA, TFT_BLACK);
    tft.print(millis() / 1000);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print(" seconds.");

}

#define TOUCH_MIN_X 293.0f
#define TOUCH_MIN_Y 329.0f
#define TOUCH_MAX_X 3733.0f
#define TOUCH_MAX_Y 3891.0f

Vector2 touch_to_screen(uint16_t x, uint16_t y)
{
    Vector2 v;
    v[0] = (x - TOUCH_MIN_X) / (TOUCH_MAX_X - TOUCH_MIN_X);
    v[1] = (y - TOUCH_MIN_Y) / (TOUCH_MAX_Y - TOUCH_MIN_Y);
    v[0] = std::max(std::min(v[0], 1.0f), 0.0f);
    v[1] = std::max(std::min(v[1], 1.0f), 0.0f);
    v[0] = v[0] * TFT_HEIGHT;
    v[1] = v[1] * TFT_WIDTH;
    return v;
}

void TFTScreen::registerTouch()
{
    uint16_t x, y;
    if (ts.tirqTouched() && ts.touched()) {
        TS_Point p = ts.getPoint();
        tX = p.x;
        tY = p.y;
        Serial.print("tX: ");
        Serial.print(tX);
        Serial.print(", tY: ");
        Serial.print(tY);
        Serial.print(", ");
        Vector2 v = touch_to_screen(p.x, p.y);
        Serial.print(v[0]);
        Serial.print(", ");
        Serial.println(v[1]);
        
        bool need_send_message = false;
        
        if (button_change_color->clicked(v))
        {
            Serial.println("Button clicked");
            color_idx = (color_idx + 1) % number_of_colors;
            need_send_message = true;
        }

        if (button_change_brightness->clicked(v))
        {
            Serial.println("Button clicked");
            brightness_level = brightness_level + BRIGHTNESS_OFFSET;
            if (brightness_level > 255)
            {
                brightness_level = 0;
            }
            need_send_message = true;
        }

        if (need_send_message)
        {
            send_message();
        }

        delay(30);
    }
}
