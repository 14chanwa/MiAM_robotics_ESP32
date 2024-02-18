#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <tasks.hpp>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)

#define OLED_ADDRESS 0x3C

Adafruit_SSD1306* display;

void initOLEDScreen(TwoWire* wire)
{
 
    display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, wire, OLED_RESET);

    if(!display->begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS, true, false)) { 
        Serial.println(F("SSD1306 allocation failed"));
    }

    display->setRotation(2);

    // Show initial display buffer contents on the screen --
    // the library initializes this with an Adafruit splash screen.
    display->clearDisplay();
    display->display();
}

void printOLEDMessage(String message, int16_t cursor_offset_x, int16_t cursor_offset_y, uint8_t size)
{
    display->setTextSize(size);             // Normal 1:1 pixel scale
    display->setTextColor(WHITE, BLACK);        // Draw white text
    display->setCursor(cursor_offset_x, cursor_offset_y);             // Start at top-left corner
    display->println(message);
    display->display();
}

bool flipMessage = false;


void update_ssd1306(DisplayInformations* display_informations)
{
    // display->clearDisplay();
    flipMessage = !flipMessage;

    printOLEDMessage(display_informations->ip_address, 0, 0, 1);
    if (flipMessage)
    {
        printOLEDMessage("/", 0, 10, 1);
    }
    else
    {
        printOLEDMessage("\\", 0, 10, 1);
    }
    printOLEDMessage(std::to_string(display_informations->id).c_str(), 0, 20, 2);
    
}
