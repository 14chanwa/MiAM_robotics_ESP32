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
    display->display();
}

void printOLEDMessage(String message)
{
    display->clearDisplay();
    display->setTextSize(1);             // Normal 1:1 pixel scale
    display->setTextColor(WHITE);        // Draw white text
    display->setCursor(0,0);             // Start at top-left corner
    display->println(message);
    display->display();
}

bool flipMessage = false;
void update_ssd1306()
{
    display->startscrollleft(0x00, 0x0F);
    if (flipMessage)
        printOLEDMessage("Hello");
    else
        printOLEDMessage("World");
    flipMessage = !flipMessage;
}

// void task_update_ssd1306(void* parameters)
// {
//     // char str[3];
//     // char tmp = 0;
//     for (;;)
//     {
//         // snprintf(str, sizeof(str), "%2d", tmp);
//         // printOLEDMessage(strcat("Message ", str));
//         // tmp++;
//         // if (tmp > 10)
//         //     tmp = 0;

//         display->startscrollleft(0x00, 0x0F);
//         printOLEDMessage("Hello");
//         vTaskDelay(1000 / portTICK_PERIOD_MS);
//         printOLEDMessage("World");
//         vTaskDelay(1000 / portTICK_PERIOD_MS);
//     }
// }