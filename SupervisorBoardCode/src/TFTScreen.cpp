#include <TFTScreen.hpp>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>

#define TFT_CS 17
#define TFT_RST 5
#define TFT_DC 33
#define TFT_LED 32

#define TFT_HEIGHT 320
#define TFT_WIDTH 240

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
n
void TFTScreen::init()
{
    tft.init(TFT_WIDTH, TFT_HEIGHT); 
    tft.setRotation(3);
    pinMode(TFT_LED, OUTPUT);
    digitalWrite(TFT_LED, HIGH);
    tft.invertDisplay(false);

    tft.setTextWrap(true);
    tft.setCursor(10, 10);
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.println("Sketch has been");
    tft.println("running for: ");
}

void TFTScreen::update()
{
    tft.setCursor(10, 42);
    tft.setTextColor(ST77XX_MAGENTA, ST77XX_BLACK);
    tft.print(millis() / 1000);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.print(" seconds.");
}