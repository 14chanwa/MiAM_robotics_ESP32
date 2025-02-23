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

std::shared_ptr<ButtonDrawable > button_change_color;

void TFTScreen::init()
{
    
    Vector2 top_left_corner = Vector2(215, 150);
    Vector2 dimensions = Vector2(100, 40);
    button_change_color = std::make_shared<ButtonDrawable >(
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
}

void TFTScreen::update()
{
    // Update match time
    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    button_change_color->update(
        "BLUE",
        TFT_BLACK,
        TFT_BLACK,
        2,
        TFT_BLUE
    );
    button_change_color->draw(tft);

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
        
        if (button_change_color->clicked(v))
        {
            Serial.println("Button clicked");
        }
        delay(30);
    }
}
