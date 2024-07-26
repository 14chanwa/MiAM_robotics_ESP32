#include <TFTScreen.hpp>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
#include <vector>
#include <Types.h>
#include <Match.hpp>
#include <PAMIStates.hpp>
#include <XPT2046_Touchscreen.h>

#include <PAMIDrawable.hpp>
#include <ButtonDrawable.hpp>

#define TFT_CS 17
#define TFT_RST 5
#define TFT_DC 33

#define TFT_HEIGHT 320
#define TFT_WIDTH 240

#define TOUCHSCREEN_CS 32

// #define DEBUG_TFT_SCREEN

#define PAMI_RECT_XSIZE 100
#define PAMI_RECT_YSIZE 100

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
XPT2046_Touchscreen ts(TOUCHSCREEN_CS);

std::vector<std::shared_ptr<PAMIDrawable > > pami_drawables;
std::shared_ptr<ButtonDrawable > button_change_color;

void TFTScreen::init()
{
    // Create PAMIs
    pami_drawables.clear();
    for (uint i=1; i<=5; i++)
    {
        uint8_t gridX;
        uint8_t gridY;
        switch (i)
        {
            case 1:
                gridX = 0;
                gridY = 0;
                break;
            case 2:
                gridX = 1;
                gridY = 0;
                break;
            case 3:
                gridX = 2;
                gridY = 0;
                break;
            case 4:
                gridX = 0;
                gridY = 1;
                break;
            case 5:
                gridX = 1;
                gridY = 1;
                break;
        };

        Vector2 top_left_corner(5 + gridX * (PAMI_RECT_XSIZE + 5), 5 + gridY * (PAMI_RECT_YSIZE + 5));
        Vector2 dimensions(PAMI_RECT_XSIZE, PAMI_RECT_YSIZE);

        std::shared_ptr<PAMIDrawable > new_drawable(new PAMIDrawable(i, top_left_corner, dimensions));
        pami_drawables.push_back(new_drawable);

        top_left_corner = Vector2(215, 150);
        dimensions = Vector2(PAMI_RECT_XSIZE, 40);
        button_change_color = std::make_shared<ButtonDrawable >(
            top_left_corner,
            dimensions
        );
    }

    tft.init(TFT_WIDTH, TFT_HEIGHT); 
    tft.setRotation(3); 
    tft.invertDisplay(false);

    ts.begin();
    ts.setRotation(3); 

    tft.setTextWrap(true);
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(1);

    tft.setCursor(10, 230);
    tft.println("Sketch has been running for");
}

void TFTScreen::update(IPAddress localIP)
{

    for (uint8_t i=1; i<pami_drawables.size()+1; i++)
    {
        drawPAMI(PAMIStates::readPAMIMessage(i), i);
    }

    // Update match time
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.setCursor(230, 120);
    if (Match::getMatchStarted())
    {
        tft.print("      ");
        tft.setCursor(230, 120);
        tft.print(Match::getMatchTimeSeconds());
    }
    else
    {
        tft.print("Wait..");
    }

    // Update current side
    if (Match::getSide() == PlayingSide::BLUE_SIDE)
    {
        button_change_color->update(
            "BLUE",
            ST77XX_BLACK,
            ST77XX_BLACK,
            2,
            ST77XX_BLUE
        );
    }
    else
    {
        button_change_color->update(
            "YELLOW",
            ST77XX_BLACK,
            ST77XX_BLACK,
            2,
            ST77XX_YELLOW
        );
    }

    button_change_color->draw(tft);

    // tft.setCursor(230, 150);
    // if (Match::getSide() == PlayingSide::BLUE_SIDE)
    // {
    //     tft.setTextSize(2);
    //     tft.setTextColor(ST77XX_BLUE, ST77XX_BLACK);
    //     tft.print("BLUE  ");
    // }
    // else
    // {
    //     tft.setTextSize(2);
    //     tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    //     tft.print("YELLOW");
    // }

    // Update seconds count
    tft.setTextSize(1);
    tft.setCursor(180, 230);
    tft.setTextColor(ST77XX_MAGENTA, ST77XX_BLACK);
    tft.print(millis() / 1000);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.print(" seconds.");

    // Update local IP
    if (localIP != lastIP)
    {
        tft.setCursor(10, 220);
        tft.print("               ");
        tft.setCursor(10, 220);
        tft.print(localIP);
        lastIP = localIP;
    }

#ifdef DEBUG_TFT_SCREEN
    tft.setCursor(10, 210);
    tft.print("Last received message at time: ");
    tft.print(PAMIStates::readLastMessageTime());
#endif

    // // PAMI motor lock
    // tft.setTextSize(1);
    // tft.setCursor(230, 180);
    // if (Match::getStopMotors())
    // {
    //     tft.print("Motors LOCKED");
    // }
    // else
    // {
    //     tft.print("             ");
    // }

}

#define TOUCH_MIN_X 3900.0f
#define TOUCH_MIN_Y 3700.0f
#define TOUCH_MAX_X 330.0f
#define TOUCH_MAX_Y 220.0f

Vector2 touch_to_screen(TS_Point p)
{
    Vector2 v;
    v[0] = (p.x - TOUCH_MIN_X) / (TOUCH_MAX_X - TOUCH_MIN_X);
    v[1] = (p.y - TOUCH_MIN_Y) / (TOUCH_MAX_Y - TOUCH_MIN_Y);
    v[0] = std::max(std::min(v[0], 1.0f), 0.0f);
    v[1] = std::max(std::min(v[1], 1.0f), 0.0f);
    v[0] = v[0] * TFT_HEIGHT;
    v[1] = v[1] * TFT_WIDTH;
    return v;
}

void TFTScreen::registerTouch()
{
    if (ts.touched()) {
        TS_Point p = ts.getPoint();
        // Serial.print("Pressure = ");
        // Serial.print(p.z);
        // Serial.print(", x = ");
        // Serial.print(p.x);
        // Serial.print(", y = ");
        // Serial.print(p.y);
        Vector2 v = touch_to_screen(p);
        // Serial.print(v[0]);
        // Serial.print(", ");
        // Serial.println(v[1]);
        // Serial.println();
        for (auto p : pami_drawables)
        {
            if (p->clicked(v))
            {
                Serial.print("PAMI Clicked: ");
                Serial.print(p->pami_id_);
                Serial.println();
            }
        }
        if (button_change_color->clicked(v))
        {
            Serial.println("Button clicked");
            if (Match::getSide() == PlayingSide::YELLOW_SIDE)
            {
                Match::setSide(PlayingSide::BLUE_SIDE);
            }
            else
            {
                Match::setSide(PlayingSide::YELLOW_SIDE);
            }
        }
        delay(30);
    }
}


void TFTScreen::drawPAMI(PamiReportMessage pamiReport, uint8_t pamiID)
{
   pami_drawables.at(pamiID-1)->update(pamiReport);
   pami_drawables.at(pamiID-1)->draw(tft);
}
