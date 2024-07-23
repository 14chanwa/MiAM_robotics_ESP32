#include <TFTScreen.hpp>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
#include <vector>
#include <Types.h>
#include <Match.hpp>
#include <PAMIStates.hpp>
#include <XPT2046_Touchscreen.h>

#define TFT_CS 17
#define TFT_RST 5
#define TFT_DC 33

#define TFT_HEIGHT 320
#define TFT_WIDTH 240

#define TOUCHSCREEN_CS 32

#define DEBUG_TFT_SCREEN

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
XPT2046_Touchscreen ts(TOUCHSCREEN_CS);

void TFTScreen::init()
{
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

    for (uint8_t i=1; i<=5; i++)
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
    tft.setCursor(230, 150);
    if (Match::getSide() == PlayingSide::BLUE_SIDE)
    {
        tft.setTextSize(2);
        tft.setTextColor(ST77XX_BLUE, ST77XX_BLACK);
        tft.print("BLUE  ");
    }
    else
    {
        tft.setTextSize(2);
        tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
        tft.print("YELLOW");
    }

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

    // PAMI motor lock
    tft.setTextSize(1);
    tft.setCursor(230, 180);
    if (Match::getStopMotors())
    {
        tft.print("Motors LOCKED");
    }
    else
    {
        tft.print("             ");
    }

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
        Serial.print(v[0]);
        Serial.print(", ");
        Serial.println(v[1]);
        delay(30);
        Serial.println();
    }
}

#define PAMI_RECT_XSIZE 100
#define PAMI_RECT_YSIZE 100

void TFTScreen::drawPAMI(PamiReportMessage pamiReport, uint8_t pamiID)
{
    // pamiID is the desired PAMI, id is the id read in the message

    // Grid
    // 1 2 3
    // 4 5

    uint8_t gridX;
    uint8_t gridY;
    switch (pamiID)
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
        default:
            return;
    };
    
    // Drawing color is black if the message is invalid
    uint16_t drawingColor;
    uint8_t id = pamiReport.get_sender_id()-10;
    if (id >= 1 && id <= 5)
    {
        drawingColor = (pamiReport.playingSide_ == PlayingSide::BLUE_SIDE ? ST77XX_BLUE : ST77XX_YELLOW);
    }
    else
    {
        drawingColor = ST77XX_BLACK;
    }
    
    tft.fillRect(
        gridX * (PAMI_RECT_XSIZE + 10), 
        gridY * (PAMI_RECT_YSIZE + 10), 
        PAMI_RECT_XSIZE, 
        PAMI_RECT_YSIZE,
        // (pamiReport.playingSide_ == PlayingSide::BLUE_SIDE ? (uint16_t) strtol(0x7ED1E6, NULL, 16) : (uint16_t) strtol(OxFFF27A, NULL, 16)));
        drawingColor);

    if (id >= 1 && id <= 5)
    {
        // battery reading
        tft.setTextSize(1);
        tft.setTextColor(ST77XX_BLACK, drawingColor);
        tft.setCursor( gridX * (PAMI_RECT_XSIZE + 10) + 10, 
            gridY * (PAMI_RECT_YSIZE + 10) + 10);
        tft.print(pamiReport.batteryReading_);
    }
}
