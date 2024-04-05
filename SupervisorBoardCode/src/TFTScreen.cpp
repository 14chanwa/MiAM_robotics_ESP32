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

void TFTScreen::init()
{
    tft.init(TFT_WIDTH, TFT_HEIGHT); 
    tft.setRotation(3);
    pinMode(TFT_LED, OUTPUT);
    digitalWrite(TFT_LED, HIGH);
    tft.invertDisplay(false);

    tft.setTextWrap(true);
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(1);

    tft.setCursor(10, 230);
    tft.println("Sketch has been running for");
}

void TFTScreen::update(IPAddress localIP)
{
    // Update PAMIs
    PamiReportMessage pami_1(
        false, 
        0.0, 
        PlayingSide::BLUE_SIDE, 
        1);
    PamiReportMessage pami_2(
        false, 
        0.0, 
        PlayingSide::YELLOW_SIDE, 
        2);
    drawPAMI(pami_1);
    drawPAMI(pami_2);

    // Update seconds count
    tft.setCursor(180, 230);
    tft.setTextColor(ST77XX_MAGENTA, ST77XX_BLACK);
    tft.print(millis() / 1000);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.print(" seconds.");
    tft.setCursor(10, 220);

    // Update local IP
    tft.print(localIP);
}

#define PAMI_RECT_XSIZE 100
#define PAMI_RECT_YSIZE 100

void TFTScreen::drawPAMI(PamiReportMessage pamiReport)
{
    // Grid
    // 1 2 3
    // 4 5
    uint8_t id = pamiReport.get_sender_id();
    uint8_t gridX;
    uint8_t gridY;
    switch (id)
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
            gridX = 0;
            gridY = 2;
            break;
        default:
            return;
    };
         
    
    tft.fillRect(
        gridX * (PAMI_RECT_XSIZE + 10), 
        gridY * (PAMI_RECT_YSIZE + 10), 
        PAMI_RECT_XSIZE, 
        PAMI_RECT_YSIZE,
        // (pamiReport.playingSide_ == PlayingSide::BLUE_SIDE ? (uint16_t) strtol(0x7ED1E6, NULL, 16) : (uint16_t) strtol(OxFFF27A, NULL, 16)));
        (pamiReport.playingSide_ == PlayingSide::BLUE_SIDE ? ST77XX_BLUE : ST77XX_YELLOW));
}