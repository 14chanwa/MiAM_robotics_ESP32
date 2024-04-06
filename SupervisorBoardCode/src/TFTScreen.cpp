#include <TFTScreen.hpp>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
#include <vector>
#include <Match.hpp>

#define TFT_CS 17
#define TFT_RST 5
#define TFT_DC 33
#define TFT_LED 32

#define TFT_HEIGHT 320
#define TFT_WIDTH 240

#define PAMI_TIMEOUT 2000

// #define DEBUG_TFT_SCREEN

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

    for (uint8_t i=1; i<=5; i++)
    {
        drawPAMI(TFTScreen::readPAMIMessage(i), i);
    }

    // Update match time
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.setCursor(250, 120);
    tft.print("      ");
    tft.setCursor(250, 120);
    if (Match::getMatchStarted())
        tft.print(Match::getMatchTimeSeconds());
    else
        tft.print("Wait..");

    // Update seconds count
    tft.setCursor(180, 230);
    tft.setTextColor(ST77XX_MAGENTA, ST77XX_BLACK);
    tft.print(millis() / 1000);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.print(" seconds.");

    // Update local IP
    tft.setCursor(10, 220);
    tft.print("               ");
    tft.setCursor(10, 220);
    tft.print(localIP);

#ifdef DEBUG_TFT_SCREEN
    tft.setCursor(10, 210);
    tft.print("Last received message at time: ");
    tft.print(TFTScreen::readLastMessageTime());
#endif
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
}

long lastMillisRegisterMessage[5] = {0, 0, 0, 0, 0};
PamiReportMessage default_message = PamiReportMessage(false, 0, PlayingSide::BLUE_SIDE, 255);
std::vector<PamiReportMessage > pamiReportMessage({default_message, default_message, default_message, default_message, default_message});

void TFTScreen::registerMessage(std::shared_ptr<Message > message)
{
    uint8_t senderID = message->get_sender_id();
#ifdef DEBUG_TFT_SCREEN
    // lastMillisRegisterMessage = millis();
    Serial.print("Received message from: ");
    Serial.println(message->get_sender_id());
    Serial.print("Message type is ");
    Serial.print(message->get_message_type());
    Serial.print(" expected ");
    Serial.println(MessageType::PAMI_REPORT);
#endif
    if (message->get_message_type() == MessageType::PAMI_REPORT && senderID-10 >= 1 && senderID-10 <= 5)
    {
#ifdef DEBUG_TFT_SCREEN
        Serial.println("Registering message");
#endif
        PamiReportMessage newMessage = *static_cast<PamiReportMessage* >(message.get());
        pamiReportMessage[senderID-10-1] = newMessage;
        lastMillisRegisterMessage[senderID-10-1] = millis();
    }
}

PamiReportMessage TFTScreen::readPAMIMessage(uint8_t pamiID)
{
    if (pamiID >= 1 && pamiID <= 5)
    {
        if (millis() - lastMillisRegisterMessage[pamiID-1] <= PAMI_TIMEOUT)
        {
            return pamiReportMessage[pamiID-1];
        }
    }
    return default_message;
}

long TFTScreen::readLastMessageTime()
{
    long maxValue = lastMillisRegisterMessage[0];
    for (char i = 1; i<5; i++)
        maxValue = std::max(maxValue, lastMillisRegisterMessage[i]);
    return maxValue;
}