#include <TFTScreen.hpp>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <vector>

#include <ButtonDrawable.hpp>
#include <XPT2046_Touchscreen.h>

int randint(int Min, int Max) {
    return std::rand() % (Max + 1 - Min) + Min;
}


SPIClass mySpi = SPIClass(VSPI);
XPT2046_Touchscreen ts(XPT2046_CS, XPT2046_IRQ);
uint16_t tX = 0, tY = 0;

TFT_eSPI tft = TFT_eSPI();       // Invoke custom library

#define DRAW_MARGIN 10
#define DRAW_DIMENSIONS_WIDTH 150
#define DRAW_DIMENSIONS_HEIGHT 33

// cycle colors
std::shared_ptr<ButtonDrawable > button_change_color;
uint color_idx = 0;
uint16_t colors[] = { TFT_GREEN, TFT_WHITE, TFT_RED, TFT_BLUE, TFT_PURPLE, TFT_ORANGE };
char* color_names[] = { "GREEN", "WHITE", "RED", "BLUE", "PURPLE", "ORANGE" };
const uint number_of_colors = 6;
// uint16_t colors[] = { TFT_GREEN, TFT_WHITE };
// char* color_names[] = { "GREEN", "WHITE" };
// const uint number_of_colors = 2;


// brightness control
// std::shared_ptr<ButtonDrawable > button_change_brightness;
uint brightness_level = 255;
#define BRIGHTNESS_OFFSET 30;

// showcase buttons
char* showcase_names[] = { "Elec", "Batterie", "Moteurs" };
char number_of_showcase = 3;
std::vector<std::shared_ptr<ButtonDrawable > > buttons_showcase;

enum DisplayState
{
    NONE,
    ELEC,
    BATTERIE,
    MOTEURS
};

enum BlinkState
{
    NORMAL,
    CHAOTIC
};

int bpm_modes[] = {30, 90, 120, 180};
char num_bpm_modes = 4;
char current_bpm_index = 0;
char old_bpm_index = 255;

DisplayState current_state = DisplayState::NONE;
BlinkState current_blink_state = BlinkState::NORMAL;
long last_blinked = 0;

std::shared_ptr<ButtonDrawable> button_blink_state;
std::shared_ptr<ButtonDrawable> button_bpm;

std::shared_ptr<ButtonDrawable> fakebutton_text;

#define HIGHLIGHTED_COLOR TFT_WHITE
#define HIDDEN_COLOR TFT_RED

#define NONE_TEXT {"Selectionner un", "composant a gauche"}
#define NONE_TEXT_NLINES 2

#define ELEC_TEXT {"L'electronique du robot", "est concentree sur cet", "etage."}
#define ELEC_TEXT_NLINES 3
#define ELEC_LEDS {0, 1}
#define ELEC_LEDS_SIZE 2

#define BATTERIE_TEXT {"La batterie du robot", "est une batterie ", "d'outils portatifs."}
#define BATTERIE_TEXT_NLINES 3
#define BATTERIE_LEDS {2, 3}
#define BATTERIE_LEDS_SIZE 2

#define MOTEURS_TEXT {"Automathilde utilise", "des moteurs brushless et", "des drivers maison."}
#define MOTEURS_TEXT_NLINES 3
#define MOTEURS_LEDS {4, 5}
#define MOTEURS_LEDS_SIZE 2

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

void setColor(struct_message& message, uint led_idx, uint16_t color)
{
    message.red[led_idx] = (color & 0xF800) >> 8;
    message.green[led_idx] = (color & 0x7E0) >> 3;
    message.blue[led_idx] = (color & 0x1F) << 3;
}

xSemaphoreHandle xMutex;
bool need_send_message = false;

void send_message()
{
    if (current_state == DisplayState::NONE)
    {
        for (uint i=0; i<COM_NUM_LED; i++)
        {
            setColor(myData, i, colors[color_idx]);
        }
    }
    else if (current_state == DisplayState::ELEC)
    {
        for (uint i=0; i<COM_NUM_LED; i++)
        {
            setColor(myData, i, HIDDEN_COLOR);
        }
        uint highlighted_leds[] = ELEC_LEDS;
        for (uint i=0; i < ELEC_LEDS_SIZE; i++)
        {
            setColor(myData, highlighted_leds[i], HIGHLIGHTED_COLOR);
        }
    }
    else if (current_state == DisplayState::BATTERIE)
    {
        for (uint i=0; i<COM_NUM_LED; i++)
        {
            setColor(myData, i, HIDDEN_COLOR);
        }
        uint highlighted_leds[] = BATTERIE_LEDS;
        for (uint i=0; i < BATTERIE_LEDS_SIZE; i++)
        {
            setColor(myData, highlighted_leds[i], HIGHLIGHTED_COLOR);
        }
    }
    else if (current_state == DisplayState::MOTEURS)
    {
        for (uint i=0; i<COM_NUM_LED; i++)
        {
            setColor(myData, i, HIDDEN_COLOR);
        }
        uint highlighted_leds[] = MOTEURS_LEDS;
        for (uint i=0; i < MOTEURS_LEDS_SIZE; i++)
        {
            setColor(myData, highlighted_leds[i], HIGHLIGHTED_COLOR);
        }
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

// void task_send_message(void* parameters)
// {
//     if (xSemaphoreTake(xMutex, portMAX_DELAY))
//     {
//         if (need_send_message)
//         {
//             send_message();
//             need_send_message = false;
//         }
//         xSemaphoreGive(xMutex);
//     }
//     delay(100);
// }


void TFTScreen::init()
{
    xMutex = xSemaphoreCreateMutex();
    
    Vector2 top_left_corner = Vector2(DRAW_MARGIN, DRAW_MARGIN * (3+1) + 3 * DRAW_DIMENSIONS_HEIGHT);
    Vector2 dimensions = Vector2(DRAW_DIMENSIONS_WIDTH, DRAW_DIMENSIONS_HEIGHT);
    button_change_color = std::make_shared<ButtonDrawable >(
        top_left_corner,
        dimensions
    );

    
    for (char i=0; i<number_of_showcase; i++)
    {
        top_left_corner = Vector2(DRAW_MARGIN, DRAW_MARGIN * (1+i) + i * DRAW_DIMENSIONS_HEIGHT);
        dimensions = Vector2(DRAW_DIMENSIONS_WIDTH, DRAW_DIMENSIONS_HEIGHT);
        buttons_showcase.push_back(std::make_shared<ButtonDrawable >(
            top_left_corner,
            dimensions
        ));
    }

    top_left_corner = Vector2(DRAW_MARGIN, DRAW_MARGIN * (1+4) + 4 * DRAW_DIMENSIONS_HEIGHT);
    dimensions = Vector2(DRAW_DIMENSIONS_WIDTH, DRAW_DIMENSIONS_HEIGHT);
    button_blink_state = std::make_shared<ButtonDrawable >(
        top_left_corner,
        dimensions
    );

    top_left_corner = Vector2(DRAW_MARGIN * 2 + DRAW_DIMENSIONS_WIDTH, DRAW_MARGIN * (1+4) + 4 * DRAW_DIMENSIONS_HEIGHT);
    dimensions = Vector2(DRAW_DIMENSIONS_WIDTH, DRAW_DIMENSIONS_HEIGHT);
    button_bpm = std::make_shared<ButtonDrawable >(
        top_left_corner,
        dimensions
    );

    top_left_corner = Vector2(DRAW_MARGIN * 2 + DRAW_DIMENSIONS_WIDTH, DRAW_MARGIN);
    dimensions = Vector2(DRAW_DIMENSIONS_WIDTH, 5 * DRAW_DIMENSIONS_HEIGHT);
    fakebutton_text = std::make_shared<ButtonDrawable >(
        top_left_corner,
        dimensions
    );
    
    // top_left_corner[0] += DRAW_DIMENSIONS + DRAW_MARGIN;
    // button_change_brightness = std::make_shared<ButtonDrawable >(
    //     top_left_corner,
    //     dimensions
    // );

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

    // xTaskCreatePinnedToCore(
    //     task_send_message,
    //     "task_send_message",
    //     100000,
    //     NULL,
    //     30,
    //     NULL,
    //     1
    // );
}

void TFTScreen::update()
{
    // Update match time
    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    if (current_state != NONE)
    {
        button_change_color->update(
            "< Retour",
            TFT_BLACK,
            TFT_BLACK,
            2,
            TFT_PINK
        );
    }
    else if (current_blink_state == BlinkState::CHAOTIC)
    {
        button_change_color->update(
            "ALL COLORS",
            TFT_BLACK,
            TFT_BLACK,
            2,
            TFT_PINK
        );
    }
    else if (current_state == NONE)
    {
        button_change_color->update(
            color_names[color_idx],
            TFT_BLACK,
            TFT_BLACK,
            2,
            colors[color_idx]
        );
    }
    button_change_color->draw(tft);



    for (char i=0; i<number_of_showcase; i++)
    {
        uint16_t color = TFT_WHITE;
        if (current_state != DisplayState::NONE)
        {
            if (
                (i == 0 && current_state == DisplayState::ELEC) ||
                (i == 1 && current_state == DisplayState::BATTERIE) ||
                (i == 2 && current_state == DisplayState::MOTEURS)
            )
            {
                color = HIGHLIGHTED_COLOR;
            }
            else
            {
                color = HIDDEN_COLOR;
            }
        }
        buttons_showcase.at(i)->update(
            showcase_names[i],
            TFT_BLACK,
            TFT_BLACK,
            2,
            color
        );
        buttons_showcase.at(i)->draw(tft);
    }

    fakebutton_text->update(
        "",
        TFT_BLACK,
        TFT_BLACK,
        1,
        TFT_WHITE
    );
    fakebutton_text->draw(tft);
    if (current_state == DisplayState::ELEC)
    {
        char* to_draw[] = ELEC_TEXT;
        for (char i=0; i<ELEC_TEXT_NLINES; i++)
        {
            fakebutton_text->draw_text(tft, to_draw[i], i);
        }
    }
    else if (current_state == DisplayState::BATTERIE)
    {
        char* to_draw[] = BATTERIE_TEXT;
        for (char i=0; i<BATTERIE_TEXT_NLINES; i++)
        {
            fakebutton_text->draw_text(tft, to_draw[i], i);
        }
    }
    else if (current_state == DisplayState::MOTEURS)
    {
        char* to_draw[] = MOTEURS_TEXT;
        for (char i=0; i<MOTEURS_TEXT_NLINES; i++)
        {
            fakebutton_text->draw_text(tft, to_draw[i], i);
        }
    }
    else
    {
        char* to_draw[] = NONE_TEXT;
        for (char i=0; i<NONE_TEXT_NLINES; i++)
        {
            fakebutton_text->draw_text(tft, to_draw[i], i);
        }
    }

    button_blink_state->update(
        current_blink_state == BlinkState::NORMAL ? "Static" : (current_blink_state == BlinkState::CHAOTIC ? "Chaotic" : ""),
        TFT_BLACK,
        TFT_BLACK,
        2,
        current_blink_state == BlinkState::CHAOTIC ? TFT_RED : TFT_WHITE
    );
    button_blink_state->draw(tft);

    std::string new_string = std::string("BPM: ") + std::to_string(bpm_modes[current_bpm_index]);
    button_bpm->update(
        new_string,
        TFT_BLACK,
        TFT_BLACK,
        2,
        current_blink_state == BlinkState::CHAOTIC ? TFT_WHITE : TFT_BLACK
    );
    if (old_bpm_index != current_bpm_index)
    {
        button_bpm->trigger_redraw();
        old_bpm_index = current_bpm_index;
    }
    button_bpm->draw(tft);

    // std::string s = std::to_string(brightness_level);
    // button_change_brightness->update(
    //     s,
    //     TFT_BLACK,
    //     TFT_BLACK,
    //     2,
    //     TFT_DARKGREY
    // );
    // button_change_brightness->draw(tft);

    // Update seconds count
    tft.setTextSize(1);
    tft.setCursor(180, 230);
    tft.setTextColor(TFT_MAGENTA, TFT_BLACK);
    tft.print(millis() / 1000);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print(" seconds.");

    // Update colors
    if (xSemaphoreTake(xMutex, portMAX_DELAY))
        {
        if (current_blink_state == BlinkState::CHAOTIC)
        {
            long lhs = (millis() - last_blinked);
            float rhs = 60.0 * 1000.0 / bpm_modes[current_bpm_index];
            // Serial.print("lhs: ");
            // Serial.print(lhs);
            // Serial.print(", rhs: ");
            // Serial.println(rhs);
            if (lhs > rhs)
            {
                // Select a new color index (different than the current index)
                uint newindex = randint(0, number_of_colors);
                while (newindex == color_idx)
                {
                    newindex = randint(0, number_of_colors);
                }
                color_idx = newindex;
                last_blinked = millis();
                need_send_message = true;
            }
        }

        if (need_send_message)
        {
            send_message();
            need_send_message = false;
        }

        xSemaphoreGive(xMutex);
    }
    

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

        if (xSemaphoreTake(xMutex, portMAX_DELAY))
        {
            bool blink_state_clicked = button_blink_state->clicked(v);
            if (blink_state_clicked)
            {
                Serial.println("Button clicked");
                if (current_blink_state == BlinkState::NORMAL)
                {
                    current_blink_state = BlinkState::CHAOTIC;
                }
                else
                {
                    current_blink_state = BlinkState::NORMAL;
                }
                current_state = DisplayState::NONE;
                need_send_message = true;
            }
            
            bool button_change_color_clicked = button_change_color->clicked(v);
            if (blink_state_clicked || button_change_color_clicked)
            {
                if (button_change_color_clicked)
                {
                    Serial.println("Button clicked");
                    if (current_state == DisplayState::NONE)
                    {
                        color_idx = (color_idx + 1) % number_of_colors;
                    }
                    else
                    {
                        current_state = DisplayState::NONE;
                    }
                    need_send_message = true;
                }
                fakebutton_text->trigger_redraw();
                button_change_color->trigger_redraw();
            }

            for (char i=0; i<number_of_showcase; i++)
            {
                if (buttons_showcase.at(i)->clicked(v))
                {
                    Serial.println("Button clicked");
                    if (current_state == DisplayState::NONE)
                    {
                        current_blink_state = BlinkState::NORMAL;
                        button_blink_state->trigger_redraw();
                        button_bpm->trigger_redraw();
                    }
                    if (i == 0)
                    {
                        current_state = DisplayState::ELEC;
                    }
                    else if (i == 1)
                    {
                        current_state = DisplayState::BATTERIE;
                    }
                    else if (i == 2)
                    {
                        current_state = DisplayState::MOTEURS;
                    }
                    //need_send_message = true;
                    fakebutton_text->trigger_redraw();
                    button_change_color->trigger_redraw();
                }
                need_send_message = true;
            }

            // if (button_change_brightness->clicked(v))
            // {
            //     Serial.println("Button clicked");
            //     brightness_level = brightness_level + BRIGHTNESS_OFFSET;
            //     if (brightness_level > 255)
            //     {
            //         brightness_level = 0;
            //     }
            //     need_send_message = true;
            // }


            
            if (button_bpm->clicked(v))
            {
                Serial.println("Button clicked");
                current_bpm_index = (current_bpm_index + 1) % num_bpm_modes;
                Serial.print("current index: ");
                Serial.println(std::to_string(current_bpm_index).c_str());
                need_send_message = true;
            }
            
            xSemaphoreGive(xMutex);
        }

        delay(30);
    }
}
