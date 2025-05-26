#include <TFTScreen.hpp>
#include <Button.hpp>
#include <Match.hpp>

#include <NetworkHandler.hpp>
#include <ServoHandler.hpp>
#include <MessageReceiver.hpp>

#define SET_SIDE_PIN 36
#define FUNCTION_PIN 39
#define START_SWITCH_PIN 35

// Screen
TFTScreen tftScreen;

// Read buttons with hysteresis
Button set_side_button(SET_SIDE_PIN);
Button function_button(FUNCTION_PIN);
Button start_switch_button(START_SWITCH_PIN);

void task_update_screen(void *parameters)
{
    char counter = 0;
    for (;;)
    {
        counter = (counter + 1) % 2;
        tftScreen.registerTouch();
        if (counter == 0)
        {
            tftScreen.update(NetworkHandler::localIP());
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void task_read_pins(void *parameters)
{
    // set_side_button.init();
    // function_button.init();
    start_switch_button.init();

    for (;;)
    {
        // set_side_button.update();
        // function_button.update();
        start_switch_button.update();
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}

void task_monitor_buttons(void *parameters)
{
    for (;;)
    {
        // Check buttons
        ButtonEvent buttonEvent;

        // Set side button triggers messages
        buttonEvent = set_side_button.getEvent();
        if (buttonEvent == ButtonEvent::NEW_STATE_LOW)
        {
            // Change color
            if (Match::getSide() == PlayingSide::BLUE_SIDE)
            {
                Match::setSide(PlayingSide::YELLOW_SIDE);
            }
            else
            {
                Match::setSide(PlayingSide::BLUE_SIDE);
            }
        }

        // // Function button toggles pami motor lock
        // buttonEvent = function_button.getEvent();
        // if (buttonEvent == ButtonEvent::NEW_STATE_LOW)
        // {
        //     if (Match::getMatchStarted())
        //     {
        //         Match::stopMatch();
        //     }
        //     else
        //     {
        //         Match::startMatch(85.0);
        //     }
        //     // Match::setStopMotors(!Match::getStopMotors());
        // }

        // Switch button starts match
        buttonEvent = start_switch_button.getEvent();
        if (buttonEvent == ButtonEvent::NEW_STATE_HIGH)
        {
            Match::startMatch(0.0f);
        }
        else if (buttonEvent == ButtonEvent::NEW_STATE_LOW)
        {
            Match::stopMatch();
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void task_handle_servo(void *parameters)
{
    ServoHandler::init();
    bool state = false;
    for (;;)
    {
        if (Match::getMatchStarted() &&
            Match::getMatchTimeSeconds() >= 90 &&
            Match::getMatchTimeSeconds() < 100)
        {
            if (state)
            {
                ServoHandler::servoUp();
                Serial.println("Servo down");
            }
            else
            {
                ServoHandler::servoDown();
                Serial.println("Servo up");
            }
            state = !state;
        }
        else if (Match::getMatchStarted() && Match::getMatchTimeSeconds() >= 100)
        {
            ServoHandler::servoUp();
        }
        else
        {
            ServoHandler::servoFolded();
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void setup()
{
    Serial.begin(115200);

    tftScreen.init();
    xTaskCreatePinnedToCore(
        task_update_screen,
        "task_update_screen",
        10000,
        NULL,
        6,
        NULL,
        0);

    xTaskCreatePinnedToCore(
        task_read_pins,
        "task_read_pins",
        10000,
        NULL,
        8,
        NULL,
        0);

    xTaskCreatePinnedToCore(
        task_monitor_buttons,
        "task_monitor_buttons",
        10000,
        NULL,
        7,
        NULL,
        0);

    xTaskCreatePinnedToCore(
        task_handle_servo,
        "task_handle_servo",
        10000,
        NULL,
        1,
        NULL,
        0);

    NetworkHandler::init();
    MessageReceiver::startListening();
}

void loop()
{
    taskYIELD();
}