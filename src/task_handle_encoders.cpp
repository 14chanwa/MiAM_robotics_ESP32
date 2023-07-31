#include <Arduino.h>
#include <parameters.hpp>
#include <tasks.hpp>

volatile int encoder_value[2] = {0, 0};
int old_encoder_value[2] = {0, 0};
volatile int encoder_speed[2] = {0, 0};

void IRAM_ATTR handleEncoder1();
void IRAM_ATTR handleEncoder2();

void run_handle_encoders()
{
    pinMode(ENCODER_A1, INPUT_PULLUP);
    attachInterrupt(ENCODER_A1, handleEncoder1, CHANGE);
    attachInterrupt(ENCODER_B1, handleEncoder1, CHANGE);
    attachInterrupt(ENCODER_A2, handleEncoder2, CHANGE);
    attachInterrupt(ENCODER_B2, handleEncoder2, CHANGE);
    xTaskCreate(
        task_print_encoder, 
        "task_print_encoder",
        1000,
        NULL,
        1,
        NULL
    ); 
    xTaskCreate(
        task_get_encoder_speed, 
        "task_get_encoder_speed",
        1000,
        NULL,
        1,
        NULL
    ); 
}

int get_encoder_values(unsigned char id)
{
    return encoder_value[id];
}

int get_encoder_speed(unsigned char id)
{
    return encoder_speed[id];
}

/////////////////////////////////////////////
// Functions
/////////////////////////////////////////////

bool oldB_1 = 0;
bool oldB_2 = 0;

void IRAM_ATTR handleEncoder1()
{
    // Get current status.
    bool currentA = digitalRead(ENCODER_A1);

    // The direction of the encoder is given by currentA xor oldB
    encoder_value[LEFT_ENCODER_ID] += (oldB_1 ^ currentA ? 1 : -1);
    oldB_1 = digitalRead(ENCODER_B1);
}

void IRAM_ATTR handleEncoder2()
{
    // Get current status.
    bool currentA = digitalRead(ENCODER_A2);

    // The direction of the encoder is given by currentA xor oldB
    encoder_value[RIGHT_ENCODER_ID] += (oldB_2 ^ currentA ? 1 : -1);
    oldB_2 = digitalRead(ENCODER_B2);
}

void task_print_encoder(void* parameters)
{
    for(;;)
    {
        Serial.print(">encoderLeftValue:");
        Serial.println(encoder_value[LEFT_ENCODER_ID]);
        Serial.print(">encoderRightValue:");
        Serial.println(encoder_value[RIGHT_ENCODER_ID]);
        Serial.print(">encoderLeftSpeed:");
        Serial.println(encoder_speed[LEFT_ENCODER_ID]);
        Serial.print(">encoderRightSpeed:");
        Serial.println(encoder_speed[RIGHT_ENCODER_ID]);
        vTaskDelay(100.0 / portTICK_PERIOD_MS);
    }
}

void task_get_encoder_speed(void* parameters)
{
    for(;;)
    {
        encoder_speed[LEFT_ENCODER_ID] = 
            encoder_value[LEFT_ENCODER_ID] - old_encoder_value[LEFT_ENCODER_ID];
        old_encoder_value[LEFT_ENCODER_ID] = encoder_value[LEFT_ENCODER_ID];

        encoder_speed[RIGHT_ENCODER_ID] = 
            encoder_value[RIGHT_ENCODER_ID] - old_encoder_value[RIGHT_ENCODER_ID];
        old_encoder_value[RIGHT_ENCODER_ID] = encoder_value[RIGHT_ENCODER_ID];

        vTaskDelay(10.0 / portTICK_PERIOD_MS);
    }
}