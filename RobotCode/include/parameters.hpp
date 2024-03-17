#ifndef _PARAMETERS_HEADER
#define _PARAMETERS_HEADER

#define PAMI_ID 3

#if PAMI_ID == 1 || PAMI_ID == 2 || PAMI_ID == 3
    #define USE_STEPPER_MOTORS
#else
    #define USE_DC_MOTORS
#endif

#define SEND_TELEPLOT_UDP
// #define SEND_SERIAL

// #define ENABLE_OTA_UPDATE

// #define DEBUG_MODE_MATCH
// #define DEBUG_MODE_SERVO
// #define DEBUG_MODE_SIMPLE_TRAJECTORY

/////////////////////////////////////////////
// Pins
/////////////////////////////////////////////

#define LED_BUILTIN 2

#define BAT_READING 36

#define TCRT_0 32
#define TCRT_1 39
#define TCRT_2 33

#define TOUCH_SENSOR 0

/////////////////////////////////////////////
// Motion controller specs
/////////////////////////////////////////////

#define LOW_LEVEL_LOOP_TIME_MS 10

#endif