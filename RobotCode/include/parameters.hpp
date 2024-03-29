#ifndef _PARAMETERS_HEADER
#define _PARAMETERS_HEADER

#define PAMI_ID 2

#if PAMI_ID == 1 || PAMI_ID == 2 || PAMI_ID == 3
    #define USE_STEPPER_MOTORS
#else
    #define USE_DC_MOTORS
#endif

#define SEND_TELEPLOT_UDP
#define MIAM_BROADCAST_ADDRESS "42.42.0.255"
// #define SEND_SERIAL

// #define ENABLE_OTA_UPDATE

// #define DEBUG_MODE_MATCH
// #define DEBUG_MODE_SERVO
// #define DEBUG_MODE_SIMPLE_TRAJECTORY

/////////////////////////////////////////////
// Motion controller specs
/////////////////////////////////////////////

#define LOW_LEVEL_LOOP_TIME_MS 10

/////////////////////////////////////////////
// Match parameters
/////////////////////////////////////////////

#define MATCH_DURATION_S 100.0f
#define MATCH_PAMI_START_TIME_S 90.0f

#endif