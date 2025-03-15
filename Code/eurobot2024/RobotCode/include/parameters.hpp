#ifndef _PARAMETERS_HEADER
#define _PARAMETERS_HEADER

#define PAMI_ID 2

// #define USE_STEPPER_MOTORS
#define USE_DC_MOTORS

// Set MAC address manually for router to allocate static IPs
#define WIFI_MAC_ADDRESS {0xAA, 0xAB, 0xAC, 0xAD, 0x00, PAMI_ID}

#define SEND_TELEPLOT_UDP
#define MIAM_BROADCAST_ADDRESS "10.42.0.255"//"42.42.0.255"
#define MIAM_SCD_ADDRESS IPAddress(42, 42, 0, 10)
#define MIAM_SCD_PORT 778
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
#define MATCH_PAMI_START_TIME_S 85.0f

#endif