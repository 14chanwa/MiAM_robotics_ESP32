#ifndef _PARAMETERS_HEADER
#define _PARAMETERS_HEADER

#define USE_WIFI
#define USE_MONITOR_BATTERY
#define USE_VL53L0X

#ifdef USE_WIFI
  #define ENABLE_OTA_UPDATE
  #define SEND_TELEPLOT_UDP
#endif
// #define SEND_SERIAL

// #define USE_DC_MOTORS
#define USE_STEPPER_MOTORS


/////////////////////////////////////////////
// Pins
/////////////////////////////////////////////

#define LED_BUILTIN 2

#define BAT_READING 36

#define TCRT_0 32
#define TCRT_1 39
#define TCRT_2 33

#define TOUCH_SENSOR 12

/////////////////////////////////////////////
// Resistor values
/////////////////////////////////////////////

#define RESISTOR_R1 100000.0f
#define RESISTOR_R2 10000.0f

/////////////////////////////////////////////
// Motion controller specs
/////////////////////////////////////////////

#define LOW_LEVEL_LOOP_TIME_MS 10

// Motion controller PID parameters
#define LINEAR_KP 3.5f
#define LINEAR_KD 0.01f
#define LINEAR_KI 0.0f

#define TRANSVERSE_KP 0.005f

#define ROTATION_KP 5.0f
#define ROTATION_KD 0.0f
#define ROTATION_KI 0.0f

#endif