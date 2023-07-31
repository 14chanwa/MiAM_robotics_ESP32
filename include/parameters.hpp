#ifndef _PARAMETERS_HEADER
#define _PARAMETERS_HEADER

/////////////////////////////////////////////
// Pins
/////////////////////////////////////////////

// #define LED_BUILTIN 2

#define IN1_A 25
#define IN2_A 26
#define IN1_B 32
#define IN2_B 33
#define EN_A 27
#define EN_B 14

#define ENCODER_A1 2
#define ENCODER_B1 4

#define ENCODER_A2 34
#define ENCODER_B2 35

#define BAT_READING 36

/////////////////////////////////////////////
// Resistor values
/////////////////////////////////////////////

#define R1 20000.0
#define R2 4700.0

/////////////////////////////////////////////
// Motor & encoder specs
/////////////////////////////////////////////

#define MOTOR_REDUCTION_FACTOR 100.0
#define MOTOR_RATED_RPM 220.0
#define ENCODER_PULSE_PER_REVOLUTION 28
#define LEFT_ENCODER_ID 0
#define RIGHT_ENCODER_ID 1// PID

// Motor PID
#define VELOCITY_KP 0.07
#define VELOCITY_KD 0.01
#define VELOCITY_KI 0.02

#endif