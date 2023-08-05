#ifndef _PARAMETERS_HEADER
#define _PARAMETERS_HEADER

/////////////////////////////////////////////
// Pins
/////////////////////////////////////////////

// #define LED_BUILTIN 2

#define IN1_A 16
#define IN2_A 17
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

#define R1 100000.0
#define R2 10000.0

/////////////////////////////////////////////
// Motor & encoder specs
/////////////////////////////////////////////

#define MOTOR_REDUCTION_FACTOR 100.0
#define MOTOR_RATED_RPM 220.0

#define ENCODER_PULSE_PER_REVOLUTION 28.0
#define ENCODER_SPEED_TICK_PERIOD_MS 10.0

// conversion functions
#define RPM_TO_RAD_S(VALUE) (VALUE * 2.0 * M_PI / 60.0)
#define RAD_S_TO_RPM(VALUE) (VALUE * 60.0 / (2.0 * M_PI))

// give 20% overhead
#define MAX_SPEED_RPM (0.8 * MOTOR_RATED_RPM)
#define MAX_SPEED_RAD_S (RPM_TO_RAD_S(MAX_SPEED_RPM))

// Motor PID

// Target control will be
// PWM = targetSpeed (rpm) * 255 / max speed (rpm) * MOTOR_TARGET_CONTROL_A + MOTOR_TARGET_CONTROL_B
#define MOTOR_TARGET_CONTROL_B 0
#define MOTOR_TARGET_CONTROL_A 0.55
// offset will be applied if targetSpeed (rad/s) is above threshold
#define MOTOR_ST0P_THRESHOLD_RAD_S 0.1


// PID parameters
#define VELOCITY_KP 0.5
#define VELOCITY_KD 0.01
#define VELOCITY_KI 0.02

#endif