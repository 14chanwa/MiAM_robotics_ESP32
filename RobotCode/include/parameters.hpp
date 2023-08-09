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

#define ENCODER_A1 13
#define ENCODER_B1 4

#define ENCODER_A2 34
#define ENCODER_B2 35

#define BAT_READING 36

/////////////////////////////////////////////
// Resistor values
/////////////////////////////////////////////

#define RESISTOR_R1 100000.0
#define RESISTOR_R2 10000.0

/////////////////////////////////////////////
// Wheel specs
/////////////////////////////////////////////

#define WHEEL_RADIUS_MM 30.0
#define WHEEL_SPACING_MM 73.5

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

#define MAX_WHEEL_SPEED_MM_S (MAX_SPEED_RAD_S * WHEEL_RADIUS_MM)
#define MAX_WHEEL_ACCELERATION_MM_S 250.0

/////////////////////////////////////////////
// Motion controller specs
/////////////////////////////////////////////

#define LOW_LEVEL_LOOP_TIME_MS 10

// Motor PID

// Target control will be
// PWM = targetSpeed (rpm) * 255 / max speed (rpm) * MOTOR_TARGET_CONTROL_A + MOTOR_TARGET_CONTROL_B
#define MOTOR_TARGET_CONTROL_B 20
#define MOTOR_TARGET_CONTROL_A 0.52
// offset will be applied if targetSpeed (rad/s) is above threshold
#define MOTOR_ST0P_THRESHOLD_RAD_S 0.1

// Wheel PID parameters
#define VELOCITY_KP 0.5
#define VELOCITY_KD 0.01
#define VELOCITY_KI 0.02

// Motion controller PID parameters
#define LINEAR_KP 3.5
#define LINEAR_KD 0.01
#define LINEAR_KI 0.00

#define TRANSVERSE_KP 0.005

#define ROTATION_KP 5.0
#define ROTATION_KD 0.01
#define ROTATION_KI 0.0

#endif