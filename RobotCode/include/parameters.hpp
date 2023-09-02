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

#define RESISTOR_R1 100000.0f
#define RESISTOR_R2 10000.0f

/////////////////////////////////////////////
// Wheel specs
/////////////////////////////////////////////

#define WHEEL_RADIUS_MM 30.0f
#define WHEEL_SPACING_MM 73.5

/////////////////////////////////////////////
// Motor & encoder specs
/////////////////////////////////////////////

#define MOTOR_REDUCTION_FACTOR 100.0f
#define MOTOR_RATED_RPM 220.0f

#define ENCODER_PULSE_PER_REVOLUTION 28.0f
#define ENCODER_SPEED_TICK_PERIOD_MS 10.0f

// conversion functions
#define RPM_TO_RAD_S(VALUE) (VALUE * 2.0f * M_PI / 60.0f)
#define RAD_S_TO_RPM(VALUE) (VALUE * 60.0f / (2.0f * M_PI))

// give 20% overhead
#define MAX_SPEED_RPM (MOTOR_RATED_RPM)
#define MAX_SPEED_RAD_S (RPM_TO_RAD_S(MAX_SPEED_RPM))

#define MAX_WHEEL_SPEED_MM_S (MAX_SPEED_RAD_S * WHEEL_RADIUS_MM)
#define MAX_WHEEL_ACCELERATION_MM_S 250.0f

/////////////////////////////////////////////
// Motion controller specs
/////////////////////////////////////////////

#define LOW_LEVEL_LOOP_TIME_MS 10

// Motor PID

// Target control will be
// PWM = targetSpeed (rpm) * 255 / max speed (rpm) * MOTOR_TARGET_CONTROL_A + MOTOR_TARGET_CONTROL_B
#define MOTOR_TARGET_CONTROL_B 20
#define MOTOR_TARGET_CONTROL_A 0.52f
// offset will be applied if targetSpeed (rad/s) is above threshold
#define MOTOR_ST0P_THRESHOLD_RAD_S 0.02f

// Wheel PID parameters
#define VELOCITY_KP 0.5f
#define VELOCITY_KD 0.01f
#define VELOCITY_KI 0.02f

// Motion controller PID parameters
#define LINEAR_KP 3.5f
#define LINEAR_KD 0.01f
#define LINEAR_KI 0.00f

#define TRANSVERSE_KP 0.005f

#define ROTATION_KP 5.0f
#define ROTATION_KD 0.0f
#define ROTATION_KI 0.0f

#endif