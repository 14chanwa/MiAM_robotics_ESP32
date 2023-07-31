#ifndef _TASKS_HEADER
#define _TASKS_HEADER

#include <parameters.hpp>
#include <Motor.hpp>

/////////////////////////////////////////////
// Tasks
/////////////////////////////////////////////

// blink led

void run_blink_led();
void task_blink_led(void* parameters);

// monitor battery

void run_monitor_battery();
void task_monitor_battery(void* parameters);
float get_current_battery_reading();
float get_min_battery_reading();

// handle encoders
void run_handle_encoders();
int get_encoder_values(unsigned char id);
int get_encoder_speed(unsigned char id);
void task_print_encoder(void* parameters);
void task_get_encoder_speed(void* parameters);

// low level loop

void run_low_level_loop(Motor* motor);
void task_low_level_loop(void* parameters);
void task_print_low_level(void* parameters);

#endif