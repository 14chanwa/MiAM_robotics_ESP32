#include <Arduino.h>
#include <pinout.hpp>
#include <tasks.hpp>
#include <PID.h>
#include <L298NX2.h>

// Motors
L298NX2 myMotors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);

// PID
#define VELOCITY_KP 0.07
#define VELOCITY_KD 0.01
#define VELOCITY_KI 0.02
miam::PID velocityPID(VELOCITY_KP, VELOCITY_KD, VELOCITY_KI, 3000);

int printCounter = 0;


double currentSpeed = 0; 
double rawSpeed = 0; 
int motorSpeedPWM = 0.0;
double targetSpeed = 0;

unsigned long timeLowLevel = 0;

void setSpeed(int speed_rpm)
{
    targetSpeed = speed_rpm;
}
int getSpeed()
{
    return currentSpeed;
}


void run_low_level_loop()
{
    xTaskCreate(
        task_low_level_loop, 
        "task_low_level_loop",
        10000,
        NULL,
        1,
        NULL
    ); 
}

long currentTime;
double dt_ms;
double error;
double correction;

double newTarget;
int basePWMTarget;
int newPWMTarget;

void task_low_level_loop(void* parameters)
{
  for(;;)
  {
    currentTime = micros();
    if (timeLowLevel > 0)
    {
        dt_ms = (currentTime - timeLowLevel) / 1000.0; // in ms
        Serial.print(">dt_ms:");
        Serial.println(dt_ms);
        
        // speed in rpm
        rawSpeed = get_encoder_speed(LEFT_ENCODER_ID) 
            * 60.0 / 0.01 // convert from 10ms to 60s
            / ENCODER_PULSE_PER_REVOLUTION // convert from encoder tick to encoder revolution
            / MOTOR_REDUCTION_FACTOR; // convert from encoder revolution to motor revolution

        // currentSpeed = readVelocity_Avg(rawSpeed);
        currentSpeed = rawSpeed;
        Serial.print(">currentSpeed:");
        Serial.println(currentSpeed);
        Serial.print(">targetSpeed: ");
        Serial.println(targetSpeed);  

        error = currentSpeed - targetSpeed; // in rpm

        correction = velocityPID.computeValue(error, dt_ms);
        newTarget = targetSpeed + correction;

        // convert from rpm to 0-255
        basePWMTarget = (targetSpeed < 0 ? -1.0 : 1.0) 
            * std::min((int)(std::abs(targetSpeed * 255.0 / MOTOR_RATED_RPM / 1.5)), 255);
        newPWMTarget = (newTarget < 0 ? -1.0 : 1.0) 
            * std::min((int)(std::abs(newTarget * 255.0 / MOTOR_RATED_RPM / 1.5)), 255);

        // if (targetSpeed < 5 & abs(error) < 20)
        // {
        //     // stop the motors
        //     newPWMTarget = 0;
        // }

        if (printCounter % 20 == 0)
        {
            Serial.print(">error: ");
            Serial.println(error);       
            Serial.print(">correction: ");
            Serial.println(correction);        
            // Serial.print(" - currentSpeed: ");
            // Serial.print(currentSpeed);             
            // Serial.print(" - targetSpeed: ");
            // Serial.print(targetSpeed);           
            // Serial.print(" - correction: ");
            // Serial.print(correction);           
            // Serial.print(" - dt_min: ");
            // Serial.print(dt_min);
            // Serial.print(" - error: ");
            // Serial.print(error);
            Serial.print(">basePWMTarget: ");
            Serial.println(basePWMTarget);
            Serial.print(">newPWMTarget: ");
            Serial.println(newPWMTarget);
            // Serial.print(" - newPWMTarget: ");
            // Serial.println(newPWMTarget);
        }
        // newPWMTarget = 255;
        printCounter++;
        

        // if (targetSpeed < 10)
        // {
        //     myMotors.stop();
        //     velocityPID.resetIntegral();
        // }
        // else
        // {
            // if (targetSpeed * newPWMTarget >= 0)
            // {   
                if (newPWMTarget >= 0)
                {
                    myMotors.forward();
                }
                else
                {
                    myMotors.backward();
                }
                // int newPWMTarget = 255;
                // myMotors.forward();
                myMotors.setSpeed(std::abs(newPWMTarget));
            // }
            
        // }
        
    }
    timeLowLevel = currentTime;
    vTaskDelay(10.0 / portTICK_PERIOD_MS);
  }    
}