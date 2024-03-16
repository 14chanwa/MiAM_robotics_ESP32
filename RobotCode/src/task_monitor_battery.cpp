#include <Arduino.h>
#include <parameters.hpp>
#include <tasks.hpp>

/////////////////////////////////////////////
// Resistor values
/////////////////////////////////////////////

#define RESISTOR_R1 100000.0f
#define RESISTOR_R2 10000.0f

// Moving average for battery reading
#define FILTER_LEN  5

int AN_Pot1_Raw = 0;

class ADCReading
{
  public:
    uint32_t AN_Pot1_Buffer[FILTER_LEN] = {0};
    int AN_Pot1_i = 0;
    uint32_t readADC_Avg(int ADC_Raw);
    bool MA_inited = false;
};

ADCReading battery_reading;
ADCReading tcrt0_reading;
ADCReading tcrt1_reading;
ADCReading tcrt2_reading;

ADCReading touch_sensor_reading;


// float batReading_unfiltered = 0;
float batReading = 0;
float minBatReading = 100;

float tcrt0Value = 0;
float tcrt1Value = 0;
float tcrt2Value = 0;
float touchSensorValue = 0;

void monitor_battery()
{
  AN_Pot1_Raw = analogReadMilliVolts(BAT_READING);
  // batReading_unfiltered = AN_Pot1_Raw * (RESISTOR_R1 + RESISTOR_R2) / R2 / 1000.0;
  batReading = battery_reading.readADC_Avg(AN_Pot1_Raw) * (RESISTOR_R1 + RESISTOR_R2) / RESISTOR_R2 / 1000.0;
  // batReading = battery_reading.readADC_Avg(AN_Pot1_Raw) / 1000.0;
  tcrt0Value = tcrt0_reading.readADC_Avg(analogReadMilliVolts(TCRT_0)) / 1000.0;
  tcrt1Value = tcrt1_reading.readADC_Avg(analogReadMilliVolts(TCRT_1)) / 1000.0;
  tcrt2Value = tcrt2_reading.readADC_Avg(analogReadMilliVolts(TCRT_2)) / 1000.0;
  touchSensorValue = touch_sensor_reading.readADC_Avg(touchRead(TOUCH_SENSOR)) / 1000.0;
  minBatReading = std::min(minBatReading, batReading);
}

void print_battery()
{
  Serial.print(">batReading:");
  Serial.println(batReading);
  Serial.print(">minBatReading:");
  Serial.println(minBatReading);
}


float get_current_battery_reading()
{
    return batReading;
}

float get_min_battery_reading()
{
    return minBatReading;
}

float get_current_tcrt0_reading()
{
  return tcrt0Value;
}

float get_current_tcrt1_reading()
{
  return tcrt1Value;
}

float get_current_tcrt2_reading()
{
  return tcrt2Value;
}

float get_current_touch_sensor_reading()
{
  return touchSensorValue;
}

// void task_update_analog_readings(void* parameters){
//     for(;;)
//     {
//       monitor_battery();
//       vTaskDelay(20 / portTICK_PERIOD_MS);
//     }
// }

/////////////////////////////////////////////
// Functions
/////////////////////////////////////////////

uint32_t ADCReading::readADC_Avg(int ADC_Raw)
{

  if (!MA_inited)
  {
    for (int i = 0; i < FILTER_LEN; i++)
      AN_Pot1_Buffer[i] = ADC_Raw;
    
    MA_inited = true;
  }

  int i = 0;
  uint32_t Sum = 0;
  
  AN_Pot1_Buffer[AN_Pot1_i++] = ADC_Raw;
  if(AN_Pot1_i == FILTER_LEN)
  {
    AN_Pot1_i = 0;
  }
  for(i=0; i<FILTER_LEN; i++)
  {
    Sum += AN_Pot1_Buffer[i];
  }
  return (Sum/FILTER_LEN);
}