#include <Arduino.h>
#include <parameters.hpp>
#include <tasks.hpp>

// Moving average for battery reading
#define FILTER_LEN  15

uint32_t AN_Pot1_Buffer[FILTER_LEN] = {0};
int AN_Pot1_i = 0;
int AN_Pot1_Raw = 0;
uint32_t readADC_Avg(int ADC_Raw);
bool MA_inited = false;

// float batReading_unfiltered = 0;
float batReading = 0;
float minBatReading = 100;

void monitor_battery()
{
  AN_Pot1_Raw = analogReadMilliVolts(BAT_READING);
  // batReading_unfiltered = AN_Pot1_Raw * (RESISTOR_R1 + RESISTOR_R2) / R2 / 1000.0;
  batReading = readADC_Avg(AN_Pot1_Raw) * (RESISTOR_R1 + RESISTOR_R2) / RESISTOR_R2 / 1000.0;
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




/////////////////////////////////////////////
// Functions
/////////////////////////////////////////////

uint32_t readADC_Avg(int ADC_Raw)
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