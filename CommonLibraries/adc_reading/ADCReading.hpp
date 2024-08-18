#include <Arduino.h>

// Moving average for battery reading
#define FILTER_LEN 5

class ADCReading
{
public:
    uint32_t AN_Pot1_Buffer[FILTER_LEN] = {0};
    int AN_Pot1_i = 0;
    uint32_t readADC_Avg(int ADC_Raw);
    bool MA_inited = false;
};