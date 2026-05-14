#include <Arduino.h>

// Moving average for battery reading
#define FILTER_LEN 5

class ADCReading
{
public:
    ADCReading();
    ADCReading(int filter_len);
    uint32_t* AN_Pot1_Buffer;
    int AN_Pot1_i = 0;
    uint32_t readADC_Avg(int ADC_Raw);
    uint32_t readADC_Min(int ADC_Raw);
    uint32_t readADC_Max(int ADC_Raw);
    bool MA_inited = false;
    int filter_len_ = 0;
};
