#include <ADCReading.hpp>

ADCReading::ADCReading(int filter_len) : filter_len_(filter_len) 
{
    AN_Pot1_Buffer = new uint32_t[filter_len_];
    for (uint i=0; i<filter_len_; i++)
    {
        AN_Pot1_Buffer[i] = 0;
    }
}

ADCReading::ADCReading(): ADCReading(FILTER_LEN)
{
    
}


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
    if (AN_Pot1_i == FILTER_LEN)
    {
        AN_Pot1_i = 0;
    }
    for (i = 0; i < FILTER_LEN; i++)
    {
        Sum += AN_Pot1_Buffer[i];
    }
    return (Sum / FILTER_LEN);
}

uint32_t ADCReading::readADC_Min(int ADC_Raw)
{

    if (!MA_inited)
    {
        for (int i = 0; i < FILTER_LEN; i++)
            AN_Pot1_Buffer[i] = ADC_Raw;

        MA_inited = true;
    }

    int i = 0;

    AN_Pot1_Buffer[AN_Pot1_i++] = ADC_Raw;
    if (AN_Pot1_i == FILTER_LEN)
    {
        AN_Pot1_i = 0;
    }
    uint32_t Min = 8000;
    for (i = 0; i < FILTER_LEN; i++)
    {
        Min = std::min(AN_Pot1_Buffer[i], Min);
    }
    return Min;
}


uint32_t ADCReading::readADC_Max(int ADC_Raw)
{

    if (!MA_inited)
    {
        for (int i = 0; i < FILTER_LEN; i++)
            AN_Pot1_Buffer[i] = ADC_Raw;

        MA_inited = true;
    }

    int i = 0;

    AN_Pot1_Buffer[AN_Pot1_i++] = ADC_Raw;
    if (AN_Pot1_i == FILTER_LEN)
    {
        AN_Pot1_i = 0;
    }
    uint32_t Max = 0;
    for (i = 0; i < FILTER_LEN; i++)
    {
        Max = std::max(AN_Pot1_Buffer[i], Max);
    }
    return Max;
}