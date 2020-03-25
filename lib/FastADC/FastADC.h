#ifndef FASTADC_H_
#define FASTADC_H_

#include <Arduino.h>

#if !defined(ADC_SAMPLE_TIME)
  #if defined(STM32F1xx)
    #define ADC_SAMPLE_TIME ADC_SAMPLETIME_13CYCLES_5
  #elif defined(STM32F4xx)
    #define ADC_SAMPLE_TIME ADC_SAMPLETIME_15CYCLES
  #endif
#endif

#if defined(STM32F1xx)
  #define ADC_STAB_DELAY_US 10
#endif

class FastADC
{
private:
    uint8_t m_Pin;
    ADC_HandleTypeDef m_AdcHandle = {};
    ADC_ChannelConfTypeDef  m_AdcChannelConf = {};
    uint32_t getAdcInternalChannel(PinName pin);
    uint32_t getAdcChannel(PinName pin);
public:
    FastADC(uint8_t pin) { m_Pin = pin;};
    void begin();
    uint16_t read();
};




#endif