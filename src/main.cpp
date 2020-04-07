#include <Arduino.h>
#include "FastADC.h"

#if defined(STM32F407xx)
    #define ADC_PIN PC3
    #define SH_PIN PE15
    #define ICG_PIN PE14
    #define ADC_FLAG_PIN PE13
    #define BITSET_SH (GPIOE->BSRR = GPIO_BSRR_BS15);
    #define BITCLR_SH (GPIOE->BSRR = GPIO_BSRR_BR15);
    #define BITSET_ICG (GPIOE->BSRR = GPIO_BSRR_BS14);
    #define BITCLR_ICG (GPIOE->BSRR = GPIO_BSRR_BR14);
    // just to use as a sync/marker for oscilloscope
    #define BITSET_ADC_READ (GPIOE->BSRR = GPIO_BSRR_BS13);
    #define BITCLR_ADC_READ (GPIOE->BSRR = GPIO_BSRR_BR13);
#else
    #define ADC_PIN PA7
    #define SH_PIN PC15
    #define ICG_PIN PC14
    #define ADC_FLAG_PIN PC13
    #define BITSET_SH (GPIOC->BSRR = GPIO_BSRR_BS15);
    #define BITCLR_SH (GPIOC->BSRR = GPIO_BSRR_BR15);
    #define BITSET_ICG (GPIOC->BSRR = GPIO_BSRR_BS14);
    #define BITCLR_ICG (GPIOC->BSRR = GPIO_BSRR_BR14);
    // just to use as a sync/marker for oscilloscope
    #define BITSET_ADC_READ (GPIOC->BSRR = GPIO_BSRR_BS13);
    #define BITCLR_ADC_READ (GPIOC->BSRR = GPIO_BSRR_BR13);
#endif

#define CLK_PIN PA8

// Full frame, including dark pixels
// and dead pixels.
#define PIXEL_COUNT 3691
#define CPU_CLOCK_FREQ F_CPU

uint16_t buffer[PIXEL_COUNT];
uint16_t buffer2[85];
uint16_t avg = 0;
int exposureTime = 5;

FastADC fastADC(ADC_PIN);

uint32_t measureAdcSpeed();

void setup()
{
    pinMode(SH_PIN, OUTPUT);
    pinMode(ICG_PIN, OUTPUT);
    pinMode(ADC_FLAG_PIN, OUTPUT);
    pinMode(CLK_PIN, OUTPUT);
    pinMode(ADC_PIN, INPUT_ANALOG);

    fastADC.begin();
    // measure ADC speed
    uint32_t adcFreq = measureAdcSpeed();
    uint32_t ovfCounter = F_CPU / adcFreq / 4;

    PinName clkPin = digitalPinToPinName(CLK_PIN);

    TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(clkPin, PinMap_PWM);
    HardwareTimer *HT;
    uint32_t index = get_timer_index(Instance);
    if (HardwareTimer_Handle[index] == NULL) 
    {
        HardwareTimer_Handle[index]->__this = new HardwareTimer((TIM_TypeDef *)pinmap_peripheral(clkPin, PinMap_PWM));
    }

    HT = (HardwareTimer *)(HardwareTimer_Handle[index]->__this);

    uint32_t clkChannel = STM_PIN_CHANNEL(pinmap_function(clkPin, PinMap_PWM));

    HT->setPrescaleFactor(1);
    HT->setMode(clkChannel, TIMER_OUTPUT_COMPARE_PWM1, clkPin);
    HT->setOverflow(ovfCounter, TICK_FORMAT);
    HT->setCaptureCompare(clkChannel, ovfCounter/2, TICK_COMPARE_FORMAT);
    HT->resume();
}

void readCCDInternal(int pixelsToRead)
{
    for (int x = 0; x < pixelsToRead; x++)
    {
        BITSET_SH;
        buffer[x] = fastADC.read();
        BITCLR_SH;
    }    
}


uint32_t measureAdcSpeed()
{
    uint32_t started = micros();
    readCCDInternal(1000);
    uint32_t timed = micros() - started;

    return 1000000UL / timed * 1000;
}

void readCCD(void)
{
    BITCLR_ICG;
    delayMicroseconds(1);
    BITSET_SH;  
    delayMicroseconds(5);
    BITCLR_SH;
    delayMicroseconds(15);
    BITSET_ICG;
    delayMicroseconds(1);

    BITSET_ADC_READ;
    readCCDInternal(PIXEL_COUNT);
    BITCLR_ADC_READ;
}

unsigned long copyTimer = 0;

void loop()
{
    if (millis() - copyTimer > 2000)
    {
        for (int i = 0; i< 36; ++i)
        {
            buffer2[i] = buffer[i*100];
        }
        copyTimer = millis();
    }

    readCCD();
    delay(exposureTime);
}