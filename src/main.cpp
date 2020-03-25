#include <Arduino.h>
#include "FastADC.h"

#define BITSET_SH (GPIOC->BSRR = GPIO_BSRR_BS15);
#define BITCLR_SH (GPIOC->BSRR = GPIO_BSRR_BR15);
#define BITSET_ICG (GPIOC->BSRR = GPIO_BSRR_BS14);
#define BITCLR_ICG (GPIOC->BSRR = GPIO_BSRR_BR14);

#define ADC_PIN PA7
#define CLK_PIN PA8

// Full frame, including dark pixels
// and dead pixels.
#define PIXEL_COUNT 3691

uint16_t buffer[PIXEL_COUNT];
uint16_t buffer2[85];
uint16_t avg = 0;
int exposureTime = 10;

FastADC fastADC(ADC_PIN);

void setup()
{
    pinMode(CLK_PIN, OUTPUT);
    pinMode(ADC_PIN, INPUT_ANALOG);

    analogWrite(CLK_PIN, 50);

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
    HT->setOverflow(100, TICK_FORMAT);
    HT->setCaptureCompare(clkChannel, 50, TICK_COMPARE_FORMAT);
    HT->resume();

    pinMode(PC14, OUTPUT);
    pinMode(PC15, OUTPUT);

    fastADC.begin();
}

void readCCD(void)
{
    uint16_t result;
    BITCLR_ICG;
    delayMicroseconds(1);
    BITSET_SH;  
    delayMicroseconds(5);
    BITCLR_SH;
    delayMicroseconds(15);
    BITSET_ICG;
    delayMicroseconds(1);

    for (int x = 0; x < PIXEL_COUNT; x++)
    {
        BITSET_SH;

        result = fastADC.read();

        buffer[x] = result;
        delayMicroseconds(5);

        BITCLR_SH;
    }
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