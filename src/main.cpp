#include <Arduino.h>

#define SH 0b1000000000000000 // PC15
#define ICG 0b0100000000000000 // PC14

#define BITSET(mask) GPIOC->regs->BSRR = mask
#define BITCLR(mask) GPIOC->regs->BRR = mask


// Full frame, including dark pixels
// and dead pixels.
#define PIXEL_COUNT 3691

uint8_t buffer[PIXEL_COUNT];
uint8_t buffer2[85];
uint8_t avg = 0;
int exposureTime = 10
;


void setup()
{
  pinMode(PA8, WiringPinMode::PWM);

  timer_dev *dev = PIN_MAP[PA8].timer_device;
  uint8 cc_channel = PIN_MAP[PA8].timer_channel;

  timer_set_prescaler(dev, (uint16)(0));
  timer_set_reload(dev, 100);
  timer_set_compare(dev, cc_channel, 50);

  adc_set_prescaler(ADC_PRE_PCLK2_DIV_2);
  adc_set_sample_rate(ADC1, ADC_SMPR_1_5);

  GPIOC->regs->CRL = 0x00000000;
  GPIOC->regs->CRH = 0x33300000;
}

void readCCD(void)
{
  int x;
  uint8_t result;

  //CLOCK &= ~ICG;
  BITCLR(ICG);

  //_delay_loop_1(12);
  delayMicroseconds(1);
  
  //CLOCK |= SH;
  BITSET(SH);
  
  delayMicroseconds(5);
  
  //CLOCK &= ~SH;
  BITCLR(SH);
  
  delayMicroseconds(15);
  
  //CLOCK |= ICG;
  BITSET(ICG);
  
  delayMicroseconds(1);

  for (x = 0; x < PIXEL_COUNT; x++)
  {
    //CLOCK |= SH;
    BITSET(SH);

    result = (uint8_t)(analogRead(PA7) >> 2);   

    if (x == 0)
    {
      avg = result;
    }
    else
    {
      // TODO data is inverted when we connect ADC directly
      if (result < avg)
      {
        // result = 0;
      }
      else
      {
        // result -= avg;
      }
      
      buffer[x] = result;
      delayMicroseconds(5);
    }

    //CLOCK &= ~SH;
    BITCLR(SH);
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