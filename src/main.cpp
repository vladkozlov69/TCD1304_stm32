#include <Arduino.h>

#define SH 0b1000000000000000 // PC15
#define ICG 0b0100000000000000 // PC14

#define BITSET(mask) GPIOC->regs->BSRR = mask
#define BITCLR(mask) GPIOC->regs->BRR = mask

#define ADC_PIN PA7
#define CLK_PIN PA8

// Full frame, including dark pixels
// and dead pixels.
#define PIXEL_COUNT 3691

uint16_t buffer[PIXEL_COUNT];
uint16_t buffer2[85];
uint16_t avg = 0;
int exposureTime = 10
;


void setup()
{
  pinMode(CLK_PIN, WiringPinMode::PWM);
  pinMode(ADC_PIN, WiringPinMode::INPUT_ANALOG);

  timer_dev *t_dev = PIN_MAP[CLK_PIN].timer_device;
  uint8_t cc_channel = PIN_MAP[CLK_PIN].timer_channel;

  timer_set_prescaler(t_dev, (uint16_t)(0));
  timer_set_reload(t_dev, 100);
  timer_set_compare(t_dev, cc_channel, 50);

  adc_set_prescaler(ADC_PRE_PCLK2_DIV_2);
  adc_set_sample_rate(ADC1, ADC_SMPR_7_5);

  GPIOC->regs->CRL = 0x00000000;
  GPIOC->regs->CRH = 0x33300000;
}

void readCCD(void)
{
  uint16_t result;
  BITCLR(ICG);
  delayMicroseconds(1);
  BITSET(SH);  
  delayMicroseconds(5);
  BITCLR(SH);
  delayMicroseconds(15);
  BITSET(ICG);
  delayMicroseconds(1);

  // analogRead() preparation work
  adc_dev *a_dev = PIN_MAP[ADC_PIN].adc_device;
  adc_reg_map *adc_regs = a_dev->regs;
  adc_set_reg_seqlen(a_dev, 1);
  adc_regs->SQR3 = PIN_MAP[ADC_PIN].adc_channel;

  for (int x = 0; x < PIXEL_COUNT; x++)
  {
    BITSET(SH);

    //result = (uint8_t)(analogRead(PA7) >> 2);   
	  adc_regs->CR2 |= ADC_CR2_SWSTART;
	  while (!(adc_regs->SR & ADC_SR_EOC))
            ;
	  result = (uint16_t)(adc_regs->DR & ADC_DR_DATA);

    // if (x == 0)
    // {
    //   avg = result;
    // }
    // else
    // {
    //   // TODO data is inverted when we connect ADC directly
    //   if (result > avg)
    //   {
    //     result = 0;
    //   }
    //   else
    //   {
    //     result += avg;
    //   }
    // }

    buffer[x] = result;
    delayMicroseconds(5);

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