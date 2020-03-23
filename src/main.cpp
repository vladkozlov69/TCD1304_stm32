#include <Arduino.h>

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
int exposureTime = 10
;


void setup()
{
  pinMode(CLK_PIN, OUTPUT);
  pinMode(ADC_PIN, INPUT_ANALOG);

  analogWrite(CLK_PIN, 50);

  PinName p = digitalPinToPinName(CLK_PIN);

  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(p, PinMap_PWM);
  HardwareTimer *HT;
  uint32_t index = get_timer_index(Instance);
  if (HardwareTimer_Handle[index] == NULL) 
  {
    HardwareTimer_Handle[index]->__this = new HardwareTimer((TIM_TypeDef *)pinmap_peripheral(p, PinMap_PWM));
  }

  HT = (HardwareTimer *)(HardwareTimer_Handle[index]->__this);

  uint32_t channel = STM_PIN_CHANNEL(pinmap_function(p, PinMap_PWM));

  HT->setPrescaleFactor(1);
  HT->setMode(channel, TIMER_OUTPUT_COMPARE_PWM1, p);
  HT->setOverflow(100, TICK_FORMAT);
  HT->setCaptureCompare(channel, 50, TICK_COMPARE_FORMAT);
  HT->resume();

  // timer_dev *t_dev = PIN_MAP[CLK_PIN].timer_device;
  // uint8_t cc_channel = PIN_MAP[CLK_PIN].timer_channel;

  // timer_set_prescaler(t_dev, (uint16_t)(0));
  // timer_set_reload(t_dev, 100);
  // timer_set_compare(t_dev, cc_channel, 50);

  // adc_set_prescaler(ADC_PRE_PCLK2_DIV_2);
  // adc_set_sample_rate(ADC1, ADC_SMPR_7_5);

  pinMode(PC14, OUTPUT);
  pinMode(PC15, OUTPUT);
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

  // analogRead() preparation work
  // adc_dev *a_dev = PIN_MAP[ADC_PIN].adc_device;
  // adc_reg_map *adc_regs = a_dev->regs;
  // adc_set_reg_seqlen(a_dev, 1);
  // adc_regs->SQR3 = PIN_MAP[ADC_PIN].adc_channel;

  for (int x = 0; x < PIXEL_COUNT; x++)
  {
    BITSET_SH;

    result = analogRead(ADC_PIN);
    //result = (uint8_t)(analogRead(PA7) >> 2);   
	  // adc_regs->CR2 |= ADC_CR2_SWSTART;
	  // while (!(adc_regs->SR & ADC_SR_EOC))
    //         ;
	  // result = (uint16_t)(adc_regs->DR & ADC_DR_DATA);

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