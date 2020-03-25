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
  // if (millis() - copyTimer > 2000)
  // {
  //   for (int i = 0; i< 36; ++i)
  //   {
  //     buffer2[i] = buffer[i*100];
  //   }
  //   copyTimer = millis();
  // }

  // readCCD();
  //delay(exposureTime);

      




  __IO uint16_t uhADCxConvertedValue = 0;
  /*##-3- Start the conversion process ####################*/
  // if (HAL_ADC_Start(&AdcHandle) != HAL_OK) {
  //   /* Start Conversation Error */
  //   return;
  // }
  // analogRead(ADC_PIN);

  // __IO uint32_t counter = 0U;


  // uint32_t cr2 = ADC1->CR2;
  // cr2 &= ~ADC_CR2_EXTSEL;
  // cr2 |= ADC_CR2_SWSTART;
  // cr2 |= (1 << 20);
  // ADC1->CR2 = cr2;

  // if((AdcHandle.Instance->CR2 & ADC_CR2_ADON) != ADC_CR2_ADON)
  // {
  //   /* Enable the Peripheral */
  //   __HAL_ADC_ENABLE(&AdcHandle);

  //   /* Delay for ADC stabilization time */
  //   /* Compute number of CPU cycles to wait for */
  //   counter = (ADC_STAB_DELAY_US * (SystemCoreClock / 1000000U));
  //   while(counter != 0U)
  //   {
  //     counter--;
  //   }
  // }

    // calibrate for F103 ??? TODO really need?done above...
  // ADC1->CR2 |= ADC_CR2_RSTCAL; //Initialize calibration register
  // while ((ADC1->CR2 >> ADC_CR2_RSTCAL_Pos) & 0x1UL); //Wait until calibration register is initialized
  // ADC1->CR2 |= ADC_CR2_CAL; //Enable calibration
  // while ((ADC1->CR2 >> ADC_CR2_CAL_Pos) & 0x1UL); //Wait until calibration completed

  //Configure the sampling time to 640.5 cycles.
  // uint32_t adc_smpr1_val = 0, adc_smpr2_val = 0;
  // for (int i = 0; i < 10; i++) {
  //   if (i < 8) {
  //     /* ADC_SMPR1 determines sample time for channels [10,17] */
  //     adc_smpr1_val |= ADC_SAMPLETIME_13CYCLES_5 << (i * 3);
  //   }
  //   /* ADC_SMPR2 determines sample time for channels [0,9] */
  //   adc_smpr2_val |= ADC_SAMPLETIME_13CYCLES_5 << (i * 3);
  // }

  // ADC1->SMPR1 = adc_smpr1_val;
  // ADC1->SMPR2 = adc_smpr2_val;
  //--------------------------------------------------

// TODO what is SQR1 in this context? 
  // First, set the number of channels to read during each sequence.
// (# of channels = L + 1, so set L to 0)
// ADC1->SQR1  &= ~( ADC_SQR1_L );
// // Configure the first (and only) step in the sequence to read channel 6.
// ADC1->SQR1  &= ~( 0x1F << 7 );
// ADC1->SQR1  |=  ( 7 << 6 );


  

  uint32_t started = micros();

  for (int cnt = 0; cnt < 1000; cnt++)
  {
    BITSET_SH;

    uhADCxConvertedValue = fastADC.read();

    // ADC1->CR2 |= (ADC_CR2_SWSTART | ADC_CR2_EXTTRIG);
    // while (!(ADC1->SR & ADC_SR_EOC))
    //     ;
    // uhADCxConvertedValue =  (uint16_t)(ADC1->DR & ADC_DR_DATA);
    
    BITCLR_SH;
  } 

  uint32_t ended = micros() - started;

  if (Serial && ended > 10000)
  {
    Serial.println(ended);
    started = 0;
  }
}