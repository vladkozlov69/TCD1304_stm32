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

ADC_HandleTypeDef AdcHandle = {};
ADC_ChannelConfTypeDef  AdcChannelConf = {};

static uint32_t get_adc_internal_channel(PinName pin)
{
  uint32_t channel = 0;
  switch (pin) {
#if defined(ADC_CHANNEL_TEMPSENSOR)
    case PADC_TEMP:
      channel = ADC_CHANNEL_TEMPSENSOR;
      break;
#endif
#if defined(ADC_CHANNEL_TEMPSENSOR_ADC1)
    case PADC_TEMP:
      channel = ADC_CHANNEL_TEMPSENSOR_ADC1;
      break;
#endif
#if defined(ADC5) && defined(ADC_CHANNEL_TEMPSENSOR_ADC5)
    case PADC_TEMP_ADC5:
      channel = ADC_CHANNEL_TEMPSENSOR_ADC5;
      break;
#endif
#ifdef ADC_CHANNEL_VREFINT
    case PADC_VREF:
      channel = ADC_CHANNEL_VREFINT;
      break;
#endif
#ifdef ADC_CHANNEL_VBAT
    case PADC_VBAT:
      channel = ADC_CHANNEL_VBAT;
      break;
#endif
    default:
      channel = 0;
      break;
  }
  return channel;
}

static uint32_t get_adc_channel(PinName pin)
{
  uint32_t function = pinmap_function(pin, PinMap_ADC);
  uint32_t channel = 0;
  switch (STM_PIN_CHANNEL(function)) {
#ifdef ADC_CHANNEL_0
    case 0:
      channel = ADC_CHANNEL_0;
      break;
#endif
    case 1:
      channel = ADC_CHANNEL_1;
      break;
    case 2:
      channel = ADC_CHANNEL_2;
      break;
    case 3:
      channel = ADC_CHANNEL_3;
      break;
    case 4:
      channel = ADC_CHANNEL_4;
      break;
    case 5:
      channel = ADC_CHANNEL_5;
      break;
    case 6:
      channel = ADC_CHANNEL_6;
      break;
    case 7:
      channel = ADC_CHANNEL_7;
      break;
    case 8:
      channel = ADC_CHANNEL_8;
      break;
    case 9:
      channel = ADC_CHANNEL_9;
      break;
    case 10:
      channel = ADC_CHANNEL_10;
      break;
    case 11:
      channel = ADC_CHANNEL_11;
      break;
    case 12:
      channel = ADC_CHANNEL_12;
      break;
    case 13:
      channel = ADC_CHANNEL_13;
      break;
    case 14:
      channel = ADC_CHANNEL_14;
      break;
    case 15:
      channel = ADC_CHANNEL_15;
      break;
#ifdef ADC_CHANNEL_16
    case 16:
      channel = ADC_CHANNEL_16;
      break;
#endif
    case 17:
      channel = ADC_CHANNEL_17;
      break;
#ifdef ADC_CHANNEL_18
    case 18:
      channel = ADC_CHANNEL_18;
      break;
#endif
#ifdef ADC_CHANNEL_19
    case 19:
      channel = ADC_CHANNEL_19;
      break;
#endif
#ifdef ADC_CHANNEL_20
    case 20:
      channel = ADC_CHANNEL_20;
      break;
    case 21:
      channel = ADC_CHANNEL_21;
      break;
    case 22:
      channel = ADC_CHANNEL_22;
      break;
    case 23:
      channel = ADC_CHANNEL_23;
      break;
    case 24:
      channel = ADC_CHANNEL_24;
      break;
    case 25:
      channel = ADC_CHANNEL_25;
      break;
    case 26:
      channel = ADC_CHANNEL_26;
      break;
#ifdef ADC_CHANNEL_27
    case 27:
      channel = ADC_CHANNEL_27;
      break;
    case 28:
      channel = ADC_CHANNEL_28;
      break;
    case 29:
      channel = ADC_CHANNEL_29;
      break;
    case 30:
      channel = ADC_CHANNEL_30;
      break;
    case 31:
      channel = ADC_CHANNEL_31;
      break;
#endif
#endif
    default:
      channel = 0;
      break;
  }
  return channel;
}

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

  PinName adcPin = analogInputToPinName(ADC_PIN);

  uint32_t samplingTime = ADC_SAMPLETIME_3CYCLES;
  uint32_t adcChannel = 0;

  if (adcPin & PADC_BASE) {
#if defined(STM32H7xx)
    AdcHandle.Instance = ADC3;
#else
    AdcHandle.Instance = ADC1;
#if defined(ADC5) && defined(ADC_CHANNEL_TEMPSENSOR_ADC5)
    if (adcPin == PADC_TEMP_ADC5) {
      AdcHandle.Instance = ADC5;
    }
#endif
#endif
    adcChannel = get_adc_internal_channel(adcPin);
    samplingTime = ADC_SAMPLETIME_3CYCLES;
  } else {
    AdcHandle.Instance = (ADC_TypeDef *)pinmap_peripheral(adcPin, PinMap_ADC);
    adcChannel = get_adc_channel(adcPin);
  }

  if (AdcHandle.Instance == NP) {
    return;
  }

#ifdef ADC_CLOCK_DIV
  AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_DIV;                 /* (A)synchronous clock mode, input ADC clock divided */
#endif
#ifdef ADC_RESOLUTION_12B
  AdcHandle.Init.Resolution            = ADC_RESOLUTION_12B;            /* 12-bit resolution for converted data */
#endif
#ifdef ADC_DATAALIGN_RIGHT
  AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;           /* Right-alignment for converted data */
#endif
  AdcHandle.Init.ScanConvMode          = DISABLE;                       /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
#ifdef ADC_EOC_SINGLE_CONV
  AdcHandle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;           /* EOC flag picked-up to indicate conversion end */
#endif
#if !defined(STM32F1xx) && !defined(STM32F2xx) && !defined(STM32F4xx) && \
    !defined(STM32F7xx) && !defined(STM32F373xC) && !defined(STM32F378xx)
  AdcHandle.Init.LowPowerAutoWait      = DISABLE;                       /* Auto-delayed conversion feature disabled */
#endif
#if !defined(STM32F1xx) && !defined(STM32F2xx) && !defined(STM32F3xx) && \
    !defined(STM32F4xx) && !defined(STM32F7xx) && !defined(STM32G4xx) && \
    !defined(STM32H7xx) && !defined(STM32L4xx) && !defined(STM32WBxx)
  AdcHandle.Init.LowPowerAutoPowerOff  = DISABLE;                       /* ADC automatically powers-off after a conversion and automatically wakes-up when a new conversion is triggered */
#endif
#ifdef ADC_CHANNELS_BANK_A
  AdcHandle.Init.ChannelsBank          = ADC_CHANNELS_BANK_A;
#endif
  AdcHandle.Init.ContinuousConvMode    = DISABLE;                       /* Continuous mode disabled to have only 1 conversion at each conversion trig */
#if !defined(STM32F0xx) && !defined(STM32L0xx)
  AdcHandle.Init.NbrOfConversion       = 1;                             /* Specifies the number of ranks that will be converted within the regular group sequencer. */
#endif
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;                       /* Parameter discarded because sequencer is disabled */
#if !defined(STM32F0xx) && !defined(STM32G0xx) && !defined(STM32L0xx)
  AdcHandle.Init.NbrOfDiscConversion   = 0;                             /* Parameter discarded because sequencer is disabled */
#endif
  AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */
#if !defined(STM32F1xx)
  AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE; /* Parameter discarded because software trigger chosen */
#endif
#if !defined(STM32F1xx) && !defined(STM32H7xx) && \
    !defined(STM32F373xC) && !defined(STM32F378xx)
  AdcHandle.Init.DMAContinuousRequests = DISABLE;                       /* DMA one-shot mode selected (not applied to this example) */
#endif
#ifdef ADC_CONVERSIONDATA_DR
  AdcHandle.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;      /* Regular Conversion data stored in DR register only */
#endif
#ifdef ADC_OVR_DATA_OVERWRITTEN
  AdcHandle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;      /* DR register is overwritten with the last conversion result in case of overrun */
#endif
#ifdef ADC_LEFTBITSHIFT_NONE
  AdcHandle.Init.LeftBitShift          = ADC_LEFTBITSHIFT_NONE;         /* No bit shift left applied on the final ADC convesion data */
#endif

#if defined(STM32F0xx)
  AdcHandle.Init.SamplingTimeCommon    = samplingTime;
#endif
#if defined(STM32G0xx)
  AdcHandle.Init.SamplingTimeCommon1   = samplingTime;              /* Set sampling time common to a group of channels. */
  AdcHandle.Init.SamplingTimeCommon2   = samplingTime;              /* Set sampling time common to a group of channels, second common setting possible.*/
  AdcHandle.Init.TriggerFrequencyMode  = ADC_TRIGGER_FREQ_HIGH;
#endif
#if defined(STM32L0xx)
  AdcHandle.Init.LowPowerFrequencyMode = DISABLE;                       /* To be enabled only if ADC clock < 2.8 MHz */
  AdcHandle.Init.SamplingTime          = samplingTime;
#endif
#if !defined(STM32F0xx) && !defined(STM32F1xx) && !defined(STM32F2xx) && \
    !defined(STM32F3xx) && !defined(STM32F4xx) && !defined(STM32F7xx) && \
    !defined(STM32L1xx)
  AdcHandle.Init.OversamplingMode      = DISABLE;
  /* AdcHandle.Init.Oversample ignore for STM32L0xx as oversampling disabled */
  /* AdcHandle.Init.Oversampling ignored for other as oversampling disabled */
#endif
#if defined(ADC_CFGR_DFSDMCFG) && defined(DFSDM1_Channel0)
  AdcHandle.Init.DFSDMConfig           = ADC_DFSDM_MODE_DISABLE;        /* ADC conversions are not transferred by DFSDM. */
#endif
#ifdef ADC_TRIGGER_FREQ_HIGH
  AdcHandle.Init.TriggerFrequencyMode  = ADC_TRIGGER_FREQ_HIGH;
#endif

  AdcHandle.State = HAL_ADC_STATE_RESET;
  AdcHandle.DMA_Handle = NULL;
  AdcHandle.Lock = HAL_UNLOCKED;
  /* Some other ADC_HandleTypeDef fields exists but not required */

  // g_current_pin = pin; /* Needed for HAL_ADC_MspInit*/

  if (HAL_ADC_Init(&AdcHandle) != HAL_OK) {
    return;
  }

  AdcChannelConf.Channel      = adcChannel;                          /* Specifies the channel to configure into ADC */

#if defined(STM32L4xx) || defined(STM32WBxx)
  if (!IS_ADC_CHANNEL(&AdcHandle, AdcChannelConf.Channel)) {
#elif defined(STM32G4xx)
  if (!IS_ADC_CHANNEL(&AdcHandle, AdcChannelConf.Channel)) {
#else
  if (!IS_ADC_CHANNEL(AdcChannelConf.Channel)) {
#endif /* STM32L4xx || STM32WBxx */
    return;
  }
  ///AdcChannelConf.Rank         = ADC_REGULAR_RANK_1;               /* Specifies the rank in the regular group sequencer */
#if !defined(STM32L0xx)
#if !defined(STM32G0xx)
  AdcChannelConf.SamplingTime = samplingTime;                     /* Sampling time value to be set for the selected channel */
#else
  AdcChannelConf.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;        /* Sampling time value to be set for the selected channel */
#endif
#endif
#if !defined(STM32F0xx) && !defined(STM32F1xx) && !defined(STM32F2xx) && \
    !defined(STM32F4xx) && !defined(STM32F7xx) && !defined(STM32G0xx) && \
    !defined(STM32L0xx) && !defined(STM32L1xx) && \
    !defined(STM32F373xC) && !defined(STM32F378xx)
  AdcChannelConf.SingleDiff   = ADC_SINGLE_ENDED;                 /* Single-ended input channel */
  AdcChannelConf.OffsetNumber = ADC_OFFSET_NONE;                  /* No offset subtraction */
#endif
#if !defined(STM32F0xx) && !defined(STM32F1xx) && !defined(STM32F2xx) && \
    !defined(STM32G0xx) && !defined(STM32L0xx) && !defined(STM32L1xx) && \
    !defined(STM32WBxx) && !defined(STM32F373xC) && !defined(STM32F378xx)
  AdcChannelConf.Offset = 0;                                      /* Parameter discarded because offset correction is disabled */
#endif
#if defined (STM32H7xx)
  AdcChannelConf.OffsetRightShift = DISABLE;                      /* No Right Offset Shift */
  AdcChannelConf.OffsetSignedSaturation = DISABLE;                /* Signed saturation feature is not used */
#endif

  /*##-2- Configure ADC regular channel ######################################*/
  if (HAL_ADC_ConfigChannel(&AdcHandle, &AdcChannelConf) != HAL_OK) {
    /* Channel Configuration Error */
    return;
  }

#if defined(STM32F0xx) || defined(STM32F1xx) || defined(STM32F3xx) || \
    defined(STM32G0xx) || defined(STM32G4xx) || defined(STM32H7xx) || \
    defined(STM32L0xx) || defined(STM32L4xx) || defined(STM32WBxx)
  /*##-2.1- Calibrate ADC then Start the conversion process ####################*/
#if defined(STM32F0xx) || defined(STM32G0xx) || defined(STM32F1xx) || \
    defined(STM32F373xC) || defined(STM32F378xx)
  if (HAL_ADCEx_Calibration_Start(&AdcHandle) !=  HAL_OK)
#elif defined (STM32H7xx)
  if (HAL_ADCEx_Calibration_Start(&AdcHandle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
#else
  if (HAL_ADCEx_Calibration_Start(&AdcHandle, ADC_SINGLE_ENDED) !=  HAL_OK)
#endif
  {
    /* ADC Calibration Error */
    return;
  }
#endif



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

  __IO uint32_t counter = 0U;


  uint32_t cr2 = ADC1->CR2;
  cr2 &= ~ADC_CR2_EXTSEL;
  cr2 |= ADC_CR2_SWSTART;
  cr2 |= (1 << 20);
  ADC1->CR2 = cr2;

  if((AdcHandle.Instance->CR2 & ADC_CR2_ADON) != ADC_CR2_ADON)
  {
    /* Enable the Peripheral */
    __HAL_ADC_ENABLE(&AdcHandle);

    /* Delay for ADC stabilization time */
    /* Compute number of CPU cycles to wait for */
    counter = (ADC_STAB_DELAY_US * (SystemCoreClock / 1000000U));
    while(counter != 0U)
    {
      counter--;
    }
  }

    // set ADC1 sample rate ADC_SMPR_7_5
  uint32_t adc_smpr1_val = 0, adc_smpr2_val = 0;
  for (int i = 0; i < 10; i++) {
    if (i < 8) {
      /* ADC_SMPR1 determines sample time for channels [10,17] */
      adc_smpr1_val |= ADC_SAMPLETIME_15CYCLES << (i * 3);
    }
    /* ADC_SMPR2 determines sample time for channels [0,9] */
    adc_smpr2_val |= ADC_SAMPLETIME_15CYCLES << (i * 3);
  }

  // ADC1->SMPR1 = adc_smpr1_val;
  // ADC1->SMPR2 = adc_smpr2_val;

  // First, set the number of channels to read during each sequence.
// (# of channels = L + 1, so set L to 0)
ADC1->SQR1  &= ~( ADC_SQR1_L );
// Configure the first (and only) step in the sequence to read channel 6.
ADC1->SQR1  &= ~( 0x1F << 7 );
ADC1->SQR1  |=  ( 7 << 6 );
// Configure the sampling time to 640.5 cycles.
ADC1->SMPR1 &= ~( 0x7 << ( 6 * 3 ) );
ADC1->SMPR1 |=  ( 0x7 << ( 6 * 3 ) );

   ADC1->SQR3 = 7;

  uint32_t started = micros();

  for (int cnt = 0; cnt < 1000; cnt++)
  {
    BITSET_SH;
  	AdcHandle.Instance->CR2 |= ADC_CR2_SWSTART;

    if (HAL_ADC_PollForConversion(&AdcHandle, 10) != HAL_OK) {
      /* End Of Conversion flag not set on time */
      return;
    }
	  // while (!(AdcHandle.Instance->SR & ADC_SR_EOC))
    //         ;
	  uhADCxConvertedValue = (uint16_t)(AdcHandle.Instance->DR & ADC_DR_DATA);
    BITCLR_SH;

  } 

  uint32_t ended = micros() - started;

  if (Serial)
    Serial.println(ended);
}