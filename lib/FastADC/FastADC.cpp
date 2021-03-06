#include "FastADC.h"

void FastADC::begin()
{
    PinName adcPin = analogInputToPinName(m_Pin);

    uint32_t samplingTime = ADC_SAMPLE_TIME;
    uint32_t adcChannel = 0;

    if (adcPin & PADC_BASE) 
    {
#if defined(STM32H7xx)
        m_AdcHandle.Instance = ADC3;
#else
        m_AdcHandle.Instance = ADC1;
#if defined(ADC5) && defined(ADC_CHANNEL_TEMPSENSOR_ADC5)
        if (adcPin == PADC_TEMP_ADC5) 
        {
            m_AdcHandle.Instance = ADC5;
        }
#endif
#endif
        adcChannel = getAdcInternalChannel(adcPin);
        samplingTime = ADC_SAMPLE_TIME;
    } 
    else 
    {
        m_AdcHandle.Instance = (ADC_TypeDef *)pinmap_peripheral(adcPin, PinMap_ADC);
        adcChannel = getAdcChannel(adcPin);
    }

    if (m_AdcHandle.Instance == NP) 
    {
        return;
    }

#ifdef ADC_CLOCK_DIV
    m_AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_DIV;                 /* (A)synchronous clock mode, input ADC clock divided */
#endif
#ifdef ADC_RESOLUTION_12B
    m_AdcHandle.Init.Resolution            = ADC_RESOLUTION_12B;            /* 12-bit resolution for converted data */
#endif
#ifdef ADC_DATAALIGN_RIGHT
    m_AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;           /* Right-alignment for converted data */
#endif
    m_AdcHandle.Init.ScanConvMode          = DISABLE;                       /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
#ifdef ADC_EOC_SINGLE_CONV
    m_AdcHandle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;           /* EOC flag picked-up to indicate conversion end */
#endif
#if !defined(STM32F1xx) && !defined(STM32F2xx) && !defined(STM32F4xx) && \
    !defined(STM32F7xx) && !defined(STM32F373xC) && !defined(STM32F378xx)
    m_AdcHandle.Init.LowPowerAutoWait      = DISABLE;                       /* Auto-delayed conversion feature disabled */
#endif
#if !defined(STM32F1xx) && !defined(STM32F2xx) && !defined(STM32F3xx) && \
    !defined(STM32F4xx) && !defined(STM32F7xx) && !defined(STM32G4xx) && \
    !defined(STM32H7xx) && !defined(STM32L4xx) && !defined(STM32WBxx)
    m_AdcHandle.Init.LowPowerAutoPowerOff  = DISABLE;                       /* ADC automatically powers-off after a conversion and automatically wakes-up when a new conversion is triggered */
#endif
#ifdef ADC_CHANNELS_BANK_A
    m_AdcHandle.Init.ChannelsBank          = ADC_CHANNELS_BANK_A;
#endif
    m_AdcHandle.Init.ContinuousConvMode    = DISABLE;                       /* Continuous mode disabled to have only 1 conversion at each conversion trig */
#if !defined(STM32F0xx) && !defined(STM32L0xx)
    m_AdcHandle.Init.NbrOfConversion       = 1;                             /* Specifies the number of ranks that will be converted within the regular group sequencer. */
#endif
    m_AdcHandle.Init.DiscontinuousConvMode = DISABLE;                       /* Parameter discarded because sequencer is disabled */
#if !defined(STM32F0xx) && !defined(STM32G0xx) && !defined(STM32L0xx)
    m_AdcHandle.Init.NbrOfDiscConversion   = 0;                             /* Parameter discarded because sequencer is disabled */
#endif
    m_AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */
#if !defined(STM32F1xx)
    m_AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE; /* Parameter discarded because software trigger chosen */
#endif
#if !defined(STM32F1xx) && !defined(STM32H7xx) && \
    !defined(STM32F373xC) && !defined(STM32F378xx)
    m_AdcHandle.Init.DMAContinuousRequests = DISABLE;                       /* DMA one-shot mode selected (not applied to this example) */
#endif
#ifdef ADC_CONVERSIONDATA_DR
    m_AdcHandle.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;      /* Regular Conversion data stored in DR register only */
#endif
#ifdef ADC_OVR_DATA_OVERWRITTEN
    m_AdcHandle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;      /* DR register is overwritten with the last conversion result in case of overrun */
#endif
#ifdef ADC_LEFTBITSHIFT_NONE
    m_AdcHandle.Init.LeftBitShift          = ADC_LEFTBITSHIFT_NONE;         /* No bit shift left applied on the final ADC convesion data */
#endif

#if defined(STM32F0xx)
    m_AdcHandle.Init.SamplingTimeCommon    = samplingTime;
#endif
#if defined(STM32G0xx)
    m_AdcHandle.Init.SamplingTimeCommon1   = samplingTime;              /* Set sampling time common to a group of channels. */
    m_AdcHandle.Init.SamplingTimeCommon2   = samplingTime;              /* Set sampling time common to a group of channels, second common setting possible.*/
    m_AdcHandle.Init.TriggerFrequencyMode  = ADC_TRIGGER_FREQ_HIGH;
#endif
#if defined(STM32L0xx)
    m_AdcHandle.Init.LowPowerFrequencyMode = DISABLE;                       /* To be enabled only if ADC clock < 2.8 MHz */
    m_AdcHandle.Init.SamplingTime          = samplingTime;
#endif
#if !defined(STM32F0xx) && !defined(STM32F1xx) && !defined(STM32F2xx) && \
    !defined(STM32F3xx) && !defined(STM32F4xx) && !defined(STM32F7xx) && \
    !defined(STM32L1xx)
    m_AdcHandle.Init.OversamplingMode      = DISABLE;
    /* AdcHandle.Init.Oversample ignore for STM32L0xx as oversampling disabled */
    /* AdcHandle.Init.Oversampling ignored for other as oversampling disabled */
#endif
#if defined(ADC_CFGR_DFSDMCFG) && defined(DFSDM1_Channel0)
    m_AdcHandle.Init.DFSDMConfig           = ADC_DFSDM_MODE_DISABLE;        /* ADC conversions are not transferred by DFSDM. */
#endif
#ifdef ADC_TRIGGER_FREQ_HIGH
    m_AdcHandle.Init.TriggerFrequencyMode  = ADC_TRIGGER_FREQ_HIGH;
#endif

    m_AdcHandle.State = HAL_ADC_STATE_RESET;
    m_AdcHandle.DMA_Handle = NULL;
    m_AdcHandle.Lock = HAL_UNLOCKED;
    /* Some other ADC_HandleTypeDef fields exists but not required */

    // g_current_pin = pin; /* Needed for HAL_ADC_MspInit*/

    if (HAL_ADC_Init(&m_AdcHandle) != HAL_OK) 
    {
        return;
    }

    m_AdcChannelConf.Channel      = adcChannel;                          /* Specifies the channel to configure into ADC */

#if defined(STM32L4xx) || defined(STM32WBxx)
    if (!IS_ADC_CHANNEL(&m_AdcHandle, m_AdcChannelConf.Channel)) 
    {
#elif defined(STM32G4xx)
    if (!IS_ADC_CHANNEL(&m_AdcHandle, m_AdcChannelConf.Channel)) 
    {
#else
    if (!IS_ADC_CHANNEL(m_AdcChannelConf.Channel)) 
    {
#endif /* STM32L4xx || STM32WBxx */
        return;
    }

#if defined(STM32F1xx)    
    m_AdcChannelConf.Rank         = ADC_REGULAR_RANK_1;               /* Specifies the rank in the regular group sequencer */
#endif
#if !defined(STM32L0xx)
#if !defined(STM32G0xx)
    m_AdcChannelConf.SamplingTime = samplingTime;                     /* Sampling time value to be set for the selected channel */
#else
    m_AdcChannelConf.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;        /* Sampling time value to be set for the selected channel */
#endif
#endif
#if !defined(STM32F0xx) && !defined(STM32F1xx) && !defined(STM32F2xx) && \
    !defined(STM32F4xx) && !defined(STM32F7xx) && !defined(STM32G0xx) && \
    !defined(STM32L0xx) && !defined(STM32L1xx) && \
    !defined(STM32F373xC) && !defined(STM32F378xx)
    m_AdcChannelConf.SingleDiff   = ADC_SINGLE_ENDED;                 /* Single-ended input channel */
    m_AdcChannelConf.OffsetNumber = ADC_OFFSET_NONE;                  /* No offset subtraction */
#endif
#if !defined(STM32F0xx) && !defined(STM32F1xx) && !defined(STM32F2xx) && \
    !defined(STM32G0xx) && !defined(STM32L0xx) && !defined(STM32L1xx) && \
    !defined(STM32WBxx) && !defined(STM32F373xC) && !defined(STM32F378xx)
    m_AdcChannelConf.Offset = 0;                                      /* Parameter discarded because offset correction is disabled */
#endif
#if defined (STM32H7xx)
    m_AdcChannelConf.OffsetRightShift = DISABLE;                      /* No Right Offset Shift */
    m_AdcChannelConf.OffsetSignedSaturation = DISABLE;                /* Signed saturation feature is not used */
#endif

    /*##-2- Configure ADC regular channel ######################################*/
    if (HAL_ADC_ConfigChannel(&m_AdcHandle, &m_AdcChannelConf) != HAL_OK) 
    {
    /* Channel Configuration Error */
       return;
    }

#if defined(STM32F0xx) || defined(STM32F1xx) || defined(STM32F3xx) || \
    defined(STM32G0xx) || defined(STM32G4xx) || defined(STM32H7xx) || \
    defined(STM32L0xx) || defined(STM32L4xx) || defined(STM32WBxx)
    /*##-2.1- Calibrate ADC then Start the conversion process ####################*/
#if defined(STM32F0xx) || defined(STM32G0xx) || defined(STM32F1xx) || \
    defined(STM32F373xC) || defined(STM32F378xx)
    if (HAL_ADCEx_Calibration_Start(&m_AdcHandle) !=  HAL_OK)
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

    m_AdcHandle.Instance->SQR3 = adcChannel;

    if((m_AdcHandle.Instance->CR2 & ADC_CR2_ADON) != ADC_CR2_ADON)
    {
        __IO uint32_t counter = 0U;
        /* Enable the Peripheral */
        __HAL_ADC_ENABLE(&m_AdcHandle);

        /* Delay for ADC stabilization time */
        /* Compute number of CPU cycles to wait for */
        counter = (ADC_STAB_DELAY_US * (SystemCoreClock / 1000000U));
        while(counter != 0U)
        {
            counter--;
        }
    }

      //Configure the sampling time to 640.5 cycles.
//   uint32_t adc_smpr1_val = 0, adc_smpr2_val = 0;
//   for (int i = 0; i < 10; i++) {
//     if (i < 8) {
//       /* ADC_SMPR1 determines sample time for channels [10,17] */
//       adc_smpr1_val |= ADC_SAMPLETIME_7CYCLES_5 << (i * 3);
//     }
//     /* ADC_SMPR2 determines sample time for channels [0,9] */
//     adc_smpr2_val |= ADC_SAMPLETIME_7CYCLES_5 << (i * 3);
//   }

//   ADC1->SMPR1 = adc_smpr1_val;
//   ADC1->SMPR2 = adc_smpr2_val;
}

uint16_t FastADC::read()
{
#if defined(STM32F4xx)
    m_AdcHandle.Instance->CR2 |= ADC_CR2_SWSTART;
#else
    if (ADC_IS_SOFTWARE_START_REGULAR(&m_AdcHandle) && ADC_NONMULTIMODE_OR_MULTIMODEMASTER(&m_AdcHandle)  
        )
    {
        m_AdcHandle.Instance->CR2 |= (ADC_CR2_SWSTART | ADC_CR2_EXTTRIG);
    } 
    else
    {
        m_AdcHandle.Instance->CR2 |= ADC_CR2_SWSTART;
    }
#endif 

    while (!(m_AdcHandle.Instance->SR & ADC_SR_EOC))
        ;
    return (uint16_t)(m_AdcHandle.Instance->DR & ADC_DR_DATA);
}

uint32_t FastADC::getAdcInternalChannel(PinName pin)
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

uint32_t FastADC::getAdcChannel(PinName pin)
{
    uint32_t function = pinmap_function(pin, PinMap_ADC);
    uint32_t channel = 0;
    switch (STM_PIN_CHANNEL(function)) 
    {
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