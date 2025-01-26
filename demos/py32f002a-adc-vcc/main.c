/**
 * Demo of internal temperature sensor
 * 
 * - If the internal sensor reading is always 4095(0xFFF), disconnect the UART RX(A3) pin
 * 
*/
#include "main.h"
#include "py32f0xx_bsp_clock.h"

#define ESP_RST_PIN                       (LL_GPIO_PIN_4)
#define ADC_CHANNEL_NUM                   (1)
#define STOP_SECONDS                      (3)
#define SWITCH_OFF                        (0)
#define SWITCH_ON                         (1)
#define VOLTAGE_TOOHIGH                   (2700)
#define VOLTAGE_ON                        (2300)
#define VOLTAGE_OFF                       (2000)

uint16_t adc_values[ADC_CHANNEL_NUM];
uint16_t switch_status = SWITCH_OFF;      // 0 off, 1 on

static void APP_InitADC(void);
static void APP_InitGPIO(void);
static void APP_InitLPTIM(void);
static void APP_EnterStop(void);

int main(void)
{
  // Set system clock to 24MHz
  BSP_RCC_HSI_24MConfig();
  LL_mDelay(1000);

  APP_InitGPIO();
  APP_InitADC();
  APP_InitLPTIM();

  while (1)
  {
    LL_ADC_Enable(ADC1);
    LL_mDelay(1);
    LL_ADC_ClearFlag_EOC(ADC1);
    LL_ADC_ClearFlag_EOS(ADC1);
    LL_ADC_REG_StartConversion(ADC1);

    for(int i=0; i < ADC_CHANNEL_NUM; i++){
      while (LL_ADC_IsActiveFlag_EOC(ADC1) == 0);
      LL_ADC_ClearFlag_EOC(ADC1);
      adc_values[i] = LL_ADC_REG_ReadConversionData12(ADC1);
    }

    while(LL_ADC_IsActiveFlag_EOS(ADC1) == 0);
    LL_ADC_ClearFlag_EOS(ADC1);

    LL_ADC_REG_StopConversion(ADC1);
    while(LL_ADC_REG_IsConversionOngoing(ADC1) != 0);
    LL_ADC_Disable(ADC1);

    uint32_t voltage = __LL_ADC_CALC_VREFANALOG_VOLTAGE(adc_values[0], LL_ADC_RESOLUTION_12B);

    if(voltage > VOLTAGE_ON){
      switch_status = SWITCH_ON;
    }else if(switch_status == SWITCH_ON && voltage < VOLTAGE_OFF){
      switch_status = SWITCH_OFF;
    }

    if(switch_status == SWITCH_ON){
      if(voltage > VOLTAGE_TOOHIGH){
        for(int i = 0; i < 5; i++){
          LL_GPIO_TogglePin(GPIOA, ESP_RST_PIN);
          LL_mDelay(50);
        }
      }
      LL_GPIO_SetOutputPin(GPIOA, ESP_RST_PIN);
    }else{
      LL_GPIO_ResetOutputPin(GPIOA, ESP_RST_PIN);
    }

    // enter stop model
    for(int i=0; i<STOP_SECONDS; i++)
      APP_EnterStop();
  }
}

static void APP_InitADC(void)
{
  LL_ADC_InitTypeDef ADC_Init;
  LL_ADC_REG_InitTypeDef LL_ADC_REG_InitType;

  LL_ADC_Reset(ADC1);

  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);

  // Calibrate start
  if (LL_ADC_IsEnabled(ADC1) == 0)
  {
    LL_ADC_StartCalibration(ADC1);
    while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0);
    /* Delay 1ms(>= 4 ADC clocks) before re-enable ADC */
    LL_mDelay(1);
  }
  // Calibrate end

  ADC_Init.Clock = LL_ADC_CLOCK_SYNC_PCLK_DIV64;
  ADC_Init.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_Init.LowPowerMode = LL_ADC_LP_MODE_NONE;
  ADC_Init.Resolution = LL_ADC_RESOLUTION_12B;
  LL_ADC_Init(ADC1, &ADC_Init);
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_71CYCLES_5);

  // Regular ADC config
  LL_ADC_REG_InitType.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS;
  LL_ADC_REG_InitType.Overrun = LL_ADC_REG_OVR_DATA_OVERWRITTEN;
  LL_ADC_REG_InitType.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  LL_ADC_REG_InitType.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  LL_ADC_REG_Init(ADC1, &LL_ADC_REG_InitType);

  // Set common path and internal channel
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_VREFINT);

  // Select temperature sensor
  LL_ADC_REG_SetSequencerChannels(ADC1, LL_ADC_CHANNEL_VREFINT);
}

static void APP_InitGPIO(void)
{
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  LL_GPIO_SetPinMode(GPIOA, ESP_RST_PIN, LL_GPIO_MODE_OUTPUT);
  // LL_GPIO_SetOutputPin(GPIOA, ESP_RST_PIN);
  LL_GPIO_ResetOutputPin(GPIOA, ESP_RST_PIN);
}

static void APP_InitLPTIM(void)
{

  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_LPTIM1);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  LL_RCC_LSI_Enable();
  while(LL_RCC_LSI_IsReady() == 0);
  // Set LSI as LPTIM1 clock source
  LL_RCC_SetLPTIMClockSource(LL_RCC_LPTIM1_CLKSOURCE_LSI);

  // Prescaler = 128
  LL_LPTIM_SetPrescaler(LPTIM1, LL_LPTIM_PRESCALER_DIV128);
  LL_LPTIM_SetUpdateMode(LPTIM1, LL_LPTIM_UPDATE_MODE_ENDOFPERIOD);
  LL_LPTIM_EnableIT_ARRM(LPTIM1);
  LL_LPTIM_Enable(LPTIM1);

  // 32768/128=256
  LL_LPTIM_SetAutoReload(LPTIM1, 255); /*one-second*/
  // LL_LPTIM_StartCounter(LPTIM1, LL_LPTIM_OPERATING_MODE_ONESHOT);

  NVIC_EnableIRQ(LPTIM1_IRQn);
  NVIC_SetPriority(LPTIM1_IRQn, 0);
}

static void APP_EnterStop(void){
  LL_PWR_EnableLowPowerRunMode();
  LL_LPTIM_Disable(LPTIM1);
  LL_LPTIM_Enable(LPTIM1);
  LL_mDelay(1);

  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE2);
  LL_PWR_SetSramRetentionVolt(LL_PWR_SRAM_RETENTION_VOLT_0p9);

  LL_LPTIM_StartCounter(LPTIM1, LL_LPTIM_OPERATING_MODE_ONESHOT);
  LL_LPM_EnableDeepSleep();
  __WFI();
}

void APP_ErrorHandler(void)
{
  while (1);
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  while (1);
}
#endif /* USE_FULL_ASSERT */
