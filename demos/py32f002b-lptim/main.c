/**
  ******************************************************************************
  * PY32F002B Deep Stop Wake Up By LPTIM(LSI)
  *
  * Connections
  *  PB5      --->    - LED +   ---> 3.3V
  *  VCC      ---> 3.3V
  *  GND      ---> GND
  *
  * - MCU enters deep stop mode after 3 seconds,
  * - LED blinks every 5 seconds (In low voltage LSI becomes slower and slower, the
  *   interval will be longer than expected)
  * - Current consumption is around 0.6uA in deep stop mode
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define RST_WAITTIME          (300)        //seconds
#define QUARTER_SECOND_LOAD   (256-1)
#define RST_COUNT_MOD         (RST_WAITTIME * 4)

LL_LPTIM_InitTypeDef LPTIM_InitStruct = {0};
__IO uint32_t RatioNops = 0;

static void APP_GPIO_Config(void);
static void APP_LPTIM_Config(void);
static void APP_EnterDeepStop(void);

static void APP_uDelay(uint32_t us)
{
  uint32_t temp;
  SysTick->LOAD=us*(SystemCoreClock/1000000);
  SysTick->VAL=0x00;
  SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;

  do{
    temp=SysTick->CTRL;
  }while((temp&0x01)&&!(temp&(1<<16)));

  SysTick->CTRL=SysTick_CTRL_ENABLE_Msk;
  SysTick->VAL =0x00;
}

static void APP_SystemClockConfig(void)
{
  /* Enable HSI */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  /* Set AHB divider: HCLK = SYSCLK */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* HSISYS used as SYSCLK clock source  */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS)
  {
  }

  /* Set APB1 divider */
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_Init1msTick(24000000);

  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(24000000);
}

int main(void)
{
  static uint32_t counter = 0;
  /* Configure HSI as Systemclock source */
  APP_SystemClockConfig();
  APP_GPIO_Config();

  /* Hold 3 seconds for flash download */
  LL_mDelay(3000);

  APP_LPTIM_Config();

  while (1)
  {
    counter++;
    if(counter % RST_COUNT_MOD == 0){
      LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4);
      LL_mDelay(220);
      LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_4);
    }

    /* LPTIM must be disabled to restore internal state before next time enter stop mode */
    LL_LPTIM_Disable(LPTIM);

    /* Enable LPTIM */
    LL_LPTIM_Enable(LPTIM);

    /* Wait at least three LSI times for the completion of the disable operation */
    APP_uDelay(120);

    /* Set auto-reload value */
    LL_LPTIM_SetAutoReload(LPTIM, QUARTER_SECOND_LOAD);

    /* Start in once mode */
    LL_LPTIM_StartCounter(LPTIM, LL_LPTIM_OPERATING_MODE_ONESHOT);

    /* Enable DEEP STOP mode */
    APP_EnterDeepStop();
  }
}

static void APP_GPIO_Config(void)
{
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  /* PB5 as output */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
  // LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_4, LL_GPIO_OUTPUT_PUSHPULL);
  /* Turn LED off */
  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_4);
}

static void APP_LPTIM_Config(void)
{
  /* Disable LES otherwise it will consume extra 0.6 uA */
  LL_RCC_LSE_Disable();

  /* Enable LSI */
  LL_RCC_LSI_SetCalibTrimming(LL_RCC_LSICALIBRATION_32768Hz);
  LL_RCC_LSI_Enable();
  while(LL_RCC_LSI_IsReady() != 1);

  /* Set LSI as LTPIM clock source */
  LL_RCC_SetLPTIMClockSource(LL_RCC_LPTIM1_CLKSOURCE_LSI);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_LPTIM1);
  while(LL_APB1_GRP1_IsEnabledClock(LL_APB1_GRP1_PERIPH_LPTIM1) != 1);

  // while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_LSI);
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);


  /* prescaler: 32 */
  LL_LPTIM_SetPrescaler(LPTIM, LL_LPTIM_PRESCALER_DIV32);
  // LL_LPTIM_SetUpdateMode(LPTIM, LL_LPTIM_UPDATE_MODE_IMMEDIATE);
  LL_LPTIM_SetUpdateMode(LPTIM, LL_LPTIM_UPDATE_MODE_ENDOFPERIOD);


  /* Enable LPTIM1 interrupt */
  NVIC_SetPriority(LPTIM1_IRQn, 0);
  NVIC_EnableIRQ(LPTIM1_IRQn);

  /* Enable LPTIM autoreload match interrupt  */
  LL_LPTIM_EnableIT_ARRM(LPTIM);


}

static void APP_EnterDeepStop(void)
{
  /* Enable PWR clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  /* DEEP STOP mode with deep low power regulator ON */
  LL_PWR_SetLprMode(LL_PWR_LPR_MODE_DLPR);
  /* SRAM retention voltage aligned with digital LDO output */
  LL_PWR_SetStopModeSramVoltCtrl(LL_PWR_SRAM_RETENTION_VOLT_CTRL_LDO);
  /* Enter STOP mode */
  LL_LPM_EnableDeepSleep();
  /* Wait For interrupt */
   __WFI();

   LL_LPM_EnableSleep();
}

/**
  * LPTIM interrupt callback
  */
void APP_LptimIRQCallback(void)
{
  if((LL_LPTIM_IsActiveFlag_ARRM(LPTIM) == 1) && (LL_LPTIM_IsEnabledIT_ARRM(LPTIM) == 1))
  {
    /* Clear autoreload match flag */
    LL_LPTIM_ClearFLAG_ARRM(LPTIM);
  }
}

void APP_ErrorHandler(void)
{
  while (1);
}
