/**
 * Demo of Low Power Timer Wakeup
*/
#include "main.h"
#define PACKAGE_SOP8
#include "py32f0xx_bsp_clock.h"
// #include "py32f0xx_bsp_printf.h"

#define HalfSecond    (256 - 1)
#define TIME_WAIT     (300*2)

#define ESP_RST_PIN   (LL_GPIO_PIN_4)

static void APP_GPIO_Config(void);
static void APP_ConfigLPTIMOneShot(void);
static void APP_uDelay(uint32_t us);

int main(void)
{
  static uint32_t count = 0;
  BSP_RCC_HSI_8MConfig();
  APP_GPIO_Config();

  LL_mDelay(2000);


  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_LPTIM1);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  APP_ConfigLPTIMOneShot();

  while (1)
  {
    count++;
    LL_PWR_EnableLowPowerRunMode();

    LL_LPTIM_Disable(LPTIM1);
    LL_LPTIM_Enable(LPTIM1);
    APP_uDelay(120);

    LL_LPTIM_StartCounter(LPTIM1, LL_LPTIM_OPERATING_MODE_ONESHOT);
    LL_LPM_EnableDeepSleep();
    __WFI();


    if(count % TIME_WAIT == 0){
      LL_GPIO_ResetOutputPin(GPIOA, ESP_RST_PIN);
      LL_mDelay(180);
      LL_GPIO_SetOutputPin(GPIOA, ESP_RST_PIN);
    }
  }
}

static void APP_GPIO_Config(void)
{
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  LL_GPIO_SetPinMode(GPIOA, ESP_RST_PIN, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetOutputPin(GPIOA, ESP_RST_PIN);
}

static void APP_ConfigLPTIMOneShot(void)
{

  LL_RCC_LSI_Enable();
  while(LL_RCC_LSI_IsReady() == 0);
  // Set LSI as LPTIM1 clock source
  LL_RCC_SetLPTIMClockSource(LL_RCC_LPTIM1_CLKSOURCE_LSI);
  // Prescaler = 64
  LL_LPTIM_SetPrescaler(LPTIM1, LL_LPTIM_PRESCALER_DIV64);
  LL_LPTIM_SetUpdateMode(LPTIM1, LL_LPTIM_UPDATE_MODE_ENDOFPERIOD);
  LL_LPTIM_EnableIT_ARRM(LPTIM1);
  LL_LPTIM_Enable(LPTIM1);


  /* 32768 / 128 = 256 */
  LL_LPTIM_SetAutoReload(LPTIM1, HalfSecond);

  NVIC_EnableIRQ(LPTIM1_IRQn);
  NVIC_SetPriority(LPTIM1_IRQn, 0);
}

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


void LPTIM1_IRQHandler(void)
{
  if (LL_LPTIM_IsActiveFlag_ARRM(LPTIM))
  {
    LL_LPTIM_ClearFLAG_ARRM(LPTIM);
  }
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
