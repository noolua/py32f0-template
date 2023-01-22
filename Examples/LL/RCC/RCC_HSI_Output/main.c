#include "main.h"
#include "py32f0xx_bsp_button.h"
#include "py32f0xx_bsp_led.h"
#include "py32f0xx_bsp_printf.h"


static void APP_SystemClockConfig(void);
static void APP_GPIOConfig(void);


int main(void)
{
  APP_SystemClockConfig();

  APP_GPIOConfig();

  LL_RCC_ConfigMCO(LL_RCC_MCO1SOURCE_SYSCLK,LL_RCC_MCO1_DIV_1);

  while (1)
  {
    LL_mDelay(500);
    LL_GPIO_TogglePin(GPIOB,LL_GPIO_PIN_5);
  }
}

static void APP_SystemClockConfig(void)
{
  LL_RCC_HSI_Enable();
  LL_RCC_HSI_SetCalibFreq(LL_RCC_HSICALIBRATION_24MHz);
  while(LL_RCC_HSI_IsReady() != 1);

  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS);

  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);

  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  /* Update global SystemCoreClock(or through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(24000000);
}

static void APP_GPIOConfig(void)
{
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);

  /* PA08: MCO output */
  // Add USE_FULL_LL_DRIVER to LIB_FLAGS to enable LL_GPIO_InitTypeDef
  LL_GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  /* Alternate mode */
  GPIO_InitStruct.Alternate = LL_GPIO_AF5_MCO;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  /* No pull */
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
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