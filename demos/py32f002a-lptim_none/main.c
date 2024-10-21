/**
 * Demo of Low Power Timer Wakeup
*/
#include "main.h"
#define PACKAGE_SOP8
#include "py32f0xx_bsp_clock.h"
// #include "py32f0xx_bsp_printf.h"

#define SecondPluse     (256 - 1)
#define TIME_WAIT       (10)
#define ESP_POWER_TIME  (4)

#define ESP_RST_PIN   (LL_GPIO_PIN_4)

static void APP_GPIO_Config(void);
static void APP_ConfigLPTIMOneShot(void);
static void APP_uDelay(uint32_t us);
static void APP_CheckAndDisableNRST(void);

int main(void)
{
  static uint32_t count = 0, pwr_times = 0;
  BSP_RCC_HSI_8MConfig();
  APP_GPIO_Config();

  LL_mDelay(2000);

  APP_CheckAndDisableNRST();


  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_LPTIM1);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  APP_ConfigLPTIMOneShot();

  while (1)
  {
   if(count % TIME_WAIT == 0){
      pwr_times = ESP_POWER_TIME+1;
      LL_GPIO_TogglePin(GPIOA, ESP_RST_PIN);
    }
    count++;
    if(pwr_times > 0){
      pwr_times--;
      if(pwr_times == 0)
        LL_GPIO_TogglePin(GPIOA, ESP_RST_PIN);
    }
    LL_PWR_EnableLowPowerRunMode();

    LL_LPTIM_Disable(LPTIM1);
    LL_LPTIM_Enable(LPTIM1);
    APP_uDelay(120);

    LL_LPTIM_StartCounter(LPTIM1, LL_LPTIM_OPERATING_MODE_ONESHOT);
    LL_LPM_EnableDeepSleep();
    __WFI();
  }
}

static void APP_GPIO_Config(void)
{
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  LL_GPIO_SetPinMode(GPIOA, ESP_RST_PIN, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_ResetOutputPin(GPIOA, ESP_RST_PIN);
  // LL_GPIO_SetOutputPin(GPIOA, ESP_RST_PIN);

  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_3,LL_GPIO_PULL_DOWN);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_2,LL_GPIO_PULL_DOWN);

  /* 将 SCL 引脚配置为：可选功能、高速、开漏、上拉 */
  /*
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_12;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  */
  /* 将 SDA 引脚配置为：可选功能、高速、开漏、上拉 */
  /*
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_12;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  */
}

static void APP_ConfigLPTIMOneShot(void)
{

  LL_RCC_LSI_Enable();
  while(LL_RCC_LSI_IsReady() == 0);
  // Set LSI as LPTIM1 clock source
  LL_RCC_SetLPTIMClockSource(LL_RCC_LPTIM1_CLKSOURCE_LSI);
  // Prescaler = 128
  LL_LPTIM_SetPrescaler(LPTIM1, LL_LPTIM_PRESCALER_DIV128);
  LL_LPTIM_SetUpdateMode(LPTIM1, LL_LPTIM_UPDATE_MODE_ENDOFPERIOD);
  LL_LPTIM_EnableIT_ARRM(LPTIM1);
  LL_LPTIM_Enable(LPTIM1);


  /* 32768 / 128 = 256 */
  LL_LPTIM_SetAutoReload(LPTIM1, SecondPluse);

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

static void APP_CheckAndDisableNRST(void)
{
  /* 如果没关闭则调用 */
  if(READ_BIT(FLASH->OPTR, FLASH_OPTR_NRST_MODE) == OB_RESET_MODE_RESET){
    FLASH_OBProgramInitTypeDef OBInitCfg;

    LL_FLASH_Unlock();
    LL_FLASH_OB_Unlock();

    OBInitCfg.OptionType = OPTIONBYTE_USER;
    OBInitCfg.USERType = OB_USER_BOR_EN | OB_USER_BOR_LEV | OB_USER_IWDG_SW | OB_USER_NRST_MODE | OB_USER_nBOOT1;
    /*
     * 默认的值: OB_BOR_DISABLE | OB_BOR_LEVEL_3p1_3p2 | OB_IWDG_SW | OB_RESET_MODE_RESET | OB_BOOT1_SYSTEM;
    */
    OBInitCfg.USERConfig = OB_BOR_DISABLE | OB_BOR_LEVEL_3p1_3p2 | OB_IWDG_SW | OB_RESET_MODE_GPIO | OB_BOOT1_SYSTEM;
    // OBInitCfg.USERConfig = OB_BOR_DISABLE | OB_BOR_LEVEL_3p1_3p2 | OB_IWDG_SW | OB_RESET_MODE_RESET | OB_BOOT1_SYSTEM;
    LL_FLASH_OBProgram(&OBInitCfg);

    LL_FLASH_Lock();
    LL_FLASH_OB_Lock();
    /* 重新载入OB, 这会触发软复位, MCU重启 */
    LL_FLASH_OB_Launch();
  }
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
