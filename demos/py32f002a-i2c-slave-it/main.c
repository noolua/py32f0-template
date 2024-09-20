/**
  ******************************************************************************
  * @file    main.c
  * @author  MCU Application Team
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 Puya Semiconductor Co.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by Puya under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
// #define PY32F002A_DEVKIT
// #include "py32f0xx_bsp_led.h"
#include "py32f0xx_bsp_clock.h"

/* Private define ------------------------------------------------------------*/
#define ESP_RST_PIN           (LL_GPIO_PIN_4)

#define I2C_BUFF_SZ           (64)
enum{
  i2cs_ready,
  i2cs_waiting_ack,
  i2cs_busy_rx,
  i2cs_busy_tx,
  i2cs_tx_done,
};
#define I2C_CMD_ADD           ('a')   // add
#define I2C_CMD_SUB           ('d')   // sub
#define I2C_CMD_RPC_COUNT     ('c')   // rpc_count
#define I2C_CMD_STOP          ('s')   // go stop
#define I2C_CMD_TIME          ('t')   // set timestamp
#define I2C_CMD_TIMENOW       ('n')   // get timestamp
#define I2C_CMD_ERR           (0xEE)

/* Private variables ---------------------------------------------------------*/
typedef struct app_i2c_s{
  int32_t tim1_tick, lptim_tick;
  int32_t delta_time, status;
  uint32_t stopseconds, rpc_count;
  uint8_t recv_len, resp_len, resp_send;
  uint8_t recv_buffer[I2C_BUFF_SZ], resp_buffer[I2C_BUFF_SZ];
}app_i2c_t;


#define ts_tick()     ((_app_i2c.tim1_tick>>1) + _app_i2c.lptim_tick)
#define ts_now()      (ts_tick() + _app_i2c.delta_time)

static app_i2c_t _app_i2c = {
  .tim1_tick = 0, .lptim_tick = 0, .delta_time = 0, .status = i2cs_ready,
  .stopseconds = 0U, .rpc_count = 0U, .recv_len = 0, .resp_len = 0, .resp_send = 0
};

/* Private function prototypes -----------------------------------------------*/
static void APP_ConfigLPTIMOneShot(void);
static void APP_TIM1Config(void);
static void APP_EnterStop(void);

static void  APP_InitI2cSlave(void);
static void  APP_DeinitI2cSlave(void);
static void  APP_GPIOConfig(void);
static void  APP_ESPReset(void);
static void  APP_SlaveAckReady(void);
static void  APP_HandleI2CSalve(void);
static void  APP_OnTimerUpdate(void);

#ifdef PY32F002A_SOP8
static void     APP_CheckAndDisableNRST(void);
#endif

static void APP_receive8(void){
  if(_app_i2c.recv_len < I2C_BUFF_SZ){
    _app_i2c.recv_buffer[_app_i2c.recv_len] = LL_I2C_ReceiveData8(I2C1);
    _app_i2c.recv_len++;
  }else{
    __IO uint32_t tmpreg;
    tmpreg = LL_I2C_ReceiveData8(I2C1);
    (void)tmpreg;
  }
}

static void APP_transmit8(void){
  if(_app_i2c.resp_send < _app_i2c.resp_len){
    LL_I2C_TransmitData8(I2C1, _app_i2c.resp_buffer[_app_i2c.resp_send]);
    _app_i2c.resp_send++;
  }else{
    LL_I2C_TransmitData8(I2C1, 0x00);
  }
}
int main(void)
{
  /* 配置系统时钟 */
  BSP_RCC_HSI_8MConfig();
  APP_TIM1Config();
  LL_mDelay(100);

#ifdef PY32F002A_SOP8
  APP_CheckAndDisableNRST();
#endif

  /* 初始化GPIO */
  APP_GPIOConfig();

  /*配置LPTIM*/
  APP_ConfigLPTIMOneShot();

  /* 配置I2C1（Slave模式下的I2C配置及相关GPIO初始化）,并使能*/
  APP_InitI2cSlave();
  APP_ESPReset();

  /* 处理 I2C1 事件（从机） */
  while(1){
    APP_HandleI2CSalve();
    // LL_mDelay(1);
    // do other jobs
    // ...
  }
}

static void  APP_OnTimerUpdate(void){
}

#ifdef PY32F002A_SOP8
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
#endif // PY32F002A_SOP8

static void APP_TIM1Config(void)
{
  LL_TIM_InitTypeDef TIM1CountInit = {0};

  LL_APB1_GRP2_EnableClock(RCC_APBENR2_TIM1EN);

  TIM1CountInit.ClockDivision       = LL_TIM_CLOCKDIVISION_DIV1;
  TIM1CountInit.CounterMode         = LL_TIM_COUNTERMODE_UP;
  TIM1CountInit.Prescaler           = 8000-1;
  TIM1CountInit.Autoreload          = 500-1; /* half-second */
  TIM1CountInit.RepetitionCounter   = 0;
  LL_TIM_Init(TIM1,&TIM1CountInit);

  LL_TIM_EnableIT_UPDATE(TIM1);
  LL_TIM_EnableCounter(TIM1);

  NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
  NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn,0);
}

static void APP_GPIOConfig(void)
{
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  LL_GPIO_SetPinMode(GPIOA, ESP_RST_PIN, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetOutputPin(GPIOA, ESP_RST_PIN);
}

static void APP_ESPReset(void){
  LL_GPIO_ResetOutputPin(GPIOA, ESP_RST_PIN);
  LL_mDelay(50);
  LL_GPIO_SetOutputPin(GPIOA, ESP_RST_PIN);
}

static void APP_ConfigLPTIMOneShot(void)
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

void APP_EnterStop(void){
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


static void APP_InitI2cSlave(void)
{
  /* (1) 使能 GPIO 时钟 ************************/

  /* 使能 GPIOA 的外设时钟 */
  // LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  /* (2) 使能 I2C1 外设时钟 *************************************/

  /* 启用 I2C1 的外设时钟 */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* 将 SCL 引脚配置为：可选功能、高速、开漏、上拉 */
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_12;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* 将 SDA 引脚配置为：可选功能、高速、开漏、上拉 */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_12;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);


  /* 复位I2C */
  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_I2C1);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_I2C1);

  /* 使能NVIC中断 */
  NVIC_SetPriority(I2C1_IRQn, 0);
  NVIC_EnableIRQ(I2C1_IRQn);

  /* I2C初始化 */
  LL_I2C_InitTypeDef I2C_InitStruct = {0};
  I2C_InitStruct.ClockSpeed      = 1000000;
  I2C_InitStruct.DutyCycle       = LL_I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1     = PY32_OWN_ADDRESS<<1;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_NACK;
  LL_I2C_Init(I2C1, &I2C_InitStruct);


  /* 启用时钟拉伸 */
  /* 复位值是启用时钟延长 */
  /* LL_I2C_EnableClockStretching(I2C1); */

  /* 启用广播呼叫 */
  /* 复位值为禁用广播呼叫 */
  /* LL_I2C_EnableGeneralCall(I2C1); */

}

static void  APP_DeinitI2cSlave(void)
{
  LL_I2C_Disable(I2C1);

  /* 使能NVIC中断 */
  // NVIC_SetPriority(I2C1_IRQn, 0);
  NVIC_DisableIRQ(I2C1_IRQn);

  /* 复位I2C */
  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_I2C1);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_I2C1);

  /* 启用 I2C1 的外设时钟 */
  LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_I2C1);
}

static void APP_HandleI2CSalve(void){
  if(_app_i2c.status == i2cs_tx_done){
    if(_app_i2c.stopseconds > 0){

      APP_DeinitI2cSlave();

      while(_app_i2c.stopseconds){
        _app_i2c.stopseconds--;
        APP_EnterStop();
      }

      APP_InitI2cSlave();
      APP_ESPReset();
    }
    APP_SlaveAckReady();
  }else if(_app_i2c.status == i2cs_ready){
    uint8_t cmd = I2C_CMD_ERR;
    if(_app_i2c.recv_len > 0)
      cmd = _app_i2c.recv_buffer[0];
    switch(cmd){
    case I2C_CMD_ADD:{
      int a = *(int*)&_app_i2c.recv_buffer[1];
      int b = *(int*)&_app_i2c.recv_buffer[5];
      *(int*)&_app_i2c.resp_buffer[0] = a + b;
      _app_i2c.resp_len = 4;
      _app_i2c.rpc_count++;
      break;
    }
    case I2C_CMD_SUB:{
      int a = *(int*)&_app_i2c.recv_buffer[1];
      int b = *(int*)&_app_i2c.recv_buffer[5];
      *(int*)&_app_i2c.resp_buffer[0] = a - b;
      _app_i2c.resp_len = 4;
      _app_i2c.rpc_count++;
      break;
    }
    case I2C_CMD_RPC_COUNT:{
      _app_i2c.rpc_count++;
      *(int*)&_app_i2c.resp_buffer[0] = _app_i2c.rpc_count;
      _app_i2c.resp_len = 4;
      break;
    }
    case I2C_CMD_STOP:{
      _app_i2c.rpc_count++;
      int32_t seconds = *(int32_t*)&_app_i2c.recv_buffer[1];
      *(int32_t*)&_app_i2c.resp_buffer[0] = seconds;
      _app_i2c.stopseconds = seconds;
      _app_i2c.resp_len = 4;
      break;
    }
    case I2C_CMD_TIME:{
      _app_i2c.rpc_count++;
      int32_t ts = *(int*)&_app_i2c.recv_buffer[1];
      _app_i2c.delta_time = ts - ts_tick();
      *(int*)&_app_i2c.resp_buffer[0] = ts_now();
      _app_i2c.resp_len = 4;
      break;
    }
    case I2C_CMD_TIMENOW:{
      _app_i2c.rpc_count++;
      *(int*)&_app_i2c.resp_buffer[0] = ts_now();
      _app_i2c.resp_len = 4;
      break;
    }
    default:
      break;
    }
    APP_SlaveAckReady();
  }
}

static void APP_SlaveAckReady()
{
  /* 清pos */
  LL_I2C_DisableBitPOS(I2C1);

  /* 修改 */
  _app_i2c.recv_len = 0;
  _app_i2c.resp_send = 0;
  _app_i2c.recv_buffer[0] = I2C_CMD_ERR;
  _app_i2c.status = i2cs_waiting_ack;

  /* 使能中断 */
  LL_I2C_EnableIT_EVT(I2C1);
  LL_I2C_EnableIT_BUF(I2C1);
  LL_I2C_EnableIT_ERR(I2C1);

  /* 使能应答 */
  LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);
}


void APP_SlaveIRQCallback(void)
{
  /* ADDR标志位置位 */
  if ((LL_I2C_IsActiveFlag_ADDR(I2C1) == 1) && (LL_I2C_IsEnabledIT_EVT(I2C1) == 1))
  {
    LL_I2C_ClearFlag_ADDR(I2C1);

    if(LL_I2C_GetTransferDirection(I2C1) == LL_I2C_DIRECTION_READ){
      // slave recieve
      _app_i2c.status = i2cs_busy_rx;
    }else{
      // slave transmit
      _app_i2c.status = i2cs_busy_tx;
    }
  }
  else if(LL_I2C_IsActiveFlag_STOP(I2C1) == 1){

    LL_I2C_DisableIT_EVT(I2C1);
    LL_I2C_DisableIT_BUF(I2C1);
    LL_I2C_DisableIT_ERR(I2C1);
    LL_I2C_ClearFlag_STOP(I2C1);

    if(LL_I2C_IsActiveFlag_RXNE(I2C1)){
      APP_receive8();
    }
    if(LL_I2C_IsActiveFlag_BTF(I2C1)){
      APP_receive8();
    }
    _app_i2c.status = i2cs_ready;
  }
  else if(_app_i2c.status == i2cs_busy_rx){
    if ((LL_I2C_IsActiveFlag_RXNE(I2C1) == 1) && (LL_I2C_IsEnabledIT_BUF(I2C1) == 1) && (LL_I2C_IsActiveFlag_BTF(I2C1) == 0))
    {
      APP_receive8();
    }
    /* BTF标志位置位 */
    else if ((LL_I2C_IsActiveFlag_BTF(I2C1) == 1) && (LL_I2C_IsEnabledIT_EVT(I2C1) == 1))
    {
      APP_receive8();
    }
  }
  else if(_app_i2c.status == i2cs_busy_tx){
    if ((LL_I2C_IsActiveFlag_TXE(I2C1) == 1) && (LL_I2C_IsEnabledIT_BUF(I2C1) == 1) && (LL_I2C_IsActiveFlag_BTF(I2C1) == 0))
    {
      APP_transmit8();
    }
    /* BTF标志位置位 */
    else if ((LL_I2C_IsActiveFlag_BTF(I2C1) == 1) && (LL_I2C_IsEnabledIT_EVT(I2C1) == 1))
    {
      APP_transmit8();
    }
  }
}
/**
  * @brief  I2C主机接收完最后一字节后，向从机发送NACK，从机NACK中断回调函数
  * @param  无
  * @retval 无
  */
void APP_SlaveIRQCallback_NACK(void)
{

  if ((LL_I2C_IsActiveFlag_AF(I2C1) == 1) && (LL_I2C_IsEnabledIT_ERR(I2C1) == 1))
  {
    if(_app_i2c.status == i2cs_busy_tx){
      LL_I2C_DisableIT_EVT(I2C1);
      LL_I2C_DisableIT_BUF(I2C1);
      LL_I2C_DisableIT_ERR(I2C1);
      LL_I2C_ClearFlag_AF(I2C1);
      // APP_SlaveAckReady();
      _app_i2c.status = i2cs_tx_done;
    }
  }
}


void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
  if(LL_TIM_IsActiveFlag_UPDATE(TIM1) && LL_TIM_IsEnabledIT_UPDATE(TIM1))
  {
    APP_OnTimerUpdate();
    _app_i2c.tim1_tick++;
    LL_TIM_ClearFlag_UPDATE(TIM1);
  }
}

void LPTIM1_IRQHandler(void)
{
  if (LL_LPTIM_IsActiveFlag_ARRM(LPTIM))
  {
    _app_i2c.lptim_tick++;
    LL_LPTIM_ClearFLAG_ARRM(LPTIM);
  }
}

/**
  * @brief  错误执行函数
  * @param  无
  * @retval 无
  */
void Error_Handler(void)
{
  /* 无限循环 */
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  输出产生断言错误的源文件名及行号
  * @param  file：源文件名指针
  * @param  line：发生断言错误的行号
  * @retval 无
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* 用户可以根据需要添加自己的打印信息,
     例如: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* 无限循环 */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT Puya *****END OF FILE****/
