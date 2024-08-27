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
// #define PY32F002A_SOP8
// #define PY32F002A_DEVKIT
#include "py32f0xx_bsp_led.h"
#include "py32f0xx_bsp_clock.h"
#define I2C_BUFF_SZ           (64)
enum{
  i2cs_busy_rx = 0,
  i2cs_busy_tx,
  i2cs_busy_tx_done,
  i2cs_ready
};

#define I2C_CMD_ADD           ('a')   // add
#define I2C_CMD_SUB           ('d')   // sub
#define I2C_CMD_RPC_COUNT     ('c')   // rcp_count

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
typedef struct app_i2c_s{
  int status;
  uint32_t rpc_count;
  uint8_t recv_len, resp_len, resp_send;
  uint8_t recv_buffer[I2C_BUFF_SZ], resp_buffer[I2C_BUFF_SZ];
}app_i2c_t;

static app_i2c_t _app_i2c = { .status = i2cs_busy_rx, .rpc_count = 0, .recv_len = 0, .resp_len = 0, .resp_send = 0};

/* Private function prototypes -----------------------------------------------*/
static void     APP_ConfigI2cSlave(void);
static void     APP_LED_BlinkFast(int count);
static void     APP_SlaveMake_IT(int status);
static void     APP_HandleI2CSalve(void);

int main(void)
{
  /* 配置系统时钟 */
  BSP_RCC_HSI_8MConfig();
  LL_mDelay(1000);

  /* 初始化LED */
  BSP_LED_Init(LED_GREEN);

  /* 配置I2C1（Slave模式下的I2C配置及相关GPIO初始化）,并使能*/
  APP_ConfigI2cSlave();

  /* 闪烁LED表示开始工作 */
  APP_LED_BlinkFast(3);

  /*开始接收i2c消息*/
  APP_SlaveMake_IT(i2cs_busy_rx);

  /* 处理 I2C1 事件（从机） */
  while(1){
    APP_HandleI2CSalve();
    LL_mDelay(1);
    // do other jobs
    // ...
  }
}

static void APP_ConfigI2cSlave(void)
{
  /* (1) 使能 GPIO 时钟 ************************/

  /* 使能 GPIOA 的外设时钟 */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

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
  I2C_InitStruct.DutyCycle       = LL_I2C_DUTYCYCLE_16_9;
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

static void APP_HandleI2CSalve(void){
  if(_app_i2c.status == i2cs_busy_tx_done){
    APP_SlaveMake_IT(i2cs_busy_rx);
  }else if(_app_i2c.status == i2cs_ready){
    uint8_t cmd = _app_i2c.recv_buffer[0];
    switch(cmd){
    case I2C_CMD_ADD:{
      int a = *(int*)&_app_i2c.recv_buffer[1];
      int b = *(int*)&_app_i2c.recv_buffer[5];
      *(int*)&_app_i2c.resp_buffer[0] = a + b;
      _app_i2c.resp_len = 4;
      _app_i2c.rpc_count++;
      APP_SlaveMake_IT(i2cs_busy_tx);
      break;
    }
    case I2C_CMD_SUB:{
      int a = *(int*)&_app_i2c.recv_buffer[1];
      int b = *(int*)&_app_i2c.recv_buffer[5];
      *(int*)&_app_i2c.resp_buffer[0] = a - b;
      _app_i2c.resp_len = 4;
      _app_i2c.rpc_count++;
      APP_SlaveMake_IT(i2cs_busy_tx);
      break;
    }
    case I2C_CMD_RPC_COUNT:{
      _app_i2c.rpc_count++;
      *(int*)&_app_i2c.resp_buffer[0] = _app_i2c.rpc_count;
      _app_i2c.resp_len = 4;
      APP_SlaveMake_IT(i2cs_busy_tx);
      break;
    }
    default:
      APP_SlaveMake_IT(i2cs_busy_rx);
      APP_LED_BlinkFast(2);
      break;
    }
  }
}

static void APP_SlaveMake_IT(int status)
{
  /* 清pos */
  LL_I2C_DisableBitPOS(I2C1);

  /* 修改 */
  _app_i2c.recv_len = 0;
  _app_i2c.resp_send = 0;
  _app_i2c.status = status;

  /* 使能应答 */
  LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);

  /* 使能中断 */
  LL_I2C_EnableIT_EVT(I2C1);
  LL_I2C_EnableIT_BUF(I2C1);
  LL_I2C_EnableIT_ERR(I2C1);
}

static void APP_LED_BlinkFast(int count)
{
  int times = count * 2;
  while(times--){
    BSP_LED_Toggle(LED_GREEN);
    LL_mDelay(150);
  }
}

void APP_SlaveIRQCallback(void)
{
  /* ADDR标志位置位 */
  if ((LL_I2C_IsActiveFlag_ADDR(I2C1) == 1) && (LL_I2C_IsEnabledIT_EVT(I2C1) == 1))
  {
    LL_I2C_ClearFlag_ADDR(I2C1);
  }
  else if(LL_I2C_IsActiveFlag_STOP(I2C1) == 1){

    if(_app_i2c.status == i2cs_busy_rx){
      _app_i2c.recv_buffer[_app_i2c.recv_len] = LL_I2C_ReceiveData8(I2C1);
      _app_i2c.recv_len++;
      _app_i2c.status = i2cs_ready;
    }else if(_app_i2c.status == i2cs_busy_tx){
      LL_I2C_GenerateStopCondition(I2C1);
    }
    LL_I2C_DisableIT_EVT(I2C1);
    LL_I2C_DisableIT_BUF(I2C1);
    LL_I2C_DisableIT_ERR(I2C1);
    LL_I2C_ClearFlag_STOP(I2C1);
  }
  else if(_app_i2c.status == i2cs_busy_rx){
    if ((LL_I2C_IsActiveFlag_RXNE(I2C1) == 1) && (LL_I2C_IsEnabledIT_BUF(I2C1) == 1) && (LL_I2C_IsActiveFlag_BTF(I2C1) == 0))
    {
      _app_i2c.recv_buffer[_app_i2c.recv_len] = LL_I2C_ReceiveData8(I2C1);
      _app_i2c.recv_len++;
    }
    /* BTF标志位置位 */
    else if ((LL_I2C_IsActiveFlag_BTF(I2C1) == 1) && (LL_I2C_IsEnabledIT_EVT(I2C1) == 1))
    {
      _app_i2c.recv_buffer[_app_i2c.recv_len] = LL_I2C_ReceiveData8(I2C1);
      _app_i2c.recv_len++;
    }
  }
  else if(_app_i2c.status == i2cs_busy_tx){
    if ((LL_I2C_IsActiveFlag_TXE(I2C1) == 1) && (LL_I2C_IsEnabledIT_BUF(I2C1) == 1) && (LL_I2C_IsActiveFlag_BTF(I2C1) == 0))
    {
      LL_I2C_TransmitData8(I2C1, _app_i2c.resp_buffer[_app_i2c.resp_send]);
      _app_i2c.resp_send++;
    }
    /* BTF标志位置位 */
    else if ((LL_I2C_IsActiveFlag_BTF(I2C1) == 1) && (LL_I2C_IsEnabledIT_EVT(I2C1) == 1))
    {
      LL_I2C_TransmitData8(I2C1, _app_i2c.resp_buffer[_app_i2c.resp_send]);
      _app_i2c.resp_send++;
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
      APP_SlaveMake_IT(i2cs_busy_rx);
      // APP_LED_BlinkFast(1);
    }
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
