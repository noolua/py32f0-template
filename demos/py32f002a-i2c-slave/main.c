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

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t aReceiveBuffer[0x11] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F','X'};
uint8_t ubReceiveIndex      = 0;
/* Private function prototypes -----------------------------------------------*/
static void     APP_ConfigI2cSlave(void);
static void     APP_SlaveOnRecieve(void);
static void     APP_SlaveOnRequest(uint8_t *pData, uint16_t size);
static void     APP_LED_Blink(int count);
static void     APP_LED_BlinkFast(int count);

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
  APP_LED_Blink(2);
  APP_LED_BlinkFast(3);

  /* 处理 I2C1 事件（从机） */
  while(1){
    ubReceiveIndex = 0;
    APP_SlaveOnRecieve();
    // LL_mDelay(2000);
    APP_SlaveOnRequest(aReceiveBuffer, 16);
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

  /* (3) 配置I2C1功能参数***********************/

  /* 在修改配置寄存器之前禁用 I2C1 */
  LL_I2C_Disable(I2C1);

  /* 配置从机地址：
    * - OwnAddress1 是 PY32_OWN_ADDRESS
    */
  LL_I2C_SetOwnAddress1(I2C1, PY32_OWN_ADDRESS<<1, 0);

  /* 启用时钟拉伸 */
  /* 复位值是启用时钟延长 */
  /* LL_I2C_EnableClockStretching(I2C1); */

  /* 启用广播呼叫 */
  /* 复位值为禁用广播呼叫 */
  /* LL_I2C_EnableGeneralCall(I2C1); */

  /* (1) 使能 I2C1 **********************************************************/
  LL_I2C_Enable(I2C1);
}


static void APP_SlaveOnRecieve(void)
{
  /* (1) 准备从机地址接收的确认 ************************/
  LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);

  /* (2) 等待 ADDR 标志并检查地址匹配码和方向 ************/
  while(!LL_I2C_IsActiveFlag_ADDR(I2C1))
  {
  }

  /* 验证传输方向，Read 时的方向，Slave 进入接收器模式 */
  if(LL_I2C_GetTransferDirection(I2C1) == LL_I2C_DIRECTION_READ)
  {
    /* 清除 ISR 寄存器中的 ADDR 标志值 */
    LL_I2C_ClearFlag_ADDR(I2C1);
  }
  else
  {
    /* 清除 ISR 寄存器中的 ADDR 标志值 */
    LL_I2C_ClearFlag_ADDR(I2C1);

    /* 调用错误函数 */
    Error_Handler();
  }

  /* (3) 循环直到接收到传输结束（STOP 标志出现） ***************/

  /* 循环直到 STOP 标志置位 */
  while(!LL_I2C_IsActiveFlag_STOP(I2C1))
  {
    /* (3.1) 接收数据（RXNE 标志置位） **********************************/

    /* 检查 ISR 寄存器中的 RXNE 标志值 */
    if(LL_I2C_IsActiveFlag_RXNE(I2C1))
    {
      /* 读取接收数据寄存器中的字符。
       通过读取 DR 寄存器中的数据清除 RXNE 标志 */
      aReceiveBuffer[ubReceiveIndex++] = LL_I2C_ReceiveData8(I2C1);
    }

    /* (3.2) 接收数据（BTF 标志置位） ***********************************/
    /* 检查 ISR 寄存器中的 BTF 标志值 */
    if(LL_I2C_IsActiveFlag_BTF(I2C1))
    {
      /* 读取接收数据寄存器中的字符。
       通过读取 DR 寄存器中的数据清除 BTF 标志 */
      aReceiveBuffer[ubReceiveIndex++] = LL_I2C_ReceiveData8(I2C1);
    }
  }

  /* (4) 清除挂起标志，检查数据一致性 **************************/
  LL_I2C_ClearFlag_STOP(I2C1);

  // APP_LED_BlinkFast(2);

}

void APP_SlaveOnRequest(uint8_t *pData, uint16_t size)
{

  /* (1) 准备从机地址接收的确认 ************************/
  LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);

  /* (2) 等待 ADDR 标志并检查地址匹配码和方向 ************/
  while(!LL_I2C_IsActiveFlag_ADDR(I2C1))
  {
  }

  /* 验证传输方向，Write 时的方向，Slave 进入接收器模式 */
  if(LL_I2C_GetTransferDirection(I2C1) == LL_I2C_DIRECTION_WRITE)
  {
    /* 清除 ISR 寄存器中的 ADDR 标志值 */
    LL_I2C_ClearFlag_ADDR(I2C1);
  }
  else
  {
    /* 清除 ISR 寄存器中的 ADDR 标志值 */
    LL_I2C_ClearFlag_ADDR(I2C1);

    /* 调用错误函数 */
    Error_Handler();
  }

  // LL_I2C_DisableBitPOS(I2C1);

  /* Transfer data */
  while (size > 0)
  {
    while (LL_I2C_IsActiveFlag_TXE(I2C1) != 1);
    LL_I2C_TransmitData8(I2C1, *pData++);
    size--;

    if ((LL_I2C_IsActiveFlag_BTF(I2C1) == 1) && (size != 0U))
    {
      LL_I2C_TransmitData8(I2C1, *pData++);
      size--;
    }
    if(size > 0){
      while (LL_I2C_IsActiveFlag_BTF(I2C1) != 1);
    }else{
      while (LL_I2C_IsActiveFlag_BTF(I2C1) != 0);
    }
  }

  /* Stop */
  LL_I2C_GenerateStopCondition(I2C1);

  // APP_LED_BlinkFast(4);
}


static void     APP_LED_Blink(int count)
{
  int times = count * 2;
  while(times--){
    BSP_LED_Toggle(LED_GREEN);
    LL_mDelay(500);
  }
}

static void     APP_LED_BlinkFast(int count)
{
  int times = count * 2;
  while(times--){
    BSP_LED_Toggle(LED_GREEN);
    LL_mDelay(150);
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
