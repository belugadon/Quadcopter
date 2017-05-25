/**
  ******************************************************************************
  * @file    main.c 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    20-September-2012
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include <stdio.h>

  RCC_ClocksTypeDef RCC_Clocks;
__IO uint8_t DataReady = 0;
__IO uint8_t PrevXferComplete = 1;
__IO uint32_t USBConnectTimeOut = 100;

uint8_t offset;

int main(void)
{
	int a = 0;
  uint8_t i = 0;

  /* SysTick end of count event each 10ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);
  RCC_Configuration();
  USART1_Configuration();
    Demo_USB();

	    PWMInput_Config();

		while(1);
}
void PWMInput_Config2()
{
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	SYSCFG_EXTILineConfig(GPIOA, GPIO_Pin_1);
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 || GPIO_Pin_2 || GPIO_Pin_3 || GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; // GPIO speed - has nothing to do with the timer timing
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // Push-pull
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; // Setup pull-up resistors
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	TIM_TimeBaseInitTypeDef timerInitStructure;
	timerInitStructure.TIM_Prescaler = 0;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = 0xFFFF;
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	timerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &timerInitStructure);
	//TIM_Cmd(TIM4, ENABLE);

    //Configure Button EXTI line
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = EXTI_Line1 || EXTI_Line2 || EXTI_Line3 || EXTI_Line4;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitTypeDef nvicStructure;
    nvicStructure.NVIC_IRQChannel = EXTI1_IRQn;
    nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
    nvicStructure.NVIC_IRQChannelSubPriority = 0;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure);

}
void PWMInput_Config()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_10);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // Input
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; // GPIO speed - has nothing to do with the timer timing
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; // Push-pull
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // Setup pull-up resistors
	GPIO_Init(GPIOD, &GPIO_InitStructure);


	TIM_TimeBaseInitTypeDef timerInitStructure;
	timerInitStructure.TIM_Prescaler = 0;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = 0xFFFF;
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	timerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &timerInitStructure);
	//TIM_Cmd(TIM4, ENABLE);

    TIM_ICInitTypeDef inputCaptureInitStructure;
    TIM_ICStructInit(&inputCaptureInitStructure);
    inputCaptureInitStructure.TIM_Channel = TIM_Channel_1 | TIM_Channel_2;
    inputCaptureInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    inputCaptureInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    inputCaptureInitStructure.TIM_ICFilter = 0x03;

    //(TIM4, &inputCaptureInitStructure);
    TIM_PWMIConfig(TIM4, &inputCaptureInitStructure);

    TIM_SelectInputTrigger(TIM4, TIM_TS_TI1FP1);
    TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset);
    TIM_SelectMasterSlaveMode(TIM4,TIM_MasterSlaveMode_Enable);

    NVIC_InitTypeDef nvicStructure;
    nvicStructure.NVIC_IRQChannel = TIM4_IRQn;
    nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
    nvicStructure.NVIC_IRQChannelSubPriority = 1;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure);

    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

    TIM_ITConfig(TIM4, TIM_IT_CC1 | TIM_IT_CC2, ENABLE);

    TIM_Cmd(TIM4, ENABLE);
}


/**
  * @brief  Configure the USB.
  * @param  None
  * @retval None
  */
void Demo_USB (void)
{
  Set_System();
  Set_USBClock();
  USB_Interrupts_Config();

  USB_Init();

  while ((bDeviceState != CONFIGURED)&&(USBConnectTimeOut != 0))
  {}
}
/**
  * @brief  Configure the Mems to gyroscope application.
  * @param  None
  * @retval None
  */


void RCC_Configuration(void)
{
  /// Enable GPIO clock
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  // Enable USART clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);


}


void USART1_Configuration(void)
{
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

//   USART resources configuration (Clock, GPIO pins and USART registers) ----
//   USART configured as follow:
//        - BaudRate = 9600 baud
//        - Word Length = 8 Bits
//        - One Stop Bit
//        - No parity
//        - Hardware flow control disabled (RTS and CTS signals)
//        - Receive and transmit enabled

  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  // USART configuration
  USART_Init(USART1, &USART_InitStructure);
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  // Enable USART
  USART_Cmd(USART1, ENABLE);
}


void USART1_IRQHandler(void)
{
	while((USART1->ISR & USART_FLAG_RXNE) == (uint16_t)RESET);

	offset = USART_ReceiveData(USART1);
	//rx = USART_ReceiveData(USART1);
    //duty_cycle1=rx;
    //set_pwm_width(1, pwm_period1, &duty_cycle1);
    //set_pwm_width(2, pwm_period1, &duty_cycle1);
    //set_pwm_width(3, pwm_period1, &duty_cycle1);
    //set_pwm_width(4, pwm_period1, &duty_cycle1);

}
void USART3_IRQHandler(void)
{

	//uint8_t temp = 0;
	//uint8_t value = 0;
	//uint8_t dec_place = 1;
	while((USART3->ISR & USART_FLAG_RXNE) == (uint16_t)RESET);
	offset = USART_ReceiveData(USART3);
	//USART1_Send(offset);


	/*
	if (USART3_rx == '$')
	{
		string_pointer = 0;
	}
	NEMA_String[string_pointer]=USART3_rx;
	//USART1_Send(USART3_rx);
	//USART1_Send(string_pointer + '0');
	string_pointer++;
	if(USART3_rx == '\r' || USART3_rx == '\n')
	{
		  //NVIC_InitTypeDef NVIC_InitStructure;
		  USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
		  //USART1_Send('\r');
		  //USART1_Send('\n');
		  //NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
		  //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		  //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		  //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		  //NVIC_Init(&NVIC_InitStructure);
	}
	*/
}

void EXTI0_IRQHandler(void){
  EXTI_ClearITPendingBit(EXTI_Line0);

}
void TIM4_IRQHandler(void)
{
	if (TIM4->SR & TIM_IT_CC1)
	{
	TIM_ClearITPendingBit(TIM4, TIM_IT_CC1 | TIM_IT_Update);

		/* Get the Input Capture value */
	offset = TIM_GetCapture2(TIM4);
	//Display_Temperature(offset);
	}
	if (TIM4->SR & TIM_IT_Update)
	{
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	  // Get the Input Capture value
	//pitch = TIM_GetCapture2(TIM4);
	//Display_Temperature(pitch);
	}
	/*
	if (TIM4->SR & TIM_IT_CC3)
	{
	TIM_ClearITPendingBit(TIM4, TIM_IT_CC3);
	  // Get the Input Capture value
	roll = TIM_GetCapture3(TIM4);
	Display_Temperature(roll);
	}
	if (TIM4->SR & TIM_IT_CC4)
	{
	TIM_ClearITPendingBit(TIM4, TIM_IT_CC4);
	  // Get the Input Capture value
	yaw = TIM_GetCapture4(TIM4);
	Display_Temperature(yaw);
	}*/

}
void EXTI1_IRQHandler(void){

  EXTI_ClearITPendingBit(EXTI_Line1);

  if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == 1)
  {
	  //start counter
	  TIM_Cmd(TIM4, ENABLE);
	  EXTI_InitTypeDef EXTI_InitStructure;
	  EXTI_InitStructure.EXTI_Line = EXTI_Line1;
	  EXTI_InitStructure.EXTI_LineCmd = DISABLE;
	  EXTI_Init(&EXTI_InitStructure);
	  EXTI_InitStructure.EXTI_Line = EXTI_Line1;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);
  }
  else
  {
	  //Stop counter and update setpoint
	  offset = TIM_GetCounter(TIM4);
	  TIM_Cmd(TIM4, DISABLE);
	  EXTI_InitTypeDef EXTI_InitStructure;
	  EXTI_InitStructure.EXTI_LineCmd = DISABLE;
	  EXTI_Init(&EXTI_InitStructure);
	  EXTI_InitStructure.EXTI_Line = EXTI_Line1;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);
	  //Display_Raw_Gyro(offset);
  }
}
void EXTI2_IRQHandler(void){
  EXTI_ClearITPendingBit(EXTI_Line2);
}
void EXTI3_IRQHandler(void){
  EXTI_ClearITPendingBit(EXTI_Line3);
}
void EXTI4_IRQHandler(void){
  EXTI_ClearITPendingBit(EXTI_Line4);
}
#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
