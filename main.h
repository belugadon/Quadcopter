/**
  ******************************************************************************
  * @file    main.h 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    20-September-2012
  * @brief   Header for main.c module
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
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f30x.h"
#include "stm32f3_discovery.h"
#include <stdio.h>
#include "stm32f3_discovery_lsm303dlhc.h"
#include "stm32f3_discovery_l3gd20.h"
#include "usb_lib.h"
#include "hw_config.h"
#include "usb_pwr.h"
#include "platform_config.h"
#include "math.h"
//#include "stm32f30x_it.h"
#include <float.h>
#include "GPS.h"
#include "BLDC_Control.h"
#include "KalmanFilter.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Demo_USB (void);
void PWMInput_Config2();
void PWMInput_Config();
void get_heading();
void Demo_GyroConfig(void);
void Calculate_Gyro_Drift();
void Demo_GyroReadAngRate (float* pfData);
void Demo_CompassConfig(void);
void Demo_CompassReadMag (float* pfData);
void Demo_CompassReadAcc(float* pfData);
void TimingDelay_Decrement(void);
void Timing_Delay(uint32_t delay);
void USART3_Configuration();
void TX_GPS();
void GPIO_Configuration2();
void Config_ADC(uint16_t GPIO_Pin, uint16_t ADC_Channel);
uint16_t Get_Voltage();
uint32_t Get_Temperature();
uint32_t Get_Baro();
uint8_t get_location();
float Calc_altitude(uint32_t sea_press, uint32_t press, uint32_t temp);
double pow_(double x, int e);
double root(double a, uint8_t n);
void Display_Pressure(uint32_t value);
void Display_Temperature(uint32_t value);
double DEG_MIN_TO_DEC(uint16_t value1, float value2 );
double Display_Longitude(uint16_t value1, float value2);
void Display_Altitude(uint16_t value);
void Display_Heading(float value);
void Display_Angular_velocity(uint16_t value);
void Display_Raw_Gyro(int8_t value);
void Display_Conditioned_Gyro(float value);
void Delay(__IO uint32_t nTime);

//GPS Structure definition
//typedef struct
//{
//	uint16_t Long_Deg;
//	float Long_Min;
//	uint16_t Lat_Deg;
//	float Lat_Min;
//	uint16_t altitude;
//}GPS_TypeDef;


#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
