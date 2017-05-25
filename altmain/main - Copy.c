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
#include <math.h>
//#include "GPS.h"
/** @addtogroup STM32F3-Discovery_Demo
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define ABS(x)         (x < 0) ? (-x) : x

#define L3G_Sensitivity_250dps     (float)   114.285f         /*!< gyroscope sensitivity with 250 dps full scale [LSB/dps] */
#define L3G_Sensitivity_500dps     (float)    57.1429f        /*!< gyroscope sensitivity with 500 dps full scale [LSB/dps] */
#define L3G_Sensitivity_2000dps    (float)    14.285f	      /*!< gyroscope sensitivity with 2000 dps full scale [LSB/dps] */
#define PI                         (float)     3.14159265f

#define LSM_Acc_Sensitivity_2g     (float)     1.0f            /*!< accelerometer sensitivity with 2 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_4g     (float)     0.5f            /*!< accelerometer sensitivity with 4 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_8g     (float)     0.25f           /*!< accelerometer sensitivity with 8 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_16g    (float)     0.0834f         /*!< accelerometer sensitivity with 12 g full scale [LSB/mg] */
/* Private variables ---------------------------------------------------------*/
  RCC_ClocksTypeDef RCC_Clocks;
__IO uint32_t TimingDelay = 0;
__IO uint32_t UserButtonPressed = 0;
__IO float HeadingValue = 0.0f;  
float MagBuffer[3] = {0.0f}, AccBuffer[3] = {0.0f}, Buffer[3] = {0.0f};
uint8_t Xval, Yval = 0x00;

__IO uint8_t DataReady = 0;
__IO uint8_t PrevXferComplete = 1;
__IO uint32_t USBConnectTimeOut = 100;

float fNormAcc,fSinRoll,fCosRoll,fSinPitch,fCosPitch = 0.0f, RollAng = 0.0f, PitchAng = 0.0f;
float fTiltedX,fTiltedY = 0.0f;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



/**
  * @brief  Main program.
  * @param  None 
  * @retval None
  */

//GPS fix data;
uint16_t altitude;
uint16_t Long_Deg;
float Long_Min;
double dec_long;
uint16_t Lat_Deg;
float Lat_Min;
double dec_lat;

uint8_t fix_state = 2;
uint8_t USART3_rx;
uint8_t *NEMA_String[70];
uint8_t string_pointer=0;
uint8_t power = 0;
uint8_t tx;
uint8_t rx;
uint8_t c =0;
int duty_cycle1 = 17;
int duty_cycle2 = 17;
int duty_cycle3 = 17;
int duty_cycle4 = 18;

uint32_t calibration_value;

int main(void)
{
	int a = 0;
  uint8_t i = 0;

  /* SysTick end of count event each 10ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);
  RCC_Configuration();
  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);
  GPIO_Configuration();
  GPIO_Configuration2();
  USART1_Configuration();
  USART3_Configuration();
  
  /* Reset UserButton_Pressed variable */
  UserButtonPressed = 0x00; 
   
    Demo_USB();
    /* Demo Gyroscope */
    Demo_GyroConfig();

    init_pwm_gpio();
    //init_BATT_SENSE();
    int pwm_period = init_pwm(300);
	__IO uint32_t temperature;
	__IO uint32_t baro;

	//char char_1, char_2, char_3, char_4;
	//char banner1[] = {'\n', '\n', '\r','T', 'e', 'm', 'p', 'e', 'r', 'a', 't', 'u', 'r', 'e', ':', '\n', '\r'};
	//char banner2[] = {'\n', '\r','A', 'i', 'r', ' ', 'P', 'r', 'e', 's', 's', 'u', 'r', 'e', ':', '\n', '\r'};;
	//uint8_t len;

	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; // Input
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // GPIO speed - has nothing to do with the timer timing
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // Push-pull
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // Setup pull-up resistors
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//while(GPIO_ReadInputDataBit(GPIOA, 0) == 0)
	//{

	//}

		while(fix_state != '4')
		{
		fix_state = get_location();
		//TX_GPS();
		}
		USART1_Send(fix_state);
		USART1_Send('\n');
		USART1_Send('\r');
		Display_Altitude(altitude);
		dec_long = Display_Longitude(Long_Deg, Long_Min);
		dec_lat = Display_Longitude(Lat_Deg, Lat_Min);

		DataReady = 0x00;
	    Demo_CompassConfig();

	    /* Wait for data ready */
	    while(DataReady !=0x05)
	    {}
	    DataReady = 0x00;
	    Demo_CompassReadMag(MagBuffer);
	    Demo_CompassReadAcc(AccBuffer);

	    for(i=0;i<3;i++)
	        AccBuffer[i] /= 100.0f;

	      fNormAcc = sqrt((AccBuffer[0]*AccBuffer[0])+(AccBuffer[1]*AccBuffer[1])+(AccBuffer[2]*AccBuffer[2]));

	      fSinRoll = -AccBuffer[1]/fNormAcc;
	      fCosRoll = sqrt(1.0-(fSinRoll * fSinRoll));
	      fSinPitch = AccBuffer[0]/fNormAcc;
	      fCosPitch = sqrt(1.0-(fSinPitch * fSinPitch));

		fTiltedX = MagBuffer[0]*fCosPitch+MagBuffer[2]*fSinPitch;
	    fTiltedY = MagBuffer[0]*fSinRoll*fSinPitch+MagBuffer[1]*fCosRoll-MagBuffer[1]*fSinRoll*fCosPitch;
	    HeadingValue = (float) ((atan2f((float)fTiltedY,(float)fTiltedX))*180)/PI;
	    Display_Heading(HeadingValue);

		arm_sequence();
		balance();
	    //for(i=0; i<sizeof(banner1);i++){USART1_Send(banner1[i]);}
		//temperature = Get_Temperature();
		//Display_Temperature(temperature);

	    //for(i=0; i<sizeof(banner2);i++){USART1_Send(banner2[i]);}
	    //baro = Get_Baro();
	    //Display_Pressure(baro);

	    //altitude = Calc_altitude(1013000000, baro, temperature);
	    //USART1_Send('\r');
		//while((USART3->ISR & USART_FLAG_RXNE) == (uint16_t)RESET);
		//USART2_rx = USART_ReceiveData(USART3);
		//USART1_Send(USART2_rx);

    // Config the PWM freq to 200Hz


    set_pwm_width(1, pwm_period, duty_cycle1);
    set_pwm_width(2, pwm_period, duty_cycle2);
    set_pwm_width(3, pwm_period, duty_cycle3);
    set_pwm_width(4, pwm_period, duty_cycle4);
    while(1);

}


uint8_t get_location()
{
	uint8_t head_state = 1;
	//while(fix_state != '0')
	//{
	for(string_pointer=0; string_pointer<=69; string_pointer++)
	{
		NEMA_String[string_pointer]=0;
	}
	string_pointer=0;
	//GPIO_Configuration2();
	USART3_Configuration();
	while((USART3_rx != '\r') && (USART3_rx != '\n'));
//	USART1_Send('X');
	fix_state = parse_GPGGA(NEMA_String, &altitude, &Long_Deg, &Long_Min, &Lat_Deg, &Lat_Min);
	//if (fix_state=='3')
	//{
		//USART1_Send(fix_state);
		USART1_Send('\r');
		USART1_Send('\n');
	//}
	if(Long_Deg == 0)
	{
		fix_state == '5';
	}
	return fix_state;
}
uint8_t get_heading()
{
	uint8_t i;
	DataReady = 0x00;
    Demo_CompassConfig();

    /* Wait for data ready */
    while(DataReady !=0x05)
    {}
    DataReady = 0x00;
    Demo_CompassReadMag(MagBuffer);
    Demo_CompassReadAcc(AccBuffer);

    for(i=0;i<3;i++)
        AccBuffer[i] /= 100.0f;

      fNormAcc = sqrt((AccBuffer[0]*AccBuffer[0])+(AccBuffer[1]*AccBuffer[1])+(AccBuffer[2]*AccBuffer[2]));

      fSinRoll = -AccBuffer[1]/fNormAcc;
      fCosRoll = sqrt(1.0-(fSinRoll * fSinRoll));
      fSinPitch = AccBuffer[0]/fNormAcc;
      fCosPitch = sqrt(1.0-(fSinPitch * fSinPitch));

	fTiltedX = MagBuffer[0]*fCosPitch+MagBuffer[2]*fSinPitch;
    fTiltedY = MagBuffer[0]*fSinRoll*fSinPitch+MagBuffer[1]*fCosRoll-MagBuffer[1]*fSinRoll*fCosPitch;
    HeadingValue = (float) ((atan2f((float)fTiltedY,(float)fTiltedX))*180)/PI;
    Display_Heading(HeadingValue);
}
void TX_GPS()
{
	for(string_pointer=0; string_pointer<=69; string_pointer++)
	{
		NEMA_String[string_pointer]=0;
	}
	string_pointer=0;
	USART3_Configuration();
	while((USART3_rx != '\r') && (USART3_rx != '\n'));
}

void balance()
{
	int pwm_period = init_pwm(300);

	duty_cycle1++;
	duty_cycle2++;
	duty_cycle3++;
	duty_cycle4++;

    // Read Gyro Angular data
    while(UserButtonPressed == 0x00){
    	int timedelay1 = 5000;
      Demo_GyroReadAngRate(Buffer);
         
      // Update autoreload and capture compare registers value
      Xval = ABS((int8_t)(Buffer[0]));
      Yval = ABS((int8_t)(Buffer[1]));
      
      if ( Xval>Yval)
      {
        if ((int8_t)Buffer[0] > 5.0f)
        {
          // LD10 On
        	duty_cycle1=3;
        	duty_cycle3=6;
        }
        if ((int8_t)Buffer[0] < -5.0f)
        {
          // LD3 On
        	duty_cycle1=6;
        	duty_cycle3=3;
        }
      }
      else
      {
        if ((int8_t)Buffer[1] < -5.0f)
        {
          // LD6 on
        	duty_cycle2=8;
        	duty_cycle4=3;
        }
        if ((int8_t)Buffer[1] > 5.0f)
        {
          // LD7 On
        	duty_cycle2=3;
        	duty_cycle4=6;
        }
      }
      bounds_check();

      set_pwm_width(4, pwm_period, duty_cycle4);
      set_pwm_width(1, pwm_period, duty_cycle1);
      set_pwm_width(2, pwm_period, duty_cycle2);
      set_pwm_width(3, pwm_period, duty_cycle3);

  	while(timedelay1 >= 0)
  	{
  		timedelay1--;
  	}
    //DataReady = 0x00;
    }
    set_pwm_width(1, pwm_period, 0);
    set_pwm_width(2, pwm_period, 0);
    set_pwm_width(3, pwm_period, 0);
    set_pwm_width(4, pwm_period, 0);
}

//void Delay(uint32_t delay) {
//    while (delay--);
//}
void update_control_signals(uint8_t width)
{
	duty_cycle1 = width;
	duty_cycle2 = width;
	duty_cycle3 = width;
	duty_cycle4 = width;
}
void init_BATT_SENSE()
{


	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; // Input
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // GPIO speed - has nothing to do with the timer timing
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // Push-pull
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // Setup pull-up resistors
	GPIO_Init(GPIOA, &GPIO_InitStructure);

    //Configure Button EXTI line
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);


    //set priotity
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;


    NVIC_Init(&NVIC_InitStructure);
}
void bounds_check()
{
	if(duty_cycle1 >= 20)
	{
		duty_cycle1 = 20;
	}
	//if(duty_cycle1 <= 5)
	//{
		//duty_cycle1 = 5;
	//}
	if(duty_cycle2 >= 20)
	{
		duty_cycle2 = 20;
	}
	//if(duty_cycle2 <= 5)
	//{
		//duty_cycle2 = 5;
	//}
	if(duty_cycle3 >= 20)
	{
		duty_cycle3 = 20;
	}
	//if(duty_cycle3 <= 5)
	//{
		//duty_cycle3 = 5;
	//}
	if(duty_cycle4 >= 20)
	{
		duty_cycle4 = 20;
	}
	//if(duty_cycle4 <= 5)
	//{
		//duty_cycle4 = 5;
	//}
}
void arm_sequence()
{
	duty_cycle1 = 17;
	duty_cycle2 = 17;
	duty_cycle3 = 17;
	duty_cycle4 = 18;
	int pwm_period = init_pwm(300);
	int timedelay1 = 5000000;
	int timedelay2 = 3000;

	set_pwm_width(1, pwm_period, duty_cycle1);
	set_pwm_width(2, pwm_period, duty_cycle2);
	set_pwm_width(3, pwm_period, duty_cycle3);
	set_pwm_width(4, pwm_period, duty_cycle4);

	while(timedelay1 >= 0)
	{
		timedelay1--;
	}
	while(duty_cycle1 >= 3)
	{
		while(timedelay2 >= 0)
		{
			timedelay2--;
		}
		duty_cycle1--;
		duty_cycle2--;
		duty_cycle3--;
		duty_cycle4--;

		set_pwm_width(1, pwm_period, duty_cycle1);
		set_pwm_width(2, pwm_period, duty_cycle2);
		set_pwm_width(3, pwm_period, duty_cycle3);
		set_pwm_width(4, pwm_period, duty_cycle4);
		timedelay2 = 3000;
	}
	//balance();
}
void init_pwm_gpio()
{
	// The Timer 1 channels 1,2 and 3 are connected to the LED pins on the Discovery board
	// To drive the ECSs they could to be connected to other pins if needed.

	// Setup the LEDs
	// Check out the Discovery schematics
	// http://www.st.com/st-web-ui/static/active/en/resource/technical/document/user_manual/DM00063382.pdf
		//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);
	GPIO_InitStructure.GPIO_Pin = LED3_PIN | LED7_PIN | LED8_PIN | LED10_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // Use the alternative pin functions
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // GPIO speed - has nothing to do with the timer timing
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // Push-pull
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // Setup pull-up resistors
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	// Connect the timer output to the LED pins
	// Check the alternative function mapping in the CPU doc
	// http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00058181.pdf
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_2); // TIM1_CH1 -> LED3
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_2); // TIM1_CH2 -> LED7
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_2); // TIM1_CH2 -> LED8
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_2); // TIM1_CH3 -> LED10
}

//  * @brief  Initializes PWM
//  * @param  pwm_freq: Frequency of the PWM in Hz
//  * @retval Number of the timer pulses per one PWM period

int init_pwm(int pwm_freq)
{
	// Calculates the timing. This is common for all channels
	int clk = 72e6; // 72MHz -> system core clock. This is default on the stm32f3 discovery
	int tim_freq = 2e6; // in Hz (2MHz) Base frequency of the pwm timer
	int prescaler = ((clk / tim_freq) - 1);

	// Calculate the period for a given pwm frequency
	//		int pwm_freq = 200; // in Hz
	int pwm_period = tim_freq/pwm_freq;		// 2MHz / 200Hz = 10000
												// For 50Hz we get: 2MHz / 50Hz = 40000

	// Calculate a number of pulses per millisecond.
	// Not used in this rutine but I put it here just as an example
	int ms_pulses = (float)pwm_period / (1000.0/pwm_freq); // for 200Hz we get: 10000 / (1/200 * 1000) = 2000


//  Enable the TIM1 peripherie
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE );

// Setup the timing and configure the TIM1 timer
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseStructInit(& TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
	TIM_TimeBaseStructure.TIM_Period = pwm_period - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up ;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);


// Initialise the timer channels
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

	TIM_OCInitStructure.TIM_Pulse = ms_pulses*2; // preset pulse width 0..pwm_period
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // Pulse polarity
	//	  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

	// These settings must be applied on the timer 1.
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Set;

// Setup four channels
	// Channel 1
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

	// Channel 2
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

	// Channel 3
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

	// Channel 4
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

	// Starup the timer
	//TIM_ARRPreloadConfig(TIM1, DISABLE);
	TIM_ARRPreloadConfig(TIM1, DISABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	TIM_Cmd(TIM1 , ENABLE);

	// The PWM is running now. The pulse width can be set by
	// TIM1->CCR1 = [0..pwm_period] -> 0..100% duty cycle
	//
	// For example:
	// int pulse_width = 3000;
	// TIM1->CCR1 = pulse_width;
	//
	// The firmware offers a API to do this:
	// TIM_SetCompare1(TIM1 , pulse_width); // This is a wrapper for TIM1->CCR1, the same as TIM1->CCR1=pulse_width;

	return pwm_period;
}


//  * @brief  Sets the PWM duty cycle per channel
//  * @param  channel:  PWM channel index [1..4]
//  * @param  duty_cycle:  PWM duty cycle in percents (integer) [0..100]
//  * @retval None
//
void set_pwm_width(int channel, int pwm_period, int duty_cycle)
{
	int pwm_pulses = pwm_period*(float)duty_cycle/100.0;
	switch (channel){
		case 1: TIM_SetCompare1(TIM1, pwm_pulses); break;
		case 2: TIM_SetCompare2(TIM1, pwm_pulses); break;
		case 3: TIM_SetCompare3(TIM1, pwm_pulses); break;
		case 4: TIM_SetCompare4(TIM1, pwm_pulses); break;
	}
}


//  * @brief  Sets the PWM duty cycle per channel
//  * @param  channel:  PWM channel index [1..4]
//  * @param  duty_cycle:  PWM duty cycle (float) [0..1]
//  * @retval None

void set_pwm_width_norm(int channel, int pwm_period, float duty_cycle)
{
	int pwm_pulses = pwm_period*(float)duty_cycle;
	switch (channel){
		case 1: TIM_SetCompare1(TIM1, pwm_pulses); break;
		case 2: TIM_SetCompare2(TIM1, pwm_pulses); break;
		case 3: TIM_SetCompare3(TIM1, pwm_pulses); break;
		case 4: TIM_SetCompare4(TIM1, pwm_pulses); break;
	}
}

// Just simple exponential mapping
// Not important for this demo
float gammaCorrect(int b, int c)
{
	double f = ((double)b/(float)c);
	return f*f*f*f*f; // gamma = 5
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
void Demo_GyroConfig(void)
{
  L3GD20_InitTypeDef L3GD20_InitStructure;
  L3GD20_FilterConfigTypeDef L3GD20_FilterStructure;
  
  /* Configure Mems L3GD20 */
  L3GD20_InitStructure.Power_Mode = L3GD20_MODE_ACTIVE;
  L3GD20_InitStructure.Output_DataRate = L3GD20_OUTPUT_DATARATE_1;
  L3GD20_InitStructure.Axes_Enable = L3GD20_AXES_ENABLE;
  L3GD20_InitStructure.Band_Width = L3GD20_BANDWIDTH_4;
  L3GD20_InitStructure.BlockData_Update = L3GD20_BlockDataUpdate_Continous;
  L3GD20_InitStructure.Endianness = L3GD20_BLE_LSB;
  L3GD20_InitStructure.Full_Scale = L3GD20_FULLSCALE_500; 
  L3GD20_Init(&L3GD20_InitStructure);
   
  L3GD20_FilterStructure.HighPassFilter_Mode_Selection =L3GD20_HPM_NORMAL_MODE_RES;
  L3GD20_FilterStructure.HighPassFilter_CutOff_Frequency = L3GD20_HPFCF_0;
  L3GD20_FilterConfig(&L3GD20_FilterStructure) ;
  
  L3GD20_FilterCmd(L3GD20_HIGHPASSFILTER_ENABLE);
}

/**
  * @brief  Calculate the angular Data rate Gyroscope.
  * @param  pfData : Data out pointer
  * @retval None
  */
void Demo_GyroReadAngRate (float* pfData)
{
  uint8_t tmpbuffer[6] ={0};
  int16_t RawData[3] = {0};
  uint8_t tmpreg = 0;
  float sensitivity = 0;
  int i =0;

  L3GD20_Read(&tmpreg,L3GD20_CTRL_REG4_ADDR,1);
  
  L3GD20_Read(tmpbuffer,L3GD20_OUT_X_L_ADDR,6);
  
  /* check in the control register 4 the data alignment (Big Endian or Little Endian)*/
  if(!(tmpreg & 0x40))
  {
    for(i=0; i<3; i++)
    {
      RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i+1] << 8) + tmpbuffer[2*i]);
    }
  }
  else
  {
    for(i=0; i<3; i++)
    {
      RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i] << 8) + tmpbuffer[2*i+1]);
    }
  }
  
  /* Switch the sensitivity value set in the CRTL4 */
  switch(tmpreg & 0x30)
  {
  case 0x00:
    sensitivity=L3G_Sensitivity_250dps;
    break;
    
  case 0x10:
    sensitivity=L3G_Sensitivity_500dps;
    break;
    
  case 0x20:
    sensitivity=L3G_Sensitivity_2000dps;
    break;
  }
  /* divide by sensitivity */
  for(i=0; i<3; i++)
  {
    pfData[i]=(float)RawData[i]/sensitivity;
  }
}
/**
  * @brief  Configure the Mems to compass application.
  * @param  None
  * @retval None
  */
void Demo_CompassConfig(void)
{
  LSM303DLHCMag_InitTypeDef LSM303DLHC_InitStructure;
  LSM303DLHCAcc_InitTypeDef LSM303DLHCAcc_InitStructure;
  LSM303DLHCAcc_FilterConfigTypeDef LSM303DLHCFilter_InitStructure;

  /* Configure MEMS magnetometer main parameters: temp, working mode, full Scale and Data rate */
  LSM303DLHC_InitStructure.Temperature_Sensor = LSM303DLHC_TEMPSENSOR_DISABLE;
  LSM303DLHC_InitStructure.MagOutput_DataRate =LSM303DLHC_ODR_30_HZ ;
  LSM303DLHC_InitStructure.MagFull_Scale = LSM303DLHC_FS_8_1_GA;
  LSM303DLHC_InitStructure.Working_Mode = LSM303DLHC_CONTINUOS_CONVERSION;
  LSM303DLHC_MagInit(&LSM303DLHC_InitStructure);

   /* Fill the accelerometer structure */
  LSM303DLHCAcc_InitStructure.Power_Mode = LSM303DLHC_NORMAL_MODE;
  LSM303DLHCAcc_InitStructure.AccOutput_DataRate = LSM303DLHC_ODR_50_HZ;
  LSM303DLHCAcc_InitStructure.Axes_Enable= LSM303DLHC_AXES_ENABLE;
  LSM303DLHCAcc_InitStructure.AccFull_Scale = LSM303DLHC_FULLSCALE_2G;
  LSM303DLHCAcc_InitStructure.BlockData_Update = LSM303DLHC_BlockUpdate_Continous;
  LSM303DLHCAcc_InitStructure.Endianness=LSM303DLHC_BLE_LSB;
  LSM303DLHCAcc_InitStructure.High_Resolution=LSM303DLHC_HR_ENABLE;
  /* Configure the accelerometer main parameters */
  LSM303DLHC_AccInit(&LSM303DLHCAcc_InitStructure);

  /* Fill the accelerometer LPF structure */
  LSM303DLHCFilter_InitStructure.HighPassFilter_Mode_Selection =LSM303DLHC_HPM_NORMAL_MODE;
  LSM303DLHCFilter_InitStructure.HighPassFilter_CutOff_Frequency = LSM303DLHC_HPFCF_16;
  LSM303DLHCFilter_InitStructure.HighPassFilter_AOI1 = LSM303DLHC_HPF_AOI1_DISABLE;
  LSM303DLHCFilter_InitStructure.HighPassFilter_AOI2 = LSM303DLHC_HPF_AOI2_DISABLE;

  /* Configure the accelerometer LPF main parameters */
  LSM303DLHC_AccFilterConfig(&LSM303DLHCFilter_InitStructure);
}
/**
* @brief Read LSM303DLHC output register, and calculate the acceleration ACC=(1/SENSITIVITY)* (out_h*256+out_l)/16 (12 bit rappresentation)
* @param pnData: pointer to float buffer where to store data
* @retval None
*/
void Demo_CompassReadAcc(float* pfData)
{
  int16_t pnRawData[3];
  uint8_t ctrlx[2];
  uint8_t buffer[6], cDivider;
  uint8_t i = 0;
  float LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_2g;

  /* Read the register content */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, ctrlx,2);
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_L_A, buffer, 6);

  if(ctrlx[1]&0x40)
    cDivider=64;
  else
    cDivider=16;

  /* check in the control register4 the data alignment*/
  if(!(ctrlx[0] & 0x40) || (ctrlx[1] & 0x40)) /* Little Endian Mode or FIFO mode */
  {
    for(i=0; i<3; i++)
    {
      pnRawData[i]=((int16_t)((uint16_t)buffer[2*i+1] << 8) + buffer[2*i])/cDivider;
    }
  }
  else /* Big Endian Mode */
  {
    for(i=0; i<3; i++)
      pnRawData[i]=((int16_t)((uint16_t)buffer[2*i] << 8) + buffer[2*i+1])/cDivider;
  }
  /* Read the register content */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, ctrlx,2);


  if(ctrlx[1]&0x40)
  {
    /* FIFO mode */
    LSM_Acc_Sensitivity = 0.25;
  }
  else
  {
    /* normal mode */
    /* switch the sensitivity value set in the CRTL4*/
    switch(ctrlx[0] & 0x30)
    {
    case LSM303DLHC_FULLSCALE_2G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_2g;
      break;
    case LSM303DLHC_FULLSCALE_4G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_4g;
      break;
    case LSM303DLHC_FULLSCALE_8G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_8g;
      break;
    case LSM303DLHC_FULLSCALE_16G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_16g;
      break;
    }
  }

  /* Obtain the mg value for the three axis */
  for(i=0; i<3; i++)
  {
    pfData[i]=(float)pnRawData[i]/LSM_Acc_Sensitivity;
  }

}
/**
  * @brief  calculate the magnetic field Magn.
* @param  pfData: pointer to the data out
  * @retval None
  */
void Demo_CompassReadMag (float* pfData)
{
  static uint8_t buffer[6] = {0};
  uint8_t CTRLB = 0;
  uint16_t Magn_Sensitivity_XY = 0, Magn_Sensitivity_Z = 0;
  uint8_t i =0;
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_CRB_REG_M, &CTRLB, 1);

  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_H_M, buffer, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_L_M, buffer+1, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_H_M, buffer+2, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_L_M, buffer+3, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_H_M, buffer+4, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_L_M, buffer+5, 1);
  /* Switch the sensitivity set in the CRTLB*/
  switch(CTRLB & 0xE0)
  {
  case LSM303DLHC_FS_1_3_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_1_3Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_1_3Ga;
    break;
  case LSM303DLHC_FS_1_9_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_1_9Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_1_9Ga;
    break;
  case LSM303DLHC_FS_2_5_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_2_5Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_2_5Ga;
    break;
  case LSM303DLHC_FS_4_0_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_4Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_4Ga;
    break;
  case LSM303DLHC_FS_4_7_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_4_7Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_4_7Ga;
    break;
  case LSM303DLHC_FS_5_6_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_5_6Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_5_6Ga;
    break;
  case LSM303DLHC_FS_8_1_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_8_1Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_8_1Ga;
    break;
  }

  for(i=0; i<2; i++)
  {
    pfData[i]=(float)((int16_t)(((uint16_t)buffer[2*i] << 8) + buffer[2*i+1])*1000)/Magn_Sensitivity_XY;
  }
  pfData[2]=(float)((int16_t)(((uint16_t)buffer[4] << 8) + buffer[5])*1000)/Magn_Sensitivity_Z;
}
void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
	  if (TimingDelay != 0x00)
	  {
		  TimingDelay--;
	  }
}
void Timing_Delay(uint32_t Delay)
{
  if (Delay != 0x00)
  { 
    Delay--;
  }
}
/**
  * @brief  Basic management of the timeout situation.
  * @param  None.
  * @retval None.
  */
uint32_t L3GD20_TIMEOUT_UserCallback(void)
{
  return 0;
}
/**
  * @brief  Basic management of the timeout situation.
  * @param  None.
  * @retval None.
  */
uint32_t LSM303DLHC_TIMEOUT_UserCallback(void)
{
  return 0;
}

void RCC_Configuration(void)
{
  /// Enable GPIO clock
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  // Enable USART clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);


}

void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // Connect PA9 to USART1_Tx
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_7);

  // Connect PA10 to USART1_Rx
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_7);

  // Configure USART Tx as alternate function push-pull
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // Configure USART Rx as alternate function push-pull
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}
void GPIO_Configuration2(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // Connect PD5 to USART1_Tx
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_7);

  // Connect PD6 to USART1_Rx
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_7);

  // Configure USART Tx as alternate function push-pull
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // Configure USART Rx as alternate function push-pull
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void USART1_Configuration(void)
{
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

//   USART resources configuration (Clock, GPIO pins and USART registers) ----
//   USART configured as follow:
//        - BaudRate = 115200 baud
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
void USART3_Configuration(void)
{
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  // USART configuration
  USART_Init(USART3, &USART_InitStructure);
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  // Enable USART
  USART_Cmd(USART3, ENABLE);
}
void USART1_Send(char character)
{
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); // Wait for Empty

    USART_SendData(USART1, character);
}


void Config_ADC(uint16_t GPIO_PIN, uint16_t ADC_Channel)
{
	uint32_t timedelay = 100;
    ADC_InitTypeDef       ADC_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    GPIO_InitTypeDef      GPIO_InitStructure;

    // Configure the ADC clock
    RCC_ADCCLKConfig( RCC_ADC12PLLCLK_Div2 );

    // Enable ADC1 clock
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_ADC12, ENABLE );

    // ADC Channel configuration
    // GPIOC Periph clock enable
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOC, ENABLE );

    // Configure ADC Channel7 as analog input
    GPIO_InitStructure.GPIO_Pin = GPIO_PIN ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init( GPIOC, &GPIO_InitStructure );

    ADC_StructInit( &ADC_InitStructure );

    // Calibration procedure
    ADC_VoltageRegulatorCmd( ADC1, ENABLE );

    // Insert delay equal to 10 µs
    //TaskDelay(5);
    Timing_Delay(timedelay);

    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Clock = ADC_Clock_SynClkModeDiv4; //ADC_Clock_AsynClkMode;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_OneShot;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;
    ADC_CommonInit( ADC1, &ADC_CommonInitStructure );

    ADC_SelectCalibrationMode( ADC1, ADC_CalibrationMode_Single );
    ADC_StartCalibration( ADC1 );

    while ( ADC_GetCalibrationStatus( ADC1 ) != RESET );
    calibration_value = ADC_GetCalibrationValue( ADC1 );

    ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Disable;
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;
    ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;
    ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;
    ADC_InitStructure.ADC_NbrOfRegChannel = 1;
    ADC_Init( ADC1, &ADC_InitStructure );

    // ADC1 regular channel configuration
    ADC_RegularChannelConfig( ADC1, ADC_Channel, 1, ADC_SampleTime_7Cycles5 );

    // Enable ADC1
    ADC_Cmd( ADC1, ENABLE );

    // wait for ADRDY
    while( !ADC_GetFlagStatus( ADC1, ADC_FLAG_RDY ) );

    // Start ADC1 Software Conversion
    ADC_StartConversion( ADC1 );
}
uint16_t Get_Voltage()
{
	 	uint16_t  ADC1ConvertedValue = 0;

	    // Test EOC flag
	    while ( ADC_GetFlagStatus( ADC1, ADC_FLAG_EOC ) == RESET );

	    // Get ADC1 converted data
	    ADC1ConvertedValue = ADC_GetConversionValue( ADC1 );

	    // Compute the voltage (3.3V @ 12bit resolution)
	    ADC1ConvertedValue = ( ADC1ConvertedValue * 3300 ) / 0xFFF;
	    //ADC1ConvertedValue = ( ADC1ConvertedValue * 3000 ) / 0xFFF;

	    ADC_StopConversion(ADC1);
	    //ADC_DisableCmd(ADC1);
	    ADC_DeInit(ADC1);

	    return ADC1ConvertedValue;
}
uint32_t Get_Temperature()
{
	__IO uint32_t Temp;
	__IO uint32_t Voltage;
	Config_ADC(GPIO_Pin_1, ADC_Channel_7);
	Voltage = Get_Voltage();

	Voltage = Voltage * 10000;
	Temp = Voltage * 27.992;
	Temp = Temp - 227350000;

	return Temp;
}
uint32_t Get_Baro()
{
	__IO uint32_t Pressure = 0;
	__IO uint32_t Voltage = 0;
	Config_ADC(GPIO_Pin_0, ADC_Channel_6);
	Voltage = Get_Voltage();

	Voltage = Voltage * 10000;
	Pressure = Voltage / 0.036278;//0.68*Vdd*a
	//Pressure = Pressure / 1000;
	Pressure = Pressure + 306157000;//-a/b

	return Pressure;
}

float Calc_altitude(uint32_t sea_press, uint32_t press, uint32_t temp)
{
	uint8_t i=0;
	long double exp=0;
	double height;
	//double exp=1/5.257;
	height = (double)sea_press / (double)press;
	exp=height*height;
	for(i=1; i<100;i=i+1)
	{
		exp=exp*height;
	}
	height=root(height, 525);
    //height = powf(height, 0.5f);// 0.1902225604f);
    height = height-1;
    temp = temp + 2731500;
    temp=temp/10000;
    height = height * temp;
    height = height / 0.0065;

    return height;
}
inline double abs_(double x) { return x >= 0 ? x : -x; }
double pow_(double x, int e)
{
	double ret = 1;
	for (ret = 1; e; x *= x, e >>= 1)
		if ((e & 1)) ret *= x;
	return ret;
}
/*
double root(double a, uint8_t n)
{
	double d, x = 1;
	if (!a) return 0;
	if (n < 1 || (a < 0 && !(n&1))) return 0./0.; // NaN

	do {	d = (a / pow_(x, n - 1) - x) / n;
		x+= d;
	} while (abs_(d) >= abs_(x) * (DBL_EPSILON * 10));

	return x;
}*/
void Display_Pressure(uint32_t value)
{
	char dig, i;
	char message[10];
	uint32_t a = 1;
	for(i=0; i <= 10; i=i+1)
	{
		dig = value % 10;
		value = value / 10;
		if (i==6)
		{
			message[i]='.';
			i=i+1;
		}
		message[i]=dig+48;
		//a=a*10;
	}
	for(i=10; i > 0; i=i-1)
	{
		USART1_Send(message[i]);
	}
	USART1_Send('m');
	USART1_Send('b');
	USART1_Send('a');
	USART1_Send('r');
	for(i=6;i>0;i=i-1)
	{
		USART1_Send(0x1B);
		USART1_Send(0x5B);
		USART1_Send(0x41);
	}
		USART1_Send('\r');
}
void Display_Temperature(uint32_t value)
{
	char dig, i;
	char message[9];
	uint32_t a = 1;
	for(i=0; i <= 9; i=i+1)
	{
		dig = value % 10;
		value = value / 10;
		if (i==7)
		{
			message[i]='.';
			i=i+1;
		}
		message[i]=dig+48;
		//a=a*10;
	}
	for(i=9; i > 0; i=i-1)
	{
		USART1_Send(message[i]);
	}
	USART1_Send('C');
    USART1_Send('\n');
    USART1_Send('\r');

}
double Display_Longitude(uint16_t value1, float value2 )
{
	char dig, i;
	uint16_t dec = value1;
	char message1[3];
	char message2[6];
	double dec_deg = 0;
	uint16_t a = 1;
	uint32_t num;
	for(i=0; i <= 3; i=i+1)
	{
		dig = value1 % 10;
		value1 = value1 / 10;
		message1[i]=dig+48;
		//a=a*10;
	}
	for(i=3; i > 0; i=i-1)
	{
		USART1_Send(message1[i-1]);
	}
	USART1_Send('.');
	num = value2 * 10000;
	num = num / 60;
	for(i=0; i <= 6; i=i+1)
	{
		dig = num % 10;
		num = num / 10;
		message1[i]=dig+48;
		//a=a*10;
	}
	for(i=4; i > 0; i=i-1)
	{
		USART1_Send(message1[i-1]);
	}
	USART1_Send(0xF8);//degrees
    USART1_Send('\n');
    USART1_Send('\r');

    dec_deg = num / 100000;
    dec_deg = dec_deg + dec;

    return dec_deg;
}
void Display_Altitude(uint16_t value)
{
	char dig, i;
	char message[5];
	dig = value % 10;
	value = value / 10;
	message[0] = dig + 48;
	message[1] = '.';
	dig = value % 10;
	value = value / 10;
	message[2] = dig + 48;
	dig = value % 10;
	value = value / 10;
	message[3] = dig + 48;
	dig = value % 10;
	value = value / 10;
	message[4] = dig + 48;
	dig = value % 10;
	value = value / 10;
	message[5] = dig + 48;
	for(i=5; i != 0; i=i-1)
	{
		USART1_Send(message[i-1]);
	}
	USART1_Send('M');
	USART1_Send('\n');
	USART1_Send('\r');
}

void Display_Heading(float value)
{
	uint8_t dig, i;
	float temp;
	char message[7];
	temp = value;
	//temp = (uint8_t)temp - temp;
	dig = temp*1000;
	dig = dig %10;
	message[0] = dig+48;
	dig = temp*100;
	dig = dig %10;
	message[1] = dig+48;
	dig = temp*10;
	dig = dig %10;
	message[2] = dig+48;
	message[3] = '.';
	dig = ((uint8_t)value % 10)+48;
	message[4] = dig;
	value = value/10;
	dig = ((uint8_t)value % 10)+48;
	message[5] = dig;
	value = value/10;
	dig = ((uint8_t)value % 10)+48;
	message[6] = dig;
	for(i=7; i != 0; i=i-1)
	{
		USART1_Send(message[i-1] );
	}
	USART1_Send(0xF8);//degrees
    USART1_Send('\n');
    USART1_Send('\r');
}
void USART1_IRQHandler(void)
{
	while((USART1->ISR & USART_FLAG_RXNE) == (uint16_t)RESET);
	//if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	//{
	rx = USART_ReceiveData(USART1);

   duty_cycle1=rx;
   duty_cycle2=rx;
   duty_cycle3=rx;
   duty_cycle4=rx;
	//}
}
void USART3_IRQHandler(void)
{

	while((USART3->ISR & USART_FLAG_RXNE) == (uint16_t)RESET);
	USART3_rx = USART_ReceiveData(USART3);
	if (USART3_rx == '$')
	{
		string_pointer = 0;
	}
	NEMA_String[string_pointer]=USART3_rx;
	USART1_Send(USART3_rx);
	//USART1_Send(string_pointer + '0');
	string_pointer++;
	if(USART3_rx == '\r' || USART3_rx == '\n')
	{
		  //NVIC_InitTypeDef NVIC_InitStructure;
		  USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
		  USART1_Send('\r');
		  USART1_Send('\n');
		  //NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
		  //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		  //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		  //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		  //NVIC_Init(&NVIC_InitStructure);
	}
}
/*
void EXTI0_IRQHandler(void){
  EXTI_ClearITPendingBit(EXTI_Line0);
  power = 1;
  //arm_sequence();
}
*/
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
