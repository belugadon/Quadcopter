#include "BLDC_Control.h"
//#include "float.h"
//#include "arm_math.h"
//#include "core_CM4.h"


#define ABS(x)         (x < 0) ? (-x) : x
#define PI                         (float)     3.14159265f
#define RadToDeg                   (int)  57295

uint8_t PID = ENABLE;
uint8_t Xval, Yval = 0x00;
float Buffer[3] = {0.0f}, AccBuffer2[3] = {0.0f};
int offsetA = 7000;
int offsetB = 7000;
int offsetC = 7000;
int offsetD = 7000;
int offsetA_High;
int offsetB_High;
int offsetC_High;
int offsetD_High;
int offsetA_Low;
int offsetB_Low;
int offsetC_Low;
int offsetD_Low;
int duty_cycleC;
int duty_cycleB;
int duty_cycleA;
int duty_cycleD;
int XSum_Of_Gyro;
int YSum_Of_Gyro;
int XTotal_Rotation;
int YTotal_Rotation;
float AccYanglefloat;
int AccXangle;
int AccYangle;
int chasetheX = 0.0;//positional setpoint
int chasetheY = 0.0;//positional setpoint
int SUMof_XError = 0;
int SUMof_YError = 0;
int XLastError = 0;
int YLastError = 0;
float ControlX_Out = 0;
float ControlY_Out = 0;
float KpX =  0.0155;//0.031;//0.08;
float KiX = 0.0005;//0.0007;
float KdX= 0.065;//0.089;
float KpY = 0.0155;//0.08;
float KiY = 0.0005;//0.5;
float KdY= 0.065;
int pwm_period;
int ms_pulses2;
int prescaler2;
int interrupt_frequency = 45;//150;
int interrupt_period_int;
float interrupt_period_float;

//Set up the timer and schedule interruptions
void schedule_PI_interrupts()
{
	//cortexm4f_enable_fpu();
	interrupt_period_int = (1/(float)interrupt_frequency)*1000;
	interrupt_period_float = 1/(float)interrupt_frequency;
	int clk = 36e6; // 36MHz -> system core clock. This is default on the stm32f3 discovery
	/*
	 * prescaler = ((clk/interrupt_frequency)/interrupt_frequency) - 1
	 * This equation returns the PID interrupt frequency configuration value for TIM_Prescaler
	 */

	int prescaler = ((clk / interrupt_frequency)/interrupt_frequency)-1;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	pwm_period = slow_init_pwm(700);
	TIM_TimeBaseInitTypeDef timerInitStructure;
	timerInitStructure.TIM_Prescaler = prescaler;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = interrupt_frequency - 1;
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	timerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &timerInitStructure);
	TIM_Cmd(TIM2, ENABLE);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    NVIC_InitTypeDef nvicStructure;
    nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
    nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
    nvicStructure.NVIC_IRQChannelSubPriority = 1;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure);

}
void cortexm4f_enable_fpu() {
    /* set CP10 and CP11 Full Access */
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));
}

void disable_PI_control()
{
	PID = DISABLE;
}
void Set_Offset(int* value, float* roll, float* pitch, int* yaw)
{
	//chasetheY = (*pitch * 0.1) + (*roll * 0.11);
	//chasetheX = (*roll * 0.11) - (*pitch * 0.1);
	offsetA = 6900 + *value + (*value * (*roll))/2 + (*value * (0 - *pitch))/2;
	offsetB = 6900 + *value + (*value * (*roll))/2 + (*value * (*pitch))/2;
	offsetC = 6900 + *value + (*value * (0 - *roll))/2 + (*value * (*pitch))/2;
	offsetD = 6900 + *value + (*value * (0 - *roll))/2 + (*value * (0 - *pitch))/2;
	//offsetB = offsetB * ((*roll) + (*pitch)/2);
	//offsetC = offsetC * ((0 - *roll) + (*pitch)/2);
	//offsetD = offsetD * ((0 - *roll) + (0 - *pitch)/2);
	//offsetA = 7000 + ((*value * (*roll)) * (0 - *pitch));
	//offsetB = 7000 + ((*value * (*roll)) * (*pitch));
	//offsetC = 7000 + ((*value * (0 - *roll)) * (*pitch));
	//offsetD = 7000 + ((*value * (0 - *roll)) * (0 - *pitch));
	offsetA = offsetA + *yaw/2;
	offsetB = offsetB - *yaw/2;
	offsetC = offsetC + *yaw/2;
	offsetD = offsetD - *yaw/2;
	offsetA_High = offsetA + 2000;
	offsetB_High = offsetB + 2000;
	offsetC_High = offsetC + 2000;
	offsetD_High = offsetD + 2000;
	offsetA_Low = offsetA - 2000;
	offsetB_Low = offsetB - 2000;
	offsetC_Low = offsetC - 2000;
	offsetD_Low = offsetD - 2000;

}
void Set_Offset1(int* value, int* roll, int* pitch, int* yaw)
{
	chasetheY = (*pitch * 0.1) + (*roll * 0.11);
	chasetheX = (*roll * 0.11) - (*pitch * 0.1);
	offsetA = (*value + 6900);//7000);// * 0.78;
	offsetB = (*value + 6900);//7000);// * 0.79;
	offsetC = (*value + 6900);// * 1.24;
	offsetD = (*value + 6900);//* 1.17;
	offsetA = offsetA + *yaw;
	offsetB = offsetB - *yaw;
	offsetC = offsetC + *yaw;
	offsetD = offsetD - *yaw;
	offsetA_High = offsetA + 2000;
	offsetB_High = offsetB + 2000;
	offsetC_High = offsetC + 2000;
	offsetD_High = offsetD + 2000;
	offsetA_Low = offsetA - 2000;
	offsetB_Low = offsetB - 2000;
	offsetC_Low = offsetC - 2000;
	offsetD_Low = offsetD - 2000;

}
void Set_Offset2(int* value, int* roll, int* pitch, int* yaw)
{
	//chasetheX = *pitch + *roll;
	//chasetheY = *roll - *pitch;
	offsetA = *value + *pitch + *roll;//+1;
	offsetB = *value - *roll + *pitch;
	offsetC = *value - *pitch - *roll;
	offsetD = *value + *roll - *pitch;//+1;
	offsetA = offsetA + *yaw;
	offsetB = offsetB - *yaw;
	offsetC = offsetC + *yaw;
	offsetD = offsetD - *yaw;
	offsetA_High = offsetA + 2000;
	offsetB_High = offsetB + 2000;
	offsetC_High = offsetC + 2000;
	offsetD_High = offsetD + 2000;
	offsetA_Low = offsetA - 1800;
	offsetB_Low = offsetB - 1800;
	offsetC_Low = offsetC - 1800;
	offsetD_Low = offsetD - 1800;

}

void Adjust_Yaw(int* value)
{
	duty_cycleC = duty_cycleC + *value;
	duty_cycleB = duty_cycleB - *value;
	duty_cycleA = duty_cycleA + *value;
	duty_cycleD = duty_cycleD - *value;
}

//void Delay(uint32_t delay) {
//    while (delay--);
//}
void update_control_signals(uint16_t* width)
{
	duty_cycleC = *width;
	duty_cycleB = *width;
	duty_cycleA = *width;
	duty_cycleD = *width;
}
void init_BT_Pair_Sense()
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
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    //EXTI_Init(&EXTI_InitStructure);


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
	if(duty_cycleC >= offsetC_High)
	{
		duty_cycleC = offsetC_High;
	}
	else if(duty_cycleC <= offsetC_Low)
	{
		duty_cycleC = offsetC_Low;
	}
	if(duty_cycleB >= offsetB_High)
	{
		duty_cycleB = offsetB_High;
	}
	else if(duty_cycleB <= offsetB_Low)
	{
		duty_cycleB = offsetB_Low;
	}
	if(duty_cycleA >= offsetA_High)
	{
		duty_cycleA = offsetA_High;
	}
	else if(duty_cycleA <= offsetA_Low)
	{
		duty_cycleA = offsetA_Low;
	}
	if(duty_cycleD >= offsetD_High)
	{
		duty_cycleD = offsetD_High;
	}
	else if(duty_cycleD <= offsetD_Low)
	{
		duty_cycleD = offsetD_Low;
	}
}
void low_bounds_check()
{
	if(duty_cycleC >= offsetC)
	{
		duty_cycleC = offsetC;
	}
	else if(duty_cycleC <= offsetC-1000)
	{
		duty_cycleC = offsetC-1000;
	}
	if(duty_cycleB >= offsetB)
	{
		duty_cycleB = offsetB;
	}
	else if(duty_cycleB <= offsetB-1000)
	{
		duty_cycleB = offsetB-1000;
	}
	if(duty_cycleA >= offsetA)
	{
		duty_cycleA = offsetA;
	}
	else if(duty_cycleA <= offsetA-1000)
	{
		duty_cycleA = offsetA-1000;
	}
	if(duty_cycleD >= offsetD)
	{
		duty_cycleD = offsetD;
	}
	else if(duty_cycleD <= offsetD-1000)
	{
		duty_cycleD = offsetD-1000;
	}
}
void arm_sequence()
{
	/*
	 * This function is used to arm the ESC by setting the PWM throttle control
	 * to it's max value, holding for 2 seconds and then ramping the PWM output to
	 * it's minimum throttle value.
	 */
	duty_cycleC = 18000;
	duty_cycleB = 18000;
	duty_cycleA = 18000;
	duty_cycleD = 18000;
	pwm_period = slow_init_pwm(700);
	int timedelay1 = 5000000;
	int timedelay2 = 3000;
	int timedelay3 = 3000000;

	set_pwm_width(1, pwm_period, duty_cycleC);
	set_pwm_width(2, pwm_period, duty_cycleB);
	set_pwm_width(3, pwm_period, duty_cycleA);
	set_pwm_width(4, pwm_period, duty_cycleD);

	while(timedelay1 >= 0)
	{
		timedelay1--;
	}
	while(duty_cycleC >= 8000)
	{
		while(timedelay2 >= 0)
		{
			timedelay2--;
		}
		duty_cycleC=duty_cycleC-1000;
		duty_cycleB=duty_cycleB-1000;
		duty_cycleA=duty_cycleA-1000;
		duty_cycleD=duty_cycleD-1000;

		set_pwm_width(1, pwm_period, duty_cycleC);
		set_pwm_width(2, pwm_period, duty_cycleB);
		set_pwm_width(3, pwm_period, duty_cycleA);
		set_pwm_width(4, pwm_period, duty_cycleD);
		timedelay2 = 3000;
	}
	while(timedelay3 >= 0)
	{
		timedelay3--;
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
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_2); // TIM1_CH4 -> LED8
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_2); // TIM1_CH3 -> LED10
}

//  * @brief  Initializes PWM
//  * @param  pwm_freq: Frequency of the PWM in Hz
//  * @retval Number of the timer pulses per one PWM period

int init_pwm()
{

//  Enable the TIM1 peripherie
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE );

// Setup the timing and configure the TIM1 timer
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseStructInit(& TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = prescaler2;
	TIM_TimeBaseStructure.TIM_Period = 2860;//2857;//pwm_period - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up ;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);


// Initialise the timer channels
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

	TIM_OCInitStructure.TIM_Pulse = ms_pulses2*2; // preset pulse width 0..pwm_period
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
}
int slow_init_pwm(int pwm_freq)
{
	// Calculates the timing. This is common for all channels
	int clk = 72e6; // 72MHz -> system core clock. This is default on the stm32f3 discovery
	int tim_freq = 2e6; // in Hz (2MHz) Base frequency of the pwm timer
	int prescaler = ((clk / tim_freq) - 1);
	prescaler2 = ((clk / tim_freq) - 1);
	// Calculate the period for a given pwm frequency
	//		int pwm_freq = 200; // in Hz
	int pwm_period = tim_freq/pwm_freq;		// 2MHz / 200Hz = 10000
												// For 50Hz we get: 2MHz / 50Hz = 40000

	// Calculate a number of pulses per millisecond.
	// Not used in this routine but I put it here just as an example
	int ms_pulses = (float)pwm_period / (1000.0/pwm_freq); // for 200Hz we get: 10000 / (1/200 * 1000) = 2000
	ms_pulses2 = (float)pwm_period / (1000.0/pwm_freq);

//  Enable the TIM1 peripherie
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE );

// Setup the timing and configure the TIM1 timer
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseStructInit(& TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
	TIM_TimeBaseStructure.TIM_Period = 2857;//pwm_period - 1;
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

	return pwm_period;
}


//  * @brief  Sets the PWM duty cycle per channel
//  * @param  channel:  PWM channel index [1..4]
//  * @param  duty_cycle:  PWM duty cycle in percents (integer) [0..100]
//  * @retval None
//
void set_pwm_width(int channel, int pwm_period, uint32_t duty_cycle)//float* duty_cycle)
{
	int pwm_pulses = (pwm_period*duty_cycle)/100000;
	//int pwm_pulses = pwm_period*(*duty_cycle)/100.0;
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
void Display_Axis(int value)
{
	int dig, i;
	int temp;
	char message[8];
	//temp = (uint8_t)temp - temp;
	if (value < 0)
	{
		temp = value*-1;
		dig = temp %10;
		temp = temp/10;
		message[0] = dig+48;
		dig = temp %10;
		temp = temp/10;
		message[1] = dig+48;
		dig = temp %10;
		temp = temp/10;
		message[2] = dig+48;
		//dig = temp %10;
		//temp = temp/10;
		message[3] = '.';
		dig = temp %10;
		temp = temp/10;
		message[4] = dig+48;
		dig = temp %10;
		temp = temp/10;
		message[5] = dig+48;
		dig = temp %10;
		temp = temp/10;
		message[6] = dig+48;
		message[7] = '-';
		//USART1_Send('-');
		for(i=8; i != 0; i=i-1)
		{
			USART1_Send(message[i-1] );
		}
	}
	else {
		temp = value;
		dig = temp %10;
		temp = temp/10;
		message[0] = dig+48;
		dig = temp %10;
		temp = temp/10;
		message[1] = dig+48;
		dig = temp %10;
		temp = temp/10;
		message[2] = dig+48;
		//dig = temp %10;
		//temp = temp/10;
		message[3] = '.';
		dig = temp %10;
		temp = temp/10;
		message[4] = dig+48;
		dig = temp %10;
		temp = temp/10;
		message[5] = dig+48;
		dig = temp %10;
		temp = temp/10;
		message[6] = dig+48;
		for(i=7; i != 0; i=i-1)
		{
			USART1_Send(message[i-1] );
		}
	}
	//USART1_Send(0xF8);//degrees
    //USART1_Send('\n');
    //USART1_Send('\r');
}
void Decrease_Angular_Position(uint8_t value)
{
	if (value == 'x')
	{
		XTotal_Rotation--;
	}
	else if (value == 'y')
	{
		YTotal_Rotation--;
	}
}
void Increase_Angular_Position(uint8_t value)
{
	if (value == 'x')
	{
		XTotal_Rotation++;
	}
	else if (value == 'y')
	{
		YTotal_Rotation++;
	}
}
Display_DC(int value)
{
	uint8_t dig1, dig2;
	dig1 = value % 10;
	value = value/10;
	dig2 = value % 10;
	USART1_Send(dig2+48);
	USART1_Send(dig1+48);
}

void TIM2_IRQHandler()
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
    	int Xerror = 0; //instantaneous error
    	int Yerror = 0;
    	int SlopeofXError = 0;
    	int SlopeofYError = 0;
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        init_pwm();
        if (offsetA >= 8500){
        Demo_GyroReadAngRate(Buffer);//read the angular rate from the gyroscope and store in Buffer[]

        //XSamples[2] = XSamples[1];
        //XSamples[3] = XSamples[2];
        //XSamples[4] = XSamples[3];
        //XSamples[5] = XSamples[4];

        Demo_CompassReadAcc(AccBuffer2);
        //AccYangle = ((atan2f((float)AccBuffer2[1],(float)AccBuffer2[2]))*180)/PI;
        //AccXangle = ((atan2f((float)AccBuffer2[2],(float)AccBuffer2[0]))*180)/PI;
        AccYangle = ((atan2f((float)AccBuffer2[1],(float)AccBuffer2[2]))*RadToDeg);//*180)/PI;
        //AccXangle = ((atan2f((float)AccBuffer2[2],(float)AccBuffer2[0]))*RadToDeg);//*180)/PI;
        AccXangle = ((atan2f((float)AccBuffer2[0],(float)AccBuffer2[2]))*RadToDeg);//*180)/PI;

        /*
         * The following lines of code create the control components. The respective axis' angular rate(Buffer[n])
         * is multiplied by the time since the containing interrupt was last called(in seconds(1/interrupt_frequency)).
         * angular rate multiplied by time passed gives us the amount of displacement(in degrees) that took place
         * over that period. Summing each periods individual displacement gives the total(current) angular
         * displacement. The difference between the current displacement and the setpoint(chasetheN) gives us the
         *  N axis error and P component of the algorithm. A summation of the P component over time is equivalent,
         *  in this context, to the integral of the error and is used as the I component.
        */

        //Multiplying angular rate(Buffer[n]) by the period gives displacement for that period
        //adding the last displacement to the total returns the gyro's current angular displacement
        //if((Buffer[0] > 1.5) || (Buffer[0] < -3)){
        XSum_Of_Gyro = (XSum_Of_Gyro + Buffer[0]*108);//interrupt_period_int);//0.98*(XTotal_Rotation + Buffer[0]*0.06) + (0.02*AccXangle);
        //}
        //XTotal_Rotation = XSum_Of_Gyro;
        XTotal_Rotation = ((XSum_Of_Gyro*-0.98) + (AccXangle*0.02));///10;
        //if((Buffer[1] > 1.5) || (Buffer[1] < -3)){
        YSum_Of_Gyro = (YSum_Of_Gyro + Buffer[1]*108);//interrupt_period_int);//0.98*(XTotal_Rotation + Buffer[0]*0.06) + (0.02*AccXangle);
        //}
        //YTotal_Rotation = YSum_Of_Gyro;
        YTotal_Rotation = ((YSum_Of_Gyro*-0.98) + (AccYangle*0.02));///10;
        //YMean =
        //YTotal_Rotation = AccYangle*-1;
        //the difference between the current displacement and the setpoint is the error and P component
        Xerror = chasetheX - XTotal_Rotation;
        Yerror = chasetheY - YTotal_Rotation;

        //The integral(I) component is created by multiplying the error by the period
        //and summing each individual periods error
        if (((duty_cycleA >= offsetA_High) || (duty_cycleC >= offsetC_High)) || ((duty_cycleA <= offsetA_Low) || (duty_cycleC <= offsetC_Low)))
        {
        	SUMof_XError = SUMof_XError;
        }
        else {
        SUMof_XError = SUMof_XError + Xerror;//*interrupt_period_float;
        }
        if (((duty_cycleB >= offsetB_High) || (duty_cycleD >= offsetD_High))|| ((duty_cycleB <= offsetB_Low) || (duty_cycleD <= offsetD_Low)))
        {
        	SUMof_YError = SUMof_YError;
        }
        else {
        SUMof_YError = SUMof_YError + Yerror;//*interrupt_period_float;
        }

        //Derivative(D) Component
        SlopeofXError = (Xerror - XLastError);//*interrupt_frequency;///interrupt_period_float;
        XLastError = Xerror;
        SlopeofYError = (Yerror - YLastError);//*interrupt_frequency;///interrupt_period_float;
        YLastError = Yerror;


        //We can now assemble the control output by multiplying each control component by it's associated
        //gain coefficient and summing the results
        //ControlX_Out = (KpX*Xerror)+(KiX*SUMof_XError)+(KdX*SlopeofXError);
        //ControlY_Out = (KpY*Yerror)+(KiY*SUMof_YError)+(KdY*SlopeofYError);
        ControlX_Out = (0.5 * Xerror)+(0.2 * SUMof_XError)+(0.5 * SlopeofXError);
        ControlY_Out = (0.5 * Yerror)+(0.2 * SUMof_YError)+(0.5 * SlopeofYError);
        }
        else{
        ControlX_Out = 0;
        ControlY_Out = 0;
        }
        duty_cycleC = ControlX_Out + offsetC;
        duty_cycleA = 0 - (ControlX_Out) + offsetA;// * 1.1);
        duty_cycleD = (ControlY_Out) + offsetD;// * 1.6);
        duty_cycleB = 0 - ControlY_Out + offsetB;

        //duty_cycleC = duty_cycleC;
        //duty_cycleA = duty_cycleA;
        //duty_cycleB = duty_cycleB;
        //duty_cycleD = duty_cycleD;

        if(PID == ENABLE)
        {
        	bounds_check();
        }
        else if(PID == DISABLE)
        {
        	low_bounds_check();
        }
        	//Display_DC(duty_cycleD);
        	//USART1_Send(' ');
        	set_pwm_width(2, pwm_period, duty_cycleD);
        	//Display_DC(duty_cycleC);
        	//USART1_Send(' ');
        	set_pwm_width(1, pwm_period, duty_cycleC);
        	//Display_DC(duty_cycleB);
        	//USART1_Send(' ');
        	set_pwm_width(4, pwm_period, duty_cycleB);
        	//Display_DC(duty_cycleA);
        	//USART1_Send(' ');
        	set_pwm_width(3, pwm_period, duty_cycleA);



        //USART1_Send('X');
        //USART1_Send(':');
        //USART1_Send(',');
        //Display_Axis(XTotal_Rotation);
        //USART1_Send(' ');
        //Display_Axis(chasetheX);
        //USART1_Send(' ');
        //USART1_Send('\n');
        //USART1_Send('\r');
        //USART1_Send('Y');
        //USART1_Send(':');
        //USART1_Send(' ');
        //Display_Axis(YTotal_Rotation);
        //USART1_Send(',');
        //Display_Axis(Xerror);
        //USART1_Send(',');
        //USART1_Send('\n');
        //USART1_Send('\r');

/*
        USART1_Send('A');
        USART1_Send(':');
        Display_Axis(duty_cycleA);
        USART1_Send(',');
        USART1_Send(' ');
        USART1_Send('B');
        USART1_Send(':');
        Display_Axis(duty_cycleB);
        USART1_Send(',');
        USART1_Send(' ');
        USART1_Send('C');
        USART1_Send(':');
        Display_Axis(duty_cycleC);
        USART1_Send(',');
        USART1_Send(' ');
        USART1_Send('D');
        USART1_Send(':');
        Display_Axis(duty_cycleD);
        //USART1_Send(',');
        //USART1_Send(' ');
        //USART1_Send('\n');
        USART1_Send('\r');
*/
    }
}
