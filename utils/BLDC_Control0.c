#include "BLDC_Control.h"

#define ABS(x)         (x < 0) ? (-x) : x
#define PI                         (float)     3.14159265f

uint8_t Xval, Yval = 0x00;
float Buffer[3] = {0.0f}, AccBuffer2[3] = {0.0f};
uint8_t offsetA = 7;
uint8_t offsetB = 7;
uint8_t offsetC = 7;
uint8_t offsetD = 7;
uint8_t duty_cycleC;
uint8_t duty_cycleB;
uint8_t duty_cycleA;
uint8_t duty_cycleD;
float XTotal_Rotation;
float YTotal_Rotation;
float AccXangle;
float AccYangle;
float chasetheX = 0.0;//positional setpoint
float chasetheY = 0.0;//positional setpoint
float Total_XError = 0;
float Total_YError = 0;
float Last_XPOut = 0;
float Last_YPOut = 0;
int pwm_period;

//Set up the timer and schedule interruptions
void schedule_interrupts()
{

    int pwm_period = init_pwm(300);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseInitTypeDef timerInitStructure;
	timerInitStructure.TIM_Prescaler = 40000;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = 10;
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
void Set_Offset(uint8_t* value)
{
	offsetA = *value;
	offsetB = *value;
	offsetC = *value;
	offsetD = *value;
}

//void Delay(uint32_t delay) {
//    while (delay--);
//}
void update_control_signals(uint8_t width)
{
	duty_cycleC = width;
	duty_cycleB = width;
	duty_cycleA = width;
	duty_cycleD = width;
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
	if(duty_cycleC >= offsetC+3)
	{
		duty_cycleC = offsetC+3;
	}
	if(duty_cycleC <= offsetC-3)
	{
		duty_cycleC = offsetC-3;
	}
	if(duty_cycleB >= offsetB+3)
	{
		duty_cycleB = offsetB+3;
	}
	if(duty_cycleB <= offsetB-3)
	{
		duty_cycleB = offsetB-3;
	}
	if(duty_cycleA >= offsetA+3)
	{
		duty_cycleA = offsetA+3;
	}
	if(duty_cycleA <= offsetA-3)
	{
		duty_cycleA = offsetA-3;
	}
	if(duty_cycleD >= offsetD+3)
	{
		duty_cycleD = offsetD+3;
	}
	if(duty_cycleD <= offsetD-3)
	{
		duty_cycleD = offsetD-3;
	}
}
void arm_sequence()
{
	duty_cycleC = 17;
	duty_cycleB = 17;
	duty_cycleA = 17;
	duty_cycleD = 17;
	int pwm_period = init_pwm(300);
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
	while(duty_cycleC >= 4)
	{
		while(timedelay2 >= 0)
		{
			timedelay2--;
		}
		duty_cycleC--;
		duty_cycleB--;
		duty_cycleA--;
		duty_cycleD--;

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
void Display_XAxis(float value)
{
	uint8_t dig, i;
	float temp;
	char message[9];
	temp = value;
	//temp = (uint8_t)temp - temp;
	dig = temp*100000;
	dig = dig %10;
	message[0] = dig+48;
	dig = temp*10000;
	dig = dig %10;
	message[1] = dig+48;
	dig = temp*1000;
	dig = dig %10;
	message[2] = dig+48;
	dig = temp*100;
	dig = dig %10;
	message[3] = dig+48;
	dig = temp*10;
	dig = dig %10;
	message[4] = dig+48;
	message[5] = '.';
	dig = ((uint8_t)value % 10)+48;
	message[6] = dig;
	value = value/10;
	dig = ((uint8_t)value % 10)+48;
	message[7] = dig;
	value = value/10;
	dig = ((uint8_t)value % 10)+48;
	message[8] = dig;
	for(i=9; i != 0; i=i-1)
	{
		USART1_Send(message[i-1] );
	}
	//USART1_Send(0xF8);//degrees
    //USART1_Send('\n');
    //USART1_Send('\r');
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
    	//float XPOut=0;
    	float Xerror = 0; //instantaneous error
    	//float YPOut=0;
    	float Yerror = 0;
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        int pwm_period = init_pwm(300);
        Demo_GyroReadAngRate(Buffer);//read the angular rate from the gyroscope and store in Buffer[]
        //Demo_CompassReadAcc(AccBuffer2);
        //AccXangle = ((atan2f((float)AccBuffer2[1],(float)AccBuffer2[2]))*180)/PI;
        //AccYangle = ((atan2f((float)AccBuffer2[2],(float)AccBuffer2[0]))*180)/PI;
        //if(AccBuffer2[0] != 416){
        /*
         * The following lines of code create the control components. The respective axis' angular rate(Buffer[n])
         * is multiplied by the time since the containing interrupt was last called(in seconds(s)). angular rate
         * multiplied by time passed gives us displacement(in degrees). The difference between the current displacement
         * and the setpoint(chasetheN) gives us the N axis error and P component of the algorithm. A summation of the
         * P component over time is equivalent, in this context, to the integral of the error and is used as the
         * I component.
        */
        //Xerror = chasetheX - Buffer[0]*0.06;
        //Yerror = chasetheY - Buffer[1]*0.06;

        XTotal_Rotation = (XTotal_Rotation + Buffer[0]*0.06);//0.98*(XTotal_Rotation + Buffer[0]*0.06) + (0.02*AccXangle);
        YTotal_Rotation = (YTotal_Rotation + Buffer[1]*0.06);//0.98*(XTotal_Rotation + Buffer[0]*0.06) + (0.02*AccXangle);

        Xerror = chasetheX - XTotal_Rotation;
        Total_XError = Total_XError + Xerror*0.06;
        //XPOut = Total_XError;

        Yerror = chasetheY - YTotal_Rotation;
        Total_YError = Total_YError + Yerror*0.06;
        //YPOut = Total_YError;

        if(Total_XError > 1)
        {
        	duty_cycleC++;
        	duty_cycleA--;
        }
        if(Total_XError < -1)
        {
        	duty_cycleA++;
        	duty_cycleC--;
        }
        Last_XPOut = Total_XError;
        if(Total_YError > 1)
          {
          	duty_cycleD++;
          	duty_cycleB--;
          }
          if(Total_YError < -1)
          {
          	duty_cycleB++;
          	duty_cycleD--;
          }
          Last_YPOut = Total_YError;
/*
        if((XPOut < 0) && (XPOut >= -100))
        {
        	duty_cycleA++;
        	duty_cycleC--;
        }
        else if((XPOut < -100 ) && (XPOut >= -250))
        {
        	duty_cycleA = duty_cycleA+2;
        	duty_cycleC = duty_cycleC-2;
        }
        else if(XPOut < -250 )
        {
        	duty_cycleA = duty_cycleA+3;
        	duty_cycleC = duty_cycleC-3;
        }
        if((XPOut > 0) && (XPOut <= 100))
        {
        	duty_cycleA--;
        	duty_cycleC++;
        }
        else if((XPOut > 100 ) && (XPOut <= 250))
        {
        	duty_cycleA = duty_cycleA-2;
        	duty_cycleC = duty_cycleC+2;
        }
        else if(XPOut > 250 )
        {
        	duty_cycleA = duty_cycleA-3;
        	duty_cycleC = duty_cycleC+3;
        }
        if((YPOut < 0) && (YPOut >= -100))
        {
        	duty_cycleB++;
        	duty_cycleD--;
        }
        else if((YPOut < -100 ) && (YPOut >= -250))
        {
        	duty_cycleB = duty_cycleB+2;
        	duty_cycleD = duty_cycleD-2;
        }
        else if(YPOut < -250 )
        {
        	duty_cycleB = duty_cycleB+3;
        	duty_cycleD = duty_cycleD-3;
        }
        if((YPOut > 0) && (YPOut <= 100))
        {
        	duty_cycleB--;
        	duty_cycleD++;
        }
        else if((YPOut > 100 ) && (YPOut <= 250))
        {
        	duty_cycleB = duty_cycleB-2;
        	duty_cycleD = duty_cycleD+2;
        }
        else if(YPOut > 250 )
        {
        	duty_cycleB = duty_cycleB-3;
        	duty_cycleD = duty_cycleD+3;
        }
*/
        bounds_check();
        Display_DC(duty_cycleD);
        USART1_Send(' ');
        set_pwm_width(4, pwm_period, duty_cycleD);
        Display_DC(duty_cycleC);
        USART1_Send(' ');
        set_pwm_width(1, pwm_period, duty_cycleC);
        Display_DC(duty_cycleB);
        USART1_Send(' ');
        set_pwm_width(2, pwm_period, duty_cycleB);
        Display_DC(duty_cycleA);
        USART1_Send(' ');
        set_pwm_width(3, pwm_period, duty_cycleA);

        USART1_Send('\n');
        USART1_Send('\r');
        //Display_XAxis(YPOut);
        //Display_XAxis(Total_XError);
        //USART1_Send('\n');
        //USART1_Send('\r');
        //Display_XAxis(Total_YError);
        //USART1_Send('\n');
        //USART1_Send('\r');
    }
}
