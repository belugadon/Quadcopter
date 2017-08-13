#include "BLDC_Control.h"
#include "math.h"
#include "KalmanFilter.h"


#define ABS(x)         (x < 0) ? (-x) : x
#define RadToDeg                   (int)  57295

float Buffer[3] = {0.0f}, AccBuffer2[3] = {0.0f};
int offsetA = 7000;
int offsetB = 7000;
int offsetC = 7000;
int offsetD = 7000;
int offsetA_High, offsetB_High, offsetC_High, offsetD_High;
int offsetA_Low, offsetB_Low, offsetC_Low, offsetD_Low;
int duty_cycleC, duty_cycleB, duty_cycleA, duty_cycleD;
int XSum_Of_Gyro, YSum_Of_Gyro;
float XTotal_Rotation, YTotal_Rotation;
float AccXangle, AccYangle;
float chasetheX = 0.0;//positional setpoint
float chasetheY = 0.0;//positional setpoint
float SUMof_XError = 0;
float SUMof_YError = 0;
float XLastError = 0;
float YLastError = 0;
float ControlX_Out = 0;
float ControlY_Out = 0;
int pwm_period;
int ms_pulses2;
int prescaler2;
int interrupt_frequency = 50;
int interrupt_period_int;


//Set up the timer and schedule interruptions
void schedule_PI_interrupts()
{
	//cortexm4f_enable_fpu();
	interrupt_period_int = (1/(float)interrupt_frequency)*1000;
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
void Display_Axis(int value)
{
	int dig, i;
	int temp;
	char message[8];
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
}
void cortexm4f_enable_fpu() {
    /* set CP10 and CP11 Full Access */
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));
}

void Set_Offset(int* value, float* roll, float* pitch, int* yaw)
{
	chasetheY = (*roll + *pitch)*9;
	chasetheX = (*roll + (0 - *pitch))*9;
	offsetA = 6900 + *value;
	offsetB = 6900 + *value;
	offsetC = 6900 + *value;
	offsetD = 6900 + *value;
	offsetA = offsetA + *yaw/2;
	offsetB = offsetB - *yaw/2;
	offsetC = offsetC + *yaw/2;
	offsetD = offsetD - *yaw/2;
	offsetA_High = offsetA + 2500;
	offsetB_High = offsetB + 2500;
	offsetC_High = offsetC + 2500;
	offsetD_High = offsetD + 2500;
	offsetA_Low = offsetA - 2500;
	offsetB_Low = offsetB - 2500;
	offsetC_Low = offsetC - 2500;
	offsetD_Low = offsetD - 2500;
}
void Calculate_Position()
{
    GyroReadAngRate(Buffer);//read the angular rate from the gyroscope and store in Buffer[]

    CompassReadAcc(AccBuffer2);

    AccYangle = ((atan2f((float)AccBuffer2[1],(float)AccBuffer2[2]))*RadToDeg);//*180)/PI;
    AccXangle = ((atan2f((float)AccBuffer2[0],(float)AccBuffer2[2]))*RadToDeg);//*180)/PI;

    XTotal_Rotation = kalmanFilterX(AccXangle, Buffer[0], 50)/10;
    YTotal_Rotation = kalmanFilterY(AccYangle, Buffer[1], 59)/10;
/*    USART1_Send('X');
    USART1_Send(':');
    //USART1_Send(',');
    Display_Axis((XTotal_Rotation*10));
    //Display_Axis(Buffer[0]*1000);
    USART1_Send(',');
    USART1_Send(' ');
    //USART1_Send('\n');
    //USART1_Send('\r');
    USART1_Send('Y');
    USART1_Send(':');
    //USART1_Send(' ');
    Display_Axis((YTotal_Rotation*10));
    //Display_Axis(Buffer[1]*1000);
    //USART1_Send(',');
    USART1_Send('\n');
    USART1_Send('\r');*/
}


void Adjust_Yaw(int* value)
{
	duty_cycleC = duty_cycleC + *value;
	duty_cycleB = duty_cycleB - *value;
	duty_cycleA = duty_cycleA + *value;
	duty_cycleD = duty_cycleD - *value;
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
	TIM_TimeBaseStructure.TIM_Period = 2860;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up ;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);


// Initialise the timer channels
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

	TIM_OCInitStructure.TIM_Pulse = ms_pulses2*2;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
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
	int pwm_period = tim_freq/pwm_freq;

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
	TIM_TimeBaseStructure.TIM_Period = 2857;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up ;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);


// Initialise the timer channels
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

	TIM_OCInitStructure.TIM_Pulse = ms_pulses*2;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
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


void TIM2_IRQHandler()
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
    	float Xerror = 0;
    	float Yerror = 0;
    	float SlopeofXError = 0.0;
    	float SlopeofYError = 0;
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        init_pwm();

        if (offsetA >= 8000){
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
        SUMof_XError = SUMof_XError + Xerror;
        }
        if (((duty_cycleB >= offsetB_High) || (duty_cycleD >= offsetD_High))|| ((duty_cycleB <= offsetB_Low) || (duty_cycleD <= offsetD_Low)))
        {
        	SUMof_YError = SUMof_YError;
        }
        else {
        	SUMof_YError = SUMof_YError + Yerror;
        }

        //Derivative(D) Component
        SlopeofXError = (Xerror - XLastError);
        XLastError = Xerror;
        SlopeofYError = (Yerror - YLastError);
        YLastError = Yerror;


        //We can now assemble the control output by multiplying each control component by it's associated
        //gain coefficient and summing the results
        ControlX_Out = (0.035 * Xerror) + (0.025 * SUMof_XError) + (0.04 * SlopeofYError);
        ControlY_Out = (0.035 * Yerror) + (0.025 * SUMof_YError) + (0.04 * SlopeofYError);
        }
        else{
        ControlX_Out = 0;
        ControlY_Out = 0;
        }
        duty_cycleC = 0 - (ControlX_Out) + offsetC;
        duty_cycleA = ControlX_Out + offsetA;
        duty_cycleD = 0 - (ControlY_Out) + offsetD;
        duty_cycleB = ControlY_Out + offsetB;


        	bounds_check();
        	set_pwm_width(2, pwm_period, duty_cycleD);
        	set_pwm_width(1, pwm_period, duty_cycleC);
        	set_pwm_width(4, pwm_period, duty_cycleB);
        	set_pwm_width(3, pwm_period, duty_cycleA);

    }
}
