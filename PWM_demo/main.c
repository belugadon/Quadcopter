#include "main.h"
#include "stm32f3_discovery.h"


/**
  * @brief  Initializes the putput pins for the Timer one channels. Currently
  * 		hard coded to drive the LED3 (channel 1), LED7 (channel 2), and
  * 		LED10 (channel 3) on the STM32F3 Discovery board.
  * @param  pwm_freq: Frequency of the PWM in Hz
  * @retval Number of the timer pulses per one PWM period
  */
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
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource12, GPIO_AF_12); // TIM1_CH1 -> LED3
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource10, GPIO_AF_12); // TIM1_CH2 -> LED7
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_12); // TIM1_CH2 -> LED8
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_12); // TIM1_CH3 -> LED10
}

/**
  * @brief  Initializes PWM
  * @param  pwm_freq: Frequency of the PWM in Hz
  * @retval Number of the timer pulses per one PWM period
  */
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

/**
  * @brief  Sets the PWM duty cycle per channel
  * @param  channel:  PWM channel index [1..4]
  * @param  duty_cycle:  PWM duty cycle in percents (integer) [0..100]
  * @retval None
  */
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

/**
  * @brief  Sets the PWM duty cycle per channel
  * @param  channel:  PWM channel index [1..4]
  * @param  duty_cycle:  PWM duty cycle (float) [0..1]
  * @retval None
  */
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

int main(void)
{
	// Check this interesting doc
	//http://www.compel.ru/wordpress/wp-content/uploads/2013/02/STM32F3-trening.pdf

	init_pwm_gpio();
	// Config the PWM freq to 200Hz
	int pwm_period = init_pwm(200);

	// Change the brightness of the LED3,7 and 10
	int brightness = 0;
	int up = 1;
	while (1) {

		up ? brightness++ : brightness--;
		if (brightness >= pwm_period) up = 0;
		if (brightness <= 0) up = 1;

		// The gammaCorrect() just compensates for the non linear brightness perception of human eye.
		// I's just an eye candy :-)
		float level = gammaCorrect(brightness, pwm_period);
		set_pwm_width_norm(1, pwm_period, level);
		set_pwm_width_norm(3, pwm_period, level);

		// Make the LED7 to fade inversely
		level = gammaCorrect(pwm_period - brightness, pwm_period);
		set_pwm_width_norm(2, pwm_period, level);

		// Delay
		int i;
		for (i = 0; i < 300; i++);
	}
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
