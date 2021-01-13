#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "misc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_usart.h"
#include "math.h"

GPIO_InitTypeDef gpio_init;
TIM_ICInitTypeDef TIM_ICInitStructure;
TIM_TimeBaseInitTypeDef time_init;
TIM_OCInitTypeDef oc_init;

int sign(int a);
int constr(int a);
int deg_to_pwm(int a);
void INTTIM_Config(void);
void USART_test(void);
void USART_eho(void);
void pwm_init1(void);
void pwm_init2(void);
void pwm_init3(void);

void cfg_pack1();
void init_pack1(void);
int ch_pack=1;

void cfg_pack2();
void init_pack2(void);

void cfg_pack3();
void init_pack3(void);
int gam=0;
uint16_t eho_stat[12]={0};
uint16_t eho_angle[12]={0};
uint16_t eho_mes[12]={0};


void send_to_uart(uint8_t data)
{
	while(!(USART2->SR & USART_SR_TC));
	USART2->DR=data;
}

//Функция отправляет строку в UART, по сути пересылая по байту в send_to_uart
void send_str(char * string)
{
	int i=0;
	while(string[i])
	{
		send_to_uart(string[i]);
		i++;
	}
}

void INTTIM_Config(void)
{
	NVIC_InitTypeDef nvic_struct;
	nvic_struct.NVIC_IRQChannel = TIM7_IRQn;
	nvic_struct.NVIC_IRQChannelPreemptionPriority = 5;
	nvic_struct.NVIC_IRQChannelSubPriority = 5;
	nvic_struct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_struct);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
	TIM_TimeBaseStructure.TIM_Period = 10000 - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 168 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM7, ENABLE);
}

void USART_test(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2); //PD5 to TX USART2 на плате наоборот
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2); //PD6 to RX USART2
	// PD5 -> TX UART.
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	//PD6  -> RX UART.
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	//USART
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);
	/* NVIC configuration */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	/* Enable USART */
	USART_Cmd(USART2, ENABLE);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}

void USART_eho(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3); //PD8 to TX USART2 на плате наоборот
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3); //PD9 to RX USART2
	// PD5 -> TX UART.
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	//PD6  -> RX UART.
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	//USART
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	/* Enable USART */
	USART_Cmd(USART3, ENABLE);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
}

void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
	{
		while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
		USART_SendData(USART2, USART_ReceiveData(USART2));
	}
}
void USART3_IRQHandler(void)
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
	{
		gam = USART_ReceiveData(USART3)*10;
	}
}

int main(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	gpio_init.GPIO_Speed = GPIO_Speed_100MHz;
	gpio_init.GPIO_OType = GPIO_OType_PP;
	gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;

	time_init.TIM_Period = 5000;
	time_init.TIM_Prescaler = 168 - 1;
	time_init.TIM_ClockDivision = 0;
	time_init.TIM_CounterMode = TIM_CounterMode_Up;

	oc_init.TIM_OCMode = TIM_OCMode_PWM1;
	oc_init.TIM_OutputState = TIM_OutputState_Enable;
	oc_init.TIM_Pulse = 30;
	oc_init.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;

	USART_test();
	USART_eho();
	init_pack1();
	init_pack2();
	init_pack3();
	pwm_init1();
	pwm_init2();
	pwm_init3();
	INTTIM_Config();
	while(1)
	{
	}
}

void pwm_init1(void)
{
	GPIO_InitTypeDef gpio_init1;
	gpio_init1.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_14;
	gpio_init1.GPIO_Mode = GPIO_Mode_AF;
	gpio_init1.GPIO_Speed = GPIO_Speed_100MHz;
	gpio_init1.GPIO_OType = GPIO_OType_PP;
	gpio_init1.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &gpio_init1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	TIM1->CCER = 0;
	TIM1->ARR = 3000;
	TIM1->PSC = 168-1;
	TIM1->BDTR |= TIM_BDTR_MOE;
	TIM1->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	TIM1->CCER |= TIM_CCER_CC1E;
	TIM1->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
	TIM1->CCER |= TIM_CCER_CC2E;
	TIM1->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
	TIM1->CCER |= TIM_CCER_CC3E;
	TIM1->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;
	TIM1->CCER |= TIM_CCER_CC4E;
	TIM1->CR1 |= TIM_CR1_CEN;
	TIM1->CCR1 = 50;
	TIM1->CCR2 = 133;
	TIM1->CCR3 = 217;
	TIM1->CCR4 = 600;
}

void pwm_init2(void){

	GPIO_InitTypeDef gpio_init1;
	gpio_init1.GPIO_Pin = GPIO_Pin_15;
	gpio_init1.GPIO_Mode = GPIO_Mode_AF;
	gpio_init1.GPIO_Speed = GPIO_Speed_100MHz;
	gpio_init1.GPIO_OType = GPIO_OType_PP;
	gpio_init1.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &gpio_init1);
	gpio_init1.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_10|GPIO_Pin_11;
	GPIO_Init(GPIOB, &gpio_init1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_TIM2);
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	TIM2->CCER = 0;
	TIM2->ARR = 3000;
	TIM2->PSC = 168-1;
	TIM2->BDTR |= TIM_BDTR_MOE;
	TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	TIM2->CCER |= TIM_CCER_CC1E;
	TIM2->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
	TIM2->CCER |= TIM_CCER_CC2E;
	TIM2->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
	TIM2->CCER |= TIM_CCER_CC3E;
	TIM2->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;
	TIM2->CCER |= TIM_CCER_CC4E;
	TIM2->CR1 |= TIM_CR1_CEN;
	TIM2->CCR1 = 50;
	TIM2->CCR2 = 133;
	TIM2->CCR3 = 217;
	TIM2->CCR4 = 300;
}

void pwm_init3(void)
{
	GPIO_InitTypeDef gpio_init1;
	gpio_init1.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	gpio_init1.GPIO_Mode = GPIO_Mode_AF;
	gpio_init1.GPIO_Speed = GPIO_Speed_100MHz;
	gpio_init1.GPIO_OType = GPIO_OType_PP;
	gpio_init1.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &gpio_init1);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM8);
	RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
	TIM8->CCER = 0;
	TIM8->ARR = 3000;
	TIM8->PSC = 168-1;
	TIM8->BDTR |= TIM_BDTR_MOE;
	TIM8->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	TIM8->CCER |= TIM_CCER_CC1E;
	TIM8->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
	TIM8->CCER |= TIM_CCER_CC2E;
	TIM8->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
	TIM8->CCER |= TIM_CCER_CC3E;
	TIM8->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;
	TIM8->CCER |= TIM_CCER_CC4E;
	TIM8->CR1 |= TIM_CR1_CEN;
	TIM8->CCR1 = 50;
	TIM8->CCR2 = 133;
	TIM8->CCR3 = 217;
	TIM8->CCR4 = 300;
}

void init_pack1(void){
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIM_TimeBaseInit(TIM3, &time_init);
	TIM_OC3Init(TIM3, &oc_init);
	TIM_OC4Init(TIM3, &oc_init);
	TIM_UpdateRequestConfig(TIM3, TIM_UpdateSource_Global);
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
	TIM_SelectMasterSlaveMode(TIM3,TIM_MasterSlaveMode_Enable);
	TIM3->CR1 |= TIM_CR1_CEN;
	TIM3->DIER |= TIM_IT_CC1;
	TIM3->DIER |= TIM_IT_CC2;
	TIM_PWMIConfig(TIM3, &TIM_ICInitStructure);
	TIM_SelectInputTrigger(TIM3, TIM_TS_TI1FP1);
	cfg_pack1();
}

void cfg_pack1(){
	switch (ch_pack){
	case 1:
		gpio_init.GPIO_Pin = GPIO_Pin_4;
		gpio_init.GPIO_Mode = GPIO_Mode_AF;
		GPIO_Init(GPIOB, &gpio_init);
		gpio_init.GPIO_Pin = GPIO_Pin_5;
		gpio_init.GPIO_Mode = GPIO_Mode_IN;
		GPIO_Init(GPIOB, &gpio_init);
		gpio_init.GPIO_Pin = GPIO_Pin_0;
		gpio_init.GPIO_Mode = GPIO_Mode_AF;
		GPIO_Init(GPIOB, &gpio_init);
		gpio_init.GPIO_Pin = GPIO_Pin_1;
		gpio_init.GPIO_Mode = GPIO_Mode_IN;
		GPIO_Init(GPIOB, &gpio_init);

		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
		TIM_PWMIConfig(TIM3, &TIM_ICInitStructure);
		break;
	case 2:
		gpio_init.GPIO_Pin = GPIO_Pin_4;
		gpio_init.GPIO_Mode = GPIO_Mode_AF;
		GPIO_Init(GPIOB, &gpio_init);
		gpio_init.GPIO_Pin = GPIO_Pin_5;
		gpio_init.GPIO_Mode = GPIO_Mode_IN;
		GPIO_Init(GPIOB, &gpio_init);
		gpio_init.GPIO_Pin = GPIO_Pin_0;
		gpio_init.GPIO_Mode = GPIO_Mode_IN;
		GPIO_Init(GPIOB, &gpio_init);
		gpio_init.GPIO_Pin = GPIO_Pin_1;
		gpio_init.GPIO_Mode = GPIO_Mode_AF;
		GPIO_Init(GPIOB, &gpio_init);

		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
		TIM_PWMIConfig(TIM3, &TIM_ICInitStructure);
		break;
	case 3:
		gpio_init.GPIO_Pin = GPIO_Pin_4;
		gpio_init.GPIO_Mode = GPIO_Mode_IN;
		GPIO_Init(GPIOB, &gpio_init);
		gpio_init.GPIO_Pin = GPIO_Pin_5;
		gpio_init.GPIO_Mode = GPIO_Mode_AF;
		GPIO_Init(GPIOB, &gpio_init);
		gpio_init.GPIO_Pin = GPIO_Pin_0;
		gpio_init.GPIO_Mode = GPIO_Mode_AF;
		GPIO_Init(GPIOB, &gpio_init);
		gpio_init.GPIO_Pin = GPIO_Pin_1;
		gpio_init.GPIO_Mode = GPIO_Mode_IN;
		GPIO_Init(GPIOB, &gpio_init);

		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
		TIM_PWMIConfig(TIM3, &TIM_ICInitStructure);
		break;
	case 4:
		gpio_init.GPIO_Pin = GPIO_Pin_4;
		gpio_init.GPIO_Mode = GPIO_Mode_IN;
		GPIO_Init(GPIOB, &gpio_init);
		gpio_init.GPIO_Pin = GPIO_Pin_5;
		gpio_init.GPIO_Mode = GPIO_Mode_AF;
		GPIO_Init(GPIOB, &gpio_init);
		gpio_init.GPIO_Pin = GPIO_Pin_0;
		gpio_init.GPIO_Mode = GPIO_Mode_IN;
		GPIO_Init(GPIOB, &gpio_init);
		gpio_init.GPIO_Pin = GPIO_Pin_1;
		gpio_init.GPIO_Mode = GPIO_Mode_AF;
		GPIO_Init(GPIOB, &gpio_init);

		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
		TIM_PWMIConfig(TIM3, &TIM_ICInitStructure);
		break;
	}
}

void init_pack2(void){
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM4);

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
		TIM_TimeBaseInit(TIM4, &time_init);
		TIM_OC3Init(TIM4, &oc_init);
		TIM_OC4Init(TIM4, &oc_init);
		TIM_UpdateRequestConfig(TIM4, TIM_UpdateSource_Global);
		TIM_ARRPreloadConfig(TIM4, ENABLE);
		TIM_Cmd(TIM4, ENABLE);
		TIM_SelectMasterSlaveMode(TIM4,TIM_MasterSlaveMode_Enable);
		TIM4->CR1 |= TIM_CR1_CEN;
		TIM4->DIER |= TIM_IT_CC1;
		TIM4->DIER |= TIM_IT_CC2;

		TIM_PWMIConfig(TIM4, &TIM_ICInitStructure);
		TIM_SelectInputTrigger(TIM4, TIM_TS_TI1FP1);
		cfg_pack2();
}

void cfg_pack2(){
	switch (ch_pack){
	case 1:
		gpio_init.GPIO_Pin = GPIO_Pin_6;
		gpio_init.GPIO_Mode = GPIO_Mode_AF;
		GPIO_Init(GPIOB, &gpio_init);
		gpio_init.GPIO_Pin = GPIO_Pin_7;
		gpio_init.GPIO_Mode = GPIO_Mode_IN;
		GPIO_Init(GPIOB, &gpio_init);
		gpio_init.GPIO_Pin = GPIO_Pin_8;
		gpio_init.GPIO_Mode = GPIO_Mode_AF;
		GPIO_Init(GPIOB, &gpio_init);
		gpio_init.GPIO_Pin = GPIO_Pin_9;
		gpio_init.GPIO_Mode = GPIO_Mode_IN;
		GPIO_Init(GPIOB, &gpio_init);

		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
		TIM_PWMIConfig(TIM4, &TIM_ICInitStructure);
		break;
	case 2:
		gpio_init.GPIO_Pin = GPIO_Pin_6;
		gpio_init.GPIO_Mode = GPIO_Mode_AF;
		GPIO_Init(GPIOB, &gpio_init);
		gpio_init.GPIO_Pin = GPIO_Pin_7;
		gpio_init.GPIO_Mode = GPIO_Mode_IN;
		GPIO_Init(GPIOB, &gpio_init);
		gpio_init.GPIO_Pin = GPIO_Pin_8;
		gpio_init.GPIO_Mode = GPIO_Mode_IN;
		GPIO_Init(GPIOB, &gpio_init);
		gpio_init.GPIO_Pin = GPIO_Pin_9;
		gpio_init.GPIO_Mode = GPIO_Mode_AF;
		GPIO_Init(GPIOB, &gpio_init);

		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
		TIM_PWMIConfig(TIM4, &TIM_ICInitStructure);
		break;
	case 3:
		gpio_init.GPIO_Pin = GPIO_Pin_6;
		gpio_init.GPIO_Mode = GPIO_Mode_IN;
		GPIO_Init(GPIOB, &gpio_init);
		gpio_init.GPIO_Pin = GPIO_Pin_7;
		gpio_init.GPIO_Mode = GPIO_Mode_AF;
		GPIO_Init(GPIOB, &gpio_init);
		gpio_init.GPIO_Pin = GPIO_Pin_8;
		gpio_init.GPIO_Mode = GPIO_Mode_AF;
		GPIO_Init(GPIOB, &gpio_init);
		gpio_init.GPIO_Pin = GPIO_Pin_9;
		gpio_init.GPIO_Mode = GPIO_Mode_IN;
		GPIO_Init(GPIOB, &gpio_init);

		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
		TIM_PWMIConfig(TIM4, &TIM_ICInitStructure);
		break;
	case 4:
		gpio_init.GPIO_Pin = GPIO_Pin_6;
		gpio_init.GPIO_Mode = GPIO_Mode_IN;
		GPIO_Init(GPIOB, &gpio_init);
		gpio_init.GPIO_Pin = GPIO_Pin_7;
		gpio_init.GPIO_Mode = GPIO_Mode_AF;
		GPIO_Init(GPIOB, &gpio_init);
		gpio_init.GPIO_Pin = GPIO_Pin_8;
		gpio_init.GPIO_Mode = GPIO_Mode_IN;
		GPIO_Init(GPIOB, &gpio_init);
		gpio_init.GPIO_Pin = GPIO_Pin_9;
		gpio_init.GPIO_Mode = GPIO_Mode_AF;
		GPIO_Init(GPIOB, &gpio_init);

		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
		TIM_PWMIConfig(TIM4, &TIM_ICInitStructure);
		break;
	}
}

void init_pack3(void){
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM5);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	TIM_TimeBaseInit(TIM5, &time_init);
	TIM_OC3Init(TIM5, &oc_init);
	TIM_OC4Init(TIM5, &oc_init);
	TIM_UpdateRequestConfig(TIM5, TIM_UpdateSource_Global);
	TIM_ARRPreloadConfig(TIM5, ENABLE);
	TIM_Cmd(TIM5, ENABLE);
	TIM_SelectMasterSlaveMode(TIM5,TIM_MasterSlaveMode_Enable);
	TIM5->CR1 |= TIM_CR1_CEN;
	TIM5->DIER |= TIM_IT_CC1;
	TIM5->DIER |= TIM_IT_CC2;
	TIM_PWMIConfig(TIM5, &TIM_ICInitStructure);
	TIM_SelectInputTrigger(TIM5, TIM_TS_TI1FP1);
	cfg_pack3();
}

void cfg_pack3(){
	switch (ch_pack){
	case 1:
		gpio_init.GPIO_Pin = GPIO_Pin_0;
		gpio_init.GPIO_Mode = GPIO_Mode_AF;
		GPIO_Init(GPIOA, &gpio_init);
		gpio_init.GPIO_Pin = GPIO_Pin_1;
		gpio_init.GPIO_Mode = GPIO_Mode_IN;
		GPIO_Init(GPIOA, &gpio_init);
		gpio_init.GPIO_Pin = GPIO_Pin_2;
		gpio_init.GPIO_Mode = GPIO_Mode_AF;
		GPIO_Init(GPIOA, &gpio_init);
		gpio_init.GPIO_Pin = GPIO_Pin_3;
		gpio_init.GPIO_Mode = GPIO_Mode_IN;
		GPIO_Init(GPIOA, &gpio_init);

		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
		TIM_PWMIConfig(TIM5, &TIM_ICInitStructure);
		break;
	case 2:
		gpio_init.GPIO_Pin = GPIO_Pin_0;
		gpio_init.GPIO_Mode = GPIO_Mode_AF;
		GPIO_Init(GPIOA, &gpio_init);
		gpio_init.GPIO_Pin = GPIO_Pin_1;
		gpio_init.GPIO_Mode = GPIO_Mode_IN;
		GPIO_Init(GPIOA, &gpio_init);
		gpio_init.GPIO_Pin = GPIO_Pin_2;
		gpio_init.GPIO_Mode = GPIO_Mode_IN;
		GPIO_Init(GPIOA, &gpio_init);
		gpio_init.GPIO_Pin = GPIO_Pin_3;
		gpio_init.GPIO_Mode = GPIO_Mode_AF;
		GPIO_Init(GPIOA, &gpio_init);

		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
		TIM_PWMIConfig(TIM5, &TIM_ICInitStructure);
		break;
	case 3:
		gpio_init.GPIO_Pin = GPIO_Pin_0;
		gpio_init.GPIO_Mode = GPIO_Mode_IN;
		GPIO_Init(GPIOA, &gpio_init);
		gpio_init.GPIO_Pin = GPIO_Pin_1;
		gpio_init.GPIO_Mode = GPIO_Mode_AF;
		GPIO_Init(GPIOA, &gpio_init);
		gpio_init.GPIO_Pin = GPIO_Pin_2;
		gpio_init.GPIO_Mode = GPIO_Mode_AF;
		GPIO_Init(GPIOA, &gpio_init);
		gpio_init.GPIO_Pin = GPIO_Pin_3;
		gpio_init.GPIO_Mode = GPIO_Mode_IN;
		GPIO_Init(GPIOA, &gpio_init);

		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
		TIM_PWMIConfig(TIM5, &TIM_ICInitStructure);
		break;
	case 4:
		gpio_init.GPIO_Pin = GPIO_Pin_0;
		gpio_init.GPIO_Mode = GPIO_Mode_IN;
		GPIO_Init(GPIOA, &gpio_init);
		gpio_init.GPIO_Pin = GPIO_Pin_1;
		gpio_init.GPIO_Mode = GPIO_Mode_AF;
		GPIO_Init(GPIOA, &gpio_init);
		gpio_init.GPIO_Pin = GPIO_Pin_2;
		gpio_init.GPIO_Mode = GPIO_Mode_IN;
		GPIO_Init(GPIOA, &gpio_init);
		gpio_init.GPIO_Pin = GPIO_Pin_3;
		gpio_init.GPIO_Mode = GPIO_Mode_AF;
		GPIO_Init(GPIOA, &gpio_init);

		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
		TIM_PWMIConfig(TIM5, &TIM_ICInitStructure);
		break;
	}
}
int sign(int a){
	if (a>0) return 1;
	else if(a<0) return -1;
	else return 0;

}
int constr(int a){
	if (a<0) return 0;
	else if (a>180) return 180;
	else return a;
}
int deg_to_pwm(int a){
	return a*250/180+50;
}
void TIM7_IRQHandler(void) {
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET) {
		TIM_ClearFlag(TIM3, TIM_FLAG_Update);
		TIM_ClearFlag(TIM4, TIM_FLAG_Update);
		TIM_ClearFlag(TIM5, TIM_FLAG_Update);
		while(!TIM_GetFlagStatus(TIM3, TIM_FLAG_Update)||!TIM_GetFlagStatus(TIM4, TIM_FLAG_Update)||!TIM_GetFlagStatus(TIM5, TIM_FLAG_Update));
		eho_mes[ch_pack-1] = (TIM_GetCapture2(TIM3)-TIM_GetCapture1(TIM3))*165/1000;
		eho_mes[3+ch_pack] = (TIM_GetCapture2(TIM4)-TIM_GetCapture1(TIM4))*165/1000;
		eho_mes[7+ch_pack] = (TIM_GetCapture2(TIM5)-TIM_GetCapture1(TIM5))*165/1000;
		if (ch_pack==4) {
			ch_pack=1;

			char str4[200]={0};

			sprintf(str4,"1_1: %d; 1_2: %d; 1_3: %d; 1_4: %d; 2_1: %d; 2_2: %d; 2_3: %d; 2_4: %d; 3_1: %d; 3_2: %d; 3_3: %d; 3_4: %d;\r\n",
					eho_mes[0],eho_mes[1],eho_mes[2],eho_mes[3],eho_mes[4],eho_mes[5],eho_mes[6],eho_mes[7],eho_mes[8],eho_mes[9],eho_mes[10],eho_mes[11]);
			send_str(str4);
			for(int war=100;war<1000;war+=100){
				if (eho_mes[0]<war||eho_mes[1]<war||eho_mes[2]<war||eho_mes[3]<war||eho_mes[4]<war||eho_mes[5]<war||
						eho_mes[6]<war||eho_mes[7]<war||eho_mes[8]<war||eho_mes[9]<war||eho_mes[10]<war||eho_mes[11]<war){
					USART_SendData(USART2,war/100);
					break;
				}
			}

		}
		else ch_pack++;

		eho_angle[0]=gam+45;
		eho_angle[1]=gam+30;
		eho_angle[2]=gam+30+90;
		eho_angle[3]=gam+45+90;

		eho_angle[4]=gam+30+90;
		eho_angle[5]=gam+30+90+90;

		eho_angle[6]=180-sign(fabs(gam))*(180-fabs(gam))-45-90;
		eho_angle[7]=180-sign(fabs(gam))*(180-fabs(gam))-30;
		eho_angle[8]=180-sign(fabs(gam))*(180-fabs(gam))-60-90;
		eho_angle[9]=180-sign(fabs(gam))*(180-fabs(gam))-45;

		eho_angle[10]=gam+30;
		eho_angle[11]=gam+30+90;

		TIM1->CCR1=deg_to_pwm(eho_angle[0]);
		TIM1->CCR2=deg_to_pwm(eho_angle[1]);
		TIM1->CCR3=deg_to_pwm(eho_angle[2]);
		TIM1->CCR4=deg_to_pwm(eho_angle[3]);

		TIM2->CCR1=deg_to_pwm(eho_angle[4]);
		TIM2->CCR2=deg_to_pwm(eho_angle[5]);
		TIM2->CCR3=deg_to_pwm(eho_angle[6]);
		TIM2->CCR4=deg_to_pwm(eho_angle[7]);

		TIM8->CCR1=deg_to_pwm(eho_angle[8]);
		TIM8->CCR2=deg_to_pwm(eho_angle[9]);
		TIM8->CCR3=deg_to_pwm(eho_angle[10]);
		TIM8->CCR4=deg_to_pwm(eho_angle[11]);


		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	}
}
