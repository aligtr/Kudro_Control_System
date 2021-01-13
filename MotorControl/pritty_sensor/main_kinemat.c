#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "misc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_usart.h"
#include "math.h"
#include "MCP.h"
#include "MotorControl.h"
#include "bsp.h"
int ost(double a);
double sign(double a);
void normaliz();
void speedControl(motorType motor, double Vel);
void motorStatus(motorType motor);
void kinematica(double gam, double R, double V, double L, double C);
motorType motorfl;
motorType motorfr;
motorType motorrl;
motorType motorrr;
double_t gam=0;
double_t R=9999;
double_t V=1;
double_t L=1.5;
double_t C=0.75;
int vel_period_max=70*2;
int vel_period_mean=3910*2;
int dir_period_max=70*2;
int dir_period_mean=3910*2;
int rx_period_mean=3910*2;
int ry_period_mean=3910*2;
double k_vel=5;
double k_dir=M_PI_2;

double gfl, gfr, grl, grr;
double Vfl, Vfr, Vrl, Vrr;

void INTTIM_Config(void);
void ic_init_ch1_2(void);
void ic_init_ch3_4(void);
void oc_init_ch1_4(void);
void send_to_uart(uint8_t data);
void send_str(char * string);

void tim1_ch1(void);
void tim1_ch2(void);
void tim4_ch1(void);
void tim4_ch2(void);

int d_pwm_ch1=0;
int d_pwm_ch2=0;
int d_pwm_ch3=0;
int d_pwm_ch4=0;

char t1_ch=1;
char t4_ch=1;
int vel_count=0;
int vel_mean=0;
int vel_weidth=0;
int vel_period=0;

int rx_count=0;
int rx_mean=0;
int rx_weidth=0;
int rx_period=0;

int dir_count=0;
int dir_mean=0;
int dir_weidth=0;
int dir_period=0;
int ry_count=0;
int ry_mean=0;
int ry_weidth=0;
int ry_period=0;
int pwm_count_ch1=0;
int pwm_count_ch2=0;
int pwm_count_ch3=0;
int pwm_count_ch4=0;
int pwm_aim_ch1=0;
int pwm_aim_ch2=0;
int pwm_aim_ch3=0;
int pwm_aim_ch4=0;

int main(void)
{
	constract=0x39;
	uart1Init(9600);//PA9-TX
	//PA10-RX

	uart2Init(9600);//PD5-TX
	//PD6-RX
	//Tim9Init();
	/*uart3Init(9600);//PD8-TX
	//PD9-RX

	uart4Init(9600);//PC10-TX
	//PC11-RX

	uart5Init(9600);//PC12-TX
	//PD2-RX

	uart6Init(9600);//PC6-TX
	//PC7-RX


	motorfl.channel=USART1;
	motorfr.channel=USART2;
	motorrl.channel=USART3;
	motorrr.channel=UART4;
	motorType motor;
	USART_TypeDef * MOTOR = USART2;
			motor.refSpeed=5000;
	/*oc_init_ch1_4(); //PA0-3
	ic_init_ch1_2(); //PE9,11
	ic_init_ch3_4(); //PD12,13
	INTTIM_Config();*/
			//startMotor(USART1);
	while(1)
	{

		/*if(motor.lastSpeed!=motor.refSpeed)
		{


			if(setSpeed(motor.refSpeed,100,MOTOR)==ERROR_NONE)
			{
				motor.lastSpeed=motor.refSpeed;
			}
		}*/
		USART_SendData(USART1,116);
		for(int i=100000; i>0;i++);
	}
}

void oc_init_ch1_4(void)
{
	GPIO_InitTypeDef gpio_init1;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	gpio_init1.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	gpio_init1.GPIO_Mode = GPIO_Mode_OUT;
	gpio_init1.GPIO_Speed = GPIO_Speed_100MHz;
	gpio_init1.GPIO_OType = GPIO_OType_OD;
	gpio_init1.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &gpio_init1);

	GPIO_InitTypeDef gpio_init;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	gpio_init.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	gpio_init.GPIO_Mode = GPIO_Mode_AF;
	gpio_init.GPIO_Speed = GPIO_Speed_100MHz;
	gpio_init.GPIO_OType = GPIO_OType_PP;
	gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &gpio_init);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);
	TIM_TimeBaseInitTypeDef time_init;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	time_init.TIM_Period = 150;
	time_init.TIM_Prescaler = 168-1;
	time_init.TIM_ClockDivision = 0;
	time_init.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &time_init);
	NVIC_InitTypeDef NVIC_InitStructure2;
	NVIC_InitStructure2.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure2.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure2.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure2.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure2);
	TIM_OCInitTypeDef oc_init;
	oc_init.TIM_OCMode = TIM_OCMode_PWM1;
	oc_init.TIM_OutputState = TIM_OutputState_Enable;
	oc_init.TIM_Pulse = 0;
	oc_init.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC1Init(TIM2, &oc_init);
	TIM_OC2Init(TIM2, &oc_init);
	TIM_OC3Init(TIM2, &oc_init);
	TIM_OC4Init(TIM2, &oc_init);
	//TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	//TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_UpdateRequestConfig(TIM2, TIM_UpdateSource_Global);      // Обновление только при переполнении
	TIM_ITConfig(TIM2, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,ENABLE);
	TIM_Cmd(TIM2, ENABLE);

	TIM2->CCR1 = 0;
	TIM2->CCR2 = 0;
	TIM2->CCR3 = 0;
	TIM2->CCR4 = 0;
}
void TIM2_IRQHandler(){
	if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET) {
		if (pwm_count_ch1==pwm_aim_ch1) TIM2->CCR1 = 0xffff;
		else pwm_count_ch1+=d_pwm_ch1;
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
	}
	if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET) {
		if (pwm_count_ch2==pwm_aim_ch2) TIM2->CCR2 =  0xffff;
		else pwm_count_ch2+=d_pwm_ch2;
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
	}
	if (TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET) {
		if (pwm_count_ch3==pwm_aim_ch3) TIM2->CCR3 =  0xffff;
		else pwm_count_ch3+=d_pwm_ch3;
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);
	}
	if (TIM_GetITStatus(TIM2, TIM_IT_CC4) != RESET) {
		if (pwm_count_ch4==pwm_aim_ch4) TIM2->CCR4 =  0xffff;
		else pwm_count_ch4+=d_pwm_ch4;
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
	}
}

void ic_init_ch1_2() {
	// TIM1 clock enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	// GPIOC clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	// Enable the TIM1 global Interrupt
	NVIC_InitTypeDef NVIC_InitStructure2;
	NVIC_InitStructure2.NVIC_IRQChannel = TIM1_CC_IRQn;
	NVIC_InitStructure2.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure2.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure2.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure2);
	// Time base configuration
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 0xffff;
	TIM_TimeBaseStructure.TIM_Prescaler = 168/4-1;   //17;   //20
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	//
	tim1_ch1();
	// Select the slave Mode: Reset Mode
	TIM_SelectSlaveMode(TIM1, TIM_SlaveMode_Reset);
	TIM_SelectMasterSlaveMode(TIM1,TIM_MasterSlaveMode_Enable);
	//
	TIM1->CR1 |= (uint16_t)TIM_CR1_ARPE;
	// Enable the TIM1 Counter
	TIM1->CR1 |= TIM_CR1_CEN;
	// Enable the CC1 Interrupt Request
	TIM1->DIER |= TIM_IT_CC1;
	// Enable the CC2 Interrupt Request
	TIM1->DIER |= TIM_IT_CC2;
}

void tim1_ch1(){
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;
	//
	TIM_PWMIConfig(TIM1, &TIM_ICInitStructure);
	// Select the TIM1 Input Trigger: TI1FP1
	TIM_SelectInputTrigger(TIM1, TIM_TS_TI1FP1);
}
void tim1_ch2(){
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;
	//
	TIM_PWMIConfig(TIM1, &TIM_ICInitStructure);
	// Select the TIM1 Input Trigger: TI1FP1
	TIM_SelectInputTrigger(TIM1, TIM_TS_TI2FP2);
}

void ic_init_ch3_4() {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	NVIC_InitTypeDef NVIC_InitStructure2;
	NVIC_InitStructure2.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure2.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure2.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure2.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure2);
	// Time base configuration
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 0xffff;
	TIM_TimeBaseStructure.TIM_Prescaler = 168/4-1;   //17;   //20
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	//
	tim4_ch1();
	// Select the slave Mode: Reset Mode
	TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset);
	TIM_SelectMasterSlaveMode(TIM4,TIM_MasterSlaveMode_Enable);
	//
	TIM4->CR1 |= (uint16_t)TIM_CR1_ARPE;   //TIM_ARRPreloadConfig(TIM4, ENABLE);
	// Enable the TIM4 Counter
	TIM4->CR1 |= TIM_CR1_CEN;
	// Enable the CC1 Interrupt Request
	TIM4->DIER |= TIM_IT_CC1;
	// Enable the CC2 Interrupt Request
	TIM4->DIER |= TIM_IT_CC2;
}

void tim4_ch1(){
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	// Connect TIM pin to AF2
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;
	//
	TIM_PWMIConfig(TIM4, &TIM_ICInitStructure);

	// Select the TIM2 Input Trigger: TI1FP1
	TIM_SelectInputTrigger(TIM4, TIM_TS_TI1FP1);
}
void tim4_ch2(){
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	// Connect TIM pin to AF2
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;
	//
	TIM_PWMIConfig(TIM4, &TIM_ICInitStructure);

	// Select the TIM2 Input Trigger: TI1FP1
	TIM_SelectInputTrigger(TIM4, TIM_TS_TI2FP2);
}


void TIM1_CC_IRQHandler(void)
{
	if (t1_ch==1){
		if ( TIM1->SR & TIM_IT_CC1 )
		{
			vel_period += TIM1->CCR1;
			// Clear TIM1 Capture compare interrupt pending bit
			TIM1->SR &= (uint16_t)~TIM_IT_CC1;
		}
		if ( TIM1->SR & TIM_IT_CC2 )
		{
			// Clear TIM4 Capture compare interrupt pending bit
			TIM1->SR &= (uint16_t)~TIM_IT_CC2;
			vel_weidth += TIM1->CCR2;
			vel_count++;
		}
		if (vel_count==20){
			//print_data(vel_count, vel_period, vel_mean,(uint16_t)pwm_count_ch1);
			t1_ch=2;
			tim1_ch2();
			vel_mean=vel_weidth/vel_count;
			vel_weidth=0;
			vel_count=0;
			vel_period=0;
		}
	}
	else{
		if ( TIM1->SR & TIM_IT_CC1 )
		{
			dir_weidth += TIM1->CCR1;
			// Clear TIM1 Capture compare interrupt pending bit
			TIM1->SR &= (uint16_t)~TIM_IT_CC1;
		}
		if ( TIM1->SR & TIM_IT_CC2 )
		{
			// Clear TIM4 Capture compare interrupt pending bit
			TIM1->SR &= (uint16_t)~TIM_IT_CC2;
			dir_period += TIM1->CCR2;
			dir_count++;
		}
		if (dir_count==20){
			//print_data(dir_count, dir_period, dir_weidth,(uint16_t)pwm_count_ch2);
			t1_ch=1;
			tim1_ch1();
			dir_mean=dir_weidth/dir_count;
			dir_count=0;
			dir_period=0;
			dir_weidth=0;
		}
	}
}
void TIM4_IRQHandler(void)
{
	if (t4_ch==1){
		if ( TIM4->SR & TIM_IT_CC1 )
		{
			rx_period += TIM4->CCR1;
			// Clear TIM1 Capture compare interrupt pending bit
			TIM4->SR &= (uint16_t)~TIM_IT_CC1;
		}
		if ( TIM4->SR & TIM_IT_CC2 )
		{
			// Clear TIM4 Capture compare interrupt pending bit
			TIM4->SR &= (uint16_t)~TIM_IT_CC2;
			rx_weidth += TIM4->CCR2;
			rx_count++;
		}
		if (rx_count==20){
			//print_data(rx_count, rx_period, rx_weidth,(uint16_t)pwm_count_ch3);
			t4_ch=2;
			tim4_ch2();
			rx_mean=rx_weidth/rx_count;
			rx_count=0;
			rx_period=0;
			rx_weidth=0;
		}
	}
	else{
		if ( TIM4->SR & TIM_IT_CC1 )
		{
			ry_weidth += TIM4->CCR1;
			// Clear TIM1 Capture compare interrupt pending bit
			TIM4->SR &= (uint16_t)~TIM_IT_CC1;
		}
		if ( TIM4->SR & TIM_IT_CC2 )
		{
			// Clear TIM4 Capture compare interrupt pending bit
			TIM4->SR &= (uint16_t)~TIM_IT_CC2;
			ry_period += TIM4->CCR2;
			ry_count++;
		}
		if (ry_count==20){
			//print_data(ry_count, ry_period, ry_weidth,(uint16_t)pwm_count_ch4);
			t4_ch=1;
			tim4_ch1();
			ry_mean=ry_weidth/ry_count;
			ry_count=0;
			ry_period=0;
			ry_weidth=0;
		}
	}
}

double sign(double a){
	if (a>0) return 1;
	else if (a<0) return -1;
	else return 0;
}

void kinematica(double gam, double R, double V, double L, double C){
	double x,y;
	double Rfl, Rfr, Rrl, Rrr;
	if (fabs(R)>10000) R=10000;
	x=R*cos(M_PI_2+gam);
	y=R*sin(M_PI_2+gam);
	if (R>0){
		gfl=atan2(L/2-x, y-C/2);
		gfr=atan2(L/2-x, y+C/2);
		grl=atan2(-L/2-x, y-C/2);
		grr=atan2(-L/2-x, y+C/2);
	}
	else{
		grr=atan2(L/2+x, -y-C/2);
		grl=atan2(L/2+x, -y+C/2);
		gfr=atan2(-L/2+x, -y-C/2);
		gfl=atan2(-L/2+x, -y+C/2);
	}

	Rfl=fabs(y-C/2)/fabs(cos(gfl));
	Rfr=fabs(y+C/2)/fabs(cos(gfr));
	Rrl=fabs(y-C/2)/fabs(cos(grl));
	Rrr=fabs(y+C/2)/fabs(cos(grr));

	char str4[200]={0};
	sprintf(str4,"Rfl:%d.%d; Rfr:%d.%d; Rrl:%d.%d; Rrr:%d.%d;\r\n",
			(int)Rfl,ost(Rfl),(int)Rfr,ost(Rfr), (int)Rrl,ost(Rrl), (int)Rrr,ost(Rrr));
	//send_str(str4);

	if(Rfl>10000) Rfl=10000;
	if(Rfr>10000) Rfr=10000;
	if(Rrl>10000) Rrl=10000;
	if(Rrr>10000) Rrr=10000;

	double Rm=Rfl;
	if (Rm<Rfr) Rm=Rfr;
	if (Rm<Rrl) Rm=Rrl;
	if (Rm<Rrr) Rm=Rrr;

	Vfl=V*Rfl/Rm;
	Vfr=V*Rfr/Rm;
	Vrl=V*Rrl/Rm;
	Vrr=V*Rrr/Rm;
}

void normaliz(){
	double d_dir=(dir_mean-dir_period_mean);
	double d_vel=-(vel_mean-vel_period_mean);
	if (fabs(d_dir)<10) d_dir=0;
	if (fabs(d_vel)<10) d_vel=0;
	double Rg0=d_dir*k_dir/dir_period_max;
	if (fabs(Rg0)>k_dir) Rg0=sign(Rg0)*k_dir;
	R=L/2/tan(Rg0)*10;
	if (fabs(R)<0.2) R=0.001*sign(R);
	V=(d_vel)*k_vel/vel_period_max;
	double d_ry=(ry_mean-ry_period_mean)/10*10;
	double d_rx=-(rx_mean-rx_period_mean)/10*10;
	if (fabs(d_rx)<20) d_rx=0;
	if (fabs(d_ry)<20) d_ry=0;
	gam=-atan2(d_ry,d_rx);
}

void INTTIM_Config(void)
{
	NVIC_InitTypeDef nvic_struct;
	nvic_struct.NVIC_IRQChannel = TIM7_IRQn;
	nvic_struct.NVIC_IRQChannelPreemptionPriority = 3;
	nvic_struct.NVIC_IRQChannelSubPriority = 3;
	nvic_struct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_struct);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
	TIM_TimeBaseStructure.TIM_Period = 10000 - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 1680 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM7, ENABLE);
}

void send_to_uart(uint8_t data)
{
	while(!(USART6->SR & USART_SR_TC));
	USART6->DR=data;
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
int ost(double a){
	double intpart;                     // целая часть
	return (int)(modf(a,&intpart)*1000);
}
void TIM7_IRQHandler(void) {//зона нечувствительности около нуля
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET) {
		normaliz();
		k_vel=5*(constract-0x30)/9;
		if(V<10)
			kinematica(gam, R, V, L, C);
		pwm_aim_ch1=gfl*57.3*400/360;
		pwm_aim_ch2=gfr*57.3*400/360;
		pwm_aim_ch3=grl*57.3*400/360;
		pwm_aim_ch4=grr*57.3*400/360;

		if (pwm_aim_ch1>pwm_count_ch1) {//ch1
			GPIO_SetBits(GPIOC,GPIO_Pin_0);
			TIM2->CCR1 = 50;
			d_pwm_ch1=1;
		}
		else if (pwm_aim_ch1<pwm_count_ch1) {
			TIM2->CCR1 = 50;
			d_pwm_ch1=-1;
			GPIO_ResetBits(GPIOC,GPIO_Pin_0);
		}
		else TIM2->CCR1 = 0xffff;

		if (pwm_aim_ch2>pwm_count_ch2) {//ch2
			GPIO_SetBits(GPIOC,GPIO_Pin_1);
			TIM2->CCR2 = 50;
			d_pwm_ch2=1;
		}
		else if (pwm_aim_ch2<pwm_count_ch2) {
			TIM2->CCR2 = 50;
			GPIO_ResetBits(GPIOC,GPIO_Pin_1);
			d_pwm_ch2=-1;
		}
		else TIM2->CCR2 = 0xffff;

		if (pwm_aim_ch3>pwm_count_ch3) {//ch3
			GPIO_SetBits(GPIOC,GPIO_Pin_2);
			TIM2->CCR3 = 50;
			d_pwm_ch3=1;
		}
		else if (pwm_aim_ch3<pwm_count_ch3) {
			TIM2->CCR3 = 50;
			GPIO_ResetBits(GPIOC,GPIO_Pin_2);
			d_pwm_ch3=-1;
		}
		else TIM2->CCR3 = 0xffff;

		if (pwm_aim_ch4>pwm_count_ch4) {//ch4
			GPIO_SetBits(GPIOC,GPIO_Pin_3);
			TIM2->CCR4 = 50;
			d_pwm_ch4=1;
		}
		else if (pwm_aim_ch4<pwm_count_ch4) {
			TIM2->CCR4 = 50;
			GPIO_ResetBits(GPIOC,GPIO_Pin_3);
			d_pwm_ch4=-1;
		}
		else TIM2->CCR4 = 0xffff;

		speedControl(motorfl,Vfl);
		speedControl(motorfr,Vfr);
		speedControl(motorrl,Vrl);
		speedControl(motorrr,Vrr);

		while(!(UART5->SR & USART_SR_TC));
		USART_SendData(UART5,(uint16_t)(gam*57.3+180)/10);

		char str1[200]={0};
		char str2[200]={0};
		char str3[200]={0};
		char str4[200]={0};
		sprintf(str1,"gam:%d.%d; R:%d.%d; V:%d.%d; L:%d.%d; C:%d.%d;\r\n",
				(int)gam,ost(gam),(int)R,ost(R), (int)V,ost(V), (int)L,ost(L), (int)C,ost(C));
		sprintf(str2,"gfl:%d.%d; gfr:%d.%d; grl:%d.%d; grr:%d.%d;\r\n",
				(int)gfl,ost(gfl),(int)gfr,ost(gfr), (int)grl,ost(grl), (int)grr,ost(grr));
		sprintf(str3,"Vfl:%d.%d; Vfr:%d.%d; Vrl:%d.%d; Vrr:%d.%d;\r\n",
				(int)Vfl,ost(Vfl),(int)Vfr,ost(Vfr), (int)Vrl,ost(Vrl), (int)Vrr,ost(Vrr));
		sprintf(str4,"a1:%d; c1:%d;\r\n",
				(int)pwm_aim_ch1,(int)pwm_count_ch1);
		//print_data(vel_mean, dir_mean, rx_mean, ry_mean);
		//send_str(str1);
		send_str(str2);
		send_str(str3);
		send_str(str4);
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	}
}

void speedControl(motorType motor, double Vel){
	USART_TypeDef * MOTOR = motor.channel;
	motor.refSpeed=Vel*5000/5;
	if(motor.lastSpeed!=motor.refSpeed)
	{

		if(fabs(motor.refSpeed)<1500 && fabs(motor.lastSpeed)>=1500)
		{
			setSpeedPID(1000,1000,500,MOTOR);
		}
		if(fabs(motor.refSpeed)>1500 && fabs(motor.refSpeed)<3000)
		{
			if(fabs(motor.lastSpeed)<=1500 || fabs(motor.lastSpeed)>=3000)
				setSpeedPID(2100,1750,700,MOTOR);
		}
		if(fabs(motor.refSpeed)>3000 && fabs(motor.lastSpeed)<=3000)
		{
			setSpeedPID(4000,2850,1500,MOTOR);
		}

		if(setSpeed(motor.refSpeed,100,MOTOR)==ERROR_NONE)
		{
			motor.lastSpeed=motor.refSpeed;
		}
	}
}
void motorStatus(motorType motor){
	USART_TypeDef * MOTOR = motor.channel;
	int speed,torque,state,flux;
	getSpeed(MOTOR,&speed);
	getMotorState(MOTOR,&state);
	getTorque(MOTOR,&torque);
	getFlux(MOTOR,&flux);
	char str4[200]={0};
	sprintf(str4,"speed:%d; torque:%d; state:%d; flux:%d;\r\n",
			speed,torque,state,flux);
	//send_str(str4);
}
