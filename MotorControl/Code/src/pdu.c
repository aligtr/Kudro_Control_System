#include "pdu.h"
#define pi 3.14159265
#define deg 57.29577951
uint32_t cnt=0;
uint8_t flag=0;
uint32_t period=0;
double gam=0;
double R=9999;
double V=1;
double L=0.945;
double C=0.63;
double vel_period_max=320;
double vel_period_mean=596;
double dir_period_max=320;
double dir_period_mean=597;
double rx_period_mean=439;
double rx_period_max=320;
double ry_period_mean=596;
double ry_period_max=320;
double k_vel=5;
double k_dir=3.14;

	double d_dir;
double gfl, gfr, grl, grr;
double Vfl, Vfr, Vrl, Vrr;

void send_to_uart(uint8_t data);
void send_str(char * string);

int d_pwm_ch1=0;
int d_pwm_ch2=0;
int d_pwm_ch3=0;
int d_pwm_ch4=0;

int vel_mean=0;

int rx_mean=0;

int dir_mean=0;
int ry_mean=0;
int pwm_count_ch1=0;
int pwm_count_ch2=0;
int pwm_count_ch3=0;
int pwm_count_ch4=0;
int pwm_aim_ch1=0;
int pwm_aim_ch2=0;
int pwm_aim_ch3=0;
int pwm_aim_ch4=0;

int vel_src=0;
int smt_src1=0;
int smt_src2=0;
int smt_src3=0;

extern uint16_t constract;
char ch_pack=0;

extern uint8_t mode;

void TIM1_CC_IRQHandler(void)
{
	if (TIM1->SR & TIM_SR_CC1IF) {
		if (pwm_count_ch1==pwm_aim_ch1) TIM1->CCR1 = 0xffff;
		else pwm_count_ch1+=d_pwm_ch1;
		TIM1->SR&=~TIM_SR_CC1IF;
	}
}

void TIM8_CC_IRQHandler(void)
{
	if (TIM8->SR & TIM_SR_CC1IF) {
		if (pwm_count_ch2==pwm_aim_ch2) TIM8->CCR1 =  0xffff;
		else pwm_count_ch2+=d_pwm_ch2;
		TIM8->SR&=~TIM_SR_CC1IF;
	}
}

void TIM1_BRK_TIM9_IRQHandler(void)
{
	if (TIM9->SR & TIM_SR_CC1IF) {
		if (pwm_count_ch3==pwm_aim_ch3) TIM9->CCR1 =  0xffff;
		else pwm_count_ch3+=d_pwm_ch3;
		TIM9->SR&=~TIM_SR_CC1IF;
	}
}
void TIM8_BRK_TIM12_IRQHandler(void)
{
	if (TIM12->SR & TIM_SR_CC1IF) {
		if (pwm_count_ch4==pwm_aim_ch4) TIM12->CCR1 =  0xffff;
		else pwm_count_ch4+=d_pwm_ch4;
		TIM12->SR&=~TIM_SR_CC1IF;
	}
}

void tim1Init(void)
{
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOCEN;
	GPIOC->MODER|=GPIO_MODER_MODE0_0 | GPIO_MODER_MODE1_0 | GPIO_MODER_MODE2_0 | GPIO_MODER_MODE3_0;//PC0-PC3 в режим выходов
	GPIOC->OTYPER|=GPIO_OTYPER_OT0 | GPIO_OTYPER_OT1 | GPIO_OTYPER_OT2 | GPIO_OTYPER_OT3;
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOEEN;
	GPIOE->MODER|=GPIO_MODER_MODE9_1;//PE9,PC6,PE5,PB14 в режим альтернативной функции
	GPIOE->OTYPER|=GPIO_OTYPER_OT9;
	GPIOE->AFR[1]=(1<<4);//AF1 enable
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	TIM1->CCER = 0;
	TIM1->ARR = 150;
	TIM1->PSC = 1680/4-1;
	TIM1->BDTR |= TIM_BDTR_MOE;
	TIM1->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	TIM1->CCER |= TIM_CCER_CC1E;
	TIM1->CR1 |= TIM_CR1_CEN;
	TIM1->DIER|=TIM_DIER_CC1IE;//Настройка прерываний по совпадению
	TIM1->CCR1=0xffff;
}

void tim8Init(void)
{
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOCEN;
	GPIOC->MODER|=GPIO_MODER_MODE6_1;//PE9,PC6,PE5,PB14 в режим альтернативной функции
	GPIOC->OTYPER|=GPIO_OTYPER_OT6;
	GPIOC->AFR[0]=(3<<24);//AF1 enable
	RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
	TIM8->CCER = 0;
	TIM8->ARR = 150;
	TIM8->PSC = 1680/4-1;
	TIM8->BDTR |= TIM_BDTR_MOE;
	TIM8->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	TIM8->CCER |= TIM_CCER_CC1E;
	TIM8->CR1 |= TIM_CR1_CEN;
	TIM8->DIER|=TIM_DIER_CC1IE;//Настройка прерываний по совпадению
	TIM8->CCR1=0xffff;
}

void tim9Init(void)
{
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOEEN;
	GPIOE->MODER|=GPIO_MODER_MODE5_1;//PE5 в режим AF
	GPIOE->OTYPER|=GPIO_OTYPER_OT5;
	GPIOE->AFR[0]=(3<<20);
	RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
	TIM9->CCER = 0;
	TIM9->ARR = 150;
	TIM9->PSC = 1680/4-1;
	TIM9->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	TIM9->CCER |= TIM_CCER_CC1E;
	TIM9->DIER|=TIM_DIER_CC1IE;
	TIM9->CR1 |= TIM_CR1_CEN;
	TIM9->CCR1=0xffff;
}

void tim12Init(void)
{
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOBEN;
	GPIOB->MODER|=GPIO_MODER_MODE14_1;//PB14 в режим AF
	GPIOB->OTYPER|=GPIO_OTYPER_OT14;
	GPIOB->AFR[1]=(9<<24);
	RCC->APB1ENR |= RCC_APB1ENR_TIM12EN;
	TIM12->CCER = 0;
	TIM12->ARR = 150;
	TIM12->PSC = 1680/4-1;
	TIM12->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	TIM12->CCER |= TIM_CCER_CC1E;
	TIM12->DIER|=TIM_DIER_CC1IE;
	TIM12->CR1 |= TIM_CR1_CEN;
	TIM12->CCR1=0xffff;
}

void tim2Init1(void)
{
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN;//Тактирование потра A
	GPIOB->MODER&=~GPIO_MODER_MODE3_1;;//PA5 на AF
	GPIOB->AFR[0]&=~(1<<12);
	GPIOA->MODER|=GPIO_MODER_MODE5_1;;//PA5 на AF
	GPIOA->AFR[0]|=1<<20;
	RCC->APB1ENR|=RCC_APB1ENR_TIM2EN;//Включить тактирование таймера-счетчика 9
	TIM2->PSC=1680/4;//Делитель на 10КГц
	TIM2->ARR=0xFFFF;//Считаем до конца
	
	TIM2->CR1=0;
	TIM2->CCER=0;
	TIM2->SMCR=0;
	TIM2->DIER=0;
	TIM2->CCMR1=0;
	
	//Канал 1
	TIM2->CCMR1|=TIM_CCMR1_CC1S_0;
	TIM2->CCMR1|=TIM_CCMR1_IC1F_0;
	//Канал 2
	TIM2->CCMR1|=TIM_CCMR1_CC2S_1;
	TIM2->CCMR1|=TIM_CCMR1_IC2F_0;
	
	TIM2->SMCR|=TIM_SMCR_TS_0 | TIM_SMCR_TS_2;
	TIM2->DIER|=TIM_DIER_CC1IE;
	//TIM2->DIER|=TIM_DIER_CC2IE;
	TIM2->SMCR|=TIM_SMCR_SMS_2;//Master mode
	
	TIM2->CCER|=TIM_CCER_CC2P;
	
	TIM2->CCER|=TIM_CCER_CC1E;//Режим захвата
	TIM2->CCER|=TIM_CCER_CC2E;//Режим захвата
	
	TIM2->CR1|=TIM_CR1_CEN;
}

void tim2Init2(void)
{
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOBEN;//Тактирование потра A
	GPIOA->MODER&=~GPIO_MODER_MODE5_1;;//PA5 на AF
	GPIOA->AFR[0]&=~(1<<20);
	GPIOB->MODER|=GPIO_MODER_MODE3_1;;//PB3 на AF
	GPIOB->AFR[0]|=(1<<12);
	RCC->APB1ENR|=RCC_APB1ENR_TIM2EN;//Включить тактирование таймера-счетчика 9
	TIM2->PSC=1680/4;//Делитель на 10КГц
	TIM2->ARR=0xFFFF;//Считаем до конца
	
	TIM2->CR1=0;
	TIM2->CCER=0;
	TIM2->SMCR=0;
	TIM2->DIER=0;
	TIM2->CCMR1=0;
	
	//Канал 1
	TIM2->CCMR1|=TIM_CCMR1_CC1S_1;
	TIM2->CCMR1|=TIM_CCMR1_IC1F_0;
	//Канал 2
	TIM2->CCMR1|=TIM_CCMR1_CC2S_0;
	TIM2->CCMR1|=TIM_CCMR1_IC2F_0;
	
	TIM2->SMCR|=TIM_SMCR_TS_1 | TIM_SMCR_TS_2;
	TIM2->DIER|=TIM_DIER_CC1IE;
	
	TIM2->SMCR|=TIM_SMCR_SMS_2;//Master mode

	TIM2->CCER|=TIM_CCER_CC1P;
	
	TIM2->CCER|=TIM_CCER_CC1E;//Режим захвата
	TIM2->CCER|=TIM_CCER_CC2E;//Режим захвата
	
	TIM2->CR1|=TIM_CR1_CEN;
}

void tim3Init1(void)
{
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN;//Тактирование потра A
	GPIOA->MODER&=~GPIO_MODER_MODE7_1;;//PA6 на AF
	GPIOA->AFR[0]&=~(2<<28);
	GPIOA->MODER|=GPIO_MODER_MODE6_1;;//PA6 на AF
	GPIOA->AFR[0]|=2<<24;
	RCC->APB1ENR|=RCC_APB1ENR_TIM3EN;//Включить тактирование таймера-счетчика 9
	TIM3->PSC=1680/4;//Делитель на 10КГц
	TIM3->ARR=0xFFFF;//Считаем до конца

	TIM3->CR1=0;
	TIM3->CCER=0;
	TIM3->SMCR=0;
	TIM3->DIER=0;
	TIM3->CCMR1=0;
	//Канал 1
	TIM3->CCMR1|=TIM_CCMR1_CC1S_0;
	TIM3->CCMR1|=TIM_CCMR1_IC1F_0;
	//Канал 2
	TIM3->CCMR1|=TIM_CCMR1_CC2S_1;
	TIM3->CCMR1|=TIM_CCMR1_IC2F_0;
	
	TIM3->SMCR|=TIM_SMCR_TS_0 | TIM_SMCR_TS_2;
	TIM3->DIER|=TIM_DIER_CC1IE;
	
	TIM3->SMCR|=TIM_SMCR_SMS_2;//Master mode
	
	TIM3->CCER|=TIM_CCER_CC2P;
	
	TIM3->CCER|=TIM_CCER_CC1E;//Режим захвата
	TIM3->CCER|=TIM_CCER_CC2E;//Режим захвата
	
	TIM3->CR1|=TIM_CR1_CEN;
}
void tim3Init2(void)
{
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN;//Тактирование потра A
	GPIOA->MODER&=~GPIO_MODER_MODE6_1;;//PA6 на AF
	GPIOA->AFR[0]&=~(2<<24);
	GPIOA->MODER|=GPIO_MODER_MODE7_1;;//PA6 на AF
	GPIOA->AFR[0]|=2<<28;
	RCC->APB1ENR|=RCC_APB1ENR_TIM3EN;//Включить тактирование таймера-счетчика 9
	TIM3->PSC=1680/4;//Делитель на 10КГц
	TIM3->ARR=0xFFFF;//Считаем до конца

	TIM3->CR1=0;
	TIM3->CCER=0;
	TIM3->SMCR=0;
	TIM3->DIER=0;
	TIM3->CCMR1=0;
	//Канал 1
	TIM3->CCMR1|=TIM_CCMR1_CC1S_1;
	TIM3->CCMR1|=TIM_CCMR1_IC1F_0;
	//Канал 2
	TIM3->CCMR1|=TIM_CCMR1_CC2S_0;
	TIM3->CCMR1|=TIM_CCMR1_IC2F_0;
	
	TIM3->SMCR|=TIM_SMCR_TS_1 | TIM_SMCR_TS_2;
	TIM3->DIER|=TIM_DIER_CC1IE;
	
	TIM3->SMCR|=TIM_SMCR_SMS_2;//Master mode
	
	TIM3->CCER|=TIM_CCER_CC1P;
	
	TIM3->CCER|=TIM_CCER_CC1E;//Режим захвата
	TIM3->CCER|=TIM_CCER_CC2E;//Режим захвата
	
	TIM3->CR1|=TIM_CR1_CEN;
}

void tim4Init1(void)
{
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIODEN;//Тактирование потра D
	GPIOD->MODER&=~GPIO_MODER_MODE13_1;//PD13 на AF
	GPIOD->AFR[1]&=~(2<<20);
	GPIOD->MODER|=GPIO_MODER_MODE12_1;;//PD12 на AF
	GPIOD->AFR[1]|=(2<<16);
	RCC->APB1ENR|=RCC_APB1ENR_TIM4EN;//Включить тактирование таймера-счетчика 9
	TIM4->PSC=1680/4;//Делитель на 10КГц
	TIM4->ARR=0xFFFF;//Считаем до конца

	TIM4->CR1=0;
	TIM4->CCER=0;
	TIM4->SMCR=0;
	TIM4->DIER=0;
	TIM4->CCMR1=0;
		
	//Канал 1
	TIM4->CCMR1|=TIM_CCMR1_CC1S_0;
	TIM4->CCMR1|=TIM_CCMR1_IC1F_0;
	//Канал 2
	TIM4->CCMR1|=TIM_CCMR1_CC2S_1;
	TIM4->CCMR1|=TIM_CCMR1_IC2F_0;
	TIM4->CCER|=TIM_CCER_CC2P;
	
	TIM4->SMCR|=TIM_SMCR_TS_0 | TIM_SMCR_TS_2;
	TIM4->DIER|=TIM_DIER_CC1IE;
	TIM4->DIER|=TIM_DIER_CC2IE;
	TIM4->SMCR|=TIM_SMCR_SMS_2;//Master mode
	
	TIM4->CCER|=TIM_CCER_CC1E;//Режим захвата
	TIM4->CCER|=TIM_CCER_CC2E;//Режим захвата
	TIM4->CR1|=TIM_CR1_CEN;
}

void tim4Init2(void)
{
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIODEN;//Тактирование потра A
	GPIOD->MODER&=~GPIO_MODER_MODE12_1;;//PD12 на AF
	GPIOD->AFR[1]&=~(2<<16);
	GPIOD->MODER|=GPIO_MODER_MODE13_1;//PD13 на AF
	GPIOD->AFR[1]|=(2<<20);
	RCC->APB1ENR|=RCC_APB1ENR_TIM4EN;//Включить тактирование таймера-счетчика 9
	TIM4->PSC=1680/4;//Делитель на 10КГц
	TIM4->ARR=0xFFFF;//Считаем до конца

	TIM4->CR1=0;
	TIM4->CCER=0;
	TIM4->SMCR=0;
	TIM4->DIER=0;
	TIM4->CCMR1=0;
		
	//Канал 1
	TIM4->CCMR1|=TIM_CCMR1_CC1S_1;
	TIM4->CCMR1|=TIM_CCMR1_IC1F_0;
	//Канал 2
	TIM4->CCMR1|=TIM_CCMR1_CC2S_0;
	TIM4->CCMR1|=TIM_CCMR1_IC2F_0;
	TIM4->CCER|=TIM_CCER_CC1P;
	
	TIM4->SMCR|=TIM_SMCR_TS_1 | TIM_SMCR_TS_2;
	TIM4->DIER|=TIM_DIER_CC1IE;
	TIM4->DIER|=TIM_DIER_CC2IE;
	TIM4->SMCR|=TIM_SMCR_SMS_2;//Master mode
	
	TIM4->CCER|=TIM_CCER_CC1E;//Режим захвата
	TIM4->CCER|=TIM_CCER_CC2E;//Режим захвата
	TIM4->CR1|=TIM_CR1_CEN;
}

void tim5Init1(void)
{
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN;//Тактирование потра A
	GPIOA->MODER&=~GPIO_MODER_MODE1_1;//PA0 на AF
	GPIOA->AFR[0]&=~(2<<4);
	GPIOA->MODER|=GPIO_MODER_MODE0_1;//PA0 на AF
	GPIOA->AFR[0]|=2<<0;
	RCC->APB1ENR|=RCC_APB1ENR_TIM5EN;//Включить тактирование таймера-счетчика 9
	TIM5->PSC=1680/4 ;//Делитель на 10КГц
	TIM5->ARR=0xFFFF;//Считаем до конца
	
	TIM5->CR1=0;
	TIM5->CCER=0;
	TIM5->SMCR=0;
	TIM5->DIER=0;
	TIM5->CCMR1=0;
	//Канал 1
	TIM5->CCMR1|=TIM_CCMR1_CC1S_0;
	TIM5->CCMR1|=TIM_CCMR1_IC1F_0;
	//Канал 2
	TIM5->CCMR1|=TIM_CCMR1_CC2S_1;
	TIM5->CCMR1|=TIM_CCMR1_IC2F_0;
	
	TIM5->SMCR|=TIM_SMCR_TS_0 | TIM_SMCR_TS_2;
	TIM5->DIER|=TIM_DIER_CC1IE;
	TIM5->SMCR|=TIM_SMCR_SMS_2;//Master mode
	
	TIM5->CCER|=TIM_CCER_CC2P;
	
	TIM5->CCER|=TIM_CCER_CC1E;//Режим захвата
	TIM5->CCER|=TIM_CCER_CC2E;//Режим захвата
	TIM5->CR1|=TIM_CR1_CEN;
}

void tim5Init2(void)
{
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN;//Тактирование потра A
	GPIOA->MODER&=~GPIO_MODER_MODE0_1;//PA0 на AF
	GPIOA->AFR[0]&=~(2<<0);
	GPIOA->MODER|=GPIO_MODER_MODE1_1;//PA0 на AF
	GPIOA->AFR[0]|=2<<4;
	RCC->APB1ENR|=RCC_APB1ENR_TIM5EN;//Включить тактирование таймера-счетчика 9
	TIM5->PSC=1680/4 ;//Делитель на 10КГц
	TIM5->ARR=0xFFFF;//Считаем до конца
	
	TIM5->CR1=0;
	TIM5->CCER=0;
	TIM5->SMCR=0;
	TIM5->DIER=0;
	TIM5->CCMR1=0;
	//Канал 1
	TIM5->CCMR1|=TIM_CCMR1_CC1S_1;
	TIM5->CCMR1|=TIM_CCMR1_IC1F_0;
	//Канал 2
	TIM5->CCMR1|=TIM_CCMR1_CC2S_0;
	TIM5->CCMR1|=TIM_CCMR1_IC2F_0;
	
	TIM5->SMCR|=TIM_SMCR_TS_1 | TIM_SMCR_TS_2;
	TIM5->DIER|=TIM_DIER_CC1IE;
	TIM5->SMCR|=TIM_SMCR_SMS_2;//Master mode
	
	TIM5->CCER|=TIM_CCER_CC1P;
	
	TIM5->CCER|=TIM_CCER_CC1E;//Режим захвата
	TIM5->CCER|=TIM_CCER_CC2E;//Режим захвата
	TIM5->CR1|=TIM_CR1_CEN;
}

void send_to_uart(uint8_t data)
{
	while(!(USART6->SR & USART_SR_TC));
	USART6->DR=data;
}

//������� ���������� ������ � UART, �� ���� ��������� �� ����� � send_to_uart
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
	double intpart;                     // ����� �����
	return (int)(modf(a,&intpart)*1000);
}

double sign(double a){
	if (a>0) return 1;
	else if (a<0) return -1;
	else return 0;
}

void kinematica(){
	double x,y;
	double Rfl, Rfr, Rrl, Rrr;   
	double dgfl, dgfr, dgrl, dgrr;
	
	uint16_t gamma;
	char str1[200]={0};
	char str2[200]={0};
	char str3[200]={0};
	char str4[200]={0};
	double Rm, gm;
	normaliz();
	switch(mode)
	{
		case 1:
			k_vel=5*(constract)/40;// добавить 3 режима: 0, V, constract
			break;
		case 2:
			k_vel=5;// добавить 3 режима: 0, V, constract
			break;
		case 3:
			k_vel=0;
		break;
	}
	if (fabs(R)>900) R=10000;
	x=R*cos(3.14/2+gam);
	y=R*sin(3.14/2+gam);
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

	if(Rfl>1000) Rfl=100000;
	if(Rfr>1000) Rfr=100000;
	if(Rrl>1000) Rrl=100000;
	if(Rrr>1000) Rrr=100000;
	Rm=Rfl;
	if (Rm<Rfr) Rm=Rfr;
	if (Rm<Rrl) Rm=Rrl;
	if (Rm<Rrr) Rm=Rrr;

	Vfl=V*Rfl/Rm;
	Vfr=-V*Rfr/Rm;
	Vrl=V*Rrl/Rm;
	Vrr=-V*Rrr/Rm;
	if (fabs(gfl)<0.01) gfl=0;
	if (fabs(gfr)<0.01) gfr=0;
	if (fabs(grl)<0.01) grl=0;
	if (fabs(grr)<0.01) grr=0;
	
	pwm_aim_ch1=gfl*deg*800*15/360;
	pwm_aim_ch2=gfr*deg*800*15/360;
	pwm_aim_ch3=grl*deg*800*15/360;
	pwm_aim_ch4=grr*deg*800*15/360;
	
	dgfl=fabs(pwm_aim_ch1-pwm_count_ch1); 
	dgfr=fabs(pwm_aim_ch2-pwm_count_ch2);
	dgrl=fabs(pwm_aim_ch3-pwm_count_ch3);
	dgrr=fabs(pwm_aim_ch4-pwm_count_ch4);	
	
	gm=dgfl;
	if (gm<dgfr) gm=dgfr;
	if (gm<dgrl) gm=dgrl;
	if (gm<dgrr) gm=dgrr;
	TIM1->ARR=150*gm/dgfl;
	TIM8->ARR=150*gm/dgfr;
	TIM9->ARR=150*gm/dgrl;
	TIM12->ARR=150*gm/dgrr;
							 
	if (pwm_aim_ch1>pwm_count_ch1) {//ch1
		GPIOC->BSRR=GPIO_BSRR_BS0;
		TIM1->CCR1 = 50;
		d_pwm_ch1=1;
	}
	else if (pwm_aim_ch1<pwm_count_ch1) {
		TIM1->CCR1 = 50;
		d_pwm_ch1=-1;
		GPIOC->BSRR=GPIO_BSRR_BR0;
	}
	else TIM1->CCR1 = 0xffff;
	if (pwm_aim_ch2>pwm_count_ch2) {//ch2
		GPIOC->BSRR=GPIO_BSRR_BR1;
		TIM8->CCR1 = 50;
		d_pwm_ch2=1;
	}
	else if (pwm_aim_ch2<pwm_count_ch2) {
		TIM8->CCR1 = 50;
		GPIOC->BSRR=GPIO_BSRR_BS1;
		d_pwm_ch2=-1;
	}
	else TIM8->CCR1 = 0xffff;
	if (pwm_aim_ch3>pwm_count_ch3) {//ch3
		GPIOC->BSRR=GPIO_BSRR_BR2;
		TIM9->CCR1 = 50;
		d_pwm_ch3=1;
	}
	else if (pwm_aim_ch3<pwm_count_ch3) {
		TIM9->CCR1 = 50;
		GPIOC->BSRR=GPIO_BSRR_BS2;
		d_pwm_ch3=-1;
	}
	else TIM9->CCR1 = 0xffff;
	if (pwm_aim_ch4>pwm_count_ch4) {//ch4
		GPIOC->BSRR=GPIO_BSRR_BS3;
		TIM12->CCR1 = 50;
		d_pwm_ch4=1;
	}
	else if (pwm_aim_ch4<pwm_count_ch4) {
		TIM12->CCR1 = 50;
		GPIOC->BSRR=GPIO_BSRR_BR3;
		d_pwm_ch4=-1;
	}
	else TIM12->CCR1 = 0xffff;
	gam=gam*deg;
	if (V<0) gam=-sign(gam)*(180-fabs(gam));
	if (V<0 && gam==0) gam=-180;
	if (fmod(gam,5)>=2.5) gam=gam+(5-fmod(gam,5));
	else gam=gam-fmod(gam,5);
	gamma=(uint16_t)(gam+180);
	uartTransmitt(0x0B,UART5);
	uartTransmittBuff((uint8_t*)&gamma,sizeof(uint16_t),UART5);
	uartTransmitt(mode,UART5);
}

void normaliz(){
	double d_vel;
	double d_ry;
	double Rg0;
	double d_rx;
	
	if(vel_mean>800 || vel_mean<400) vel_mean=vel_period_mean;
	if(dir_mean>800 || dir_mean<400) dir_mean=vel_period_mean;
	if(rx_mean>800 || rx_mean<400) rx_mean=vel_period_mean;
	if(ry_mean>800 || ry_mean<400) ry_mean=vel_period_mean;
	
	d_dir=(dir_mean-dir_period_mean);
	d_vel=(vel_mean-vel_period_mean);
	if (fabs(d_dir)<20) d_dir=0;
	if (fabs(d_vel)<20) d_vel=0;
	Rg0=d_dir*k_dir/dir_period_max;
	if (fabs(Rg0)>k_dir) Rg0=sign(Rg0)*k_dir;
	R=L/2/tan(Rg0);
	if (fabs(R)<0.2) R=0.2*sign(R);
	R=R*3;//ограничение радиуса (угол около 50)
	d_rx=(rx_mean-rx_period_mean);
	if (fabs(d_rx)<10) d_rx=0;
	V=(d_vel)*k_vel/vel_period_max*(d_rx/rx_period_max);
	d_ry=ry_mean-ry_period_mean;
	if (fabs(d_ry)<20) d_ry=0;
	gam=pi/2*(d_ry)/ry_period_max; // пределы измерения угла
	if (fabs(gam)<0.01) gam=0;
	if (fabs(gam)>(pi/4-0.1)) gam=pi/4*sign(gam);//мертвая зона по углу
	if (gam!=0) R=100000;
}
