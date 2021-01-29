#include "pdu.h"
//uint16_t frameBuff[2];
//uint8_t reciveStatus;
double gam;
double echo_angle[12];
uint32_t echo_mes[12];
uint32_t start_mes[4]={0,0,0,0};
uint32_t echo_filter[12][4];
uint8_t senseCount=0;
int pduTimer;
int echo_count=0;
uint8_t sensFlag[4]={0,0,0,0};
uint8_t pulseState=0;
uint8_t echoState=0;
uint8_t recivePulseFlag[4]={0,0,0,0};

extern uint8_t idleFlag;

void tim1Init(void)
{
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOEEN;
	GPIOE->MODER|=GPIO_MODER_MODE9_0 | GPIO_MODER_MODE11_0 | GPIO_MODER_MODE13_0;//PE9,PE11,PE13 в режим альтернативной функции
	GPIOE->OSPEEDR|=GPIO_OSPEEDER_OSPEEDR9_1 | GPIO_OSPEEDER_OSPEEDR11_1 | GPIO_OSPEEDER_OSPEEDR13_1;
	//GPIOE->AFR[1]|=(1<<4)|(1<<12)|(1<<20);//AF1 enable
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	TIM1->CCER = 0;
	TIM1->ARR = 10;
	TIM1->PSC = 168-1;
	TIM1->BDTR |= TIM_BDTR_MOE;
	/*TIM1->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	TIM1->CCER |= TIM_CCER_CC1E;
	TIM1->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
	TIM1->CCER |= TIM_CCER_CC2E;
	TIM1->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
	TIM1->CCER |= TIM_CCER_CC3E;
	TIM1->CR1 |= TIM_CR1_CEN;
	TIM1->CCR1=0;
	TIM1->CCR2=0;
	TIM1->CCR3=0; */
	sensFlag[0]=1;
	sensFlag[1]=1;
	sensFlag[2]=1;
	sensFlag[3]=1;
	TIM1->DIER|=TIM_DIER_UIE;
	TIM1->CR1 |= TIM_CR1_CEN;
	NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
}

void tim2Init(void)
{
	
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN;//PA5 PB3 PB10 PB11
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOBEN;
	
	GPIOB->OSPEEDR|=GPIO_OSPEEDER_OSPEEDR10_1|GPIO_OSPEEDER_OSPEEDR11_1| GPIO_OSPEEDER_OSPEEDR3_1;
	GPIOB->MODER|=GPIO_MODER_MODE3_1 | GPIO_MODER_MODE10_1 | GPIO_MODER_MODE11_1;
	GPIOA->OSPEEDR|=GPIO_OSPEEDER_OSPEEDR5_1;
	GPIOA->MODER|=GPIO_MODER_MODE5_1;
	GPIOB->AFR[0]|=(1<<12);//AF3 enable
	GPIOB->AFR[1]|=(1<<8)|(1<<12);
	GPIOA->AFR[0]|=(1<<20);//AF1 enable
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	TIM2->CCER = 0;
	TIM2->ARR = 3000;
	TIM2->PSC = 1680-1;	
	//TIM3->BDTR |= TIM_BDTR_MOE;
	TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	TIM2->CCER |= TIM_CCER_CC1E;
	TIM2->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
	TIM2->CCER |= TIM_CCER_CC2E;
	TIM2->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
	TIM2->CCER |= TIM_CCER_CC3E;
	TIM2->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;
	TIM2->CCER |= TIM_CCER_CC4E;
	TIM2->CR1 |= TIM_CR1_ARPE|TIM_CR1_URS|TIM_CR1_CEN;
	/*TIM2->CCR1=50;
	TIM2->CCR2=50;
	TIM2->CCR3=50;
	TIM2->CCR4=50;*/
}

void tim4Init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOBEN;
	GPIOB->OSPEEDR|=GPIO_OSPEEDER_OSPEEDR6_1|GPIO_OSPEEDER_OSPEEDR7_1|GPIO_OSPEEDER_OSPEEDR8_1|GPIO_OSPEEDER_OSPEEDR9_1;
	GPIOB->MODER|=GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1 | GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1;
	GPIOB->AFR[0]|=(2<<24)|(2<<28);//AF3 enable
	GPIOB->AFR[1]|=(2<<4)|(2<<0);//AF3 enable
	TIM4->CCER = 0;
	TIM4->ARR = 3000;
	TIM4->PSC = 1680-1;
	//TIM4->BDTR |= TIM_BDTR_MOE;
	TIM4->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	TIM4->CCER |= TIM_CCER_CC1E;
	TIM4->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
	TIM4->CCER |= TIM_CCER_CC2E;
	TIM4->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
	TIM4->CCER |= TIM_CCER_CC3E;
	TIM4->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;
	TIM4->CCER |= TIM_CCER_CC4E;
	TIM4->CR1 |= TIM_CR1_CEN;
}

void tim5Init(void)
{
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN;//???????????? ????? A
	GPIOA->MODER|=GPIO_MODER_MODE0_1|GPIO_MODER_MODE1_1|GPIO_MODER_MODE2_1|GPIO_MODER_MODE3_1;//PA0 ?? AF
	GPIOA->AFR[0]|=(2<<0)|(2<<4)|(2<<8)|(2<<12);
	RCC->APB1ENR|=RCC_APB1ENR_TIM5EN;//???????? ???????????? ???????-???????? 9
	TIM5->PSC=168-1;//???????? ?? 10???
	TIM5->ARR=40000;//??????? ?? ?????
	//????? 1
	TIM5->CCMR1|=1|(1<<8);
	//????? 3
	TIM5->CCMR2|=1|(1<<8);
		
	TIM5->CCER|=TIM_CCER_CC1E;//????? ???????
	TIM5->CCER|=TIM_CCER_CC2E;//????? ???????
	TIM5->CCER|=TIM_CCER_CC3E;//????? ???????
	TIM5->CCER|=TIM_CCER_CC4E;//????? ???????
	TIM5->CCER|=TIM_CCER_CC1P | TIM_CCER_CC1NP;
	TIM5->CCER|=TIM_CCER_CC2P | TIM_CCER_CC2NP;
	TIM5->CCER|=TIM_CCER_CC3P | TIM_CCER_CC3NP;
	TIM5->CCER|=TIM_CCER_CC4P | TIM_CCER_CC4NP;
	//TIM5->CR1|=TIM_CR1_OPM;
	TIM5->DIER|=TIM_DIER_CC1IE;
	TIM5->DIER|=TIM_DIER_CC2IE;
	TIM5->DIER|=TIM_DIER_CC3IE;
	TIM5->DIER|=TIM_DIER_CC4IE;
	TIM5->DIER|=TIM_DIER_UIE;
	
	NVIC_EnableIRQ(TIM5_IRQn);
}

void tim8Init(void)
{
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOCEN;
	GPIOC->MODER|=GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1 | GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1;//PC6-9 в режим альтернативной функции
	GPIOC->AFR[0]|=(3<<24)|(3<<28);//AF3 enable
	GPIOC->AFR[1]|=(3<<4)|(3<<0);//AF3 enable
		RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
	TIM8->CCER = 0;
	TIM8->ARR = 3000;
	TIM8->PSC = 1680-1;
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
}

void tim6Init(void)
{
	RCC->APB1ENR|=RCC_APB1ENR_TIM6EN;
	TIM6->PSC=16800;
	TIM6->ARR=9;
	TIM6->DIER|=TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
	TIM6->CR1|=TIM_CR1_CEN;
}

double sign(double a){
	if (a>0) return 1;
	else if (a<0) return -1;
	else return 0;
}
int constr(int a){
	if (a<0) return 0;
	else if (a>180) return 180;
	else return a;
}
int deg_to_pwm(int a){
	return constr(a)*250/180+50;
}

long map(long x,long inMin,long inMax,long outMin, long outMax)
{
	return (((x-inMin)*(outMax-outMin)/(inMax-inMin))+outMin);
}

void TIM5_IRQHandler(void){
	uint8_t i;
	if (TIM5->SR & TIM_SR_UIF)  
	{
		TIM5->SR &=~ TIM_SR_UIF;
		if(sensFlag[0]!=1)
		{
			echo_filter[echoState+0][senseCount]=700;
			sensFlag[0]=1;
		}
		if(sensFlag[1]!=1)
		{
			echo_filter[echoState+3][senseCount]=700;
			sensFlag[1]=1;
		}
		if(sensFlag[2]!=1)
		{
			echo_filter[echoState+6][senseCount]=700;
			sensFlag[2]=1;
		}
		if(sensFlag[3]!=1)
		{
			echo_filter[echoState+9][senseCount]=700;
			sensFlag[3]=1;
		}	
	}
	///////////////////////////////
	if (TIM5->SR & TIM_SR_CC1IF){
		if (sensFlag[0]!=1 && recivePulseFlag[0]==1) 
		{
			echo_filter[echoState+0][senseCount] = (TIM5->CCR1-start_mes[0])/58.2-15;
			sensFlag[0]=1;
		}
		if(recivePulseFlag[0]==0 && sensFlag[0]!=1)
		{
			start_mes[0]=TIM5->CNT;
			recivePulseFlag[0]=1;
		}
		TIM5->SR &=~TIM_SR_CC1IF;
	}
	///////////////////////////////
	if (TIM5->SR & TIM_SR_CC2IF){
		if (sensFlag[1]!=1 && recivePulseFlag[1]==1)
		{
			echo_filter[echoState+3][senseCount] = (TIM5->CCR2-start_mes[1])/58.2;
			sensFlag[1]=1;
		}
		if(recivePulseFlag[1]==0 && sensFlag[1]!=1)
		{
			start_mes[1]=TIM5->CNT;
			recivePulseFlag[1]=1;
		}
		TIM5->SR &=~TIM_SR_CC2IF;
	}
	//////////////////////////////
	if (TIM5->SR & TIM_SR_CC3IF){
		if (sensFlag[2]!=1 && recivePulseFlag[2]==1) 
		{
			echo_filter[echoState+6][senseCount] = (TIM5->CCR3-start_mes[2])/58.2;
			sensFlag[2]=1;
		}
		if(recivePulseFlag[2]==0 && sensFlag[2]!=1)
		{
			start_mes[2]=TIM5->CNT;
			recivePulseFlag[2]=1;
		}
		TIM5->SR &=~TIM_SR_CC3IF;
	}  
	//////////////////////////////
	if (TIM5->SR & TIM_SR_CC4IF)
	{
		if (sensFlag[3]!=1 && recivePulseFlag[3]==1) 
		{
			echo_filter[echoState+9][senseCount] = (TIM5->CCR4-start_mes[3])/58.2;
			sensFlag[3]=1;
		}
		if(recivePulseFlag[3]==0 && sensFlag[3]!=1)
		{
			start_mes[3]=TIM5->CNT;
			recivePulseFlag[3]=1;
		}
		TIM5->SR &=~TIM_SR_CC4IF;
	}
	/////////////////////////////
	if(sensFlag[0] && sensFlag[1] && sensFlag[2] && sensFlag[3])
	{	
		if(echoState!=2) echoState++;
		else 
		{
			echoState=0;
			if(senseCount==2)
			{
				senseCount=0;
				for(i=0;i<12;i++)
				{
					echo_mes[i]=med_filt(i);
				}
			}
			else
				senseCount++;
		}
		recivePulseFlag[0]=0;
		recivePulseFlag[1]=0;
		recivePulseFlag[2]=0;
		recivePulseFlag[3]=0;
		sensFlag[0]=0;
		sensFlag[1]=0;
		sensFlag[2]=0;
		sensFlag[3]=0;
		TIM5->CR1&=~TIM_CR1_CEN;
		TIM5->CNT=0;
		TIM1->CNT=0;
		TIM1->ARR=5000;
		NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
		TIM1->CR1|=TIM_CR1_CEN;
	}
}
double med_filt(uint8_t sensor){
	uint32_t middle;
	uint32_t a=echo_filter[sensor][0];
	uint32_t b=echo_filter[sensor][1];
	uint32_t c=echo_filter[sensor][2];
	if ((a <= b) && (a <= c)) {
		middle = (b <= c) ? b : c;
	} else {
		if ((b <= a) && (b <= c)) {
			middle = (a <= c) ? a : c;
		} else {
			middle = (a <= b) ? a : b;
		}
	}
	return middle;
}

void TIM1_UP_TIM10_IRQHandler(void)
{
	if ((TIM1->SR & TIM_SR_UIF)){
		TIM1->CR1&=~TIM_CR1_CEN;
		TIM1->ARR=10;
		switch(echoState)
		{
			case 0:
				if(pulseState)
				{
					GPIOE->BSRR=GPIO_BSRR_BR9;
					TIM5->CR1|=TIM_CR1_CEN;
					pulseState=0;
					NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
					TIM1->CR1&=~TIM_CR1_CEN;
				}
				else
				{
					GPIOE->BSRR=GPIO_BSRR_BS9;
					TIM1->CNT=0;
					TIM1->CR1|=TIM_CR1_CEN;
					pulseState=1;
				}
				break;
			case 1:
				if(pulseState)
				{
					GPIOE->BSRR=GPIO_BSRR_BR11;
					TIM5->CR1|=TIM_CR1_CEN;
					pulseState=0;
					NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
					TIM1->CR1&=~TIM_CR1_CEN;
				}
				else
				{
					GPIOE->BSRR=GPIO_BSRR_BS11;
					TIM1->CNT=0;
					TIM1->CR1|=TIM_CR1_CEN;
					pulseState=1;
				}
				break;
			case 2:
				if(pulseState)
				{
					GPIOE->BSRR=GPIO_BSRR_BR13;
					TIM5->CR1|=TIM_CR1_CEN;
					pulseState=0;
					NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
					TIM1->CR1&=~TIM_CR1_CEN;
				}
				else
				{
					GPIOE->BSRR=GPIO_BSRR_BS13;
					TIM1->CNT=0;
					TIM1->CR1|=TIM_CR1_CEN;
					pulseState=1;
				}
				break;	
		}
		TIM1->SR &=~ TIM_SR_UIF;
	}
}   

void TIM6_DAC_IRQHandler(void){
	if (TIM6->SR & TIM_SR_UIF){
		TIM6->SR &=~ TIM_SR_UIF;
		pduTimer++;
	}
}