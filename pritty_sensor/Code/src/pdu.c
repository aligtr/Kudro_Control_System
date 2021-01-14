#include "pdu.h"
//uint16_t frameBuff[2];
//uint8_t reciveStatus;
double gam;
double echo_angle[12];
double echo_mes[12];
double start_mes;
char pack;
int pduTimer;
int echo_count;
void tim1Init(void)
{
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOEEN;
	GPIOE->MODER|=GPIO_MODER_MODE9_1 | GPIO_MODER_MODE11_1 | GPIO_MODER_MODE13_1;//PE9,PE11,PE13 в режим альтернативной функции
	GPIOE->AFR[1]|=(1<<4)|(1<<12)|(1<<20);//AF1 enable
		RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	TIM1->CCER = 0;
	TIM1->ARR = 3000;
	TIM1->PSC = 1680-1;
	TIM1->BDTR |= TIM_BDTR_MOE;
	TIM1->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	TIM1->CCER |= TIM_CCER_CC1E;
	TIM1->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
	TIM1->CCER |= TIM_CCER_CC2E;
	TIM1->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
	TIM1->CCER |= TIM_CCER_CC3E;
	TIM1->CR1 |= TIM_CR1_CEN;
	TIM1->CCR1=30;
	TIM1->CCR2=0;
	TIM1->CCR3=0;
	TIM1->DIER|=TIM_DIER_CC1IE|TIM_DIER_CC2IE|TIM_DIER_CC3IE;
	NVIC_EnableIRQ(TIM1_CC_IRQn);
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
	TIM2->CCR1=50;
	TIM2->CCR2=50;
	TIM2->CCR3=50;
	TIM2->CCR4=50;
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
	TIM4->CCR1=50;
	TIM4->CCR2=50;
	TIM4->CCR3=50;
	TIM4->CCR4=50;
}

void tim5Init(void)
{
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN;//???????????? ????? A
	GPIOA->MODER|=GPIO_MODER_MODE0_1|GPIO_MODER_MODE1_1|GPIO_MODER_MODE2_1|GPIO_MODER_MODE3_1;//PA0 ?? AF
	GPIOA->AFR[0]|=(2<<0)|(2<<4)|(2<<8)|(2<<12);
	RCC->APB1ENR|=RCC_APB1ENR_TIM5EN;//???????? ???????????? ???????-???????? 9
	TIM5->PSC=1680-1;//???????? ?? 10???
	TIM5->ARR=3000;//??????? ?? ?????
	//????? 1
	TIM5->CCMR1|=1|(1<<8);
	//????? 3
	TIM5->CCMR2|=1|(1<<8);
		
	TIM5->CCER|=TIM_CCER_CC1E;//????? ???????
	TIM5->CCER|=TIM_CCER_CC2E;//????? ???????
	TIM5->CCER|=TIM_CCER_CC3E;//????? ???????
	TIM5->CCER|=TIM_CCER_CC4E;//????? ???????
	TIM5->CCER|=TIM_CCER_CC1P;
	TIM5->CCER|=TIM_CCER_CC2P;
	TIM5->CCER|=TIM_CCER_CC3P;
	TIM5->CCER|=TIM_CCER_CC4P;
	TIM5->CR1|=TIM_CR1_CEN|TIM_CR1_ARPE|TIM_CR1_URS;
	TIM5->DIER|=TIM_DIER_CC1IE;
	TIM5->DIER|=TIM_DIER_CC2IE;
	TIM5->DIER|=TIM_DIER_CC3IE;
	TIM5->DIER|=TIM_DIER_CC4IE;
	
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
	TIM8->CCR1=50;
	TIM8->CCR2=50;
	TIM8->CCR3=50;
	TIM8->CCR4=50;
}

void tim6Init(void)
{
	RCC->APB1ENR|=RCC_APB1ENR_TIM6EN;
	TIM6->PSC=16800;
	TIM6->ARR=99;
	TIM6->DIER|=TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
	TIM6->CR1|=TIM_CR1_CEN;
}


/*void send_to_uart(uint16_t data)
{
	while(!(USART2->SR & USART_SR_TC));
	USART2->DR=data;
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
}*/

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

/*void uart3Init(uint32_t coreFreq, uint32_t baudRate)
{
	//PB10-TX
	//PB11-RX
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOBEN;//Включить тактирвание порта B
	RCC->APB1ENR|=RCC_APB1ENR_USART3EN;//Включить тактирование UART3
	GPIOB->MODER|=GPIO_MODER_MODE10_1 | GPIO_MODER_MODE11_1;//PB10,PB11 в режим альтернативной функции
	GPIOB->AFR[1]|=7<<8 | 7<<12;//Включить AF7
	GPIOB->PUPDR|=GPIO_PUPDR_PUPD11_0;//PB11 Pull up
	USART3->CR1 = 0;//Сбрасываем конфигурацию
	USART3->CR1|=USART_CR1_RE|USART_CR1_TE;//Включить приём и передачу
	USART3->BRR|=coreFreq/(baudRate);//Скорость работы uart
	NVIC_EnableIRQ(USART3_IRQn);
	USART3->CR1|=USART_CR1_UE;//Включить uart
}

void uart2Init(uint32_t coreFreq, uint32_t baudRate)
{
	//PD5-TX
	//PD6-RX
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIODEN;//Включить тактирвание порта D
	RCC->APB1ENR|=RCC_APB1ENR_USART2EN;//Включить тактирование UART2
	GPIOD->MODER|=GPIO_MODER_MODE5_1 | GPIO_MODER_MODE6_1;//PD5,6 в режим альтернативной функции
	GPIOD->AFR[0]|=(7<<20)|(7<<24);//Включить AF7
	GPIOD->PUPDR|=GPIO_PUPDR_PUPD6_0;//PD6 Pull up
	USART2->CR1 = 0;//Сбрасываем конфигурацию
	USART2->CR1|=USART_CR1_RE|USART_CR1_TE;//Включить приём и передачу
	USART2->BRR|=coreFreq/(baudRate);//Скорость работы uart
	NVIC_EnableIRQ(USART2_IRQn);
	USART2->CR1|=USART_CR1_UE;//Включить uart
}

void USART2_IRQHandler(void)
{
	if(USART2->SR & USART_SR_RXNE)
	{
		frameBuff[reciveStatus]=USART2->DR;
		if (reciveStatus==0)reciveStatus=1;
		else reciveStatus=1;
	}
}

void USART3_IRQHandler(void)
{
	if(USART3->SR & USART_SR_RXNE)
	{
		frameBuff[reciveStatus]=USART3->DR;
		reciveStatus++;
	}
}*/

void TIM5_IRQHandler(void){
	double a;
	if (TIM5->SR & TIM_SR_CC1IF){
		TIM5->SR &=~TIM_SR_CC1IF;
		if (TIM5->CCR1>start_mes) echo_mes[pack+0] = (TIM5->CCR1-start_mes)/58 *10-24;
		//else echo_mes[pack+0]=1000;
	}
	if (TIM5->SR & TIM_SR_CC2IF){
		TIM5->SR &=~TIM_SR_CC2IF;
		if (TIM5->CCR2>start_mes) echo_mes[pack+3] = (TIM5->CCR2-start_mes)/58 *10-24;
		//else echo_mes[pack+3]=1000;
	}
	if (TIM5->SR & TIM_SR_CC3IF){
		TIM5->SR &=~TIM_SR_CC3IF;
		if (TIM5->CCR3>start_mes) echo_mes[pack+6] = (TIM5->CCR3-start_mes)/58 *10-24;
		//else echo_mes[pack+6]=1000;
	}
	if (TIM5->SR & TIM_SR_CC4IF){
		TIM5->SR &=~TIM_SR_CC4IF;
		if (TIM5->CCR4>start_mes) echo_mes[pack+9] = (TIM5->CCR4-start_mes)/58 *10-24;
		//else echo_mes[pack+9]=1000;
	}
}

void TIM1_CC_IRQHandler(void){
	if ((TIM1->SR & TIM_SR_UIF)){
		TIM1->SR &=~ TIM_SR_CC1IF;
		TIM1->SR &=~ TIM_SR_CC2IF;
		TIM1->SR &=~ TIM_SR_CC3IF;
		TIM1->SR &=~ TIM_SR_UIF;
		start_mes=TIM5->CNT;
		//UPD();
		//tim5Init();
		if (echo_count>10){
			echo_count=0;
			switch (pack){
				case 0:		
					TIM1->CCR1=0;
					TIM1->CCR2=30;
					TIM1->CCR3=0;
					pack=1;
				break;
				case 1:		
					TIM1->CCR1=0;
					TIM1->CCR2=0;
					TIM1->CCR3=30;
					pack=2;
				break;
				case 2:		
					TIM1->CCR1=30;
					TIM1->CCR2=0;
					TIM1->CCR3=0;
					pack=0;
				break;
			}
		}
		else {
			/*if (echo_count>25){
			}*/
			echo_count++;
		}
	}
}

void TIM6_DAC_IRQHandler(void){
	if (TIM6->SR & TIM_SR_UIF){
		TIM6->SR &=~ TIM_SR_UIF;
		pduTimer++;
	}
}