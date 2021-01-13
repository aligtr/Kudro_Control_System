#include "bsp.h"

uint8_t frameBuff[3];
uint8_t temp;
uint8_t reciveStatus=0;
uint8_t reciveState=0;

/////////////////////////////Прерывания UART для приёма данных//////////////////////////////////////////////

void USART1_IRQHandler(void)
{
	if(USART1->SR & USART_SR_RXNE)
	{
		frameBuff[reciveStatus]=USART1->DR;
		reciveStatus++;
	}
}

void USART2_IRQHandler(void)
{
	if(USART2->SR & USART_SR_RXNE)
	{
		temp=USART2->DR;
		if(temp==0x0B)
		{
			reciveStatus=1;
			reciveState=0;
		}
		if(reciveStatus && temp!=0x0B)
		{
			frameBuff[reciveState]=temp;
			reciveState++;
			if(reciveState==3) reciveStatus=0;
		}
	}
}

void USART3_IRQHandler(void)
{
	if(USART3->SR & USART_SR_RXNE)
	{
		frameBuff[reciveStatus]=USART3->DR;
		reciveStatus++;
	}
}

void UART4_IRQHandler(void)
{
	if(UART4->SR & USART_SR_RXNE)
	{
		frameBuff[reciveStatus]=UART4->DR;
		reciveStatus++;
	}
}

void UART5_IRQHandler(void)
{
	if(UART5->SR & USART_SR_RXNE)
	{
	
	}
}

void USART6_IRQHandler(void)
{
	if(USART6->SR & USART_SR_RXNE)
	{
		frameBuff[reciveStatus]=USART6->DR;
		reciveStatus++;
	}
}


__STATIC_INLINE void Tim9Init(uint32_t coreFreq)//Таймер 9 используется для ожидания при чтении по UART
{
	RCC->APB2ENR|=RCC_APB2ENR_TIM9EN;//Включить тактирование таймера-счетчика 9
	TIM9->PSC=2*coreFreq/10000;//Делитель на 10КГц
	TIM9->CR1|=TIM_CR1_CEN;//Включить таймер
	TIM9->ARR=0xFFFF;//Считаем до конца
}

/////////////////////////////Настройка UART//////////////////////////////////////////////
void uart6Init(uint32_t coreFreq, uint32_t baudRate)
{
	//PC6-TX
	//PC7-RX
	Tim9Init(coreFreq);
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOCEN;//Включить тактирвание порта B
	RCC->APB2ENR|=RCC_APB2ENR_USART6EN;//Включить тактирование UART3
	GPIOC->MODER|=GPIO_MODER_MODE7_1 | GPIO_MODER_MODE6_1;//PC7,PC6 в режим альтернативной функции
	GPIOC->AFR[0]|=8<<24 | 8<<28;//Включить AF8
	GPIOC->PUPDR|=GPIO_PUPDR_PUPD7_0;//PC7 Pull up
	USART6->CR1 = 0;//Сбрасываем конфигурацию
	USART6->CR1|=USART_CR1_RE|USART_CR1_TE;//Включить приём и передачу
	USART6->BRR|=coreFreq/(baudRate);//Скорость работы uart
	NVIC_EnableIRQ(USART6_IRQn);
	USART6->CR1|=USART_CR1_UE;//Включить uart
}

void uart5Init(uint32_t coreFreq, uint32_t baudRate)
{
	//PC12-TX
	//PD2	-RX
	Tim9Init(coreFreq);
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIODEN;//Включить тактирвание порта D
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIODEN;//Включить тактирвание порта C
	RCC->APB1ENR|=RCC_APB1ENR_UART5EN;//Включить тактирование UART3
	GPIOD->MODER|=GPIO_MODER_MODE2_1;//PD2 в режим альтернативной функции
	GPIOC->MODER|=GPIO_MODER_MODE12_1;//PC12 режим альтернативной функции
	GPIOC->AFR[1]|=8<<16;//Включить AF8
	GPIOD->AFR[0]|=8<<8;//Включить AF8
	GPIOD->PUPDR|=GPIO_PUPDR_PUPD2_0;//PD2 Pull up
	UART5->CR1 = 0;//Сбрасываем конфигурацию
	UART5->CR1|=USART_CR1_RE|USART_CR1_TE;//Включить приём и передачу
	UART5->BRR|=coreFreq/(baudRate);//Скорость работы uart
	NVIC_EnableIRQ(UART5_IRQn);
	UART5->CR1|=USART_CR1_UE;//Включить uart
}

void uart4Init(uint32_t coreFreq, uint32_t baudRate)
{
	//PA0-TX
	//PA1-RX
	Tim9Init(coreFreq);
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN;//Включить тактирвание порта A
	RCC->APB1ENR|=RCC_APB1ENR_UART4EN;//Включить тактирование UART4
	GPIOA->MODER|=GPIO_MODER_MODE0_1 | GPIO_MODER_MODE1_1;//PA0,PA1 в режим альтернативной функции
	GPIOA->AFR[0]|=8<<0 | 8<<4;//Включить AF8
	GPIOA->PUPDR|=GPIO_PUPDR_PUPD1_0;//PA1 Pull up
	UART4->CR1 = 0;//Сбрасываем конфигурацию
	UART4->CR1|=USART_CR1_RE|USART_CR1_TE;//Включить приём и передачу
	UART4->BRR|=coreFreq/(baudRate);//Скорость работы uart
	NVIC_EnableIRQ(UART4_IRQn);
	UART4->CR1|=USART_CR1_UE;//Включить uart
}

void uart3Init(uint32_t coreFreq, uint32_t baudRate)
{
	//PB10-TX
	//PB11-RX
	Tim9Init(coreFreq);
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
	Tim9Init(coreFreq);
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIODEN;//Включить тактирвание порта А
	RCC->APB1ENR|=RCC_APB1ENR_USART2EN;//Включить тактирование UART2
	GPIOD->MODER|=GPIO_MODER_MODE5_1 | GPIO_MODER_MODE6_1;//PD5,PD6 в режим альтернативной функции
	GPIOD->AFR[0]|=(7<<20)|(7<<24);//Включить AF7
	GPIOD->PUPDR|=GPIO_PUPDR_PUPD6_0;//PD6 Pull up
	USART2->CR1 = 0;//Сбрасываем конфигурацию
	USART2->CR1|=USART_CR1_RE|USART_CR1_TE;//Включить приём и передачу
	USART2->BRR|=coreFreq/(baudRate);//Скорость работы uart
	NVIC_EnableIRQ(USART2_IRQn);
	USART2->CR1|=USART_CR1_RXNEIE;
	USART2->CR1|=USART_CR1_UE;//Включить uart
}

void uart1Init(uint32_t coreFreq, uint32_t baudRate)
{
	//PA9-TX
	//PA10-RX
	Tim9Init(coreFreq);
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN;//Включить тактирвание порта А
	RCC->APB2ENR|=RCC_APB2ENR_USART1EN;//Включить тактирование  
	GPIOA->MODER|=GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1;//PA9,PA10 в режим альтернативной функции
	GPIOA->AFR[1]|=GPIO_AFRH_AFRH1_0 | GPIO_AFRH_AFRH1_1 | GPIO_AFRH_AFRH1_2 | GPIO_AFRH_AFRH2_0 | GPIO_AFRH_AFRH2_1 | GPIO_AFRH_AFRH2_2;//Включить AF7
	GPIOA->PUPDR|=GPIO_PUPDR_PUPD10_0;//PA10 Pull up
	USART1->CR1 = 0;//Сбрасываем конфигурацию
	USART1->CR1|=USART_CR1_RE|USART_CR1_TE;//Включить приём и передачу
	USART1->BRR|=coreFreq/(baudRate);//Скорость работы uart
	NVIC_EnableIRQ(USART1_IRQn);
	USART1->CR1|=USART_CR1_UE;//Включить uart
}
/////////////////////////////Функции приёма-передачи//////////////////////////////////////////////

void uartTransmitt(uint8_t data,USART_TypeDef * UART)
{
	while (!(UART->SR & USART_SR_TXE)){}//Ждем, пока UART освлбодиться
	UART->DR = data;//Отправляем данные
}

void uartTransmittBuff(uint8_t* data,uint32_t size,USART_TypeDef * UART)
{
	while(size)
	{
		uartTransmitt(*data++,UART);
		size--;
	}
}

uint8_t uartRecive(USART_TypeDef * UART)
{
	reciveStatus=0;
	UART->CR1|=USART_CR1_RXNEIE;//Разрешить прерывание по приёму
	TIM9->CNT=0;
	while(reciveStatus!=frameBuff[1]+3)
	{
		if(TIM9->CNT>=UART_MAX_TIMEOUT_MS*10)
		{
			UART->CR1&=~USART_CR1_RXNEIE;//Запретить прерывание по приёму
			return 0;//Если закончился тайм аут завершаем ожидание
		}
	}
	UART->CR1&=~USART_CR1_RXNEIE;//Запретить прерывание по приёму
	return 1;
}