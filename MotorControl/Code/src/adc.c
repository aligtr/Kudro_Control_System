#include "adc.h"

#define ADC_COUNT 200

uint16_t wheel_pos[4];
uint32_t wheel_pos_temp[4];
char adc_flag;
char wheel_flag[4];
uint8_t potState=0;

void ADC_IRQHandler(void)
{
	if((ADC1->SR & ADC_SR_JEOC_Msk) !=0)
	{
		ADC1->SR&=~ADC_SR_JEOC;//Сбросить флаг
		wheel_pos_temp[0]+=(ADC1->JDR1)>>0;
		wheel_pos_temp[1]+=(ADC1->JDR2)>>0;
		wheel_pos_temp[2]+=(ADC1->JDR3)>>0;
		wheel_pos_temp[3]+=(ADC1->JDR4)>>0;
		potState++;
		if(potState==ADC_COUNT)
		{
			wheel_pos[0]=wheel_pos_temp[0]/ADC_COUNT;
			wheel_pos[1]=wheel_pos_temp[1]/ADC_COUNT;
			wheel_pos[2]=wheel_pos_temp[2]/ADC_COUNT;
			wheel_pos[3]=wheel_pos_temp[3]/ADC_COUNT;
			wheel_pos_temp[0]=0;
			wheel_pos_temp[1]=0;
			wheel_pos_temp[2]=0;
			wheel_pos_temp[3]=0;
			adc_flag=1;
			potState=0;
		}
		else
		{
			ADC1->CR2|=ADC_CR2_JSWSTART;
		}	
	}
}

void adcInit(void)
{
	//PC5,PC4,PB1,PB0
	RCC->APB2ENR|=RCC_APB2ENR_ADC1EN;//Включить тактирование ADC1
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOCEN;//Включить тактирование GPIOC
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOBEN;//Включить тактирование GPIOB
	GPIOC->MODER|=GPIO_MODER_MODE4 | GPIO_MODER_MODE5;
	GPIOB->MODER|=GPIO_MODER_MODE0 | GPIO_MODER_MODE1;
	ADC->CCR|=ADC_CCR_ADCPRE_0;//Делитель АЦП на 4
	ADC1->CR2|=ADC_CR2_ADON;//Включить АЦП
	ADC1->CR1|= ADC_CR1_SCAN//Режим сканирования
	| ADC_CR1_JEOCIE;//Включить прерывание по окончанию преобразования инжективного канала
	ADC1->JSQR|= ADC_JSQR_JL | (8<<ADC_JSQR_JSQ4_Pos) | (9<<ADC_JSQR_JSQ3_Pos) | (14<<ADC_JSQR_JSQ2_Pos) | (15<<ADC_JSQR_JSQ1_Pos);//Выбор канала
	NVIC_EnableIRQ(ADC_IRQn);
}

