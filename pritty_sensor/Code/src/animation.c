#include "animation.h"

#define UZ_MIN_LENGTH 120

uint16_t blinkTimer=0;
uint16_t blinkTimer1=0;
uint16_t updateTimer = 0;
int war=0;
extern uint8_t mode;
extern uint32_t echo_mes[12];
extern uint32_t control_mes[12];
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
	if((TIM14->SR &TIM_SR_UIF)!=0)
	{
		TIM14->SR&=~TIM_SR_UIF;//Сброс флага прерывания
		blinkTimer++;
		blinkTimer1++;
		updateTimer++;
	}
}

void animationInit(void)
{
	RCC->APB1ENR|=RCC_APB1ENR_TIM14EN;//Тактирование таймера 14
	TIM14->PSC|=16800;//Предделитель на 10кГц
	TIM14->ARR=9;//Делитель на 1кГц
	TIM14->DIER|=TIM_DIER_UIE;//Разрешить прерывание по переполнению
	NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn);
	TIM14->CR1|=TIM_CR1_CEN;//Запустить таймер
}

void animationLoop(void)
{
	static uint8_t blinkState=0;
	static uint8_t blinkState1=0;
	static uint8_t redBlink=0;
	static uint16_t hueAnimation=0;
	static uint8_t hueDir=0;
	if(mode==0)
	{
		if(blinkTimer1>500)
		{
			if(blinkState1==0)
				{
					ledSetBlockRGB(0,88,255,100,0);
					blinkState1=1;
				}
				else
				{
					ledSetBlockRGB(0,88,0,0,0);
					blinkState1=0;
				}
				blinkTimer1=0;
				ledUpdate();
		}
	}
	else if(redBlink && blinkTimer>200 && (mode==1))
	{
		if(blinkState==0)
		{
			ledSetBlockRGB(0,88,255,0,0);
			blinkState=1;
		}
		else
		{
			ledSetBlockRGB(0,88,0,0,0);
			blinkState=0;
		}
		blinkTimer=0;
		ledUpdate();
	}
	else if((mode==1) ? updateTimer>100 : updateTimer>20)
	{
		if(mode==1)
		{
			if(war>50)
			{
				redBlink=0;
				//rangeIndication(0,88,war,400,40);
				rangeIndication(0,5,(uint16_t)control_mes[10],500,UZ_MIN_LENGTH);//10
				rangeIndication(5,7,(uint16_t)control_mes[11],500,UZ_MIN_LENGTH);//11
				rangeIndication(12,6,(uint16_t)control_mes[0],500,UZ_MIN_LENGTH);//0
				rangeIndication(18,6,(uint16_t)control_mes[1],500,UZ_MIN_LENGTH);//1
				rangeIndication(24,6,(uint16_t)control_mes[2],500,UZ_MIN_LENGTH);//2
				rangeIndication(30,7,(uint16_t)control_mes[3],500,UZ_MIN_LENGTH);//3
				rangeIndication(37,7,(uint16_t)control_mes[4],500,UZ_MIN_LENGTH);//5
				rangeIndication(44,7,(uint16_t)control_mes[5],500,UZ_MIN_LENGTH);//6
				rangeIndication(51,7,(uint16_t)control_mes[6],500,UZ_MIN_LENGTH);//6
				rangeIndication(58,6,(uint16_t)control_mes[7],500,UZ_MIN_LENGTH);//7
				rangeIndication(64,7,(uint16_t)control_mes[8],500,UZ_MIN_LENGTH);//8
				rangeIndication(71,8,(uint16_t)control_mes[9],500,UZ_MIN_LENGTH);//9
			}
			else
			{
				redBlink=1;
				return;
			}	
		}
		else if(mode==2 || mode==3)
		{
			ledSetBlockHSV(0,79,hueAnimation,255,255);
			if(hueAnimation==359)
			{
					hueDir=1;
			}
			else if(hueAnimation==0)
			{
				hueDir=0;
			}
			if(hueDir)
				hueAnimation--;
			else
				hueAnimation++;
		}
		ledUpdate();
		updateTimer=0;
	}
}

static void rangeIndication(uint16_t ledStart,uint16_t ledNum, uint16_t range,uint16_t rangeMax,uint16_t rangeMin)
{
	uint16_t hue=(((range-rangeMin)*(80)/(rangeMax-rangeMin)));
	if(range>rangeMax) hue=80;
	if(range<rangeMin) hue=0;
	ledSetBlockHSV(ledStart,ledNum,hue,255,255);
}