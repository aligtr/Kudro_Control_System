#include "main.h"
#include "pdu.h"
 #include "math.h"

#define SERVO_MIN 50
#define SERVO_MAX 300

extern int pduTimer;
extern char pack;
extern uint32_t echo_mes[12];
extern double gam;
extern uint8_t frameBuff[3];
extern uint8_t reciveStatus;
extern double echo_angle[12];
uint16_t rej;
uint8_t mode=0;
extern int war;
int main(void)
{
	uint16_t temp;
	uint16_t gamTemp;
	uint8_t timerEnableFlag=1;
	RccClockInit();
	tim4Init();
	tim2Init();
	tim5Init();
	tim8Init();
	tim1Init();
	frameBuff[0]=180;
	frameBuff[1]=0;
	tim6Init();
	ledInit();
	animationInit();
	//uart3Init(84000000,9600);
	uart2Init(84000000,115200);
  while (1)
  {
		if(pduTimer>200)
		{
			mode = frameBuff[2];
			mode =1;
			if(mode!=1 && timerEnableFlag==1)
			{
				TIM1->CR1&=~TIM_CR1_CEN;
				TIM5->CR1&=~TIM_CR1_CEN;
				NVIC_DisableIRQ(TIM1_CC_IRQn);
				NVIC_DisableIRQ(TIM5_IRQn);
				timerEnableFlag=0;
			}
			else if(timerEnableFlag==0 && mode==1)
			{
				TIM1->CR1|=TIM_CR1_CEN;
				TIM5->CR1|=TIM_CR1_CEN;
				NVIC_EnableIRQ(TIM1_CC_IRQn);
				NVIC_EnableIRQ(TIM5_IRQn);
				timerEnableFlag=1;
			}
			
			pduTimer=0;
				gam=-1*((uint16_t)(frameBuff[0]+(frameBuff[1]<<8))-180);
				//9,10,11
			//gam=sign(gam)*(180-fabs(gam));
			//if (gam==0) gam=180;
				if((gam<=0 && gam>=-180) || gam==180)
				{
					echo_angle[9]=map(fabs(gam),0,180,30,90);
					echo_angle[10]=echo_angle[9];
					echo_angle[11]=echo_angle[9];
				}
				else
				{
					echo_angle[9]=60;
					echo_angle[10]=60;
					echo_angle[11]=60;
					echo_mes[9]=1000;
					echo_mes[10]=1000;
					echo_mes[11]=1000;
					
				}
				//3,4,5
				if((gam>=0 && gam<=180) ||gam==-180)
				{
					echo_angle[3]=map(fabs(gam),0,180,120,60);
					echo_angle[4]=echo_angle[3];
					echo_angle[5]=echo_angle[3];
				}
				else
				{
					echo_angle[3]=60;
					echo_angle[4]=60;
					echo_angle[5]=60;
					echo_mes[3]=1000;
					echo_mes[4]=1000;
					echo_mes[5]=1000;
				}
				//1
				if(gam>=-90 && gam<=90)
				{
					echo_angle[1]=map(gam,-90,90,130,20);
				}
				else
				{
					echo_angle[1]=75;
					echo_mes[1]=1000;
				}
				//0
				if(gam>=-180 && gam<=90)
				{
					echo_angle[0]=map(gam,-180,90,180,15);
				}
				else
				{
					echo_angle[0]=60+45;
					echo_mes[0]=1000;
				}
				//2
				if(gam>=-90 && gam<=180)
				{
					echo_angle[2]=map(gam,-90,180,180,15);
				}
				else
				{
					echo_angle[2]=30+45;
					echo_mes[2]=1000;
				}
				
				gam=sign(gam)*(180-fabs(gam));
				//8
				if((gam>=-180 && gam<=90))
				{
					echo_angle[8]=map(gam,90,-180,180,15);
				}
				else
				{
					echo_angle[8]=60+45;
					echo_mes[8]=1000;
				}
				//7
				if(gam<=90 && gam>=-90)
				{
					echo_angle[7]=map(gam,90,-90,130,20);
				}
				else
				{
					echo_angle[7]=75;
					echo_mes[7]=1000;
				}
				//6
				if((gam>=-90 && gam<=180))
				{
					echo_angle[6]=map(gam,180,-90,180,15);

				}
				else
				{
					echo_angle[6]=30+45;
					echo_mes[6]=1000;
				}
				for(war=0;war<450;war+=20){
					if (echo_mes[0]<war||echo_mes[1]<war||echo_mes[2]<war||echo_mes[3]<war||echo_mes[4]<war||echo_mes[5]<war||
							echo_mes[6]<war||echo_mes[7]<war||echo_mes[8]<war||echo_mes[9]<war||echo_mes[10]<war||echo_mes[11]<war){
								if(war>40)
									temp=(war<400) ? (uint16_t)(war/80) : (uint16_t)5;
								else 
									temp=0;
								uartTransmitt(0x0B,USART2);
								uartTransmittBuff((uint8_t*)&temp,2,USART2);
						break;
					}
				}
				TIM2->CCR1=deg_to_pwm(echo_angle[0]);//1
				TIM2->CCR2=deg_to_pwm(echo_angle[1]);//2
				TIM2->CCR3=deg_to_pwm(echo_angle[2]);//3
				TIM2->CCR4=deg_to_pwm(echo_angle[3]);//4

				TIM4->CCR1=deg_to_pwm(echo_angle[4]);//5
				TIM4->CCR2=deg_to_pwm(echo_angle[5]);//6
				TIM4->CCR3=deg_to_pwm(echo_angle[6]);//7
				TIM4->CCR4=deg_to_pwm(echo_angle[7]);//8

				TIM8->CCR1=deg_to_pwm(echo_angle[8]);//9
				TIM8->CCR2=deg_to_pwm(echo_angle[9]);//10
				TIM8->CCR3=deg_to_pwm(echo_angle[10]);//11
				TIM8->CCR4=deg_to_pwm(echo_angle[11]);//12	
				
		}
		animationLoop();		
  }
}



