#include "main.h"
#include "pdu.h"

__INLINE void allMotorInit(void);
void allMotorStart(void);
void allMotorChangeSpeed(void);
void allMotorGetStatus(void);
void motorStopAll(void);
void motorChangeZeroSpeed(void);
void allMotorFaultAck(void);
void reset(void);
void interruptInit(void);
void servoInit(void);
motor_t motorFrontLeft=
{
	USART1,
	0,
	0,
	0
};
motor_t motorFrontRight=
{
	USART2,
	0,
	0,
	0
};
motor_t motorRearLeft=
{
	USART3,
	0,
	0,
	0
};
motor_t motorRearRight=
{
	UART4,
	0,
	0,
	0
};

uint8_t mode=0;//3 режима
uint8_t motorState=0;//1-движки активны, 0-выключены
uint8_t motorStopFlag=0;
uint8_t motorResetFlag=0;
uint16_t checkStatusTimer=0;
uint16_t pduTimer=0;
extern double Vfl, Vfr, Vrl, Vrr;
double speed;
extern char ch_pack;

extern int vel_mean;

extern int rx_mean;

extern int dir_mean;
extern int ry_mean;

extern int vel_src;
extern int smt_src1;
extern int smt_src2;
extern int smt_src3;

extern uint16_t wheel_pos[4];
extern char adc_flag;
extern char wheel_flag[4];
extern void TIM6_DAC_IRQHandler(void)
{
	if(TIM6->SR & TIM_SR_UIF)
	{
		TIM6->SR&=~TIM_SR_UIF;
		pduTimer++;
		checkStatusTimer++;
	}
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

void tim7Init(void)
{
	RCC->APB1ENR|=RCC_APB1ENR_TIM7EN;
	TIM7->PSC=16800;
	TIM7->ARR=0xFFFF;
	TIM7->CR1|=TIM_CR1_CEN;
}

void reset(void)
{
	RCC->AHB1RSTR=0xFFFFFFFF;
	RCC->AHB1ENR=0x0;
	RCC->APB1RSTR = 0xFFFFFFFF;
	RCC->APB1RSTR = 0x0; 
	RCC->APB2RSTR = 0xFFFFFFFF;
	RCC->APB2RSTR = 0x0; 
	RCC->APB1ENR = 0x0;
	RCC->APB2ENR = 0x0;
	SCB->AIRCR = 0x05FA0004;
}

void EXTI9_5_IRQHandler(void) {
	if (EXTI->PR & EXTI_PR_PR9) {
		EXTI->PR |= EXTI_PR_PR9;
		if (GPIOB->IDR & GPIO_IDR_IDR_9)
			motorStopFlag=1;
	}
}
void interruptInit(){

	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOBEN;
	GPIOB->PUPDR|=(1<<18);
	SYSCFG->EXTICR[2]|=(1<<4);
	EXTI->IMR|=(1<<9);
	EXTI->RTSR|=(1<<9);
	EXTI->FTSR|=(1<<9);
	NVIC_EnableIRQ(EXTI9_5_IRQn);
}
void servoInit(){	
	NVIC_EnableIRQ(TIM1_CC_IRQn);
	NVIC_EnableIRQ(TIM8_CC_IRQn);
	NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
	NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
	while (!wheel_flag[0] | !wheel_flag[1] | !wheel_flag[2] | !wheel_flag[3])
	{
			ADC1->CR2|=ADC_CR2_JSWSTART;
			while(!adc_flag);
			adc_flag=0;		
			if (wheel_pos[0]>3029 && !wheel_flag[0]) {//ch1
				GPIOC->BSRR=GPIO_BSRR_BS0;
				TIM1->CCR1 = 50;
			}
			else if (wheel_pos[0]<3027 && !wheel_flag[0]) {
				TIM1->CCR1 = 50;
				GPIOC->BSRR=GPIO_BSRR_BR0;
			}
			else {
				TIM1->CCR1 = 0xffff;
				wheel_flag[0]=1;
			}
			if (wheel_pos[1]>2049 && !wheel_flag[1]) {//ch2
				GPIOC->BSRR=GPIO_BSRR_BR1;
				TIM8->CCR1 = 50;
			}
			else if (wheel_pos[1]<2047 && !wheel_flag[1]) {
				TIM8->CCR1 = 50;
				GPIOC->BSRR=GPIO_BSRR_BS1;
			}
			else {
				TIM8->CCR1 = 0xffff;
				wheel_flag[1]=1;
			}
			if (wheel_pos[2]>2049 && !wheel_flag[2]) {//ch3
				GPIOC->BSRR=GPIO_BSRR_BR2;
				TIM9->CCR1 = 50;
			}
			else if (wheel_pos[2]<2047 && !wheel_flag[2]) {
				TIM9->CCR1 = 50;
				GPIOC->BSRR=GPIO_BSRR_BS2;
			}
			else {
				TIM9->CCR1 = 0xffff;
				wheel_flag[2]=1;
			}
			if (wheel_pos[3]>2975 && !wheel_flag[3]) {//ch4
				GPIOC->BSRR=GPIO_BSRR_BS3;
				TIM12->CCR1 = 50;
			}
			else if (wheel_pos[3]<2973 && !wheel_flag[3]) {
				TIM12->CCR1 = 50;
				GPIOC->BSRR=GPIO_BSRR_BR3;
			}
			else {
				TIM12->CCR1 = 0xffff;
				wheel_flag[3]=1;
			}
			smt_src2=TIM4->CCR1;
			if(smt_src2>700)
			{
				//reset();
			}
	}
	TIM1->PSC = 1680/4-1;
	TIM8->PSC = 1680/4-1;
	TIM9->PSC = 1680/4-1;
	TIM12->PSC = 1680/4-1;
}

int main()
{
	uint16_t gamma=0;
	RccClockInit();
	delayInit(); 
	tim2Init1();
	tim3Init1();
	tim4Init2();
	tim5Init1();
	tim6Init();
	tim1Init();
	tim8Init();
	tim9Init();
	tim12Init();
 	tim7Init();
	
	//uart6Init(84000000,9600);
	uart5Init(84000000,115200);
	uart4Init(84000000,9600);
	uart3Init(84000000,9600);
	uart2Init(84000000,9600);
	uart1Init(84000000,9600);
	
	adcInit();
	uartTransmitt(0x0B,UART5);
	gamma=180;
	uartTransmittBuff((uint8_t*)&gamma,sizeof(uint16_t),UART5);
	uartTransmitt(0,UART5);

	interruptInit();
	motorStopAll();
	motorState=0;
	allMotorInit();
	tim4Init1();
	if (GPIOB->IDR & GPIO_IDR_IDR_9)
			motorStopFlag=1;
	while(1)
	{
			/*ADC1->CR2|=ADC_CR2_JSWSTART;
			while(!adc_flag);
			adc_flag=0;*/
		if(pduTimer>100)
		{
			
			pduTimer=0;
			if (ch_pack==0){
				ch_pack=1;
				vel_mean=TIM2->CCR2;
				dir_mean=TIM5->CCR2;
				rx_mean=TIM4->CCR2;
				tim2Init2();
				tim3Init2();
				tim4Init2();
				tim5Init2();
			}
			else{
				ch_pack=0;
				smt_src1=TIM3->CCR1;
				smt_src2=TIM4->CCR1;
				smt_src3=TIM5->CCR1;
				ry_mean=TIM2->CCR1;
				if(motorStopFlag)
				{
					motorStopAll();
					motorResetFlag=1;
					uartTransmitt(0x0B,UART5);
					gamma=180;
					uartTransmittBuff((uint8_t*)&gamma,sizeof(uint16_t),UART5);
					uartTransmitt(0,UART5);
				}
				if(smt_src2>700 || smt_src2<200)
				{
					reset();
				}
				if(smt_src3>700 && motorState==0 && !motorResetFlag)
				{
					
					servoInit();
					allMotorStart();
				}
				else if(motorState==1 && smt_src3<520/* && motorFrontLeft.refSpeed==0 && motorFrontRight.refSpeed==0 && motorRearLeft.refSpeed==0 && motorRearRight.refSpeed==0*/)
				{
					motorStopAll();
					uartTransmitt(0x0B,UART5);
					gamma=180;
					uartTransmittBuff((uint8_t*)&gamma,sizeof(uint16_t),UART5);
					uartTransmitt(0,UART5);
				}
				if(smt_src1>520 && smt_src1<700)
				{
					mode = 2;
				}
				else if(smt_src1>700)
				{
					mode = 3;
				}
				else 
				{
					mode=1;
				}
				tim2Init1();
				tim3Init1();
				tim4Init1();
				tim5Init1();
			}
			if(smt_src3>700 && motorState==1)
			{
				kinematica();
				allMotorChangeSpeed();
			}
			
		}
		if(checkStatusTimer>CHECK_STATUS_PERIOD)
		{
			allMotorGetStatus();
			checkStatusTimer=0;
			
		}
	}
}

void allMotorGetStatus(void)
{
	uint8_t motorState1=0;
	uint8_t motorState2=0;
	uint8_t motorState3=0;
	uint8_t motorState4=0;
	mcpErrorCode error1,error2,error3,error4;
	error1=getMotorState(motorFrontLeft.UART,&motorState1);
	error2=getMotorState(motorFrontRight.UART,&motorState2);
	error3=getMotorState(motorRearLeft.UART,&motorState3);
	error4=getMotorState(motorRearRight.UART,&motorState4);
	if(error1!=ERROR_NONE || error2!=ERROR_NONE || error3!=ERROR_NONE || error4!=ERROR_NONE || (motorState1!=0 && motorState1!=6 && motorState1!=8)\
		|| (motorState2!=0 && motorState2!=6 && motorState2!=8) || (motorState3!=0 && motorState3!=6 && motorState3!=8) || (motorState4!=0 && motorState4!=6 && motorState4!=8))
	{
		motorChangeZeroSpeed();
		allMotorFaultAck();
		if(motorState==1)
			allMotorStart();
	}
}

void allMotorChangeSpeed(void)
{
	motorFrontLeft.refSpeed=(int32_t)(Vfl*60/(6.28*MOTOR_RADIUS));
	motorFrontRight.refSpeed=(int32_t)(Vfr*60/(6.28*MOTOR_RADIUS));
	motorRearLeft.refSpeed=(int32_t)(Vrl*60/(6.28*MOTOR_RADIUS));
	motorRearRight.refSpeed=(int32_t)(Vrr*60/(6.28*MOTOR_RADIUS));
	if(motorFrontLeft.refSpeed!=motorFrontLeft.prevSpeed)
	{
		TIM7->CNT=0;
		while(setSpeed(motorFrontLeft.refSpeed,MOTOR_RAMP_TIME_MS,motorFrontLeft.UART)!=ERROR_NONE)
		{
			if(TIM7->CNT>=INIT_MAX_TIMEOUT_MS*10) 
			{
			 	//motorStopAll();
				//allMotorFaultAck();
				//allMotorStart();
				failDriveChangeSpeedCallback(motorFrontLeft);
				break;
			}
		}
		motorFrontLeft.prevSpeed=motorFrontLeft.refSpeed;
	}
	if(motorFrontRight.refSpeed!=motorFrontRight.prevSpeed)
	{
		TIM7->CNT=0;
		while(setSpeed(motorFrontRight.refSpeed,MOTOR_RAMP_TIME_MS,motorFrontRight.UART)!=ERROR_NONE)
		{
			if(TIM7->CNT>=INIT_MAX_TIMEOUT_MS*10) 
			{
				//motorStopAll();
				//allMotorFaultAck();
				//allMotorStart();
				failDriveChangeSpeedCallback(motorFrontRight);
				break;
			}
		}
		motorFrontRight.prevSpeed=motorFrontRight.refSpeed;
	}
	if(motorRearLeft.refSpeed!=motorRearLeft.prevSpeed)
	{
		TIM7->CNT=0;
		while(setSpeed(motorRearLeft.refSpeed,MOTOR_RAMP_TIME_MS,motorRearLeft.UART)!=ERROR_NONE)
		{
			if(TIM7->CNT>=INIT_MAX_TIMEOUT_MS*10) 
			{
				//motorStopAll();
				//allMotorFaultAck();
				//allMotorStart();
				failDriveChangeSpeedCallback(motorRearLeft);
				break;
			}
		}
		motorRearLeft.prevSpeed=motorRearLeft.refSpeed;
	}
	if(motorRearRight.refSpeed!=motorRearRight.prevSpeed)
	{
		TIM7->CNT=0;
		while(setSpeed(motorRearRight.refSpeed,MOTOR_RAMP_TIME_MS,motorRearRight.UART)!=ERROR_NONE)
		{
			if(TIM7->CNT>=INIT_MAX_TIMEOUT_MS*10) 
			{
				//motorStopAll();
				//allMotorFaultAck();
				//allMotorStart();
				failDriveChangeSpeedCallback(motorRearRight);
				break;
			}
		}
		motorRearRight.prevSpeed=motorRearRight.refSpeed;
	}
}

__INLINE void allMotorInit(void)
{
	//Настройка регулятора скорости
	TIM7->CNT=0;
	while(setSpeedPID(SPEED_P,SPEED_I,0,motorFrontLeft.UART)!=ERROR_NONE)
	{
		if(TIM7->CNT>=INIT_MAX_TIMEOUT_MS*10) 
		{
			failDriveInitCallback(motorFrontLeft);
			break;
		}
	}
	TIM7->CNT=0;
	while(setSpeedPID(SPEED_P,SPEED_I,0,motorFrontRight.UART)!=ERROR_NONE)
	{
		if(TIM7->CNT>=INIT_MAX_TIMEOUT_MS*10) 
		{
			failDriveInitCallback(motorFrontRight);
			break;
		}
	}
	TIM7->CNT=0;
	while(setSpeedPID(SPEED_P,SPEED_I,0,motorRearLeft.UART)!=ERROR_NONE)
	{
		if(TIM7->CNT>=INIT_MAX_TIMEOUT_MS*10) 
		{
			failDriveInitCallback(motorRearLeft);
			break;
		}
	}
	TIM7->CNT=0;
	while(setSpeedPID(SPEED_P,SPEED_I,0,motorRearRight.UART)!=ERROR_NONE)
	{
		if(TIM7->CNT>=INIT_MAX_TIMEOUT_MS*10) 
		{
			failDriveInitCallback(motorRearRight);
			break;
		}
	}
	//Настройка регулятора момента
	TIM7->CNT=0;
	while(setTorquePID(CURRENT_P,CURRENT_I,0,motorFrontLeft.UART)!=ERROR_NONE)
	{
		if(TIM7->CNT>=INIT_MAX_TIMEOUT_MS*10) 
		{
			failDriveInitCallback(motorFrontLeft);
			break;
		}
	}
	TIM7->CNT=0;
	while(setTorquePID(CURRENT_P,CURRENT_I,0,motorFrontRight.UART)!=ERROR_NONE)
	{
		if(TIM7->CNT>=INIT_MAX_TIMEOUT_MS*10) 
		{
			failDriveInitCallback(motorFrontRight);
			break;
		}
	}
	TIM7->CNT=0;
	while(setTorquePID(CURRENT_P,CURRENT_I,0,motorRearLeft.UART)!=ERROR_NONE)
	{
		if(TIM7->CNT>=INIT_MAX_TIMEOUT_MS*10) 
		{
			failDriveInitCallback(motorRearLeft);
			break;
		}
	}
	TIM7->CNT=0;
	while(setTorquePID(CURRENT_P,CURRENT_I,0,motorRearRight.UART)!=ERROR_NONE)
	{
		if(TIM7->CNT>=INIT_MAX_TIMEOUT_MS*10) 
		{
			failDriveInitCallback(motorRearRight);
			break;
		}
	}
	//Настройка регулятора потока
	TIM7->CNT=0;
	while(setFluxPID(CURRENT_P,CURRENT_I,0,motorFrontLeft.UART)!=ERROR_NONE)
	{
		if(TIM7->CNT>=INIT_MAX_TIMEOUT_MS*10) 
		{
			failDriveInitCallback(motorFrontLeft);
			break;
		}
	}
	TIM7->CNT=0;
	while(setFluxPID(CURRENT_P,CURRENT_I,0,motorFrontRight.UART)!=ERROR_NONE)
	{
		if(TIM7->CNT>=INIT_MAX_TIMEOUT_MS*10) 
		{
			failDriveInitCallback(motorFrontRight);
			break;
		}
	}
	TIM7->CNT=0;
	while(setFluxPID(CURRENT_P,CURRENT_I,0,motorRearLeft.UART)!=ERROR_NONE)
	{
		if(TIM7->CNT>=INIT_MAX_TIMEOUT_MS*10) 
		{
			failDriveInitCallback(motorRearLeft);
			break;
		}
	}
	TIM7->CNT=0;
	while(setFluxPID(CURRENT_P,CURRENT_I,0,motorRearRight.UART)!=ERROR_NONE)
	{
		if(TIM7->CNT>=INIT_MAX_TIMEOUT_MS*10) 
		{
			failDriveInitCallback(motorRearRight);
			break;
		}
	}
	//Задание начальной скорости
	TIM7->CNT=0;
	while(setSpeed(0,10,motorFrontLeft.UART)!=ERROR_NONE)
	{
		if(TIM7->CNT>=INIT_MAX_TIMEOUT_MS*10) 
		{
			failDriveInitCallback(motorFrontLeft);
			break;
		}
	}
	TIM7->CNT=0;
	while(setSpeed(0,10,motorFrontRight.UART)!=ERROR_NONE)
	{
		if(TIM7->CNT>=INIT_MAX_TIMEOUT_MS*10) 
		{
			failDriveInitCallback(motorFrontRight);
			break;
		}
	}
	TIM7->CNT=0;
	while(setSpeed(0,10,motorRearLeft.UART)!=ERROR_NONE)
	{
		if(TIM7->CNT>=INIT_MAX_TIMEOUT_MS*10) 
		{
			failDriveInitCallback(motorRearLeft);
			break;
		}
	}
	TIM7->CNT=0;
	while(setSpeed(0,10,motorRearRight.UART)!=ERROR_NONE)
	{
		if(TIM7->CNT>=INIT_MAX_TIMEOUT_MS*10) 
		{
			failDriveInitCallback(motorRearRight);
			break;
		}
	}
}

void allMotorStart(void)
{
	TIM7->CNT=0;
	motorState=1;
	while(startMotor(motorFrontLeft.UART)!=ERROR_NONE)
	{
		if(TIM7->CNT>=START_MOTOR_MAX_TIMEOUT*10) 
		{
			failDriveStartCallback(motorFrontLeft);
			motorState=0;
			break;
		}
	}
	TIM7->CNT=0;
	while(startMotor(motorFrontRight.UART)!=ERROR_NONE)
	{
		if(TIM7->CNT>=START_MOTOR_MAX_TIMEOUT*10) 
		{
			failDriveStartCallback(motorFrontRight);
			motorState=0;
			break;
		}
	}
	TIM7->CNT=0;
	while(startMotor(motorRearLeft.UART)!=ERROR_NONE)
	{
		if(TIM7->CNT>=START_MOTOR_MAX_TIMEOUT*10) 
		{
			failDriveStartCallback(motorRearLeft);
			motorState=0;
			break;
		}
	}
	TIM7->CNT=0;
	while(startMotor(motorRearRight.UART)!=ERROR_NONE)
	{
		if(TIM7->CNT>=START_MOTOR_MAX_TIMEOUT*10) 
		{
			failDriveStartCallback(motorRearRight);
			motorState=0;
			break;
		}
	}
	if(!motorState)
	{
		motorStopAll();
	}
}

void motorChangeZeroSpeed(void)
{
	TIM7->CNT=0;
	while(setSpeed(0,1,motorFrontLeft.UART)!=ERROR_NONE)
	{
		if(TIM7->CNT>=STOP_MOTOR_MAX_TIMEOUT*10) 
		{
			failDriveChangeSpeedCallback(motorFrontLeft);
			break;
		}
	}
	TIM7->CNT=0;
	while(setSpeed(0,1,motorFrontRight.UART)!=ERROR_NONE)
	{
		if(TIM7->CNT>=STOP_MOTOR_MAX_TIMEOUT*10) 
		{
			failDriveChangeSpeedCallback(motorFrontRight);
			break;
		}
	}
	TIM7->CNT=0;
	while(setSpeed(0,1,motorRearLeft.UART)!=ERROR_NONE)
	{
		if(TIM7->CNT>=STOP_MOTOR_MAX_TIMEOUT*10) 
		{
			failDriveChangeSpeedCallback(motorRearLeft);
			break;
		}
	}
	TIM7->CNT=0;
	while(setSpeed(0,1,motorRearRight.UART)!=ERROR_NONE)
	{
		if(TIM7->CNT>=STOP_MOTOR_MAX_TIMEOUT*10) 
		{
			failDriveChangeSpeedCallback(motorRearRight);
			break;
		}
	}
}

void allMotorFaultAck(void)
{
	TIM7->CNT=0;
	while(faultAck(motorFrontLeft.UART)!=ERROR_NONE)
	{
		if(TIM7->CNT>=FAULT_ACK_MAX_TIMEOUT*10) 
		{
			failDriveFaultAckCallback(motorFrontLeft);
			break;
		}
	}
	TIM7->CNT=0;
	while(faultAck(motorFrontRight.UART)!=ERROR_NONE)
	{
		if(TIM7->CNT>=FAULT_ACK_MAX_TIMEOUT*10) 
		{
			failDriveFaultAckCallback(motorFrontRight);
			break;
		}
	}
	TIM7->CNT=0;
	while(faultAck(motorRearLeft.UART)!=ERROR_NONE)
	{
		if(TIM7->CNT>=FAULT_ACK_MAX_TIMEOUT*10) 
		{
			failDriveFaultAckCallback(motorRearLeft);
			break;
		}
	}
	TIM7->CNT=0;
	while(faultAck(motorRearRight.UART)!=ERROR_NONE)
	{
		if(TIM7->CNT>=FAULT_ACK_MAX_TIMEOUT*10) 
		{
			failDriveFaultAckCallback(motorRearRight);
			break;
		}
	}
}

void motorStopAll(void)
{
	TIM7->CNT=0;
	motorState=0;
	while(stopMotor(motorFrontLeft.UART)!=ERROR_NONE)
	{
		if(TIM7->CNT>=START_MOTOR_MAX_TIMEOUT*10) 
		{
			failDriveStopCallback(motorFrontLeft);
			motorState=1;
			break;
		}
	}
	TIM7->CNT=0;
	while(stopMotor(motorFrontRight.UART)!=ERROR_NONE)
	{
		if(TIM7->CNT>=START_MOTOR_MAX_TIMEOUT*10) 
		{
			failDriveStopCallback(motorFrontRight);
			motorState=1;
			break;
		}
	}
	TIM7->CNT=0;
	while(stopMotor(motorRearLeft.UART)!=ERROR_NONE)
	{
		if(TIM7->CNT>=START_MOTOR_MAX_TIMEOUT*10) 
		{
			failDriveStopCallback(motorRearLeft);
			motorState=1;
			break;
		}
	}
	TIM7->CNT=0;
	while(stopMotor(motorRearRight.UART)!=ERROR_NONE)
	{
		if(TIM7->CNT>=START_MOTOR_MAX_TIMEOUT*10) 
		{
			failDriveStopCallback(motorRearRight);
			motorState=1;
			break;
		}
	}
}


