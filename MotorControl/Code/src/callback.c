#include "callback.h"

void failDriveInitCallback(motor_t motor)
{
	//Что-то делаем
}

void failDriveStartCallback(motor_t motor)
{
	//Что-то пошло не так при старте двигателя
}

void failDriveChangeSpeedCallback(motor_t motor)
{
	//Что-то пошло не так при изменении скорости
}

void failDriveStopCallback(motor_t motor)
{
	//Что-то не так при остановке
}

void failDriveFaultAckCallback(motor_t motor)
{
	//Что-то не так при сбросе
}

void bigRedButton(void)
{
	//Остановить всё
}