#ifndef CALLBACK_H
#define CALLBACK_H

#include "stm32f407xx.h"
#include "MotorControl.h"

void failDriveInitCallback(motor_t motor);
void failDriveStartCallback(motor_t motor);
void failDriveChangeSpeedCallback(motor_t motor);
void failDriveStopCallback(motor_t motor);
void failDriveFaultAckCallback(motor_t motor);
void bigRedButton(void);

#endif