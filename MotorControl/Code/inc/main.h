#ifndef MAIN_H
#define MAIN_H

#include "stm32f407xx.h"
#include "rcc.h"
#include "delay.h"
#include "bsp.h"
#include "MCP.h"
#include "MotorControl.h"
#include "encoder.h"
#include <stdio.h>
#include "pdu.h"
#include "callback.h"
#include "adc.h"

#define INIT_MAX_TIMEOUT_MS 500
#define START_MOTOR_MAX_TIMEOUT 100
#define STOP_MOTOR_MAX_TIMEOUT 50
#define FAULT_ACK_MAX_TIMEOUT 50
#define PDU_PERIOD 100
#define CHECK_STATUS_PERIOD 200
#define MOTOR_RADIUS 0.2
#define MOTOR_RAMP_TIME_MS 10

#define SPEED_P 32000
#define SPEED_I	1500
#define CURRENT_P 15000
#define CURRENT_I 9000

#endif