#ifndef PDU_H
#define PDU_H

#include "stm32f407xx.h"
#include "math.h"
#include "bsp.h"

void tim1Init(void);
void tim2Init1(void);
void tim2Init2(void);
void tim3Init1(void);
void tim3Init2(void);
void tim4Init1(void);
void tim4Init2(void);
void tim5Init1(void);
void tim5Init2(void);

void tim8Init(void);
void tim9Init(void);
void tim12Init(void);

int ost(double a);
double sign(double a);
void normaliz();
void kinematica();
void send_to_uart(uint8_t data);
void send_str(char * string);
#endif