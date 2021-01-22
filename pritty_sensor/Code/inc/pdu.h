#ifndef PDU_H
#define PDU_H

#include "stm32f407xx.h"
 
void tim1Init(void);
void tim2Init(void);
void tim4Init(void);
void tim5Init(void);
void tim6Init(void);
void tim8Init(void);
double med_filt(char pack);
double sign(double a);
void send_to_uart(uint16_t data);
void send_str(char * string);
int deg_to_pwm(int a);
int constr(int a);
long map(long x,long inMin,long inMax,long outMin, long outMax);
#endif