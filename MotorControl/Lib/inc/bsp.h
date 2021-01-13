#ifndef MC_BSP
#define MC_BSP

#include "stm32f407xx.h"
#include "MCP.h"


#define UART_MAX_TIMEOUT_MS 20

void uart6Init(uint32_t coreFreq, uint32_t baudRate);
void uart5Init(uint32_t coreFreq, uint32_t baudRate);
void uart4Init(uint32_t coreFreq, uint32_t baudRate);
void uart3Init(uint32_t coreFreq, uint32_t baudRate);
void uart2Init(uint32_t coreFreq, uint32_t baudRate);
void uart1Init(uint32_t coreFreq, uint32_t baudRate);
void uartTransmitt(uint8_t data,USART_TypeDef * UART);
void uartTransmittBuff(uint8_t* data,uint32_t size,USART_TypeDef * UART);
uint8_t uartRecive(USART_TypeDef * UART);

#endif