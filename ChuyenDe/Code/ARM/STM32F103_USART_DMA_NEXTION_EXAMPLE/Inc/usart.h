#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "main.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;


extern void _Error_Handler(char *, int);

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);

void uart2_send_data(char *string);

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

