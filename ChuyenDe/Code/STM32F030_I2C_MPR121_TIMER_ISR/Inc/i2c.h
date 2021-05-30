#ifndef __i2c_H
#define __i2c_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include "main.h"


extern I2C_HandleTypeDef hi2c1;

extern void _Error_Handler(char *, int);
void MX_I2C1_Init(void);
void i2c_write(uint8_t slave_addr,uint8_t addr_register, uint8_t send_data);
uint8_t i2c_read(uint8_t slave_addr, uint8_t register_addr);

#ifdef __cplusplus
}
#endif
#endif /*__ i2c_H */

