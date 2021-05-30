/*
 * mpr121.h
 *
 *  Created on: Apr 24, 2021
 *      Author: PC
 */

#ifndef MPR121_H_
#define MPR121_H_
#include "i2c.h"

#define SLAVE_ADDR				   	0x5A<<1
#define MPR121_TOUCHSTATUS_L       	0x00
#define MPR121_TOUCHSTATUS_H       	0x01
#define MPR121_FILTDATA_0L        	0x04
#define MPR121_FILTDATA_0H         	0x05
#define MPR121_BASELINE_0          	0x1E
#define MPR121_MHDR                	0x2B
#define MPR121_NHDR                	0x2C
#define MPR121_NCLR                	0x2D
#define MPR121_FDLR                	0x2E
#define MPR121_MHDF                	0x2F
#define MPR121_NHDF                	0x30
#define MPR121_NCLF                	0x31
#define MPR121_FDLF                	0x32
#define MPR121_NHDT                	0x33
#define MPR121_NCLT                	0x34
#define MPR121_FDLT                	0x35
#define MPR121_TOUCHTH_0           	0x41
#define MPR121_RELEASETH_0         	0x42
#define MPR121_DEBOUNCE            	0x5B
#define MPR121_CONFIG1             	0x5C
#define MPR121_CONFIG2             	0x5D
#define MPR121_CHARGECURR_0        	0x5F
#define MPR121_CHARGETIME_1        	0x6C
#define MPR121_ECR                 	0x5E
#define MPR121_AUTOCONFIG0         	0x7B
#define MPR121_AUTOCONFIG1         	0x7C
#define MPR121_UPLIMIT             	0x7D
#define MPR121_LOWLIMIT            	0x7E
#define MPR121_TARGETLIMIT         	0x7F
#define MPR121_GPIOGTL0			    0x73
#define MPR121_GPIOGTL1				0x74
#define MPR121_GPIODATA				0x75
#define MPR121_GPIODIR             	0x76
#define MPR121_GPIOEN              	0x77
#define MPR121_GPIOSET             	0x78
#define MPR121_GPIOCLR             	0x79
#define MPR121_GPIOTOGGLE          	0x7A
#define MPR121_SOFTRESET           	0x80

#define MPR121_TOUCH_THRESHOLD	    0
#define MPR121_RELEASE_THRESHOLD	1
#define MPR121_FILTER_L				0
#define MPR121_FILTER_H				1

void mpr121_write(uint8_t slave_addr, uint8_t addr_register, uint8_t send_data);
uint8_t mpr121_read(uint8_t slave_addr, uint8_t register_addr);

void mpr121_int(void);
void mpr121_set_thresholds(uint8_t touch, uint8_t release, uint8_t Nb_pin_used);
void mpr121_set_baseline(uint8_t base_line, uint8_t Nb_pin_used);
uint8_t mpr121_get_thresholds(uint8_t pin, uint8_t type);
uint8_t mpr121_filtered_data(uint8_t pin, uint8_t type);
uint8_t mpr121_baseline_data(uint8_t pin);
uint16_t mpr121_touched(void);
void mpr121_set_output(void);
void mpr121_write_data(uint8_t data);
void mpr121_display_touch(void);
#endif /* MPR121_H_ */
