/*
 * #define MPR121.c
 *
 *  Created on: Apr 24, 2021
 *      Author: PC
 */

#include "mpr121.h"

// define VALUE WRITE FOR MPR121
#define MPR121_RESET 0x63
#define MPR121_DEFAULT_CONFIG2 0x24

void mpr121_write(uint8_t slave_addr, uint8_t addr_register, uint8_t send_data)
{
	i2c_write(slave_addr, addr_register, send_data);
}

uint8_t mpr121_read(uint8_t slave_addr, uint8_t register_addr)
{
	return i2c_read(slave_addr, register_addr);
}
/*
*	This function will init #define MPR121 with step:
*	1. 	Reset #define MPR121.
*	2. 	Set electrode configure to default value.
*	3. 	Read MPR121_CONFIG2 to check default value after reset.
*	4.	Set touch and thresholds.
*	5. 	Configure baseline filtering control register.
*	6. 	Set other configuration register.
*	7.	Enable ectrodes
*/
void mpr121_int(void)
{
	uint8_t rdata = 0;
	// Step 1
	mpr121_write(SLAVE_ADDR, MPR121_SOFTRESET, MPR121_RESET);
	HAL_Delay(1);
	// Step 2
	mpr121_write(SLAVE_ADDR, MPR121_ECR, 0X0);
	// Step 3
	rdata = mpr121_read(SLAVE_ADDR, MPR121_CONFIG2);
	if ((rdata & MPR121_DEFAULT_CONFIG2) != MPR121_DEFAULT_CONFIG2)
	{
		// Should Set Led to high
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		return;
	}

	// Step 4
	mpr121_set_thresholds(12, 6, 12);
// Set BaseLine
#if 0
	mpr121_set_baseline(0x4e, 12);
#endif
	// Step 5
	mpr121_write(SLAVE_ADDR, MPR121_MHDR, 0x1);
	mpr121_write(SLAVE_ADDR, MPR121_NHDR, 0X1);
	mpr121_write(SLAVE_ADDR, MPR121_NCLR, 0XE);
	mpr121_write(SLAVE_ADDR, MPR121_FDLR, 0X0);
	mpr121_write(SLAVE_ADDR, MPR121_MHDF, 0X1);
	mpr121_write(SLAVE_ADDR, MPR121_NHDF, 0X5);
	mpr121_write(SLAVE_ADDR, MPR121_NCLF, 0X1);
	mpr121_write(SLAVE_ADDR, MPR121_FDLF, 0X0);
	mpr121_write(SLAVE_ADDR, MPR121_NHDT, 0X0);
	mpr121_write(SLAVE_ADDR, MPR121_NCLT, 0X0);
	mpr121_write(SLAVE_ADDR, MPR121_FDLT, 0X0);

	// Step 6
	mpr121_write(SLAVE_ADDR, MPR121_DEBOUNCE, 0X0);
	mpr121_write(SLAVE_ADDR, MPR121_CONFIG1, 0X10);
	mpr121_write(SLAVE_ADDR, MPR121_CONFIG2, 0X20);

	// Step 7
	mpr121_write(SLAVE_ADDR, MPR121_ECR, 0X83); // 0000 1000 0011 0 -> 2
	mpr121_set_output();
}

void mpr121_set_thresholds(uint8_t touch, uint8_t release, uint8_t Nb_pin_used)
{
	// Init all 12 register touch and release
	uint8_t loop;
	for (loop = 0; loop < Nb_pin_used; loop++)
	{
		mpr121_write(SLAVE_ADDR, MPR121_TOUCHTH_0 + (loop * 2), touch);
		mpr121_write(SLAVE_ADDR, MPR121_RELEASETH_0 + (loop * 2), release);
	}
}

void mpr121_set_baseline(uint8_t base_line, uint8_t Nb_pin_used)
{
	// Init all 12 register touch and release
	uint8_t loop;
	for (loop = 0; loop < Nb_pin_used; loop++)
	{
		mpr121_write(SLAVE_ADDR, MPR121_BASELINE_0 + (loop * 2), base_line);
	}
}

uint8_t mpr121_get_thresholds(uint8_t pin, uint8_t type)
{
	uint8_t rdata;
	if (type == MPR121_TOUCH_THRESHOLD)
	{
		rdata = mpr121_read(SLAVE_ADDR, MPR121_TOUCHTH_0 + (pin * 2));
	}
	else if (type == MPR121_RELEASE_THRESHOLD)
	{
		rdata = mpr121_read(SLAVE_ADDR, MPR121_RELEASETH_0 + (pin * 2));
	}
	return rdata;
}

uint8_t mpr121_filtered_data(uint8_t pin, uint8_t type)
{
	uint8_t rdata;
	if (type == MPR121_FILTER_L)
	{
		rdata = mpr121_read(SLAVE_ADDR, MPR121_FILTDATA_0L + (pin * 2));
	}
	else
	{
		rdata = mpr121_read(SLAVE_ADDR, MPR121_FILTDATA_0H + (pin * 2));
	}
	return rdata;
}

uint8_t mpr121_baseline_data(uint8_t pin)
{
	uint8_t rdata;
	rdata = mpr121_read(SLAVE_ADDR, MPR121_BASELINE_0 + (pin));
	return rdata;
}

uint16_t mpr121_touched(void)
{
	uint8_t rdatal;
	uint16_t rdata;
	rdatal = mpr121_read(SLAVE_ADDR, MPR121_TOUCHSTATUS_L);
	rdata = mpr121_read(SLAVE_ADDR, MPR121_TOUCHSTATUS_H);
	rdata = ((rdata << 8) | (rdatal));

#if 0
	printf("\n\r\n\r-------------------------------------\n\r");
	printf("rdatal: %x, Touch Status: %x \n\r", rdatal, rdata);
	
	// Display mpr121_filtered_data
	rdatal = mpr121_filtered_data(0, MPR121_FILTER_L);
	printf("filtered_L: %x, ", rdatal);
	rdatal = mpr121_filtered_data(0, MPR121_FILTER_H);
	printf("filtered_H: %x \n\r", rdatal);
	// Display base line
	rdatal = mpr121_baseline_data(0);
	printf("baseline: %x \n\r", rdatal);
	// get thresold
	rdatal = mpr121_get_thresholds(0, MPR121_TOUCH_THRESHOLD);
	printf("touch threshold: %x \n\r", rdatal);
	rdatal = mpr121_get_thresholds(0, MPR121_RELEASE_THRESHOLD);
	printf("release threshold: %x \n\r", rdatal);
	printf("-------------------------------------\n\r");
#endif
	return rdata & 0xFFF;
}

void mpr121_set_output(void)
{
	// Enable register
	mpr121_write(SLAVE_ADDR, MPR121_GPIOEN, 0xF0);
	// Set direction
	mpr121_write(SLAVE_ADDR, MPR121_GPIODIR, 0xF0);
	// Set GTL
	mpr121_write(SLAVE_ADDR, MPR121_GPIOGTL0, 0x0);
	mpr121_write(SLAVE_ADDR, MPR121_GPIOGTL1, 0x0);

	// Set data
	mpr121_write(SLAVE_ADDR, MPR121_GPIODATA, 0xF0);
}

void mpr121_write_data(uint8_t data)
{
	//printf("Led Display: %x \n\r", ~(data));
	mpr121_write(SLAVE_ADDR, MPR121_GPIODATA, data);
}

void mpr121_display_touch(void)
{
	uint8_t rdata_touch;
	rdata_touch = mpr121_touched();

	//printf("rdata touch: %x \n\r", rdata_touch);
	mpr121_write_data(rdata_touch << 4);
}
