#include "i2c.h"
#include "ds3231.h"
#include "common.h"
#include "usart.h"

#define DS3231_SLAVE_ADDR   0x68<<1 //0x0110 1000 -> 0x1101 0000
#define DS3231_BUSY         0x04

ds3231_t time;

uint8_t BDC2DEC(uint8_t data)
{
    return (int)( (data/16*10) + (data%16) );
}

uint8_t DEC2BCD(uint8_t data)
{
    return (uint8_t)( (data/10*16) + (data%10) );
}

uint8_t ds3231_is_busy(void)
{
	uint8_t rdata;
    rdata = ds3231_read(DS3231_STATUS);
	if(rdata & DS3231_BUSY)
        return DS3231_BUSY;
    return DS3231_OK;
}

void ds3231_write(uint8_t addr, uint8_t data_write)
{
    i2c_write(DS3231_SLAVE_ADDR, addr, data_write);
}

uint8_t ds3231_read(uint8_t addr)
{
    uint8_t rdata;
    rdata = i2c_read(DS3231_SLAVE_ADDR, addr);
    return rdata;
}

void ds3231_init(void)
{
	if(ds3231_is_busy())
        return;
    ds3231_set_time(00,45,15,05,06,21);
}
void ds3231_set_time(uint8_t second, uint8_t minute, uint8_t hour, uint8_t date, uint8_t month, uint8_t year)
{
    ds3231_write(DS3231_SECOND, DEC2BCD(second));
    ds3231_write(DS3231_MINUTE, DEC2BCD(minute));
    ds3231_write(DS3231_HOURS, DEC2BCD(hour));
    ds3231_write(DS3231_DATE, DEC2BCD(date));
    ds3231_write(DS3231_MONTH, DEC2BCD(month));
    ds3231_write(DS3231_YEARS, DEC2BCD(year));
}
void ds3231_get_time(ds3231_t *get_time)
{
    get_time->second = BDC2DEC(ds3231_read(DS3231_SECOND));
    get_time->minute = BDC2DEC(ds3231_read(DS3231_MINUTE));
    get_time->hours = BDC2DEC(ds3231_read(DS3231_HOURS));
    get_time->date = BDC2DEC(ds3231_read(DS3231_DATE));
    get_time->month = BDC2DEC(ds3231_read(DS3231_MONTH));
    get_time->year = BDC2DEC(ds3231_read(DS3231_YEARS));
}

void ds3231_dump(void)
{
    ds3231_get_time(&time);
    printf("%d(Y) - %d(M) - %d(D) -- %d(H) - %d(Mi) - %d(S) \n\r",time.year, time.month, time.date \
            , time.hours, time.minute, time.second);
}

