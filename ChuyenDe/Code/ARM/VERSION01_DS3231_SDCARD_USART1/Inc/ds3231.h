#include "stm32f1xx_hal.h"

#define DS3231_SECOND       0x0
#define DS3231_MINUTE       0x1
#define DS3231_HOURS        0x2
#define DS3231_DATE         0x4
#define DS3231_MONTH        0x5
#define DS3231_YEARS        0x6

#define DS3231_CONTROL      0xE
#define DS3231_STATUS       0xF

typedef enum{
    DS3231_OK = 0,
    DS3231_BUSY = 1
}ds3231_status;

typedef struct ds3231 {
    uint8_t second;
    uint8_t minute;
    uint8_t hours;
    uint8_t date;
    uint8_t month;
    uint8_t year;
    uint8_t temp;
}ds3231_t;

uint8_t BDC2DEC(uint8_t data);
uint8_t DEC2BCD(uint8_t data);

uint8_t ds3231_is_busy(void);
void ds3231_write(uint8_t addr, uint8_t data_write);
uint8_t ds3231_read(uint8_t addr);
void ds3231_init(void);
void ds3231_set_time(uint8_t second, uint8_t minute, uint8_t hour, uint8_t date, uint8_t month, uint8_t year);
void ds3231_get_time(ds3231_t *get_time);
float ds3231_get_temp(ds3231_t *get_temp);

//------------------ DUMP VALUE --------------------//
void ds3231_dump(void);

