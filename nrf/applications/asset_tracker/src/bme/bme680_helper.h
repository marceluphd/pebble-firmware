#ifndef _IOTEX_BME680_H_
#define _IOTEX_BME680_H_

#include <stdint.h>

#define I2C_DEV_BME680    "I2C_2"
#define I2C_ADDR_BME680   0x77


int8_t iotex_bme680_init(void);
int8_t iotex_bme680_get_sensor_data(uint8_t *str, size_t size);

#endif //_IOTEX_BME680_H_