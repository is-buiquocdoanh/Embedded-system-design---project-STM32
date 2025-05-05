#ifndef AHT10_H
#define AHT10_H

#include "stm32f1xx_hal.h"

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t address;
} AHT10_HandleTypeDef;

void AHT10_Init(AHT10_HandleTypeDef *dev);
uint8_t AHT10_ReadRaw(AHT10_HandleTypeDef *dev, uint32_t *humidity_raw, uint32_t *temperature_raw);
float AHT10_ReadTemperature(uint32_t temp_raw);
float AHT10_ReadHumidity(uint32_t hum_raw);

#endif
