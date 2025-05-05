#include "aht10.h"
#include "main.h"
#include "stm32f1xx_hal.h"

extern UART_HandleTypeDef huart1;


void AHT10_Init(AHT10_HandleTypeDef *dev) {
    uint8_t cmd[3] = {0xE1, 0x08, 0x00};
    HAL_I2C_Master_Transmit(dev->hi2c, dev->address, cmd, 3, HAL_MAX_DELAY);
    HAL_Delay(20);
}

uint8_t AHT10_ReadRaw(AHT10_HandleTypeDef *dev, uint32_t *humidity_raw, uint32_t *temperature_raw) {
	uint8_t data[6];
	uint8_t cmd[3] = {0xAC, 0x33, 0x00}; // chuẩn AHT10 command
	HAL_I2C_Master_Transmit(dev->hi2c, dev->address, cmd, 3, HAL_MAX_DELAY);
	HAL_Delay(80);

	if (HAL_I2C_Master_Receive(dev->hi2c, dev->address, data, 6, HAL_MAX_DELAY) != HAL_OK)
	    return 0;

	// Debug: in toàn bộ data nhận được
//	char buf[64];
//	sprintf(buf, "Raw bytes: %02X %02X %02X %02X %02X %02X\r\n",
//	        data[0], data[1], data[2], data[3], data[4], data[5]);
//	HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	if (data[0] & 0x80)
	    return 0;  // still busy, không đọc được

	*humidity_raw = ((uint32_t)(data[1]) << 12) | ((uint32_t)(data[2]) << 4) | (data[3] >> 4);
	*temperature_raw = (((uint32_t)(data[3] & 0x0F)) << 16) | ((uint32_t)(data[4]) << 8) | (data[5]);
	return 1;
}


float AHT10_ReadTemperature(uint32_t temp_raw) {
    return ((float)temp_raw / 1048576.0) * 200.0 - 50.0;
}

float AHT10_ReadHumidity(uint32_t hum_raw) {
    return ((float)hum_raw / 1048576.0) * 100.0;
}
