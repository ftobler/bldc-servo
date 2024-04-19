/*
 * as5600.c
 *
 *  Created on: Apr 19, 2024
 *      Author: ftobler
 */

#include "as5600.h"
#include "stm32_hal.h"

enum {
	AS5600L_ADDR = 0x36*2,
};


extern I2C_HandleTypeDef hi2c1;

static uint8_t i2c_received[2] = {0};
volatile int32_t as5600_angle = 0;
volatile int32_t overflow = 0;
volatile int32_t last_angle = 0;

void as5600_poll() {
	//poll AS5600 sensor
	HAL_I2C_Mem_Read(&hi2c1, AS5600L_ADDR, 0x0C, I2C_MEMADD_SIZE_8BIT, i2c_received, 2, 2);
	int32_t angle = i2c_received[1] + ((uint16_t)i2c_received[0] << 8);

	int32_t difference = angle - last_angle;
	if (difference > 2048) {
		//underflow
		overflow--;
	} else if (difference < -2048) {
		//overflow
		overflow++;
	}
	last_angle = angle;
	as5600_angle = angle + 4096 * overflow;


//	int32_t last_angle = as5600_angle % 4096;
//	int32_t difference = angle - last_angle;
//	if (difference > 2048) {
//		//underflow
//		as5600_angle = angle - 4096;
//	} else if (difference < -2048) {
//		//overflow
//		as5600_angle = angle + 4096;
//	} else {
//		as5600_angle = (as5600_angle / 4096) * 4096 + angle;
//	}
}
