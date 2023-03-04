/*
 * lis3dh.h
 *
 *  Created on: Mar 3, 2023
 *      Author: jed
 */

#ifndef INC_LIS3DH_H_
#define INC_LIS3DH_H_

#include <stdbool.h>
#include "stm32f0xx_hal.h"

typedef struct lis3dh {
	/* The HAL I2C_HandleTypeDef. */
	void* i2c;

	/* The 7-bit i2c address. */
	uint16_t i2c_addr;

	/* Data reported by the sensor */
	int x;
	int y;
	int z;
	int t;

	/* Buffer for data read from the device. Must be 6 bytes to ready XYZ data. */
	uint16_t bufsize;
	uint8_t *buf;
} lis3dh_t;

/**
 * Boot procedure is complete "about 5ms after power up." Doubling that number.
 */
#define LID3DH_POWER_UP_MS      (10)

#define LIS3DH_ADDR             (0x18)  // 7-bit I2C address. If SA0 is pulled high, address is 0x19.

#define LIS3DH_DEVICE_ID        (0x33)  // Contents of WHO_AM_I register.

#define REG_STATUS_REG_AUX      (0x07)  // r
#define REG_OUT_ADC1_L          (0x08)  // r
#define REG_OUT_ADC1_H          (0x09)  // r
#define REG_OUT_ADC2_L          (0x0a)  // r
#define REG_OUT_ADC2_H          (0x0b)  // r
#define REG_OUT_ADC3_L          (0x0c)  // r
#define REG_OUT_ADC3_H          (0x0d)  // r
#define REG_WHO_AM_I            (0x0f)  // r
#define REG_CTRL_REG0           (0x1e)  // rw
#define REG_TEMP_CFG_REG        (0x1f)  // rw
#define REG_CTRL_REG1           (0x20)  // rw
#define REG_CTRL_REG2           (0x21)  // rw
#define REG_CTRL_REG3           (0x22)  // rw
#define REG_CTRL_REG4           (0x23)  // rw
#define REG_CTRL_REG5           (0x24)  // rw
#define REG_CTRL_REG6           (0x25)  // rw
#define REG_REFERENCE           (0x26)  // rw
#define REG_STATUS_REG          (0x27)  // r
#define REG_OUT_XYZ_BASE        (0x28)  // r  (Base for reading the six XYZ registers consecutively)
#define REG_OUT_X_L             (0x28)  // r
#define REG_OUT_X_H             (0x29)  // r
#define REG_OUT_Y_L             (0x2a)  // r
#define REG_OUT_Y_H             (0x2b)  // r
#define REG_OUT_Z_L             (0x2c)  // r
#define REG_OUT_Z_H             (0x2d)  // r
#define REG_FIFO_CTRL_REG       (0x2e)  // rw
#define REG_FIFO_SRC_REG        (0x2f)  // r
#define REG_INT1_CFG            (0x30)  // rw
#define REG_INT1_SRC            (0x31)  // r
#define REG_INT1_THS            (0x32)  // rw
#define REG_INT1_DURATION       (0x33)  // rw
#define REG_INT2_CFG            (0x34)  // rw
#define REG_INT2_SRC            (0x35)  // r
#define REG_INT2_THS            (0x36)  // rw
#define REG_INT2_DURATION       (0x37)  // rw
#define REG_CLICK_CFG           (0x38)  // rw
#define REG_CLICK_SRC           (0x39)  // r
#define REG_CLICK_THS           (0x3a)  // rw
#define REG_TIME_LIMIT          (0x3b)  // rw
#define REG_TIME_LATENCY        (0x3c)  // rw
#define REG_TIME_WINDOW         (0x3d)  // rw
#define REG_ACT_THS             (0x3c)  // rw
#define REG_ACT_DUR             (0x3f)  // rw

/**
 * TEMP_CFG_REG
 */
#define TEMP_EN                 (1 << 8) // Temp sensor enable. Default: 0
#define ADC_EN                  (1 << 7) // ADC enable. Default: 0

/**
 * CTRL_REG1
 */
#define DATA_RATE_POWER_DOWN	    (0x0) // Power-down mode. Default.
#define DATA_RATE_LOW_1Hz       (0x1 << 4)
#define DATA_RATE_LOW_10Hz      (0x2 << 4)
#define DATA_RATE_LOW_25Hz      (0x3 << 4)
#define DATA_RATE_LOW_50Hz      (0x4 << 4)
#define DATA_RATE_LOW_100Hz     (0x5 << 4)
#define DATA_RATE_LOW_200Hz     (0x6 << 4)
#define DATA_RATE_LOW_400Hz     (0x7 << 4)
#define DATA_RATE_LOW_1kHz6     (0x8 << 4)
#define DATA_RATE_NORM_1kHz344  (0x9 << 4)

/**
 * CTRL_REG3
 */
#define INT1_CLICK              (1 << 7) // Click interrupt on INT1. Default: 0
#define INT1_IA1                (1 << 8) // IA1 interrupt on INT1. Default: 0
#define INT1_IA2                (1 << 5) // IA2 interrupt on INT1. Default: 0
#define INT1_ZYXDA              (1 << 4) // ZXYDA interrupt on INT1. Default: 0
#define INT1_321DA              (1 << 3) // 321DA interrupt on INT1. Default: 0
#define INT1_WTM                (1 << 2) // FIFO watermark interrupt on INT1. Default: 0
#define INT1_OVERRUN            (1 << 1) // FIFO overrun interrupt on INT1. Default: 0

HAL_StatusTypeDef lis3dh_init(lis3dh_t *lis3dh, I2C_HandleTypeDef *i2c, uint8_t *buf, uint16_t bufsize);

bool lis3dh_xyz_available(lis3dh_t *lis3dh);

HAL_StatusTypeDef lis3dh_read(lis3dh_t *lis3dh, uint16_t reg, uint16_t bufsize);

HAL_StatusTypeDef lis3dh_write(lis3dh_t *lis3dh, uint16_t reg, uint8_t data);

/* Read the XYZ data into lis3dh->buf. Requires a buffer size of 6 bytes. */
HAL_StatusTypeDef lis3dh_get_xyz(lis3dh_t *lis3dh);

HAL_StatusTypeDef lis3dh_get_temp(lis3dh_t *lis3dh);

#endif /* INC_LIS3DH_H_ */
