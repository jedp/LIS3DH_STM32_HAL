/*
 * lis3dh.c
 *
 *  Created on: Mar 4, 2023
 *      Author: jed
 */

#include "lis3dh.h"

#define I2C_READ_BIT   (1)
#define I2C_WRITE_BIT  (0)
#define TIMEOUT_MS     (50)

HAL_StatusTypeDef lis3dh_init(lis3dh_t *lis3dh, I2C_HandleTypeDef *i2c, uint8_t *buf, uint16_t bufsize) {
	HAL_StatusTypeDef status;

	lis3dh->i2c = i2c;
	lis3dh->i2c_addr = LIS3DH_ADDR << 1;
	lis3dh->buf = buf;
	lis3dh->bufsize = bufsize;

	/* Let device wake up. */
	HAL_Delay(LID3DH_POWER_UP_MS);

	status = HAL_I2C_IsDeviceReady(lis3dh->i2c, lis3dh->i2c_addr, 1, TIMEOUT_MS);
	if (status != HAL_OK) return status;

    /* Confirm the device identifies itself as expected. */
	status = lis3dh_read(lis3dh, REG_WHO_AM_I, 1);
	if (status != HAL_OK) return status;
	if (lis3dh->buf[0] != LIS3DH_DEVICE_ID) return HAL_ERROR;

	// Set power mode to operational; Enable all axes; Normal operation.
	status = lis3dh_write(lis3dh, REG_CTRL_REG1, DATA_RATE_NORM_1kHz344 | 0x07);
	if (status != HAL_OK) return status;

	// High resolution; BDU enabled.
	status = lis3dh_write(lis3dh, REG_CTRL_REG4, 0x88);
	if (status != HAL_OK) return status;

	// Enable temp sensor.
	status = lis3dh_write(lis3dh, REG_TEMP_CFG_REG, 0x80);
	return status;
}

bool lis3dh_xyz_available(lis3dh_t *lis3dh) {
	/*
	 * Read STATUS_REG bit 2 (ZYXDA): New X, Y, Z data available.
	 */
	HAL_StatusTypeDef status;
	status = lis3dh_read(lis3dh, REG_STATUS_REG, 1);
	if (status != HAL_OK) return false;

	return (lis3dh->buf[0] & 2) > 0;
}

HAL_StatusTypeDef lis3dh_read(lis3dh_t* lis3dh, uint16_t reg, uint16_t bufsize) {
	if (bufsize > lis3dh->bufsize) return HAL_ERROR;

	return HAL_I2C_Mem_Read(lis3dh->i2c, lis3dh->i2c_addr | I2C_READ_BIT, reg, 1, lis3dh->buf, bufsize, TIMEOUT_MS);
}

HAL_StatusTypeDef lis3dh_write(lis3dh_t* lis3dh, uint16_t reg, uint8_t data) {
	return HAL_I2C_Mem_Write(lis3dh->i2c, lis3dh->i2c_addr, reg, 1, &data, 1, TIMEOUT_MS);
}

HAL_StatusTypeDef lis3dh_get_xyz(lis3dh_t* lis3dh) {
	if (lis3dh->bufsize < 6) return HAL_ERROR;
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
			lis3dh->i2c,
			lis3dh->i2c_addr | I2C_READ_BIT,
			REG_OUT_XYZ_BASE | 0x80,          // Progressively read 6 buffers.
			1,
			lis3dh->buf,
			6,
			TIMEOUT_MS);

	if (status != HAL_OK) {
		lis3dh->x = -1;
		lis3dh->y = -1;
		lis3dh->z = -1;
		return status;
	}

	lis3dh->x = (int) (((int8_t) lis3dh->buf[1]) << 8) | lis3dh->buf[0];
	lis3dh->y = (int) (((int8_t) lis3dh->buf[3]) << 8) | lis3dh->buf[2];
	lis3dh->z = (int) (((int8_t) lis3dh->buf[5]) << 8) | lis3dh->buf[4];

	return HAL_OK;
}

/* To-do */
HAL_StatusTypeDef lis3dh_get_temp(lis3dh_t* lis3dh) {
	return HAL_ERROR;
}
