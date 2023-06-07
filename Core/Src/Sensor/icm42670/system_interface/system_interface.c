/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2016-2016 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively "Software") is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "Sensor/icm42670/imu/inv_imu_transport.h"
#include "Sensor/icm42670/system_interface/spi_instance.h"
#include "Sensor/icm42670/system_interface/system_interface.h"

//#include "Invn/EmbUtils/Message.h"

/* I2C number and slave address for INV device */
#if (SM_BOARD_REV == SM_REVB_OB)
#define INV_SPI_AP   INV_SPI_ONBOARD_REVB
#define ICM_I2C_ADDR 0x69
#else
/* SM_REVB_DB and SM_REVG have same SPI/I2C configuration */
#define INV_SPI_AP   INV_SPI_REVG
#define ICM_I2C_ADDR 0x68
#endif

/* I2C address for Ak09915 */
#define AK_I2C_ADDR 0x0E

void inv_io_hal_board_init(void)
{
	inv_spi_master_init();
}

int inv_io_hal_read_reg(struct inv_imu_serif *serif, uint8_t reg, uint8_t *rbuffer, uint32_t rlen)
{
	return inv_spi_master_read_register(0x00, reg, rlen, rbuffer);
}

int  inv_io_hal_write_reg(struct inv_imu_serif *serif, uint8_t reg,uint8_t *wbuffer,uint32_t wlen)
{
	return inv_spi_master_write_register(0x00,reg,wlen,wbuffer);
}
