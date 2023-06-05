#ifndef __PORT_IIC_H
#define __PORT_IIC_H

#include "stm32f4xx.h"

#define I2Cx_TIMEOUT_MAX                300
/* Maximum number of trials for HAL_I2C_IsDeviceReady() function */
#define EEPROM_MAX_TRIALS               300

class port_iic
{
    public:
    void iic_init(I2C_TypeDef* IIC_id,uint32_t speed);
    void iic_set_dev_addr(uint8_t addr);

    HAL_StatusTypeDef iic_write_Byte(uint8_t* pBuffer, uint8_t WriteAddr);

    HAL_StatusTypeDef iic_write_buf(uint8_t* pBuffer, uint8_t WriteAddr, uint8_t NumByteToWrite);
    HAL_StatusTypeDef iic_read_buf(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);   

    I2C_HandleTypeDef  iic_Handle;
    uint8_t dev_addr;
};

#endif