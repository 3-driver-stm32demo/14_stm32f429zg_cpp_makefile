//设备接口抽象层，实际上就是包装了底层的hal库函数，给上层提供统一的接口，从而屏蔽硬件差异
#include "Driver/Port_iic.hpp"

/**
  * @brief  I2C 工作模式配置
  * @param  无
  * @retval 无
  */
void port_iic::iic_init(I2C_TypeDef* IIC_id,uint32_t speed)
{
    iic_Handle.Instance             = IIC_id;
    iic_Handle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    iic_Handle.Init.ClockSpeed      = speed;
    iic_Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    iic_Handle.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    iic_Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    iic_Handle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    iic_Handle.Init.OwnAddress1     = 0 ;
    iic_Handle.Init.OwnAddress2     = 0; 
    /* Init the I2C */
    HAL_I2C_Init(&iic_Handle);	

    HAL_I2CEx_AnalogFilter_Config(&iic_Handle, I2C_ANALOGFILTER_ENABLE);  
}   

void port_iic::iic_set_dev_addr(uint8_t addr)
{
    dev_addr = addr;
}

HAL_StatusTypeDef port_iic::iic_write_buf(uint8_t* pBuffer, uint8_t WriteAddr, uint8_t NumByteToWrite)
{
    HAL_StatusTypeDef status = HAL_OK;
	/* Write EEPROM_PAGESIZE */
	status=HAL_I2C_Mem_Write(&iic_Handle, dev_addr,WriteAddr, I2C_MEMADD_SIZE_8BIT, (uint8_t*)(pBuffer),NumByteToWrite, 100);

	while (HAL_I2C_GetState(&iic_Handle) != HAL_I2C_STATE_READY)
	{
		
	}

	/* Check if the EEPROM is ready for a new operation */
	while (HAL_I2C_IsDeviceReady(&iic_Handle, dev_addr, EEPROM_MAX_TRIALS, I2Cx_TIMEOUT_MAX) == HAL_TIMEOUT);

	/* Wait for the end of the transfer */
	while (HAL_I2C_GetState(&iic_Handle) != HAL_I2C_STATE_READY)
	{
		
	}
	return status;
}

HAL_StatusTypeDef port_iic::iic_read_buf(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
	HAL_StatusTypeDef status = HAL_OK;
	
	status=HAL_I2C_Mem_Read(&iic_Handle,dev_addr,ReadAddr, I2C_MEMADD_SIZE_8BIT, (uint8_t *)pBuffer, NumByteToRead,1000);

	return status;
}

/**
  * @brief   写一个字节到I2C EEPROM中
  * @param   
  *		@arg pBuffer:缓冲区指针
  *		@arg WriteAddr:写地址 
  * @retval  无
  */
HAL_StatusTypeDef port_iic::iic_write_Byte(uint8_t* pBuffer, uint8_t WriteAddr)
{
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_I2C_Mem_Write(&iic_Handle, dev_addr, (uint16_t)WriteAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, 1, 100); 

	/* Check the communication status */
	if(status != HAL_OK)
	{
	/* Execute user timeout callback */
	//I2Cx_Error(Addr);
	}
	while (HAL_I2C_GetState(&iic_Handle) != HAL_I2C_STATE_READY)
	{
		
	}

	/* Check if the EEPROM is ready for a new operation */
	while (HAL_I2C_IsDeviceReady(&iic_Handle, dev_addr, EEPROM_MAX_TRIALS, I2Cx_TIMEOUT_MAX) == HAL_TIMEOUT);

	/* Wait for the end of the transfer */
	while (HAL_I2C_GetState(&iic_Handle) != HAL_I2C_STATE_READY)
	{
		
	}
	return status;
}