// 设备接口抽象层，实际上就是包装了底层的hal库函数，给上层提供统一的接口，从而屏蔽硬件差异
#include "Driver/Port_spi.hpp"

#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  
#define GPIOF_ODR_Addr    (GPIOF_BASE+20) //0x40021414  
#define SPI5_CS PFout(6)

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

void memset0(uint8_t *buf,uint8_t val,uint16_t size)
{
  uint16_t i = 0 ;
  for(i = 0 ;i < size;i++)
  {
    buf[i] = val ; 
  }
}
/**
 * @brief  SPI 工作模式配置
 * @param  无
 * @retval 无
 */
void port_spi::spi_init(SPI_TypeDef* spi_id, uint32_t speed) {
  spi_Handle.Instance = spi_id;
  spi_Handle.Init.Mode = SPI_MODE_MASTER;
  spi_Handle.Init.Direction = SPI_DIRECTION_2LINES;
  spi_Handle.Init.DataSize = SPI_DATASIZE_8BIT;
  spi_Handle.Init.CLKPolarity = SPI_POLARITY_HIGH;
  spi_Handle.Init.CLKPhase = SPI_PHASE_2EDGE;
  spi_Handle.Init.NSS = SPI_NSS_SOFT;
  spi_Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  spi_Handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
  spi_Handle.Init.TIMode = SPI_TIMODE_DISABLE;
  spi_Handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  spi_Handle.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&spi_Handle) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */
  __HAL_SPI_ENABLE(&spi_Handle);
}

int port_spi::SPI_ReadWriteByte(uint8_t TxData, uint8_t* Rxdata) {
  HAL_StatusTypeDef errorcode = HAL_OK;
  errorcode = HAL_SPI_TransmitReceive(&spi_Handle, &TxData, Rxdata, 1, 1000);
  if (errorcode != HAL_OK) {
    return -1;
  }
  return 0;
}

int port_spi::SPI_ReadOneRegister(uint8_t addr, uint8_t* data) {
  uint8_t temp_rx = 0;
  int ret = 0;
  SPI5_CS = 0;
  ret = SPI_ReadWriteByte(addr | 0x80, &temp_rx);
  ret |= SPI_ReadWriteByte(0x00, data);
  SPI5_CS = 1;
  return ret;
}

int port_spi::SPI_WriteOneRegister(uint8_t addr, uint8_t data) {
  uint8_t temp_rx = 0;
  int ret = 0;
  SPI5_CS = 0;
  ret = SPI_ReadWriteByte(addr & 0x7F, &temp_rx);
  ret |= SPI_ReadWriteByte(data, &temp_rx);
  SPI5_CS = 1;
  return ret;
}

bool port_spi::WriteByte(uint8_t reg, uint8_t val) {
  if (SPI_WriteOneRegister((reg), val)) {
    return false;
  }
  HAL_Delay(15);
  return true;
}

int port_spi::SPI_ReadMultRegister(uint8_t addr, uint8_t* buf, uint16_t len) {
  if (buf == NULL) {
    return -1;
  }
  uint8_t tx_buf[len];
  uint8_t add = addr | 0x80;
  memset0(tx_buf, 0, len);
  SPI5_CS = 0;
  HAL_SPI_Transmit(&spi_Handle, &add, 1, 1000);
  HAL_SPI_TransmitReceive(&spi_Handle, tx_buf, buf, len, 1000);
  SPI5_CS = 1;
  return 0;
}

bool port_spi::ReadBlock(uint8_t first_reg, uint8_t buf[], int len) {
  if (SPI_ReadMultRegister(first_reg, buf, len)) {
    return false;
  }

  return true;
}

bool port_spi::WriteMask(uint8_t reg_addr, uint8_t reg_value, uint8_t mask) {
  uint8_t rw_buffer = 0;

  if (SPI_ReadOneRegister(reg_addr, &rw_buffer)) {
    return false;
  }
  /* generate new value */
  rw_buffer = (rw_buffer & (~mask)) | (reg_value & mask);
  /* write new value to this register */
  if (SPI_WriteOneRegister(reg_addr, rw_buffer)) {
    return false;
  }
  return true;
}

bool port_spi::ReadRegister(uint8_t addr, uint8_t* data) { return SPI_ReadOneRegister(addr, data); }

bool port_spi::WriteRegister(uint8_t addr, uint8_t data) { return SPI_WriteOneRegister(addr, data); }

bool port_spi::ReadRegisters(uint8_t addr, uint8_t* data, int len)
{
    return ReadBlock(addr,data,len);
}