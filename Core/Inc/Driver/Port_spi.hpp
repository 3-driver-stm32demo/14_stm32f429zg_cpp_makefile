#ifndef __PORT_SPI_H
#define __PORT_SPI_H

#include "stm32f4xx.h"

class port_spi {
 public:
  void spi_init(SPI_TypeDef* SPI_id, uint32_t speed);

  bool WriteByte(uint8_t reg, uint8_t val);
  bool WriteMask(uint8_t reg_addr, uint8_t reg_value, uint8_t mask);
  bool ReadBlock(uint8_t first_reg, uint8_t buf[], int len);

  SPI_HandleTypeDef spi_Handle;

 private:
  int SPI_ReadWriteByte(uint8_t TxData, uint8_t* Rxdata);
  int SPI_ReadOneRegister(uint8_t addr, uint8_t* data);
  int SPI_WriteOneRegister(uint8_t addr, uint8_t data);
  int SPI_ReadMultRegister(uint8_t addr, uint8_t* buf, uint16_t len);
};

#endif