#include "Driver/Port_spi.hpp"
//这里都是包裹函数，目的是使用c调用C++函数；

#ifdef __cplusplus
extern "C" {
#endif

port_spi port_;

int inv_spi_master_init(void)
{
    port_.spi_init(SPI5,10000000);
    return 0;
}

int inv_spi_master_read_register(unsigned spi_num, uint8_t register_addr,
		unsigned short len, uint8_t *value)
{
    if(port_.ReadRegisters(register_addr,value,len))
    {
        return 0;
    }
    else
    {
        return -1;
    }
}

int inv_spi_master_write_register(unsigned spi_num, uint8_t register_addr,
		unsigned short len, uint8_t *value)
{
    if(!port_.SPI_WriteMultRegister(register_addr,value,len))
    {
        return 0;
    }
    else
    {
        return -1;
    }
}

#ifdef __cplusplus
}
#endif