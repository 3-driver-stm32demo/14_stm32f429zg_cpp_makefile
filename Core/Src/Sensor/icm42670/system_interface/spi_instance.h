#ifndef __SPI_INSTANCE_H__
#define __SPI_INSTANCE_H__

#ifdef __cplusplus
extern "C" {
#endif

int inv_spi_master_init(void);

int inv_spi_master_read_register(unsigned spi_num, uint8_t register_addr,
		unsigned short len, uint8_t *value);


int inv_spi_master_write_register(unsigned spi_num, uint8_t register_addr,
		unsigned short len, uint8_t *value);

        
#ifdef __cplusplus
}
#endif

#endif