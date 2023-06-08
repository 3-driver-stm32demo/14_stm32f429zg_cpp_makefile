#pragma once
#include "Sensor/icm42670/example_raw/icm42670_sdk_main.h"
#include "stm32f4xx_hal.h"

namespace drvf {

class icm42670
{
private:
    /* data */
public:
    icm42670(/* args */);
    ~icm42670();

    int init(void);
    void read_data_loop(void);
};

icm42670::icm42670(/* args */)
{
}

icm42670::~icm42670()
{
}

int icm42670::init(void)
{
    return icm42670_sdk_main_init();
}

void icm42670::read_data_loop(void)
{
    while (1)
    {
        HAL_Delay(10);
        icm42670_sdk_main_data_read();
    }
}

}