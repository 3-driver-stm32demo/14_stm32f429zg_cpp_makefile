#include <stdio.h>
#include "usart.h"
#include <string>
#include "UserApp/userApp.h"
#include "stm32f4xx_hal.h"
#include "UserApp/DevalIMU.hpp"
#include "UserApp/tool.hpp"

double a = 3.1415926f;
char mychar[100];

deval::IMUService imu_service_;

void setup() {
	imu_service_.Start();
    printf("run setup\r\n");
}

void loop() {
	imu_service_.Loop();
    std::string str = "hello c++ loop";
    printf("%s\r\n", str.c_str());
    printf("test float printf\r\n");
    py_f2s4printf(mychar,a,sizeof(a));
    printf("%s\r\n",mychar);
    HAL_Delay(1000);
}