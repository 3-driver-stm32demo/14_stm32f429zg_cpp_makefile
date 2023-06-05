#include "Driver/qmc5883l.hpp"
#include "math.h"
#include "usart.h"

#define PRINTF_ANGLE 1
#define PRINTF_RAW_DATA 1

//初始化QMC5883，根据需要请参考pdf进行修改****
void qmc5883l::Init_Reg_QMC5883()
{
    uint8_t reg_data[4] = {0x0d,0x01,0x40,0x01};
	port_iic_.iic_write_Byte(&reg_data[0],0x09);  //控制寄存器配置
	port_iic_.iic_write_Byte(&reg_data[1],0x0b);  //设置清除时间寄存器
	port_iic_.iic_write_Byte(&reg_data[2],0x20);  //
	port_iic_.iic_write_Byte(&reg_data[3],0x21);  //	
}

void qmc5883l::qmc5883l_init(I2C_TypeDef* iic_id,uint32_t iic_speed,uint8_t addr_dev)
{
    port_iic_.iic_init(iic_id,iic_speed);
    port_iic_.iic_set_dev_addr(addr_dev);
    Init_Reg_QMC5883();
    HAL_Delay(100);
}

qmc5883l_t qmc5883l::qmc5883l_read()
{
    uint8_t buf[6];
    int16_t X=0,Y=0,Z=0;
    qmc5883l_t qmc5883l_;
    port_iic_.iic_read_buf(buf,0x00,6);

    X=buf[1] << 8 | buf[0]; //Combine MSB and LSB of X Data output register  最高有效位
    Y=buf[3] << 8 | buf[2]; //Combine MSB and LSB of Y Data output register
    Z=buf[5] << 8 | buf[4]; //Combine MSB and LSB of Z Data output register

    if(X>0x7fff)X-=0xffff;	  
    if(Y>0x7fff)Y-=0xffff;
    if(Z>0x7fff)Z-=0xffff;	 
	
	qmc5883l_.raw_x = (int16_t)X;
    qmc5883l_.raw_y = (int16_t)Y;
    qmc5883l_.raw_z = (int16_t)Z;

    qmc5883l_.Angle_XY= atan2((double)Y,(double)X) * (180 / 3.14159265) + 180; //计算XY平面角度
	//qmc5883l_.Angle_XY*=10;
    qmc5883l_.Angle_XZ= atan2((double)Z,(double)X) * (180 / 3.14159265) + 180; //计算XZ平面角度
    //qmc5883l_.Angle_XZ*=10;
    qmc5883l_.Angle_YZ= atan2((double)Z,(double)Y) * (180 / 3.14159265) + 180; //计算YZ平面角度
    return qmc5883l_;
}

void qmc5883l::qmc5883l_printf_loop()
{
    qmc5883l_t qmc5883l_printf_;
    while (1)
    {
        qmc5883l_printf_ = qmc5883l_read();
        #if PRINTF_ANGLE
            printf("Angle_XY Angle_XZ Angle_YZ %3.2f %3.2f %3.2f\r\n",qmc5883l_printf_.Angle_XY,qmc5883l_printf_.Angle_XZ,qmc5883l_printf_.Angle_YZ);
        #endif

        #if PRINTF_RAW_DATA
            printf("raw_x raw_y raw_z %d %d %d\r\n",qmc5883l_printf_.raw_x,qmc5883l_printf_.raw_y,qmc5883l_printf_.raw_z);
        #endif

        HAL_Delay(10);
    }
}