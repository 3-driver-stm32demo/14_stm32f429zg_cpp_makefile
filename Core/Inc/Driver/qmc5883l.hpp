#ifndef __QMC5883_H
#define __QMC5883_H

#include "stdio.h"
#include "Driver/Port_iic.hpp"

typedef struct qmc5883l_s
{
    double Angle_XY;
    double Angle_XZ;
    double Angle_YZ;
    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;
}qmc5883l_t;

class qmc5883l
{
private:
    /* data */
    port_iic port_iic_;
    void Init_Reg_QMC5883();
public:
    // qmc5883l(/* args */);
    // ~qmc5883l();

    void qmc5883l_init(I2C_TypeDef* iic_id,uint32_t iic_speed,uint8_t addr_dev);
    qmc5883l_t qmc5883l_read();
    void qmc5883l_printf_loop();

};

// qmc5883l::qmc5883l(/* args */)
// {
// }

// qmc5883l::~qmc5883l()
// {
// }

#endif