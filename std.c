
#include "std.h"
#include "i2cmaster.h"
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "Compasslib.h"
void MMA8451_init()
{
	i2c_write_reg(accel_i2c_adr_a0, CTRL_REG1, 0x02);
	i2c_write_reg(accel_i2c_adr_a0, CTRL_REG2, 0x02);
	i2c_write_reg(accel_i2c_adr_a0, XYZ_DATA_CFG, 0x02);
	i2c_write_reg(accel_i2c_adr_a0, F_SETUP, 0x00);
	i2c_write_reg(accel_i2c_adr_a0, CTRL_REG1, 0x03);
}

void MMA8451_init_1()
{
	i2c_write_reg(accel_i2c_adr_a1, CTRL_REG1, 0x02); //enables fast read mode
	i2c_write_reg(accel_i2c_adr_a1, CTRL_REG2, 0x02);
	i2c_write_reg(accel_i2c_adr_a1, XYZ_DATA_CFG, 0x00); //0x02
	i2c_write_reg(accel_i2c_adr_a1, F_SETUP, 0x40);
	i2c_write_reg(accel_i2c_adr_a1, CTRL_REG1, 0x03);
}


void get_data_accel(int *x, int *y, int *z)
{
	i2c_start(accel_i2c_adr_a0_write);
	i2c_write(OUT_X_MSB);
	i2c_start(accel_i2c_adr_a0_read);
	*x = (i2c_readAck()<<8);
	*y = (i2c_readAck()<<8);
	*z = (i2c_readNak()<<8);
	i2c_stop();
}

void get_data_accel_1(float *x_1, float *y_1, float *z_1)
{
	
	i2c_start(accel_i2c_adr_a1_write);
	i2c_write(OUT_X_MSB); //write to memory
	i2c_start(accel_i2c_adr_a1_read);
	*x_1 = (i2c_readAck()<<8);
	*y_1 = (i2c_readAck()<<8);
	*z_1 = (i2c_readNak()<<8);
	i2c_stop();
}
/*
void accel_calibrate()
{
	char x, y, z;
	i2c_start(accel_i2c_adr_a0_write);
	i2c_write(OUT_X_MSB);
	i2c_start(accel_i2c_adr_a0_read);
	x = i2c_readAck();
	y = i2c_readAck();
	z = i2c_readNak();
	i2c_stop();
	i2c_start(accel_i2c_adr_a0_write);
	i2c_write(OFF_X);
	i2c_write(-x);
	i2c_stop();
	i2c_start(accel_i2c_adr_a0_write);
	i2c_write(OFF_Y);
	i2c_write(-y);
	i2c_stop();
	i2c_start(accel_i2c_adr_a0_write);
	i2c_write(OFF_Z);
	i2c_write(4096-z);
	i2c_stop();
}*/
/*
void i2c_write_reg(char device, char reg, char data)
{
	i2c_start(device + I2C_WRITE);
	i2c_write(reg);
	i2c_write(data);
	i2c_stop();
}
char i2c_read_reg(char device, char reg)
{
	i2c_start(device + I2C_WRITE);
	i2c_write(reg);
	i2c_start(device + I2C_READ);
	char data = i2c_readNak();
	i2c_stop();
	return data;
}
*/
