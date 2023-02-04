/*
 * std.h
 *
 * Created: 19-02-2020 14:27:09
 *  Author: mikke
 */ 


#ifndef STD_H_
#define STD_H_

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )

// the prescaler is set so that timer0 ticks every 64 clock cycles, and the
// the overflow handler is called every 256 ticks.
#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))

// the whole number of milliseconds per timer0 overflow
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)

#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

#define accel_i2c_adr_a0 0x38
#define accel_i2c_adr_a0_read 0x39
#define accel_i2c_adr_a0_write 0x38
#define accel_i2c_adr_a1_read 0x3B
#define accel_i2c_adr_a1_write 0x3A
#define accel_i2c_adr_a1 0x3A
#define OUT_X_MSB 0x01
#define OUT_X_LSB 0x02
#define OUT_Y_MSB 0x03
#define OUT_Y_LSB 0x04
#define OUT_Z_MSB 0x05
#define OUT_Z_LSB 0x06
#define F_SETUP 0x09
#define XYZ_DATA_CFG 0x0E
#define CTRL_REG1 0x2A
#define CTRL_REG2 0x2B

#define OFF_X 0x30
#define OFF_Y 0x31
#define OFF_Z 0x0D

#define BAUDRATE 9600
#define BAUD_PRESCALER ((F_CPU/(16UL*BAUDRATE))-1)

void initialse();
unsigned long millis(void);
unsigned int read_adc(char);
void register_setup();
void get_data_accel(int *x, int *y, int *z);
void get_data_accel_1(float *x_1, float *y_1, float *z_1);
//void accel_calibrate();
void MMA8451_init();
void MMA8451_init_1();
//void i2c_write_reg(char device, char reg, char data);
//char i2c_read_reg(char device, char reg);
void transmit_data_with_checksum_xor(int arr[]);

#endif /* STD_H_ */