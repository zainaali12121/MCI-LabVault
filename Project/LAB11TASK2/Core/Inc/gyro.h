#ifndef GYRO_H
#define GYRO_H

#include "main.h"
#include <stdint.h>

#define CTRL_REG1 0x20
#define CTRL_REG4 0x23
#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D

void gyro_write(uint8_t reg, uint8_t value);
void gyro_init(void);
uint8_t gyro_read(uint8_t reg);

#endif