#ifndef IMU_LSM_H
#define IMU_LSM_H

#include "main.h"
#include <stdint.h>
#include <math.h>

typedef struct {
float accX;
float accY;
float accZ;
float gyroX;
float gyroY;
float gyroZ;
float roll;
float pitch;
} LSM_Offset_t;

extern LSM_Offset_t lsmOffset;

extern float accX, accY, accZ;
extern float offsetAccX;
extern float offsetAccY;
extern float offsetAccZ;

extern float roll_deg;

void Init_LSM(void);
void Read_LSM(void);
void OffsetLSM(void);

#endif