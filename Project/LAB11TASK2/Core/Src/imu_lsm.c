#include "imu_lsm.h"
#include "main.h"

#include "gyro.h"

extern I2C_HandleTypeDef hi2c1;

LSM_Offset_t lsmOffset;

float accX, accY, accZ;
float offsetAccX = 0;
float offsetAccY = 0;
float offsetAccZ = 0;
float roll_deg = 0;

void Init_LSM(void)
{
  uint8_t data;

  data = 0x67;
  HAL_I2C_Mem_Write(&hi2c1, 0x33, 0x20, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

  data = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, 0x33, 0x23, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

void Read_LSM(void)
{
    uint8_t data[6];

    HAL_I2C_Mem_Read(&hi2c1, 0x33, 0x28 | 0x80,
                     I2C_MEMADD_SIZE_8BIT, data, 6, HAL_MAX_DELAY);

    int16_t x_raw = (int16_t)((data[1] << 8) | data[0]);
    int16_t y_raw = (int16_t)((data[3] << 8) | data[2]);
    int16_t z_raw = (int16_t)((data[5] << 8) | data[4]);

    accX = (x_raw - lsmOffset.accX) * 3.9f / 1000.0f;
    accY = (y_raw - lsmOffset.accY) * 3.9f / 1000.0f;
    accZ = (z_raw - lsmOffset.accZ) * 3.9f / 1000.0f;

    roll_deg = atan2f(accX, accZ) * 57.2958f;
}

void OffsetLSM(void)
{
    float sumAccX = 0, sumAccY = 0, sumAccZ = 0;
    float sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;
    int samples = 60;

    for(int i = 0; i < samples; i++)
    {
        uint8_t data[6];

        HAL_I2C_Mem_Read(&hi2c1, 0x33, 0x28 | 0x80,
                         I2C_MEMADD_SIZE_8BIT, data, 6, HAL_MAX_DELAY);

        int16_t rawX = (int16_t)((data[1] << 8) | data[0]);
        int16_t rawY = (int16_t)((data[3] << 8) | data[2]);
        int16_t rawZ = (int16_t)((data[5] << 8) | data[4]);

        sumAccX += rawX * 3.9f / 1000.0f;
        sumAccY += rawY * 3.9f / 1000.0f;
        sumAccZ += rawZ * 3.9f / 1000.0f;

        int16_t gx_raw = (int16_t)((gyro_read(0x2B) << 8) | gyro_read(0x2A));
        int16_t gy_raw = (int16_t)((gyro_read(0x29) << 8) | gyro_read(0x28));
        int16_t gz_raw = (int16_t)((gyro_read(0x2D) << 8) | gyro_read(0x2C));

        sumGyroX += gx_raw * 0.00875f;
        sumGyroY += gy_raw * 0.00875f;
        sumGyroZ += gz_raw * 0.00875f;

        HAL_Delay(50);
    }

    lsmOffset.accX = sumAccX / samples;
    lsmOffset.accY = sumAccY / samples;
    lsmOffset.accZ = sumAccZ / samples;

    lsmOffset.gyroX = sumGyroX / samples;
    lsmOffset.gyroY = sumGyroY / samples;
    lsmOffset.gyroZ = sumGyroZ / samples;
}