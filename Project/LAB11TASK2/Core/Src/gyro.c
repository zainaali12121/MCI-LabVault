#include "gyro.h"
// * This function performs a standard SPI write:
//* - Pull CS (chip select) LOW to start communication
// * - Send register address + data
// * - Pull CS HIGH to end communication
// SPI handle used for communication with gyroscope
extern SPI_HandleTypeDef hspi1;

void gyro_write(uint8_t reg, uint8_t value)
{
  uint8_t tx[2] = {reg, value};   // First byte = register, second = data

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
}

/**
 * @brief  Initialize gyroscope with desired configuration
 * 
 * CTRL_REG1:
 * - Enables X, Y, Z axes
 * - Sets output data rate and bandwidth
 * 
 * CTRL_REG4:
 * - Sets full-scale range (e.g., ±250 dps)
 * - Defines sensitivity of gyro readings
 */
void gyro_init()
{
  gyro_write(CTRL_REG1, 0b10001111);    // Power ON + enable all axes
  gyro_write(CTRL_REG4, 0b00000000);    // Default scale (±250 dps)
}

uint8_t gyro_read(uint8_t reg)
{
  uint8_t tx = reg | 0x80;    // Set MSB to indicate read operation
  uint8_t rx;

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);   // CS LOW , start communication
  HAL_SPI_Transmit(&hspi1, &tx, 1, HAL_MAX_DELAY);     // Send register address
  HAL_SPI_Receive(&hspi1, &rx, 1, HAL_MAX_DELAY);      // Receive data
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);   // CS HIGH , end communication

  return rx;
}
