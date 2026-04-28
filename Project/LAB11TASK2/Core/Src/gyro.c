#include "gyro.h"

extern SPI_HandleTypeDef hspi1;

void gyro_write(uint8_t reg, uint8_t value)
{
  uint8_t tx[2] = {reg, value};

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
}

void gyro_init()
{
  gyro_write(CTRL_REG1, 0b10001111);
  gyro_write(CTRL_REG4, 0b00000000);
}

uint8_t gyro_read(uint8_t reg)
{
  uint8_t tx = reg | 0x80;
  uint8_t rx;

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &tx, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi1, &rx, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

  return rx;
}