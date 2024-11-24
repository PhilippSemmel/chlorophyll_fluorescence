/*
 * AS7265x.c
 *
 *  Created on: Nov 16, 2024
 *      Author: Philipp S.
 */

#include "AS7265x.h"

int8_t AS7265_read_status(const struct AS7265_dev *dev, uint8_t *status) {
  return dev->read(dev->devAddr, AS7265_I2C_SLAVE_STATUS_REG, status, 1);
}

int8_t AS7265_wait_for_tx_valid(const struct AS7265_dev *dev) {
  uint8_t status;

  while (1) {
    if (AS7265_read_status(dev, &status) == AS7265_I2C_SUCCESS) {
      if ((status & AS7265_I2C_SLAVE_TX_VALID) == 0) {
        break;
      }
    } else {
      return AS7265_I2C_COMM_ERROR;
    }
  }
  return AS7265_I2C_SUCCESS;
}

int8_t AS7265_wait_for_rx_valid(const struct AS7265_dev *dev) {
  uint8_t status;

  while (1) {
    if (AS7265_read_status(dev, &status) == AS7265_I2C_SUCCESS) {
      if ((status & AS7265_I2C_SLAVE_TX_VALID) != 0) {
        break;
      }
    } else {
      return AS7265_I2C_COMM_ERROR;
    }
  }
  return AS7265_I2C_SUCCESS;
}

int8_t AS7265_read_virtual_reg(const struct AS7265_dev *dev, uint8_t reg,
    uint8_t *data) {
  // wait for the device to be ready for write
  if (AS7265_wait_for_tx_valid(dev) != AS7265_I2C_SUCCESS)
    return AS7265_I2C_COMM_ERROR;

  // Send the virtual register address (disabling bit 7 to indicate a read).
  if (dev->write(dev->devAddr, AS7265_I2C_SLAVE_WRITE_REG, &reg,
      1) != AS7265_I2C_SUCCESS)
    return AS7265_I2C_COMM_ERROR;

  // wait for the device to be ready for read
  if (AS7265_wait_for_rx_valid(dev) != AS7265_I2C_SUCCESS)
    return AS7265_I2C_COMM_ERROR;

  // Read the data to complete the operation.
  if (dev->read(dev->devAddr, AS7265_I2C_SLAVE_READ_REG, data,
      1) != AS7265_I2C_SUCCESS)
    return AS7265_I2C_COMM_ERROR;

  return AS7265_I2C_SUCCESS;
}

