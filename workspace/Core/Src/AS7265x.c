/*
 * AS7265x.c
 *
 *  Created on: Nov 16, 2024
 *      Author: Philipp S.
 */


//HAL_StatusTypeDef A7265x_Read(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data, uint16_t size)
//{
//    HAL_StatusTypeDef status;
//
//    status = HAL_I2C_Master_Transmit(hi2c, (uint16_t)(A7265x_I2C_MAXSTER_WRITE_ADDR), &reg, 1, 1000);
//    if (status != HAL_OK)
//    {
//        return status;  // If there's an issue with writing the register address, return the error
//    }
//
//    // Then, read the data from the module
//    return HAL_I2C_Master_Receive(hi2c, (uint16_t)(A7265x_I2C_MAXSTER_READ_ADDR), data, size, 1000);
//}
