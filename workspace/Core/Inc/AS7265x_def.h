/*
 * AS7265x_def.h
 *
 *  Created on: Nov 18, 2024
 *      Author: Philipp S.
 */

#include <stdint.h>

#ifndef INC_AS7265X_DEF_H_
#define INC_AS7265X_DEF_H_

#define AS7265_I2C_ADDR                   0x49

#define AS7265_I2C_SLAVE_STATUS_REG       0x00
#define AS7265_I2C_SLAVE_WRITE_REG        0x01
#define AS7265_I2C_SLAVE_READ_REG         0x02

#define AS7265_HW_VERSION_H               0x02
#define AS7265_HW_VERSION_L               0x03

/**\name Channel R,G,A */
#define AS7265_RAW_VALUE_0_H              0x08
#define AS7265_RAW_VALUE_0_L              0x09

/**\name Channel S,H,B */
#define AS7265_RAW_VALUE_1_H              0x0A
#define AS7265_RAW_VALUE_1_L              0x0B

/**\name Channel T,I,C */
#define AS7265_RAW_VALUE_2_H              0x0C
#define AS7265_RAW_VALUE_2_L              0x0D

/**\name Channel U,J,D */
#define AS7265_RAW_VALUE_3_H              0x0E
#define AS7265_RAW_VALUE_3_L              0x0F

/**\name Channel V,K,E */
#define AS7265_RAW_VALUE_4_H              0x10
#define AS7265_RAW_VALUE_4_L              0x11

/**\name Channel W,L,F */
#define AS7265_RAW_VALUE_5_H              0x12
#define AS7265_RAW_VALUE_5_L              0x13

#define AS7265_FW_CNTRL                   0x48
#define AS7265_FW_BYTE_COUNT_H            0x49
#define AS7265_FW_BYTE_COUNT_L            0x4A
#define AS7265_DEV_SEL                    0x4F

/**\name selectable devices */
#define AS7265_DEVICE_GREEN               0b00
#define AS7265_DEVICE_RED                 0b01
#define AS7265_DEVICE_BLUE                0b10

#define AS7265_I2C_SLAVE_TX_VALID         0x02
#define AS7265_I2C_SLAVE_RX_VALID         0x01

#define AS7265_I2C_INVALID_ARGUMENT_ERROR 2
#define AS7265_I2C_COMM_ERROR             1
#define AS7265_I2C_SUCCESS                0


typedef int8_t (*as7265_com_fptr_t)(uint8_t dev_addr, uint8_t reg_addr,
    uint8_t *data, uint16_t size);

struct AS7265_dev
{
  /*! Device I2C Address */
  uint8_t devAddr;
  /*! Read function pointer */
  as7265_com_fptr_t write;
  /*! Write function pointer */
  as7265_com_fptr_t read;
};

#endif /* INC_AS7265X_DEF_H_ */
