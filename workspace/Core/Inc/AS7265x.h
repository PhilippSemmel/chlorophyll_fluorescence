/*
 * AS7265x.h
 *
 *  Created on: Nov 16, 2024
 *      Author: Philipp S.
 */

#ifndef INC_AS7265X_H_
#define INC_AS7265X_H_

#include "AS7265x_def.h"
#include "main.h"

int8_t AS7265_read_status(const struct AS7265_dev *dev, uint8_t *status);

int8_t AS7265_wait_for_tx_valid(const struct AS7265_dev *dev);
int8_t AS7265_wait_for_rx_valid(const struct AS7265_dev *dev);
int8_t AS7265_read_virtual_reg(const struct AS7265_dev *dev, uint8_t reg,
    uint8_t *data);

#endif /* INC_AS7265X_H_ */
