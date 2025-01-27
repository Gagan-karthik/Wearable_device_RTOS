#ifndef PPG_DRIVER_H
#define PPG_DRIVER_H

#include <zephyr.h>
#include <drivers/i2c.h>

int read_ppg_data(uint8_t *buffer);

#endif