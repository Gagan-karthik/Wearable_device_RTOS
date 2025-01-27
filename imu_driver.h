#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H
#include <zephyr.h>
#include <drivers/i2c.h>

int read_imu_data(uint8_t *buffer);

#endif