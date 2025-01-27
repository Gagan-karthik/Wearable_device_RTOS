#include "imu_driver.h"
#include "i2c_communication.h"
#include <sys/printk.h>

#define BMI270_I2C_ADDR 0x68

int read_imu_data(uint8_t *buffer) {
    int ret = i2c_write_read(i2c_get_device(), BMI270_I2C_ADDR, (uint8_t[]){0x0C}, 1, buffer, 12);
    if (ret != 0) {
        printk("IMU I2C read error: %d\n", ret);
    }
    return ret;
}
