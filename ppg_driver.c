// ppg_driver.c
#include "ppg_driver.h"
#include "i2c_communication.h"
#include <sys/printk.h>

#define MAX30102_I2C_ADDR 0x57

int read_ppg_data(uint8_t *buffer) {
    int ret = i2c_write_read(i2c_dev, MAX30102_I2C_ADDR, (uint8_t[]){0x07}, 1, buffer, 6);
    if (ret != 0) {
        printk("PPG I2C read error: %d\n", ret);
    }
    return ret;
}