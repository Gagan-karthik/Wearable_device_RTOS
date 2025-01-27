
#include "i2c_communication.h"
#include <drivers/i2c.h>
#include <sys/printk.h>

#define I2C_DEV_LABEL "I2C_0"

static const struct device *i2c_dev;

const struct device *i2c_init(void) {
    i2c_dev = device_get_binding(I2C_DEV_LABEL);
    if (!i2c_dev) {
        printk("No I2C device found\n");
    }
    return i2c_dev;
}