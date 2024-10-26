

#ifndef RC522_MAIN_MODULE_H
#define RC522_MAIN_MODULE_H


#include <linux/init.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/mutex.h>

#include <linux/types.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>

#define DEVICE_NAME "rc522"
#define CLASS_NAME "rc522_class"

struct rc522_data
{
    dev_t devt;
    struct device *dev;
    struct cdev cdev;
    struct mutex buf_lock;
    struct gpio_desc *reset_gpio;
};

#endif // RC522_MAIN_MODULE_H