

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
#define INFO_BUFFER_SIZE 32
#define CARD_ID_SIZE 4

struct rc522_data
{
    dev_t devt;
    struct spi_device *spi_dev;
    struct cdev cdev;
    // struct mutex buf_lock;
    struct gpio_desc *reset_gpio;
    u8 version;
};
struct rc522_card_info
{
    u8 uid_length;
    u8 tx_buffer[INFO_BUFFER_SIZE];
    u8 rx_buffer[INFO_BUFFER_SIZE];
    u8 version;
    bool initialized;
};

#endif // RC522_MAIN_MODULE_H