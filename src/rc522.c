
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/kdev_t.h>
#include <linux/device.h>

// #include <linux/spi/spi.h>
#include "rc522.h"

#define DEVICE_NAME "rc522_driver"
#define DEVICE_NODE_NAME "rc522"
dev_t dev = 0;
static struct class *dev_class;

static int __init rc522_init(void)
{
    int result;
    // 1. allocate Major numner and Minor # for rc522 device driver
    result = alloc_chrdev_region(&dev, 0, 1, DEVICE_NAME);
    if (result < 0)
    {
        printk(KERN_ALERT "Failed to register character device\n");
        return result;
    }
    pr_info("Major = %d Minor = %d \n", MAJOR(dev), MINOR(dev));
    // 2. Create a device node file in /dev directory as character device
    dev_class = class_create(DEVICE_NODE_NAME);
    if (IS_ERR(dev_class))
    {
        pr_err("Cannot create the struct class for device %s\n", DEVICE_NODE_NAME);
        goto r_class;
    }
    if (IS_ERR(device_create(dev_class, NULL, dev, NULL, DEVICE_NODE_NAME)))
    {
        pr_err("Cannot create the Device\n");
        goto r_device;
    }
    pr_info("Kernel Module Inserted Successfully...\n");
    return 0;
r_device:
    class_destroy(dev_class);
r_class:
    unregister_chrdev_region(dev, 1);
    return -1;
}

static void rc522_exit(void)
{

    device_destroy(dev_class, dev);
    class_destroy(dev_class);
    unregister_chrdev_region(dev, 1);
    pr_info("Kernel Module Removed Successfully...\n");
}

module_init(rc522_init);
module_exit(rc522_exit);

MODULE_AUTHOR("Daood Abdo abdo.daood94@gmail.com");
MODULE_DESCRIPTION("A Linux SPI Device Driver for Module RC522.");
MODULE_VERSION("1.1");
MODULE_LICENSE("GPL");