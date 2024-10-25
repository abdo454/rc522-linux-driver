
#include <linux/init.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/mutex.h>
// #include <linux/property.h>
#include <linux/types.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include "rc522_api.h"

#define DEVICE_NAME "rc522"
#define CLASS_NAME "rc522_class"

static dev_t devt;
static struct class *rc522_class;
struct rc522_data
{
    dev_t devt;
    struct device *dev;
    struct cdev cdev;
    struct mutex buf_lock;
    struct gpio_desc *reset_gpio;
};

int rc522_open(struct inode *, struct file *);
int rc522_release(struct inode *, struct file *);
ssize_t rc522_read(struct file *, char __user *, size_t, loff_t *);
ssize_t rc522_write(struct file *, const char __user *, size_t, loff_t *);
const struct file_operations rc522_fops = {
    .owner = THIS_MODULE,
    .open = rc522_open,
    .release = rc522_release,
    .read = rc522_read,
    .write = rc522_write,
};

int rc522_open(struct inode *inode, struct file *file)
{
    printk("%s: Device opened\n", DEVICE_NAME);
    return 0;
}

int rc522_release(struct inode *inode, struct file *file)
{

    printk("%s: Device closed\n", DEVICE_NAME);
    return 0;
}

ssize_t rc522_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
    // printk("%s: Read from device count %lu ,offset %llu\n", DEVICE_NAME, count, offset);
    return 0;
}

ssize_t rc522_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
    // printk("%s: Write to device count %lu ,offset %llu\n", DEVICE_NAME, count, offset);
    return count;
}
/*-------------------------------------------------------------------------*/

static int rc522_probe(struct spi_device *spi)
{
    struct rc522_data *rc522;
    struct device *dev;
    int ret;
    /* Allocate driver data */
    rc522 = devm_kzalloc(&spi->dev, sizeof(*rc522), GFP_KERNEL);
    if (!rc522)
        return -ENOMEM;
    // /* Check for device properties */
    // if (!device_property_present(&spi->dev, "reset-gpio-pin"))
    // {
    //     dev_err(&spi->dev, " Error! Device property 'reset-gpio-pin' not found!\n");
    //     return -EINVAL;
    // }
    // ret = device_property_read_u32(&spi->dev, "reset-gpio-pin", &rc522->reset_pin);
    // if (ret)
    // {
    //     dev_err(&spi->dev, "Error! Could not read 'reset-gpio-pin'\n");
    //     return ret;
    // }
    // if (!gpio_is_valid(rc522->reset_pin))
    // {
    //     dev_err(&spi->dev, "Reset GPIO %d is not valid\n", rc522->reset_pin);
    //     return -1;
    // }
    rc522->reset_gpio = devm_gpiod_get_optional(&spi->dev, "reset", GPIOD_OUT_HIGH);
    if (IS_ERR(rc522->reset_gpio))
    {
        ret = PTR_ERR(rc522->reset_gpio);
        dev_err(&spi->dev, "Failed to get reset GPIO: %d\n", ret);
        return ret;
    }
    if (rc522->reset_gpio)
    {

        gpiod_set_value(rc522->reset_gpio, 0);
        msleep(50);
        gpiod_set_value(rc522->reset_gpio, 1);
    }
    // if (gpio_request(rc522->reset_pin, "rc522_reset") < 0)
    // if (devm_gpio_request(&spi->dev, rc522->reset_pin, "rc522_reset") < 0)
    // {
    //     dev_err(&spi->dev, "ERROR: Reset GPIO %d request\n", rc522->reset_pin);
    //     return -1;
    // }
    // gpio_direction_output(rc522->reset_pin, 1);

    /* Initialize the driver data */
    rc522->devt = devt;
    rc522->dev = &spi->dev;
    mutex_init(&rc522->buf_lock);

    /* Initialize cdev */
    cdev_init(&rc522->cdev, &rc522_fops);
    rc522->cdev.owner = THIS_MODULE;

    /* Add cdev to the system */
    ret = cdev_add(&rc522->cdev, rc522->devt, 1);
    if (ret)
    {
        dev_err(&spi->dev, "Failed to add cdev\n");
        goto err_cdev_add;
    }
    /* Create device node */
    dev = device_create(rc522_class, &spi->dev, rc522->devt, rc522, DEVICE_NAME);
    if (IS_ERR(dev))
    {
        dev_err(&spi->dev, "Failed to create device node\n");
        ret = PTR_ERR(dev);
        goto err_device_create;
    }
    /* Save driver data */
    spi_set_drvdata(spi, rc522);

    /* Initialize RC522 hardware */
    // ret = rc522_hw_init(spi);
    if (ret)
    {
        dev_err(&spi->dev, "Failed to initialize RC522: %d\n", ret);
        goto err_hw_init;
    }

    dev_info(&spi->dev, "RC522 probe function completed successfully\n");

    return 0;
err_hw_init:
    device_destroy(rc522_class, rc522->devt);
err_device_create:
    cdev_del(&rc522->cdev);
err_cdev_add:
    // gpio_free(rc522->reset_pin);
    return ret;
}

static void rc522_remove(struct spi_device *spi)
{
    struct rc522_data *rc522 = spi_get_drvdata(spi);
    if (rc522->reset_gpio)
        gpiod_direction_input(rc522->reset_gpio);
    device_destroy(rc522_class, rc522->devt);
    cdev_del(&rc522->cdev);
    dev_info(&spi->dev, "Device removed successfully\n");
}

/*-------------------------------------------------------------------------*/
static const struct of_device_id rc522_of_match[] = {
    {.compatible = "nxp,rc522"},
    {}};

static struct spi_driver rc522_spi_driver = {
    .driver = {
        .name = "rc522",
        .owner = THIS_MODULE,
        .of_match_table = rc522_of_match,
    },
    .probe = rc522_probe,
    .remove = rc522_remove,
};

/*-------------------------------------------------------------------------*/

static int __init rc522_init(void)
{
    int result;
    /* Allocate character device numbers */
    result = alloc_chrdev_region(&devt, 0, 1, DEVICE_NAME);
    if (result < 0)
    {
        pr_err("Failed to allocate character device region\n");
        return result;
    }
    pr_info("Allocated Major = %d Minor = %d\n", MAJOR(devt), MINOR(devt));

    /* Create device class */
    rc522_class = class_create(CLASS_NAME);
    if (IS_ERR(rc522_class))
    {
        pr_err("Failed to create device class '%s'\n", CLASS_NAME);
        result = PTR_ERR(rc522_class);
        goto unregister_chrdev;
    }
    /* Register the SPI driver */
    result = spi_register_driver(&rc522_spi_driver);
    if (result < 0)
    {
        pr_err("Failed to register SPI driver\n");
        goto destroy_class;
    }
    pr_info("%s SPI driver registered successfully\n", DEVICE_NAME);
    return 0;
destroy_class:
    class_destroy(rc522_class);
unregister_chrdev:
    unregister_chrdev_region(devt, 1);
    return result;
}

static void __exit rc522_exit(void)
{

    spi_unregister_driver(&rc522_spi_driver);
    class_destroy(rc522_class);
    unregister_chrdev_region(devt, 1);
    printk("%s: SPI driver unregistered and module exited\n", DEVICE_NAME);
}

module_init(rc522_init);
module_exit(rc522_exit);

MODULE_DEVICE_TABLE(of, rc522_of_match);
// MODULE_ALIAS("spi:rc522");

MODULE_AUTHOR("Daood Abdo abdo.daood94@gmail.com");
MODULE_DESCRIPTION("A Linux SPI Device Driver for Module RC522.");
MODULE_VERSION("1.1");
MODULE_LICENSE("GPL");