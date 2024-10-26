// SPDX-License-Identifier: GPL-2.0

#include "rc522_main.h"
#include "rc522_api.h"

// Function to write a value to a register
// returns 0 on success, -ve on failure
static int rc522_write_register(struct spi_device *spi, u8 reg, u8 value)
{
    int ret;
    u8 tx_buf[2];

    tx_buf[0] = ((reg << 1) & 0x7E);
    tx_buf[1] = value;
    ret = spi_write(spi, tx_buf, 2);
    if (ret != 0)
        printk(KERN_ERR "RC522 Driver: failed to write reg: addr=0x%02x, value=0x%02x, ret = %d\n", reg, value, ret);
    return ret;
}

// Function to read a value from a register
// returns 0 on success, -ve on failure
// doesn't work
static int rc522_read_register2(struct spi_device *spi, u8 reg, u8 *value)
{
    u8 tx_buf[1] = {reg};
    u8 rx_buf[1];
    int ret;

    ret = spi_write_then_read(spi, tx_buf, 1, rx_buf, 1);
    if (ret < 0)
    {
        return ret;
    }
    *value = rx_buf[0];
    return 0;
}
// Function to read a value from a register
// returns 0 on success, -ve on failure
static int rc522_read_register(struct spi_device *spi, u8 reg, u8 *value)
{
    u8 tx_buf[2];
    u8 rx_buf[2];
    int ret;

    /* Construct the read command */
    tx_buf[0] = ((reg << 1) & 0x7E) | 0x80;
    tx_buf[1] = 0x00;

    struct spi_transfer t = {
        .tx_buf = tx_buf,
        .rx_buf = rx_buf,
        .len = 2,
    };
    struct spi_message m;
    spi_message_init(&m);
    spi_message_add_tail(&t, &m);

    ret = spi_sync(spi, &m);
    if (ret < 0)
    {
        dev_err(&spi->dev, "SPI read error: %d\n", ret);
        return ret;
    }

    *value = rx_buf[1];
    return 0;
}

// Function to perform a read/write operations on the RC522
static int rc522_test_write_read(struct spi_device *spi)
{
    u8 test_reg = SerialSpeedReg; // Any register that supports write/read operations, e.g., SerialSpeedReg
    u8 write_val = 0x74;          // 0x55 is arbitrary
    u8 read_val = 0x00;

    // Step 1: Write to the register
    int ret = rc522_write_register(spi, test_reg, write_val);
    if (ret)
    {
        printk(KERN_ERR "RC522 Driver: Write failed with error %d\n", ret);
        return ret;
    }

    // Step 2: Read back from the same register
    ret = rc522_read_register(spi, test_reg, &read_val);
    if (ret)
    {
        printk(KERN_ERR "RC522 Driver: Read failed with error %d\n", ret);
        return ret;
    }

    // Step 3: Compare the written value with the read value
    if (read_val == write_val)
    {
        ret = 0;
        printk(KERN_INFO "RC522 Driver: Write/Read test passed. Value: 0x%02x\n", read_val);
    }
    else
    {
        ret = -1;
        printk(KERN_ERR "RC522 Driver: Write/Read test failed. Written: 0x%02x, Read: 0x%02x\n", write_val, read_val);
    }
    return ret;
}

// Helper Functions for Bitmask Operations:
static int rc522_set_bitmask(struct spi_device *spi, u8 reg, u8 mask)
{
    u8 tmp;
    rc522_read_register(spi, reg, &tmp);
    tmp |= mask;
    return rc522_write_register(spi, reg, tmp);
}

static int rc522_clear_bitmask(struct spi_device *spi, u8 reg, u8 mask)
{
    u8 tmp;
    rc522_read_register(spi, reg, &tmp);
    tmp &= (~mask);
    return rc522_write_register(spi, reg, tmp);
}
/* -------------------------------------------------------------- */
//  check if an RFID card is present.

// rc522_transceive : handle the communication to and from the RC522.
static int rc522_transceive(struct spi_device *spi, u8 cmd, u8 *send_data, u8 send_len, u8 *back_data, u8 *back_len)
{
    int status = MI_ERR;
    u8 irq_en = 0x00; // Enable all interrupts
    u8 wait_irq = 0x00;
    u8 n;
    u32 i;
    switch (cmd)
    {
    case PCD_AUTHENT:
        irq_en = 0x12;
        wait_irq = 0x10;
        break;
    case PCD_TRANSCEIVE:
        irq_en = 0x77;   // Enable all interrupts
        wait_irq = 0x30; // RX and Idle interrupt
        break;
    default:
        break;
    }
    rc522_write_register(spi, CommIEnReg, irq_en | 0x80); // Enable interrupts
    rc522_clear_bitmask(spi, CommIrqReg, 0x80);           // Clear all interrupt flags
    rc522_set_bitmask(spi, FIFOLevelReg, 0x80);           // Flush FIFO buffer

    rc522_write_register(spi, CommandReg, PCD_IDLE); // No action, cancel current command

    /* Write data to FIFO */
    for (i = 0; i < send_len; i++)
    {
        rc522_write_register(spi, FIFODataReg, send_data[i]);
    }

    rc522_write_register(spi, CommandReg, cmd); // Start transmission
    if (cmd == PCD_TRANSCEIVE)
        rc522_set_bitmask(spi, BitFramingReg, 0x80); // Start Send

    /* Wait for the transmission to complete */
    i = 2000; // Timeout counter
    do
    {
        n = rc522_read_register(spi, CommIrqReg, &n);
        i--;
    } while ((i != 0) && !(n & 0x01) && !(n & wait_irq));

    rc522_clear_bitmask(spi, BitFramingReg, 0x80); // Stop Send

    if (i != 0)
    {
        u8 error;
        rc522_read_register(spi, ErrorReg, &error);
        if (!(error & 0x1B))
        {
            status = MI_OK;
            if (n & irq_en & 0x01)
                status = MI_NOTAGERR;
            if (cmd == PCD_TRANSCEIVE)
            {
                u8 fifo_level;
                u8 last_bits;
                rc522_read_register(spi, FIFOLevelReg, &fifo_level);
                rc522_read_register(spi, ControlReg, &last_bits);
                last_bits = last_bits & 0x07;
                if (last_bits)
                    *back_len = (fifo_level - 1) * 8 + last_bits;
                else
                    *back_len = fifo_level * 8;
                if (0 == fifo_level)
                    fifo_level = 1;
                if (fifo_level > DEF_FIFO_LENGTH)
                    fifo_level = DEF_FIFO_LENGTH;
                /* Read the data from FIFO */
                for (i = 0; i < fifo_level; i++)
                    rc522_read_register(spi, FIFODataReg, &back_data[i]);
            }
        }
        else
        {
            printk(KERN_ERR "RC522 Driver: failed to read irq reg: addr=0x%02x, value=0x%02x\n", CommIrqReg, n);
            status = MI_ERR;
        }
    }

    return status;
}

static int rc522_request(struct spi_device *spi, u8 req_mode, u8 *tag_type)
{
    int status;
    u8 back_bits;

    /* Send the REQA command */
    rc522_write_register(spi, BitFramingReg, 0x07); // Set bit framing

    status = rc522_transceive(spi, PCD_TRANSCEIVE, &req_mode, 1, tag_type, &back_bits);
    if ((status != MI_OK) || (back_bits != 0x10))
    {
        status = MI_ERR;
    }

    return status;
}

int rc522_card_present(struct spi_device *spi)
{
    u8 tag_type[25] = {0x00};
    int status;

    status = rc522_request(spi, PICC_REQIDL, tag_type);
    printk("Request %s\n", (status == MI_OK) ? "succeeded" : "failed");
    for (int i = 0; i < 3; i++)
    {
        printk("%02x ", tag_type[i]);
    }
    return (status == MI_OK) ? 1 : 0;
}
static void rc522_Reset(struct spi_device *spi)
{

    rc522_write_register(spi, CommandReg, PCD_RESETPHASE);

    msleep(50);
}

static void rc522_antenna_off(struct spi_device *spi)
{
    rc522_clear_bitmask(spi, TxControlReg, 0x03);
}
static void rc522_antenna_on(struct spi_device *spi)
{
    u8 value;
    rc522_read_register(spi, TxControlReg, &value);
    if ((value & 0x03) != 0x03)
    {
        rc522_write_register(spi, TxControlReg, value | 0x03);
    }
}
int rc522_get_version(struct spi_device *spi)
{
    u8 version;
    int ret;

    ret = rc522_read_register(spi, VersionReg, &version);
    if (ret < 0)
        return ret;
    return version;
}

int rc522_hw_init(struct spi_device *spi)
{
    struct rc522_data *rc522 = spi_get_drvdata(spi);
    /* Reset the RC522 */
    if (rc522->reset_gpio)
    {
        gpiod_set_value(rc522->reset_gpio, 0);
        msleep(50);
        gpiod_set_value(rc522->reset_gpio, 1);
    }
    rc522_write_register(spi, CommandReg, PCD_RESETPHASE);
    msleep(50);
    rc522_write_register(spi, TxModeReg, 0x00);   // Reset baud rates
    rc522_write_register(spi, RxModeReg, 0x00);   // Reset baud rates
    rc522_write_register(spi, ModWidthReg, 0x26); // Reset ModWidthReg

    // Timer: TPrescaler*TreloadVal/6.78MHz = 24ms
    rc522_write_register(spi, TModeReg, 0x80);      // 0x8D);      // Tauto=1; f(Timer) = 6.78MHz/TPreScaler
    rc522_write_register(spi, TPrescalerReg, 0xA9); // 0x34); // TModeReg[3..0] + TPrescalerReg
    rc522_write_register(spi, TReloadRegL, 0x03);   // 30);
    rc522_write_register(spi, TReloadRegH, 0xE8);   // 0);
    rc522_write_register(spi, TxAutoReg, 0x40);     // force 100% ASK modulation
    rc522_write_register(spi, ModeReg, 0x3D);       // CRC Initial value 0x6363
    rc522_antenna_off(spi);
    msleep(50);
    rc522_antenna_on(spi);
    msleep(100);
    int ret = rc522_test_write_read(spi);
    if (ret)
    {
        dev_err(&spi->dev, "RC522 Driver: Failed to write/read test: %d\n", ret);
        return ret;
    }

    int version = rc522_get_version(spi);
    if (version < 0)
    {
        dev_err(&spi->dev, "RC522 Driver: Failed to get version: %d\n", version);
        return version;
    }
    dev_info(&spi->dev, "RC522 Driver: RC522 Version: 0x%02x\n", version);
    dev_info(&spi->dev, "RC522 hardware initialization completed successfully\n");
    rc522_card_present(spi);
    return 0;
}
