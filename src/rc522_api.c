// SPDX-License-Identifier: GPL-2.0

#include "rc522_main.h"
#include "rc522_api.h"

struct spi_device *rc522_spi_device;
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
static int rc522_transceive(struct spi_device *spi, u8 cmd, u8 *send_data, u8 sendLen, u8 *backData, u8 *backLen)
{
    int status = MI_ERR;
    u8 irqEn = 0x00; // Enable all interrupts
    u8 waitIRq = 0x00;
    u8 n;
    u32 i;
    switch (cmd)
    {
    case PCD_AUTHENT:
    {
        irqEn = 0x12;
        waitIRq = 0x10;
        break;
    }
    case PCD_TRANSCEIVE:
    {
        irqEn = 0x77;
        waitIRq = 0x30;
        break;
    }
    default:
        break;
    }

    rc522_write_register(spi, CommIEnReg, irqEn | 0x80); // Enable interrupts
    rc522_clear_bitmask(spi, CommIrqReg, 0x80);          // Clear all interrupt flags
    rc522_set_bitmask(spi, FIFOLevelReg, 0x80);          // Flush FIFO buffer

    rc522_write_register(spi, CommandReg, PCD_IDLE); // No action, cancel current command

    // Writing data to the FIFO
    for (i = 0; i < sendLen; i++)
    {
        rc522_write_register(spi, FIFODataReg, send_data[i]);
    }

    // Execute the command
    rc522_write_register(spi, CommandReg, cmd); // Start transmission
    if (cmd == PCD_TRANSCEIVE)
        rc522_set_bitmask(spi, BitFramingReg, 0x80); // Start Send

    // Waiting to receive data to complete
    i = 2000; // Timeout counter
    do
    {
        rc522_read_register(spi, CommIrqReg, &n);
        i--;
    } while ((i != 0) && !(n & 0x01) && !(n & waitIRq));

    rc522_clear_bitmask(spi, BitFramingReg, 0x80); // Stop Send

    if (i != 0)
    {
        u8 error;
        rc522_read_register(spi, ErrorReg, &error);
        if (!(error & 0x1B))
        {
            status = MI_OK;
            if (n & irqEn & 0x01)
            {
                status = MI_NOTAGERR;
            }

            if (cmd == PCD_TRANSCEIVE)
            {
                u8 lastBits;
                rc522_read_register(spi, FIFOLevelReg, &n);
                rc522_read_register(spi, ControlReg, &lastBits);
                lastBits = lastBits & 0x07;
                if (lastBits)
                {
                    *backLen = (n - 1) * 8 + lastBits;
                }
                else
                {
                    *backLen = n * 8;
                }

                if (n == 0)
                {
                    n = 1;
                }
                if (n > RC522_MAX_LEN)
                {
                    n = RC522_MAX_LEN;
                }

                // Reading the received data in FIFO
                for (i = 0; i < n; i++)
                {
                    rc522_read_register(spi, FIFODataReg, &backData[i]);
                }
            }
        }
        else
        {
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
    tag_type[0] = req_mode;
    status = rc522_transceive(spi, PCD_TRANSCEIVE, tag_type, 1, tag_type, &back_bits);
    if ((status != MI_OK) || (back_bits != 0x10))
    {
        status = MI_ERR;
    }

    return status;
}

int rc522_card_present(struct spi_device *spi, u8 *tag_type)
{
    int status;
    status = rc522_request(spi, PICC_REQIDL, tag_type);
    printk("Request %s\n", (status == MI_OK) ? "succeeded" : "failed");
    for (int i = 0; i < 3; i++)
    {
        printk("%02x ", tag_type[i]);
    }
    return (status == MI_OK) ? 1 : 0;
}
/*
 * Function Name: rc522_Reset
 * Description: Reset RC522
 * Input: spi_device *spi
 * Return value: None
 */
static void rc522_Reset(struct spi_device *spi)
{
    rc522_write_register(spi, CommandReg, PCD_RESETPHASE);
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

int rc522_chip_init(struct spi_device *spi)
{
    rc522_spi_device = spi;
    struct rc522_data *rc522 = spi_get_drvdata(spi);
    /* Reset the RC522 */
    if (rc522->reset_gpio)
    {
        gpiod_set_value(rc522->reset_gpio, 1);
        msleep(10);
        gpiod_set_value(rc522->reset_gpio, 0);
        msleep(5);
        gpiod_set_value(rc522->reset_gpio, 1);
        msleep(50);
    }
    rc522_Reset(spi);
    rc522_write_register(spi, TxModeReg, 0x00); // Reset baud rates
    rc522_write_register(spi, RxModeReg, 0x00);
    // Reset ModWidthReg
    rc522_write_register(spi, ModWidthReg, 0x26);

    // When communicating with a PICC we need a timeout if something goes wrong.
    // f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
    // TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
    rc522_write_register(spi, TModeReg, 0x80);      // TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
    rc522_write_register(spi, TPrescalerReg, 0xA9); // TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25μs.
    rc522_write_register(spi, TReloadRegH, 0x03);   // Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
    rc522_write_register(spi, TReloadRegL, 0xE8);

    rc522_write_register(spi, TxAutoReg, 0x40); // Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
    rc522_write_register(spi, ModeReg, 0x3D);   // Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
    rc522_antenna_on(spi);
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
    return 0;
}

void check_card(void)
{
    printk("Checking card");
    struct spi_device *spi = rc522_spi_device;
    int version = rc522_get_version(spi);
    if (version < 0)
        dev_err(&spi->dev, "RC522 Driver: Failed to get version: %d\n", version);
    else
        dev_info(&spi->dev, "RC522 Driver: RC522 Version: 0x%02x\n", version);
    u8 tag_type[RC522_MAX_LEN];
    rc522_card_present(spi, tag_type);
}