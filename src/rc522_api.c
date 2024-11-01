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
// return 0 on success, -ve on failure
static int rc522_test_write_read(struct spi_device *spi)
{
    u8 test_reg = SerialSpeedReg; // Any register that supports write/read operations, e.g., SerialSpeedReg
    u8 write_val = 0x74;          // 0x55 is arbitrary
    u8 read_val = 0x00;

    // Step 1: Write to the register
    int ret = rc522_write_register(spi, test_reg, write_val);
    if (ret)
    {
        dev_err(&spi->dev, "Write failed with error %d\n", ret);
        return ret;
    }

    // Step 2: Read back from the same register
    ret = rc522_read_register(spi, test_reg, &read_val);
    if (ret)
    {
        dev_err(&spi->dev, "Read failed with error %d\n", ret);
        return ret;
    }

    // Step 3: Compare the written value with the read value
    if (read_val == write_val)
    {
        ret = 0;
        dev_info(&spi->dev, "Write/Read test passed.\n");
    }
    else
    {
        ret = -1;
        dev_err(&spi->dev, "Write/Read test failed. Written: 0x%02x, Read: 0x%02x\n", write_val, read_val);
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
/**
 * @brief
 *
 * @param spi
 * @param cmd
 * @param send_data
 * @param sendLen
 * @param backData
 * @param backLen
 * @return MI_OK on success, MI_ERR, MI_NOTAGERR on failure
 */
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
/**
 * @brief
 *
 * @param spi
 * @param req_mode
 * @param tag_type
 * @return MI_OK on card exist, MI_ERR on error
 */
static int rc522_request_card(struct spi_device *spi, u8 req_mode)
{
    int status;
    u8 back_bits;
    u8 tag_type[10];
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

/* Function name: rc522_detect
 * @return version > 0  on success , -ve of failure
 */
int rc522_get_version(struct rc522_data *rc522)
{
    int ret;
    u8 version;

    ret = rc522_read_register(rc522->spi_dev, VersionReg, &version);
    if (ret < 0)
        return ret;

    switch (version)
    {
    case MFRC522_VERSION_1:
    case MFRC522_VERSION_2:
        dev_info(&rc522->spi_dev->dev, "MFRC522 version 0x%2X detected", version);
        return version;
    default:
        dev_warn(&rc522->spi_dev->dev, "This chip is not an MFRC522: 0x%2X", version);
    }
    return -1;
}
/**
 * @brief initialize RC522 device
 *
 * @param spi
 * @return 0 on success, -ve on failure
 */
int rc522_chip_init(struct rc522_data *rc522)
{
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
    int ret = rc522_test_write_read(rc522->spi_dev);
    if (ret)
    {
        dev_err(&rc522->spi_dev->dev, "RC522 Driver: Failed to write/read test: %d\n", ret);
        return ret;
    }
    rc522_Reset(rc522->spi_dev);
    rc522_write_register(rc522->spi_dev, TxModeReg, 0x00); // Reset baud rates
    rc522_write_register(rc522->spi_dev, RxModeReg, 0x00);
    // Reset ModWidthReg
    rc522_write_register(rc522->spi_dev, ModWidthReg, 0x26);

    // When communicating with a PICC we need a timeout if something goes wrong.
    // f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
    // TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
    rc522_write_register(rc522->spi_dev, TModeReg, 0x80);      // TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
    rc522_write_register(rc522->spi_dev, TPrescalerReg, 0xA9); // TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25μs.
    rc522_write_register(rc522->spi_dev, TReloadRegH, 0x03);   // Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
    rc522_write_register(rc522->spi_dev, TReloadRegL, 0xE8);

    rc522_write_register(rc522->spi_dev, TxAutoReg, 0x40); // Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
    rc522_write_register(rc522->spi_dev, ModeReg, 0x3D);   // Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
    rc522_antenna_on(rc522->spi_dev);

    dev_info(&rc522->spi_dev->dev, "RC522 hardware initialization completed successfully\n");
    return 0;
}
/**
 * @brief
 *
 * @param data
 * @return MI_OK  on success, otherwise MI_ERR
 */
static int rc522_anticoll_card(struct rc522_data *data, struct rc522_card_info *card)
{
    int status;
    u8 uid_length = 0;
    u8 back_bits;
    u8 id_csum = 0;
    u8 i;
    u8 *rx_buff = card->rx_buffer;
    u8 *tx_buff = card->tx_buffer;
    tx_buff[0] = PICC_ANTICOLL;
    tx_buff[1] = 0x20;
    rc522_clear_bitmask(data->spi_dev, Status2Reg, 0x08);
    rc522_write_register(data->spi_dev, BitFramingReg, 0x00);
    rc522_clear_bitmask(data->spi_dev, CollReg, 0x80);

    status = rc522_transceive(data->spi_dev, PCD_TRANSCEIVE, tx_buff, 2, rx_buff, &back_bits);
    if (status == MI_OK)
    {
        if (back_bits == 0x28)
        { // 4-byte UID (32 bits) + 1 byte crc
            uid_length = 4;
        }
        else if (back_bits == 0x46)
        { // 7-byte UID (56 bits) +1 byte crc
            uid_length = 7;
        }
        else
        {
            uid_length = 0;
            status = MI_ERR; // Unexpected UID length
        }
        for (i = 0; i < uid_length; i++)
        {
            id_csum ^= rx_buff[i];
        }
        if (id_csum != rx_buff[uid_length])
            status = MI_ERR;
    }
    card->uid_length = uid_length;
    rc522_set_bitmask(data->spi_dev, CollReg, 0x80);
    return status;
}
/**
 * @brief
 *
 * @param data
 * @param pIndata
 * @param len
 * @param pOutData
 * @return 0 on success, -1 on failure
 */
static int rc522_calculateCRC(struct rc522_data *data, u8 *pIndata, u8 len, u8 *pOutData)
{
    int i;
    u8 reg_val = 0;
    rc522_clear_bitmask(data->spi_dev, DivIrqReg, 0x04);
    rc522_write_register(data->spi_dev, CommandReg, PCD_IDLE);
    rc522_set_bitmask(data->spi_dev, FIFOLevelReg, 0x80);

    for (i = 0; i < len; i++)
        rc522_write_register(data->spi_dev, FIFODataReg, *(pIndata + i));
    rc522_write_register(data->spi_dev, CommandReg, PCD_CALCCRC);

    i = 5;
    do
    {
        rc522_read_register(data->spi_dev, DivIrqReg, &reg_val);
        msleep(30);
        i--;
    } while ((i != 0) && !(reg_val & 0x04)); // CRCIrq = 1

    rc522_read_register(data->spi_dev, CRCResultRegL, &pOutData[0]);
    rc522_read_register(data->spi_dev, CRCResultRegM, &pOutData[1]);
    return i > 0 ? 0 : -1;
}

/**
 * @brief
 *
 * @param data
 * @param card
 * @return  >0 on success, 0 on failure
 */
int rc522_select_tag(struct rc522_data *data, struct rc522_card_info *card)
{

    int i = 0;
    u8 temp_buff[10] = {};
    int status;
    u8 back_bits;
    u8 size;
    temp_buff[0] = PICC_SElECTTAG;
    temp_buff[1] = 0x70;
    for (i = 0; i < card->uid_length + 1; i++)
    {
        temp_buff[2 + i] = card->rx_buffer[i];
    }
    int len = 2 + card->uid_length + 1;
    status = rc522_calculateCRC(data, temp_buff, len, &temp_buff[len]);
    status = rc522_transceive(data->spi_dev, PCD_TRANSCEIVE, temp_buff, len + 2, temp_buff, &back_bits);
    if ((status == MI_OK) && (back_bits == 0x18))
        size = temp_buff[0];
    else
        size = 0;
    return size;
}

void rc522_test_read_card(struct rc522_data *data)
{
    int i = 5;
    int ret = MI_ERR;
    while (i--)
    {
        ret = rc522_request_card(data->spi_dev, PICC_REQIDL);
        if (ret == MI_OK)
        {
            break;
        }
        msleep(25);
    }
    if (ret != MI_OK)
    {
        dev_info(&data->spi_dev->dev, " No card detected after multiple attempts\n");
        return;
    }

    struct rc522_card_info card_info;
    ret = rc522_anticoll_card(data, &card_info);
    if (ret != MI_OK)
    {
        dev_info(&data->spi_dev->dev, " Failed to retrieve card UID (anti-collision failed)\n");
    }
    else
    {
        dev_info(&data->spi_dev->dev, "Card UID:");

        for (int i = 0; i < card_info.uid_length; i++)
        {
            printk(" 0x%02x", card_info.rx_buffer[i]);
        }
        printk("\n");
    }
    ret = rc522_select_tag(data, &card_info);
}
void rc522_halt(struct rc522_data *rc522)
{
    u8 status;
    u8 back_bits;
    u8 buff[4];

    buff[0] = PICC_HALT;
    buff[1] = 0;
    rc522_calculateCRC(rc522, buff, 2, &buff[2]);
    status = rc522_transceive(rc522->spi_dev, PCD_TRANSCEIVE, buff, 4, buff, &back_bits);
}
