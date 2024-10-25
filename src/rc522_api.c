
#include <linux/spi/spi.h>


static int rc522_hw_init(struct spi_device *spi)
{
    /* Optional: Reset the RC522 via GPIO if connected */
    /* Configure RC522 registers */
    rc522_write_register(spi, CommandReg, PCD_RESETPHASE);
    /* Further register settings as required */

    return 0;
}