#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#define INFO_BUFFER_SIZE 32
struct rc522_card_info
{
    uint8_t uid_length;
    uint8_t tx_buffer[INFO_BUFFER_SIZE];
    uint8_t rx_buffer[INFO_BUFFER_SIZE];
    uint8_t version;
    bool initialized;
};

#define RC522_IOC_MAGIC 'r'
#define RC522_IOC_GET_UID _IOR(RC522_IOC_MAGIC, 1, struct rc522_card_info)

int main()
{
    int fd = open("/dev/rc522", O_RDONLY);
    if (fd < 0)
    {
        perror("Failed to open rc522 device");
        return -1;
    }

    rc522_card_info card_info;
    while (true)
    {
        if (ioctl(fd, RC522_IOC_GET_UID, &card_info) < 0)
        {
            usleep(1000 * 100);
        }
        else
        {
            std::cout << "-Card UID: ";
            for (ssize_t i = 0; i < card_info.uid_length; ++i)
            {
                printf("%02X ", card_info.rx_buffer[i]);
            }
            std::cout << std::endl;
            sleep(1);
        }
    }

    close(fd);
    return 0;
}
