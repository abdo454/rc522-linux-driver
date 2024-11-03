#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

int main()
{
    int fd = open("/dev/rc522", O_RDONLY);
    if (fd < 0)
    {
        perror("Failed to open rc522 device");
        return -1;
    }

    uint8_t uid[10];
    while (true)
    {
        ssize_t len = read(fd, uid, sizeof(uid));
        if (len > 0)
        {
            std::cout << "Card UID: ";
            for (ssize_t i = 0; i < len; ++i)
            {
                printf("%02X ", uid[i]);
            }
            std::cout << std::endl;
            sleep(1);
        }
        else
        {

            usleep(1000 * 100);
        }
    }

    close(fd);
    return 0;
}
