#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#define BUF_LEN 80

int main()
{
    int file_desc;
    unsigned short speed;
    unsigned short stop;
    int ret_val;
    char tmp[BUF_LEN] = {0};
    char *pend;

    while (1) {
        file_desc = open("/dev/gpio_driver_read_parallel", O_RDWR);

        if (file_desc < 0) {
            printf("Error, file not opened\n");
            return -1;
        }
        ret_val = read(file_desc, tmp, BUF_LEN);
        if (ret_val < 0) {
            printf("Error in reading\n");
            return -1;
        }
        printf("%s\n", tmp);
        close(file_desc);

        speed = strtol(tmp, &pend, 2);
        stop = strtol(pend, &pend, 2);

        printf("speed %hu stop %hu\n", speed, stop);

        if (tmp[ret_val - 1] - 48 == 1) {
        /* stop bit set, end the program */
            printf("\n*****stop.*****\n\n");
            return 0;
        } else {
        /* sleep for 1 sec */
            sleep(1);
        }
    }
    return 0;
}

