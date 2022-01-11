
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>

#define BUF_LEN (80)
#define RING_SIZE (6)
#define INSTR_LEN (4)
#define N_ARGS (16)

struct Instruction
{
    char instr[BUF_LEN];
};

struct RingBuffer
{
    unsigned int  tail;
    unsigned int  head;
    struct Instruction data[RING_SIZE];
};

static struct RingBuffer ring;
static pthread_mutex_t ringAccess;
static unsigned short stop = 0;
static unsigned short semaphore = 0;

struct Instruction ringBufGetStr (struct RingBuffer *apBuffer)
{
    int index;
    index = apBuffer->head;
    apBuffer->head = (apBuffer->head + 1) % RING_SIZE;
    return apBuffer->data[index];
}

void ringBufPutStr (struct RingBuffer *apBuffer, const struct Instruction c)
{
    apBuffer->data[apBuffer->tail] = c;
    apBuffer->tail = (apBuffer->tail + 1) % RING_SIZE;
}


void *driver_producer(void *parm)
{
    int file_desc_in;
    int ret_val_in;
    char tmp_in[BUF_LEN] = {0};
    unsigned short speed;
    char *pend;
    struct Instruction tea;

    while (1) {
        file_desc_in = open("/dev/gpio_driver_read_parallel", O_RDWR);
        if (file_desc_in < 0) {
            printf("Error, file_in not opened gpio_driver_read_parallel\n");
            pthread_exit(NULL);
        }
        ret_val_in = read(file_desc_in, tmp_in, BUF_LEN);
        if (ret_val_in < 0) {
            printf("Error in reading gpio_driver_read_parallel\n");
            pthread_exit(NULL);
        }
        close(file_desc_in);
        speed = strtol(tmp_in, &pend, 2);
        stop = strtol(pend, &pend, 2);

        if (stop == 1) {
        /* stop bit set, end the program */
            printf("\n*****stop.*****driver_producer\n\n");
            pthread_exit(NULL);
        } else {
        /* sleep for 1 sec */
            sleep(1);
        }

        /* Fill the struct */
        sprintf(tmp_in, "spd %hu", speed);
	    sprintf(tea.instr, "%s", tmp_in);

        pthread_mutex_lock(&ringAccess);
        ringBufPutStr(&ring, tea);
        ++semaphore;
        pthread_mutex_unlock(&ringAccess);
    }
}

void *terminal_producer(void *parm)
{
    struct Instruction tea;
    char tmp_in[BUF_LEN] = {0};
    char *token;

    while (1) {
        if (stop == 1) {
        /* stop bit set, end the program */
            printf("\n*****stop.*****terminal_producer\n\n");
            pthread_exit(NULL);
        } else {
           // scanf("%79c", tmp_in);
            fgets(tmp_in, 79, stdin);
            tmp_in[strcspn(tmp_in, "\n,./<>?;'\\:|[]{}`~!@#$%^&*()-=_+")] = 0;
            //token = strtok(tmp_in, ";'\\[]{}:/?.>,<~!@#$%^&*()_+-=");
            // if (token == NULL) {
            //     break;
            // }
	        sprintf(tea.instr, "%s", tmp_in);
            pthread_mutex_lock(&ringAccess);
            ringBufPutStr(&ring, tea);
            ++semaphore;
            pthread_mutex_unlock(&ringAccess);
        }
        //usleep(100000);
    }
}

void *consumer(void *parm)
{
    struct Instruction tea;
    int file_desc_out;

    while (1) {
        if (stop == 1) {
        /* stop bit set, end the program */
            printf("\n*****stop.*****consumer\n\n");
            pthread_exit(NULL);
        } else {
            while (semaphore < 1) {
                if (stop == 1) {
                /* stop bit set, end the program */
                    printf("\n*****stop.*****consumer\n\n");
                    pthread_exit(NULL);
                }
                usleep(100000);
            }
//            usleep(100000);
            pthread_mutex_lock(&ringAccess);
            tea = ringBufGetStr(&ring);
            --semaphore;
            pthread_mutex_unlock(&ringAccess);
            usleep(250000);
            file_desc_out = open("/dev/gpio_driver_pwm", O_RDWR);
            if (file_desc_out < 0) {
                printf("Error, file_in not opened gpio_driver_pwm\n");
                pthread_exit(NULL);
            }
            usleep(250000);
            write(file_desc_out, tea.instr, BUF_LEN);
            usleep(250000);
            close(file_desc_out);
            usleep(250000);
            //printf("\n\ninstr: %s \n\n", tea.instr);
        }
    }
}

int main()
{
    /* threads */
    pthread_t hProducer_term;
    pthread_t hProducer_driver;
    pthread_t hConsumer;

    /* Init mutex */
    pthread_mutex_init(&ringAccess, NULL);

    /* thread init */
    pthread_create(&hProducer_term, NULL, terminal_producer, 0);
    pthread_create(&hProducer_driver, NULL, driver_producer, 0);
    pthread_create(&hConsumer, NULL, consumer, 0);

    /* thread join */
    pthread_join(hProducer_driver, NULL);
    pthread_join(hConsumer, NULL);
    pthread_cancel(hProducer_term);

    /* free */
    pthread_mutex_destroy(&ringAccess);

    printf("\n");

    return 0;
}
