#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/string.h>
#include <linux/ioport.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <asm/io.h>
#include <asm/uaccess.h>


MODULE_LICENSE("Dual BSD/GPL");

// NOTE: Check Broadcom BCM8325 datasheet, page 91+
// NOTE: GPIO Base address is set to 0x7E200000,
//       but it is VC CPU BUS address, while the
//       ARM physical address is 0x3F200000, what
//       can be seen in pages 5-7 of Broadcom
//       BCM8325 datasheet, having in mind that
//       total system ram is 0x3F000000 (1GB - 16MB)
//       instead of 0x20000000 (512 MB)

/* GPIO registers base address. */
#define BCM2708_PERI_BASE   (0x3F000000)
#define GPIO_BASE           (BCM2708_PERI_BASE + 0x200000)
#define GPIO_ADDR_SPACE_LEN (0xB4)
//--

//Handle GPIO: 0-9
/* GPIO Function Select 0. */
#define GPFSEL0_OFFSET (0x00000000)

//Handle GPIO: 10-19
/* GPIO Function Select 1. */
#define GPFSEL1_OFFSET (0x00000004)

//Handle GPIO: 20-29
/* GPIO Function Select 2. */
#define GPFSEL2_OFFSET (0x00000008)

//Handle GPIO: 30-39
/* GPIO Function Select 3. */
#define GPFSEL3_OFFSET (0x0000000C)

//Handle GPIO: 40-49
/* GPIO Function Select 4. */
#define GPFSEL4_OFFSET (0x00000010)

//Handle GPIO: 50-53
/* GPIO Function Select 5. */
#define GPFSEL5_OFFSET (0x00000014)
//--

//GPIO: 0-31
/* GPIO Pin Output Set 0. */
#define GPSET0_OFFSET (0x0000001C)

//GPIO: 32-53
/* GPIO Pin Output Set 1. */
#define GPSET1_OFFSET (0x00000020)
//--

//GPIO: 0-31
/* GPIO Pin Output Clear 0. */
#define GPCLR0_OFFSET (0x00000028)

//GPIO: 32-53
/* GPIO Pin Output Clear 1. */
#define GPCLR1_OFFSET (0x0000002C)
//--

//GPIO: 0-31
/* GPIO Pin Level 0. */
#define GPLEV0_OFFSET (0x00000034)

//GPIO: 32-53
/* GPIO Pin Level 1. */
#define GPLEV1_OFFSET (0x00000038)
//--

//GPIO: 0-53
/* GPIO Pin Pull-up/down Enable. */
#define GPPUD_OFFSET (0x00000094)

//GPIO: 0-31
/* GPIO Pull-up/down Clock Register 0. */
#define GPPUDCLK0_OFFSET (0x00000098)

//GPIO: 32-53
/* GPIO Pull-up/down Clock Register 1. */
#define GPPUDCLK1_OFFSET (0x0000009C)
//--

/* PUD - GPIO Pin Pull-up/down */
typedef enum {PULL_NONE = 0, PULL_DOWN = 1, PULL_UP = 2} PUD;
//--

//000 = GPIO Pin 'x' is an input
//001 = GPIO Pin 'x' is an output
// By default GPIO pin is being used as an input
typedef enum {GPIO_DIRECTION_IN = 0, GPIO_DIRECTION_OUT = 1} DIRECTION;
//--

/* my definitions */
#define GO (0)
#define STOP (1)
#define RESET (2)

/* GPIO pins available on connector p1. */
#define GPIO_02 (2)
#define GPIO_03 (3)
#define GPIO_04 (4)
#define GPIO_05 (5)
#define GPIO_06 (6)
#define GPIO_07 (7)
#define GPIO_08 (8)
#define GPIO_09 (9)
#define GPIO_10 (10)
#define GPIO_11 (11)
#define GPIO_12 (12)
#define GPIO_13 (13)
#define GPIO_14 (14)
#define GPIO_15 (15)
#define GPIO_16 (16)
#define GPIO_17 (17)
#define GPIO_18 (18)
#define GPIO_19 (19)
#define GPIO_20 (20)
#define GPIO_21 (21)
#define GPIO_22 (22)
#define GPIO_23 (23)
#define GPIO_24 (24)
#define GPIO_25 (25)
#define GPIO_26 (26)
#define GPIO_27 (27)

/* Declaration of gpio_driver.c functions */
int gpio_driver_init(void);
void gpio_driver_exit(void);
static int gpio_driver_open(struct inode *, struct file *);
static int gpio_driver_release(struct inode *, struct file *);
static ssize_t gpio_driver_read(struct file *, char *buf, size_t , loff_t *);
static ssize_t gpio_driver_write(struct file *, const char *buf, size_t , loff_t *);

/* Added functions */
static unsigned short fetch_args(char *gdb, unsigned short *args, unsigned short size);
static unsigned short curve_resolve(unsigned short *args, unsigned short n_a, unsigned short size);
static void copy_contents(unsigned short *args, unsigned short size_a, const unsigned short *argss, unsigned short size_aa);
static enum hrtimer_restart gpio_counter_nanosecond(struct hrtimer *param);

/* Structure that declares the usual file access functions. */
struct file_operations gpio_driver_fops =
{
    open    :   gpio_driver_open,
    release :   gpio_driver_release,
    read    :   gpio_driver_read,
    write   :   gpio_driver_write
};

/* Declaration of the init and exit functions. */
module_init(gpio_driver_init);
module_exit(gpio_driver_exit);

/* Global variables of the driver */

/* curve arguments array */
static unsigned short args[16];
static unsigned short curve[16] = {6, 12, 18, 25, 31, 37, 43, 50, 56, 62, 68, 75, 81, 87, 93, 100};
//static unsigned short timer_curve[16];
static unsigned short c = 90;

/* pwm percent */
static unsigned short pwm_percent = 0;

/* Major number. */
int gpio_driver_major;

/* Buffer to store data. */
#define BUF_LEN 80
char* gpio_driver_buffer;

/* Buffer to store instruction data */
#define INSTR_LEN 6
char* instr_buffer;

/* 1 nanosecond timer vars */
static struct hrtimer timer_nanosecond;
static ktime_t ktn;
static const s64 secondsn = 0;
static const unsigned long nanosecondsn = 1000;
static int cnti = 0;
static int cntj = 0;

/* Virtual address where the physical GPIO address is mapped */
void* virt_gpio_base;


/*
 * GetGPFSELReg function
 *  Parameters:
 *   pin    - number of GPIO pin;
 *
 *   return - GPFSELn offset from GPIO base address, for containing desired pin control
 *  Operation:
 *   Based on the passed GPIO pin number, finds the corresponding GPFSELn reg and
 *   returns its offset from GPIO base address.
 */
unsigned int GetGPFSELReg(char pin)
{
    unsigned int addr;

    if(pin >= 0 && pin <10)
        addr = GPFSEL0_OFFSET;
    else if(pin >= 10 && pin <20)
        addr = GPFSEL1_OFFSET;
    else if(pin >= 20 && pin <30)
        addr = GPFSEL2_OFFSET;
    else if(pin >= 30 && pin <40)
        addr = GPFSEL3_OFFSET;
    else if(pin >= 40 && pin <50)
        addr = GPFSEL4_OFFSET;
    else /*if(pin >= 50 && pin <53) */
        addr = GPFSEL5_OFFSET;

  return addr;
}

/*
 * GetGPIOPinOffset function
 *  Parameters:
 *   pin    - number of GPIO pin;
 *
 *   return - offset of the pin control bit, position in control registers
 *  Operation:
 *   Based on the passed GPIO pin number, finds the position of its control bit
 *   in corresponding control registers.
 */
char GetGPIOPinOffset(char pin)
{
    if(pin >= 0 && pin <10)
        pin = pin;
    else if(pin >= 10 && pin <20)
        pin -= 10;
    else if(pin >= 20 && pin <30)
        pin -= 20;
    else if(pin >= 30 && pin <40)
        pin -= 30;
    else if(pin >= 40 && pin <50)
        pin -= 40;
    else /*if(pin >= 50 && pin <53) */
        pin -= 50;

    return pin;
}

/*
 * SetInternalPullUpDown function
 *  Parameters:
 *   pin    - number of GPIO pin;
 *   pull   - set internal pull up/down/none if PULL_UP/PULL_DOWN/PULL_NONE selected
 *  Operation:
 *   Sets to use internal pull-up or pull-down resistor, or not to use it if pull-none
 *   selected for desired GPIO pin.
 */
void SetInternalPullUpDown(char pin, PUD pull)
{
    unsigned int gppud_offset;
    unsigned int gppudclk_offset;
    unsigned int tmp;
    unsigned int mask;

    /* Get the offset of GPIO Pull-up/down Register (GPPUD) from GPIO base address. */
    gppud_offset = GPPUD_OFFSET;

    /* Get the offset of GPIO Pull-up/down Clock Register (GPPUDCLK) from GPIO base address. */
    gppudclk_offset = (pin < 32) ? GPPUDCLK0_OFFSET : GPPUDCLK1_OFFSET;

    /* Get pin offset in register . */
    pin = (pin < 32) ? pin : pin - 32;

    /* Write to GPPUD to set the required control signal (i.e. Pull-up or Pull-Down or neither
       to remove the current Pull-up/down). */
    iowrite32(pull, virt_gpio_base + gppud_offset);

    /* Wait 150 cycles � this provides the required set-up time for the control signal */

    /* Write to GPPUDCLK0/1 to clock the control signal into the GPIO pads you wish to
       modify � NOTE only the pads which receive a clock will be modified, all others will
       retain their previous state. */
    tmp = ioread32(virt_gpio_base + gppudclk_offset);
    mask = 0x1 << pin;
    tmp |= mask;
    iowrite32(tmp, virt_gpio_base + gppudclk_offset);

    /* Wait 150 cycles � this provides the required hold time for the control signal */

    /* Write to GPPUD to remove the control signal. */
    iowrite32(PULL_NONE, virt_gpio_base + gppud_offset);

    /* Write to GPPUDCLK0/1 to remove the clock. */
    tmp = ioread32(virt_gpio_base + gppudclk_offset);
    mask = 0x1 << pin;
    tmp &= (~mask);
    iowrite32(tmp, virt_gpio_base + gppudclk_offset);
}

/*
 * SetGpioPinDirection function
 *  Parameters:
 *   pin       - number of GPIO pin;
 *   direction - GPIO_DIRECTION_IN or GPIO_DIRECTION_OUT
 *  Operation:
 *   Sets the desired GPIO pin to be used as input or output based on the direcation value.
 */
void SetGpioPinDirection(char pin, DIRECTION direction)
{
    unsigned int GPFSELReg_offset;
    unsigned int tmp;
    unsigned int mask;

    /* Get base address of function selection register. */
    GPFSELReg_offset = GetGPFSELReg(pin);

    /* Calculate gpio pin offset. */
    pin = GetGPIOPinOffset(pin);

    /* Set gpio pin direction. */
    tmp = ioread32(virt_gpio_base + GPFSELReg_offset);
    if(direction) { //set as output: set 1
      mask = 0x1 << (pin*3);
      tmp |= mask;
    }
    else { //set as input: set 0
      mask = ~(0x1 << (pin*3));
      tmp &= mask;
    }
    iowrite32(tmp, virt_gpio_base + GPFSELReg_offset);
}

/*
 * SetGpioPin function
 *  Parameters:
 *   pin       - number of GPIO pin;
 *  Operation:
 *   Sets the desired GPIO pin to HIGH level. The pin should previously be defined as output.
 */
void SetGpioPin(char pin)
{
    unsigned int GPSETreg_offset;
    unsigned int tmp;

    /* Get base address of gpio set register. */
    GPSETreg_offset = (pin < 32) ? GPSET0_OFFSET : GPSET1_OFFSET;
    pin = (pin < 32) ? pin : pin - 32;

    /* Set gpio. */
    tmp = 0x1 << pin;
    iowrite32(tmp, virt_gpio_base + GPSETreg_offset);
}

/*
 * ClearGpioPin function
 *  Parameters:
 *   pin       - number of GPIO pin;
 *  Operation:
 *   Sets the desired GPIO pin to LOW level. The pin should previously be defined as output.
 */
void ClearGpioPin(char pin)
{
    unsigned int GPCLRreg_offset;
    unsigned int tmp;

    /* Get base address of gpio clear register. */
    GPCLRreg_offset = (pin < 32) ? GPCLR0_OFFSET : GPCLR1_OFFSET;
    pin = (pin < 32) ? pin : pin - 32;

    /* Clear gpio. */
    tmp = 0x1 << pin;
    iowrite32(tmp, virt_gpio_base + GPCLRreg_offset);
}

/*
 * GetGpioPinValue function
 *  Parameters:
 *   pin       - number of GPIO pin;
 *
 *   return    - the level read from desired GPIO pin
 *  Operation:
 *   Reads the level from the desired GPIO pin and returns the read value.
 */
char GetGpioPinValue(char pin)
{
    unsigned int GPLEVreg_offset;
    unsigned int tmp;
    unsigned int mask;

    /* Get base address of gpio level register. */
    GPLEVreg_offset = (pin < 32) ?  GPLEV0_OFFSET : GPLEV1_OFFSET;
    pin = (pin < 32) ? pin : pin - 32;

    /* Read gpio pin level. */
    tmp = ioread32(virt_gpio_base + GPLEVreg_offset);
    mask = 0x1 << pin;
    tmp &= mask;

    return (tmp >> pin);
}

/*
 * Initialization:
 *  1. Register device driver
 *  2. Allocate buffer
 *  3. Initialize buffer
 *  4. Map GPIO Physical address space to virtual address
 *  5. Initialize GPIO pins
 *  6. Initialize IRQ and register handler
 */
int gpio_driver_init(void)
{
    int result = -1;

    printk(KERN_INFO "gpio_driver_pwm: Inserting gpio_driver module");

    /* Registering device. */
    result = register_chrdev(0, "gpio_driver_pwm", &gpio_driver_fops);
    if (result < 0) {
        printk(KERN_INFO "gpio_driver_pwm: cannot obtain major number %d", gpio_driver_major);
        return result;
    }

    gpio_driver_major = result;
    printk(KERN_INFO "gpio_driver_pwm: major number is %d", gpio_driver_major);

    /* Allocating memory for the buffer. */
    gpio_driver_buffer = kmalloc(BUF_LEN, GFP_KERNEL);
    if (gpio_driver_buffer == NULL) {
        result = -ENOMEM;
        goto fail_no_mem;
    } else {
        /* memory allocation successful */
    }

    /* Allocating memory for the instruction buffer. */
    instr_buffer = kmalloc(INSTR_LEN, GFP_KERNEL);
    if (gpio_driver_buffer == NULL) {
        result = -ENOMEM;
        goto fail_no_mem;
    } else {
        /* memory allocation successful */
    }

    /* Initialize data buffer. */
    memset(gpio_driver_buffer, 0, BUF_LEN);

    /* map the GPIO register space from PHYSICAL address space to virtual address space */
    virt_gpio_base = ioremap(GPIO_BASE, GPIO_ADDR_SPACE_LEN);
    if (virt_gpio_base == NULL) {
        result = -ENOMEM;
        goto fail_no_virt_mem;
    } else {
        /* mapping successful */
    }

    /* Initialize GPIO pins. */
    /* PWM */
    SetGpioPinDirection(GPIO_23, GPIO_DIRECTION_OUT);
    //ClearGpioPin(GPIO_14);
    //SetGpioPin(GPIO_14);
    
    /* Initialize high resolution timer. */
    hrtimer_init(&timer_nanosecond, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    ktn = ktime_set(secondsn, nanosecondsn);
    timer_nanosecond.function = &gpio_counter_nanosecond;
    hrtimer_start(&timer_nanosecond, ktn, HRTIMER_MODE_REL);

    return 0;

fail_no_virt_mem:
    /* Freeing buffer gpio_driver_buffer. */
    if (gpio_driver_buffer) {
        kfree(gpio_driver_buffer);
    }
fail_no_mem:
    /* Freeing the major number. */
    unregister_chrdev(gpio_driver_major, "gpio_driver_pwm");
	
    return result;
}

/*
 * Cleanup:
 *  1. Release IRQ and handler
 *  2. release GPIO pins (clear all outputs, set all as inputs and pull-none to minimize the power consumption)
 *  3. Unmap GPIO Physical address space from virtual address
 *  4. Free buffer
 *  5. Unregister device driver
 */
void gpio_driver_exit(void)
{
    printk(KERN_INFO "gpio_driver_pwm: Removing gpio_driver_pwm module");
    
    /* Clear GPIO pins. */
    ClearGpioPin(GPIO_23);

    /* Set GPIO pins as inputs and disable pull-ups. */
    SetGpioPinDirection(GPIO_23, GPIO_DIRECTION_IN);

    /* Closing hrtimer */
    hrtimer_cancel(&timer_nanosecond);

    /* Unmap GPIO Physical address space. */
    if (virt_gpio_base != NULL) {
        iounmap(virt_gpio_base);
    }

    /* Freeing buffer gpio_driver_buffer. */
    if (gpio_driver_buffer != NULL) {
        kfree(gpio_driver_buffer);
    }

    /* Freeing buffer instr_buffer. */
    if (instr_buffer != NULL) {
        kfree(instr_buffer);
    }

    /* Freeing the major number. */
    unregister_chrdev(gpio_driver_major, "gpio_driver_pwm");
}

/* File open function. */
static int gpio_driver_open(struct inode *inode, struct file *filp)
{
    /* Initialize driver variables here. */

    /* Reset the device here. */

    /* Success. */
    return 0;
}

/* File close function. */
static int gpio_driver_release(struct inode *inode, struct file *filp)
{
    /* Success. */
    return 0;
}

/*
 * File read function
 *  Parameters:
 *   filp  - a type file structure;
 *   buf   - a buffer, from which the user space function (fread) will read;
 *   len - a counter with the number of bytes to transfer, which has the same
 *           value as the usual counter in the user space function (fread);
 *   f_pos - a position of where to start reading the file;
 *  Operation:
 *   The gpio_driver_read function transfers data from the driver buffer (gpio_driver_buffer)
 *   to user space with the function copy_to_user.
 */
static ssize_t gpio_driver_read(struct file *filp, char *buf, size_t len, loff_t *f_pos)
{
    /* Size of valid data in gpio_driver - data to send in user space. */
    int data_size = 0;

    if (*f_pos == 0) {
        /* Get size of valid data. */
        data_size = strlen(gpio_driver_buffer);

        /* Send data to user space. */
        if (copy_to_user(buf, gpio_driver_buffer, data_size) != 0) {
            return -EFAULT;
        }
        else {
            (*f_pos) += data_size;

            return data_size;
        }
    } else {
        return 0;
    }
}

/*
 * File write function
 *  Parameters:
 *   filp  - a type file structure;
 *   buf   - a buffer in which the user space function (fwrite) will write;
 *   len - a counter with the number of bytes to transfer, which has the same
 *           values as the usual counter in the user space function (fwrite);
 *   f_pos - a position of where to start writing in the file;
 *  Operation:
 *   The function copy_from_user transfers the data from user space to kernel space.
 */
static ssize_t gpio_driver_write(struct file *filp, const char *buf, size_t len, loff_t *f_pos)
{
    /* Temp variables */
    char *instruction;
    //unsigned short args[16];
    unsigned short size = sizeof (args) / sizeof(args[0]);
    /* number of arguments fetched */
    unsigned short n_args = 0;
    //printk(KERN_INFO "gpio_driver_pwm: unsigned short size = %hu", size);
 
    //printk(KERN_INFO "gpio_driver_pwm: write -> bufaddr: %p, len: %zu", buf, len);
 
    /* Reset memory. */
    memset(gpio_driver_buffer, '\0', BUF_LEN);
    //printk(KERN_INFO "gpio_driver_pwm: memset successful buffer: %p", gpio_driver_buffer);


    /* Get data from user space.*/
    if (buf == NULL || len < 1) {
        /* oops */
        printk(KERN_ALERT "gpio_driver_pwm: oops, skip");
        return -EFAULT;
    }
    if (copy_from_user(gpio_driver_buffer, buf, len) != 0) {
        return -EFAULT;
    } else {
        //printk(KERN_INFO "gpio_driver_pwm: copy_from_user success");
         /* Instruction formats
          * spd x
          *  x - relative speed [int 0..15]
          * crv a0 .. a15
          *  a0 .. a15 - curve points [int 0..100]
          */
         instruction = strsep(&gpio_driver_buffer, " ");
         if (instruction == NULL) {
             /* gpio_driver_buffer empty */
             printk(KERN_ALERT "gpio_driver_pwm: buffer empty");
             return -EFAULT;
         } else {
             if (strncmp(instruction, "spd\0", 4) == 0) {
                 /* possible instruction read */
                 /* speed matched, check for argument */
                 n_args = fetch_args(gpio_driver_buffer, args, size);
                 if (n_args < 1) {
                     printk(KERN_ALERT "gpio_driver_pwm: insufficient number of args in %s", instruction);
                     return -EFAULT;
                 } else {
                     /* ignore all except the first element */
                     /* floor it to 15 */
                     pwm_percent = (args[0] > 15) ? 15 : args[0];
                     //printk(KERN_INFO "gpio_driver_pwm: pwm set to %hu", pwm_percent);
                 }
             } else if (strncmp(instruction, "crv\0", 4) == 0) {
                 /* possible instruction curve */
                 /* curve matched, check for arguments */
                 n_args = fetch_args(gpio_driver_buffer, args, size);
                 if (n_args < 0) {
                     printk(KERN_ALERT "gpio_driver_pwm: error %s", instruction);
                     return -EFAULT;
                 } else {
                     curve_resolve(args, n_args, size);
                    //  printk(KERN_INFO "gpio_driver_pwm: curve set to %hu %hu %hu %hu %hu %hu %hu %hu %hu %hu %hu %hu %hu %hu %hu %hu",
                    //  args[0], args[1], args[2], args[3], args[4], args[5], args[6], args[7], 
                    //  args[8], args[9], args[10], args[11], args[12], args[13], args[14], args[15]);
                     /* copy contents of args to curve */
                     copy_contents(curve, 16, args, 16);
                 }
             } else {
                 /* instruction not implemented */
                 printk(KERN_ALERT "gpio_driver_pwm: %s instruction not implemented", instruction);
                 return -EFAULT;
             }
         }

        return len;
    }
}


/*
 * Argument parser function
 *  Parameters:
 *   gdb    - string to parse;
 *   args   - a buffer in which instruction arguments will be stored;
 *   size   - size of args;
 *   return - number of successfuly fetched arguments; 
 *  Operation:
 *   The function fetch_args parses elements in gpio_driver_buffer separated by ' '.
 *   The function will parse at most size elements, others will be ignored.
 *   
 */
static unsigned short fetch_args(char *gdb, unsigned short *args, unsigned short size)
{
    unsigned short cnt = 0;
    char *token;
    int err;
    int len;
    if (memset(args, 0, size) != args) {
        printk(KERN_ALERT "gpio_driver_pwm: fetch_args memset unsuccessful");
        return 0;
    }
    for (cnt = 0; cnt < size; ++cnt) {
        token = strsep(&gdb, " ");
        if (token != NULL) {
            /* handling big numbers */
            len = strlen(token);
            if (len > 3) {
                /* use only 3 most significant digits */
                token[3] = '\0';
            }

            err = kstrtol(token, 10, (long *)&args[cnt]);
            if (err != 0) {
                break;
            }
        } else {
            /* end of gdb reached */
            break;
        }
    }

    return cnt;
}

/*
 * Curve resolve function
 *  Parameters:
 *   args   - a buffer to ceil;
 *   n_a    - number of set elements in args;
 *   size   - size of args;
 *   return - number of successfuly fetched arguments; 
 *  Operation:
 *   The function curve_resolve ceils elements in args to 100.
 *   Other (sizeof - n_a) elements will be set to 100.
 *   
 */
static unsigned short curve_resolve(unsigned short *args, unsigned short n_a, unsigned short size)
{
    unsigned short i;
    for (i = 0; i < size; ++i) {
        if (i < n_a) {
            args[i] = (args[i] > 100) ? 100 : args[i];
        } else {
            args[i] = 100;
        }
    }
    return i;
}

/* 
 * copy contents function
 * copies contents from argss array of size size_aa to args of size size_a
 */
static void copy_contents(unsigned short *args, unsigned short size_a, const unsigned short *argss, unsigned short size_aa)
{
    unsigned short i = 0;
    while (i < size_a && i < size_aa) {
        args[i] = argss[i];     
        ++i;
    }
}

static enum hrtimer_restart gpio_counter_nanosecond(struct hrtimer *param)
{
    
    //cnti = (cnti < 100)? ++cnti : 0;
    if (cnti > 99) {
        cnti = 0;
        if (cntj > 9999) {
            /* 1 second has passed, refresh curve and pwm_percent */
            c = curve[pwm_percent];
            cntj = 0;
            //printk(KERN_INFO "gpio_driver_pwm: 1 second has passed, c = %hu", c);
            //printk(KERN_INFO "gpio_driver_pwm: pwm set to %hu percent", c);
        } else {
            ++cntj;
        }
    } else {
        ++cnti;
    }
    
    if (cnti == 0 && cnti != c) {
        SetGpioPin(GPIO_23);
        //printk(KERN_INFO "gpio_driver_buffer: setpin");
    } else if (cnti == c) {
        ClearGpioPin(GPIO_23);
        //printk(KERN_INFO "gpio_driver_buffer: clrpin");
    }

    //setPinValue((cnti < c)? 1 : 0, GPIO_14);

    hrtimer_forward(&timer_nanosecond, ktime_get(), ktn);

    return HRTIMER_RESTART;
}
