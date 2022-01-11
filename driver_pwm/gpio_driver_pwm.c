#include <asm-generic/errno-base.h>
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
static void setPinValue(u8 value, char pin);
static unsigned short fetch_args(char *gdb, unsigned short *args, unsigned short size);
static unsigned short curve_resolve(unsigned short *args, unsigned short n_a, unsigned short size);

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

/* Major number. */
int gpio_driver_major;

/* Buffer to store data. */
#define BUF_LEN 80
char* gpio_driver_buffer;

/* Buffer to store instruction data */
#define INSTR_LEN 6
char* instr_buffer;

/* Base 4 timer vars */
static struct hrtimer timer_base_4;
static ktime_t kt;
static u8 time = 0;
static const s64 seconds = 1;
static const unsigned long microseconds = 0;
static u8 cs = GO; 

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
 * setPinValue function
 *  Parameters:
 *   pin       - number of GPIO pin;
 *   value     - value (0 or 1) for pin to be set;
 *  
 *  Operation:
 *   Sets the pin low or high.
 */
static void setPinValue(u8 value, char pin)
{
    if (value == 1) {
        SetGpioPin(pin);
        
    } else if (value == 0) {
        ClearGpioPin(pin);
    }
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
    /* LEDS */
    /*SetGpioPinDirection(GPIO_06, GPIO_DIRECTION_OUT);
    SetGpioPinDirection(GPIO_13, GPIO_DIRECTION_OUT);
    SetGpioPinDirection(GPIO_19, GPIO_DIRECTION_OUT);
    SetGpioPinDirection(GPIO_26, GPIO_DIRECTION_OUT);*/
    /* SWitches */
    /*SetInternalPullUpDown(GPIO_12, PULL_UP);
    SetInternalPullUpDown(GPIO_16, PULL_UP);
    SetInternalPullUpDown(GPIO_20, PULL_UP);
    SetInternalPullUpDown(GPIO_21, PULL_UP);
    SetGpioPinDirection(GPIO_12, GPIO_DIRECTION_IN);
    SetGpioPinDirection(GPIO_16, GPIO_DIRECTION_IN);
    SetGpioPinDirection(GPIO_20, GPIO_DIRECTION_IN);
    SetGpioPinDirection(GPIO_21, GPIO_DIRECTION_IN);*/

    /* PushButtons */
    /*SetInternalPullUpDown(GPIO_03, PULL_UP);
    SetInternalPullUpDown(GPIO_22, PULL_UP);
    SetGpioPinDirection(GPIO_03, GPIO_DIRECTION_IN);
    SetGpioPinDirection(GPIO_22, GPIO_DIRECTION_IN);*/
    
    /* Initialize high resolution timer. */
    /*hrtimer_init(&timer_base_4, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    kt = ktime_set(seconds, microseconds);
    timer_base_4.function = &gpio_counter_base_4;
    hrtimer_start(&timer_base_4, kt, HRTIMER_MODE_REL);*/

    /* Initialize gpio 3 ISR. */
    /*result = gpio_request_one(GPIO_03, GPIOF_IN, "irq_gpio3");
	if (result != 0) {
        printk("Error: GPIO request failed!\n");
        goto fail_irq;
    }
    irq_gpio3 = gpio_to_irq(GPIO_03);
	result = request_irq(irq_gpio3, h_irq_gpio3,
      IRQF_TRIGGER_FALLING, "irq_gpio3", (void *)(h_irq_gpio3));
	if (result != 0) {
        printk("Error: ISR not registered!\n");
        goto fail_irq;
    }*/
    
    /* Initialize gpio 22 iSR */
    /*result = gpio_request_one(GPIO_22, GPIOF_IN, "irq_gpio22");
	if (result != 0) {
        printk("Error: GPIO request failed!\n");
        goto fail_irq;
    }
    irq_gpio22 = gpio_to_irq(GPIO_22);
	result = request_irq(irq_gpio22, h_irq_gpio22,
      IRQF_TRIGGER_FALLING, "irq_gpio22", (void *)(h_irq_gpio22));
	if (result != 0) {
        printk("Error: ISR not registered!\n");
        goto fail_irq;
    }*/

    return 0;

fail_irq:
    /* Unmap GPIO Physical address space. */
    if (virt_gpio_base) {
        iounmap(virt_gpio_base);
    }
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

    /* Release IRQ and handler. */
    /*disable_irq(irq_gpio3);
    free_irq(irq_gpio3, h_irq_gpio3);
    gpio_free(GPIO_03);*/
    /* */
    /*disable_irq(irq_gpio22);
    free_irq(irq_gpio22, h_irq_gpio22);
    gpio_free(GPIO_22);*/
    
    /* Clear GPIO pins. */
    /*ClearGpioPin(GPIO_06);
    ClearGpioPin(GPIO_13);
    ClearGpioPin(GPIO_19);
    ClearGpioPin(GPIO_26);*/

    /* Set GPIO pins as inputs and disable pull-ups. */
    /*SetGpioPinDirection(GPIO_06, GPIO_DIRECTION_IN);
    SetGpioPinDirection(GPIO_13, GPIO_DIRECTION_IN);
    SetGpioPinDirection(GPIO_19, GPIO_DIRECTION_IN);
    SetGpioPinDirection(GPIO_26, GPIO_DIRECTION_IN);
    SetInternalPullUpDown(GPIO_12, PULL_NONE);
    SetInternalPullUpDown(GPIO_16, PULL_NONE);
    SetInternalPullUpDown(GPIO_20, PULL_NONE);
    SetInternalPullUpDown(GPIO_21, PULL_NONE);
    SetInternalPullUpDown(GPIO_03, PULL_NONE);
    SetInternalPullUpDown(GPIO_22, PULL_NONE);*/

    /* Unmap GPIO Physical address space. */
    if (virt_gpio_base != NULL) {
        iounmap(virt_gpio_base);
    }

    /* Freeing buffer gpio_driver_buffer. */
    if (gpio_driver_buffer != NULL) {
        kfree(gpio_driver_buffer);
    }

    /* Freeing buffer instr_buffer. */
    if (gpio_driver_buffer != NULL) {
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
    printk(KERN_INFO "gpio_driver_pwm: char* instruciton at %p", instruction);
    unsigned short args[16];
    unsigned short size = sizeof (args) / sizeof(args[0]);
    printk(KERN_INFO "gpio_driver_pwm: unsigned short size = %hu", size);
    /* number of arguments fetched */
    unsigned short n_args = 0;
 
    printk(KERN_INFO "gpio_driver_pwm: write ->");
 
    /* Reset memory. */
    memset(gpio_driver_buffer, 0, BUF_LEN);
    printk(KERN_INFO "gpio_driver_pwm: memset successful");

    /* Get data from user space.*/
    if (buf == NULL || len < 1) {
        /* oops */
        printk(KERN_ALERT "gpio_driver_pwm: oops, skip");
        return -EFAULT;
    }
    if (copy_from_user(gpio_driver_buffer, buf, len) != 0) {
        return -EFAULT;
    } else {
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
             if (strncmp(instruction, "spd", 5) == 0) {
                 /* possible instruction read */
                 /* speed matched, check for argument */
                 n_args = fetch_args(gpio_driver_buffer, args, size);
                 if (n_args < 1) {
                     printk(KERN_ALERT "gpio_driver_pwm: insufficient number of args in %s", instruction);
                     return -EFAULT;
                 } else {
                     /* ignore all except the first element */
                     printk(KERN_INFO "gpio_driver_pwm: pwm set to %hu", (args[0] > 15) ? 15 : args[0]);
                 }
             } else if (strncmp(instruction, "crv", 5) == 0) {
                 /* possible instruction curve */
                 /* curve matched, check for arguments */
                 n_args = fetch_args(gpio_driver_buffer, args, size);
                 if (n_args < 0) {
                     printk(KERN_ALERT "gpio_driver_pwm: error %s", instruction);
                     return -EFAULT;
                 } else {
                     curve_resolve(args, n_args, size);
                     printk(KERN_INFO "gpio_driver_pwm: curve set to %hu %hu %hu %hu %hu %hu %hu %hu %hu %hu %hu %hu %hu %hu %hu %hu",
                     args[0], args[1], args[2], args[3], args[4], args[5], args[6], args[7], 
                     args[8], args[9], args[10], args[11], args[12], args[13], args[14], args[15]);
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
