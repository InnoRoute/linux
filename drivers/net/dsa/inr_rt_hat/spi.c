/**
*@file
*@brief spi access and interrupts
*@author M.Ulbricht 2021
*@copyright GNU Public License v3.
*
**/
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/msi.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/netdev_features.h>
#include <linux/kthread.h>
#include <linux/skbuff.h>
#include <linux/unistd.h>
#include <linux/ptp_classify.h>
#include <linux/timecounter.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/timex.h>
#include <linux/net_tstamp.h>
#include <linux/ethtool.h>
#include <linux/kern_levels.h>


#include <linux/gpio.h>
#include <linux/fcntl.h>
#include <linux/ioctl.h>
#include <linux/spi/spi.h>


#include <linux/fs.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/sched.h>
#include <linux/cdev.h>
#include <linux/delay.h>


#include "spi.h"
#include "realtime.h"

#define MY_BUS_NUM 0
static struct spi_device *spi_device;
static volatile uint8_t spi_enabled = 0;
static volatile uint32_t data_w = 0;
static volatile uint32_t data_r = 0;
static volatile uint8_t cs = 0;
static volatile uint8_t semaphor = 0;
static unsigned char ch1 = 0xAA;
static unsigned char ch2 = 0xBB;
static uint8_t portcount = 3;
static uint16_t INTERRRUPT_MASK = 0x13ff;
static uint32_t COMMON_INTERRUPT_MASK = 0xffffffff;
static uint8_t pollcount__MMI = 0, pollcount__MMI2 = 0;
static volatile uint32_t COMMON_INTERRUPT_STATE = 0;
static volatile uint8_t INT_count1 = 0;
static volatile uint8_t INT_count2 = 0;

static struct cdev *driver_object;
static char *devname = "gpio_irq";
static int rpi_irq_27;
#define INTERRUPT_MASK 0x1C00	// just HC0..2
#define SPI_IPG 200

static DECLARE_WAIT_QUEUE_HEAD (INR_SPI_int_waittingqueu);
DEFINE_SEMAPHORE (INR_interrupt_sem);
DEFINE_SEMAPHORE (INR_SPI_sem);
//*****************************************************************************************************************
/**
*spi interrupt thread
*@brief wakeup from is with call counter, need because of spi access from interrupt
*/
int
INR_SPI_interrupt_thread (void *nix)	// contains empirical thesholds..but it works ;)
{
    DECLARE_WAITQUEUE (wait2, current);
    allow_signal (SIGKILL);
    add_wait_queue (&INR_SPI_int_waittingqueu, &wait2);
    while (1) {
        set_current_state (TASK_INTERRUPTIBLE);
        if (INT_count1 == INT_count2)
            schedule ();
        INR_SPI_MMI_interrupt ();
        //semaphore mkit counter einbauen
        INT_count2++;
        if (signal_pending (current))
            break;			//exit on thermination

    }
    set_current_state (TASK_RUNNING);
    remove_wait_queue (&INR_SPI_int_waittingqueu, &wait2);
    return 0;
}

//*****************************************************************************************************************
/**
*SPI ISR
*@brief handle spi interrupt
*/
INR_SPI_MMI_interrupt ()
{

    down_killable (&INR_interrupt_sem);

    if (DEBUG)
        printk (KERN_DEBUG "DEBUG: Passed %s %d \n", __FUNCTION__, __LINE__);
    INR_SPI_SPI_write (0xFFFFFFFF,(C_BASE_ADDR_SPI_LOWER << 8) +C_SUB_ADDR_SPI_INT_CLR_EN);
    uint32_t Interrupt_status =
        INR_SPI_SPI_read ((C_BASE_ADDR_SPI_LOWER << 8) +
                          C_SUB_ADDR_SPI_INT_STATUS);
    //printk(KERN_DEBUG "INR_RTH: SPI_INT_STATUS 0x%08lx\n",Interrupt_status);
    uint32_t reason = 0;
    /*
      if (Interrupt_status & (1 << SPI_INT_REG_XADC))
        {
          reason =
    	INR_SPI_SPI_read ((C_BASE_ADDR_SPI_LOWER << 8) +
    			  C_SUB_ADDR_SPI_FPGA_ALARM);

          if (reason & (1 << 0))
    	printk (KERN_ERR
    		"INR_RTH: C_XADC_ANY_ALARM: Any of the following alarms\n");
          if (reason & (1 << 1))
    	printk (KERN_ERR
    		"INR_RTH: C_XADC_OVER_TEMP: Critical over-temperature -> self-shutdown of the FPGA\n");
          if (reason & (1 << 2))
    	printk (KERN_ERR
    		"INR_RTH: C_XADC_USER_TEMP: Early over-temperature warning\n");
          if (reason & (1 << 3))
    	printk (KERN_ERR
    		"INR_RTH: C_XADC_VCC_AUX:   VCC-AUX voltage out of bounds\n");
          if (reason & (1 << 4))
    	printk (KERN_ERR
    		"INR_RTH: C_XADC_VCC_INT:   VCC-INT voltage out of bounds\n");
          if (reason & (1 << 5))
    	printk (KERN_ERR
    		"INR_RTH: C_XADC_VCC_BRAM:  VCC-BRAM voltage out of bounds\n");

        }
      if (Interrupt_status & (1 << SPI_INT_REG_FRAME_ECCE2))
        {
          reason =
    	INR_SPI_SPI_read ((C_BASE_ADDR_SPI_LOWER << 8) +
    			  C_SUB_ADDR_SPI_CONFIG_CHECK);
          if (reason & (1 << 0))
    	printk (KERN_ERR "INR_RTH: CRC Error\n");
          if (reason & (1 << 1))
    	printk (KERN_ERR "INR_RTH: Error\n");
          if (reason & (1 << 2))
    	printk (KERN_ERR "INR_RTH: Single ECC Error\n");

        }
      if (Interrupt_status & (1 << SPI_INT_REG_TAMPER))
        {
          reason =
    	INR_SPI_SPI_read ((C_BASE_ADDR_SPI_LOWER << 8) +
    			  C_SUB_ADDR_SPI_LICENSE);
          printk (KERN_ERR
    	      "INR_RTH: CRITICAL: Tamper protection: your license: 0x%llx\n",
    	      reason);


        }
      if (Interrupt_status & (1 << SPI_INT_REG_SPI_ERROR))
        {
          reason =
    	INR_SPI_SPI_read ((C_BASE_ADDR_SPI_LOWER << 8) +
    			  C_SUB_ADDR_SPI_ACCESS_ERROR);
          if (reason & (1 << 0))
    	printk (KERN_ERR
    		"INR_RTH: C_ERR_BDCMD: Bad SPI Command or command does not match local-MMI address\n");
          if (reason & (1 << 1))
    	printk (KERN_ERR
    		"INR_RTH: C_ERR_IRUPT: Undersized SPI access (FSM stuck until next access) or access interrupted by new access\n");
          if (reason & (1 << 2))
    	printk (KERN_ERR "INR_RTH: C_ERR_OSIZE: Oversize SPI access\n");
          if (reason & (1 << 3))
    	printk (KERN_ERR "INR_RTH: C_ERR_ALIGN: SPI Address Misalignment\n");
          if (reason & (1 << 4))
    	printk (KERN_ERR
    		"INR_RTH: C_ERR_LOCNA: Local Address not available, i.e., access to undefined local address\n");
          if (reason & (1 << 5))
    	printk (KERN_ERR
    		"INR_RTH: C_ERR_LOCAC: Local access error, i.e., read from write-only or write to read-only address\n");
          if (reason & (1 << 6))
    	printk (KERN_ERR
    		"INR_RTH: C_ERR_LFAIL: Synchronization timeout error at reading from local address\n");
          if (reason & (1 << 7))
    	printk (KERN_ERR
    		"INR_RTH: C_ERR_MMIAC: MMI Access error, e.g., write to Read-Only address\n");
          if (reason & (1 << 8))
    	printk (KERN_ERR
    		"INR_RTH: C_ERR_MMIWR: Timeout from previous MMI write access disturbs/prevents current access\n");
          if (reason & (1 << 9))
    	printk (KERN_ERR
    		"INR_RTH: C_ERR_MMIRD: Timeout from previous or current MMI read access\n");
          if (reason & (1 << 10))
    	printk (KERN_ERR
    		"INR_RTH: C_ERR_MTIME: Timeout counter triggered at current MMI access\n");
          if (reason & (1 << 11))
    	printk (KERN_ERR
    		"INR_RTH: C_ERR_MFAIL: Synchronization timeout error at reading from MMI address, possibly just read from write-only MMI address\n");

        }
      if (Interrupt_status & (1 << SPI_INT_REG_FIFO_OVERFLOW))
        {
          reason =
    	INR_SPI_SPI_read ((C_BASE_ADDR_SPI_LOWER << 8) +
    			  C_SUB_ADDR_SPI_FIFO_OVERFLOW);
          printk (KERN_ERR "INR_RTH: FIFO overflow occurred: 0x%llx\n", reason);

        }
      if (Interrupt_status & (1 << SPI_INT_REG_FIFO_UNDERRUN))
        {
          reason =
    	INR_SPI_SPI_read ((C_BASE_ADDR_SPI_LOWER << 8) +
    			  C_SUB_ADDR_SPI_FIFO_UNDERRUN);
          printk (KERN_ERR "INR_RTH: FIFO underrun occurred: 0x%llx\n", reason);


        }
      if (Interrupt_status & (1 << SPI_INT_REG_EXT))
        {
          reason =
    	INR_SPI_SPI_read ((C_BASE_ADDR_SPI_LOWER << 8) +
    			  C_SUB_ADDR_SPI_EXT_INTERRUPT);
          if (reason & (1 << 0))
    	printk (KERN_DEBUG "INR_RTH: PHY interrupt from Port 0");
          if (reason & (1 << 1))
    	printk (KERN_DEBUG "INR_RTH: PHY interrupt from the PoE port");
          if (reason & (1 << 2))
    	printk (KERN_DEBUG
    		"INR_RTH: PHY interrupt from the middle port (connected to RasPi)");
          if (reason & (1 << 3))
    	printk (KERN_DEBUG "INR_RTH: PMIC interrupt");
          if (reason & (1 << 4))
    	printk (KERN_DEBUG "INR_RTH: Link status change on Port 0");
          if (reason & (1 << 5))
    	printk (KERN_DEBUG "INR_RTH: Link status change on the PoE port");
          if (reason & (1 << 6))
    	printk (KERN_DEBUG
    		"INR_RTH: Link status change on the middle port (connected to RasPi)");

        }
      if (Interrupt_status & (1 << SPI_INT_REG_WATCHDOG))
        {
          reason =
    	INR_SPI_SPI_read ((C_BASE_ADDR_SPI_LOWER << 8) +
    			  C_SUB_ADDR_SPI_EVENT);
          printk (KERN_DEBUG "INR_RTH: Watchdog occurred: 0x%llx\n", reason);


        }
      if (Interrupt_status & (1 << SPI_INT_REG_MMI))
        {
          reason =
    	INR_SPI_SPI_read ((C_BASE_ADDR_SPI_LOWER << 8) +
    			  C_SUB_ADDR_SPI_MMI_INT_BITMAP);
          if (reason & (1 << 0))
    	{
    	  INR_SPI_SPI_read ((C_BASE_ADDR_HC_0 << 8) +
    			    C_SUB_ADDR_HC_INTERRUPT);
    	  //INR_TIME_TX_transmit_interrupt (0);
    	  //printk(KERN_DEBUG "INR_RTH: C_MMI_INT_HC(0) : TX Confirmation or Fragment counter threshold from Port 0");
    	}
          if (reason & (1 << 1))
    	{
    	  INR_SPI_SPI_read ((C_BASE_ADDR_HC_1 << 8) +
    			    C_SUB_ADDR_HC_INTERRUPT);
    	  //INR_TIME_TX_transmit_interrupt (1);
    	  //printk(KERN_DEBUG "INR_RTH: C_MMI_INT_HC(1) : TX Confirmation or Fragment counter threshold from the PoE port");
    	}
          if (reason & (1 << 2))
    	{
    	  INR_SPI_SPI_read ((C_BASE_ADDR_HC_2 << 8) +
    			    C_SUB_ADDR_HC_INTERRUPT);
    	  //INR_TIME_TX_transmit_interrupt (2);
    	  //printk(KERN_DEBUG "INR_RTH: C_MMI_INT_HC(2) : TX Confirmation or Fragment counter threshold from the middle port (connected to RasPi)");
    	}


          if (reason & (1 << 19))
    	printk (KERN_DEBUG "INR_RTH: C_MMI_INT_RTC   : RTC Interrupt");
          if (reason & (1 << 21))
    	printk (KERN_DEBUG
    		"INR_RTH: C_MMI_INT_DEBUG : Interrupt from one of the debugging units in the system");
          if (reason & (1 << 25))
    	printk (KERN_DEBUG
    		"INR_RTH: C_MMI_INT_PPS   : Edge on the PPS input was sampled");

        }
      if (Interrupt_status & (1 << SPI_INT_REG_TEST))
        {
          reason =
    	INR_SPI_SPI_read ((C_BASE_ADDR_SPI_LOWER << 8) +
    			  C_SUB_ADDR_SPI_TEST_INPUT);
          printk (KERN_DEBUG "INR_RTH: Debug interrupt occurred: 0x%llx\n",
    	      reason);


        }
    */
    if (Interrupt_status & (1 << SPI_INT_REG_TxConf0)) {
        if (DEBUG)printk (KERN_DEBUG "INR_RTH: TxConf0\n");
        INR_TIME_TX_transmit_interrupt (0);
    }
    if (Interrupt_status & (1 << SPI_INT_REG_TxConf1)) {
        if (DEBUG)printk (KERN_DEBUG "INR_RTH: TxConf1\n");
        INR_TIME_TX_transmit_interrupt (1);
    }
    if (Interrupt_status & (1 << SPI_INT_REG_TxConf2)) {
        if (DEBUG)printk (KERN_DEBUG "INR_RTH: TxConf2\n");
        INR_TIME_TX_transmit_interrupt (2);
    }
    INR_SPI_SPI_write (INTERRUPT_MASK,
                       (C_BASE_ADDR_SPI_LOWER << 8) +
                       C_SUB_ADDR_SPI_INT_SET_EN);
    up (&INR_interrupt_sem);

}

//*****************************************************************************************************************
/**
*init SPI interrupts
*@brief init settings
*/
void
init_interrupts ()
{
    if (DEBUG)
        printk (KERN_DEBUG "DEBUG: Passed %s %d \n", __FUNCTION__, __LINE__);
    uint32_t err_data = 0;
    uint32_t err_addr =
        (RD_LOCAL << 24) + (C_BASE_ADDR_SPI_LOWER << 8) +
        C_SUB_ADDR_SPI_ACCESS_ERROR;
    nibbletwist ((uint8_t *) & err_addr, 4);
    spi_write_then_read (spi_device, &err_addr, sizeof (err_addr), &err_data, sizeof (err_data));	// clear error date
    INR_SPI_SPI_write (0x100,
                       (C_BASE_ADDR_SPI_LOWER << 8) +
                       C_SUB_ADDR_SPI_INT_SET_EN);
    INR_SPI_MMI_write (0x4, INR_HC_INTERRUPT_EN (0));
    INR_SPI_MMI_write (0x0, INR_HC_INTERRUPT_EN (1));
    INR_SPI_MMI_write (0x0, INR_HC_INTERRUPT_EN (2));
    INR_SPI_MMI_write (0x0,
                       (C_BASE_ADDR_RTC << 8) + C_SUB_ADDR_RTC_INTERRUPT_EN);
    INR_SPI_MMI_write (0x0,
                       (C_BASE_ADDR_PPS << 8) + C_SUB_ADDR_PPS_INTERRUPT_EN);
    INR_SPI_MMI_write (0x0,
                       (C_BASE_ADDR_DEBUG_LOWER << 8) +
                       C_SUB_ADDR_DEBUG_INTERRUPT_EN);


}

//*****************************************************************************************************************
/**
*spi address conversion
*@brief automatic choose command filed for spi or mmi access
*/
void
INR_SPI_autocmd (uint32_t * addr, uint8_t write)
{

    if (write) {
        if ((C_BASE_ADDR_SPI_LOWER * 256 <= (0x00ffffff & *addr))
            && ((0x00ffffff & *addr) <= C_BASE_ADDR_SPI_UPPER * 256)) {
            //mmi write
            *addr = (WR_LOCAL << 24) + (0x00ffffff & (*addr));
        } else {
            //spi write
            *addr = (WR_MMI << 24) + (0x00ffffff & (*addr));
        }

    } else {
        if ((C_BASE_ADDR_SPI_LOWER * 256 <= (0x00ffffff & *addr))
            && ((0x00ffffff & *addr) <= C_BASE_ADDR_SPI_UPPER * 256)) {
            //mmi read
            *addr = (RD_LOCAL << 24) + (0x00ffffff & (*addr));
        } else {
            //spi read
            *addr = (RD_MMI << 24) + (0x00ffffff & (*addr));
        }


    }


}

//*****************************************************************************************************************
/**
*ISR for FPGA pin
*@brief this is the first function called if there is an edge on the pin, increases the call countner and wakeup thread
*/
static irqreturn_t
rpi_gpio_isr (int irq, void *data)
{
    if (DEBUG)
        printk ("rpi_gpio_isr( %d, %p )\n", irq, data);
    INT_count1++;
    wake_up_interruptible (&INR_SPI_int_waittingqueu);
    return IRQ_HANDLED;

}

//*****************************************************************************************************************
/**
*config gpio for interrupt pin
*@brief
*/
static int
config_gpio (int gpionr)
{
    int err, rpi_irq;
    char name[20];

    snprintf (name, sizeof (name), "rpi-gpio-%d", gpionr);
    err = gpio_request (gpionr, name);
    if (err) {
        printk ("gpio_request failed %d\n", err);
        return -1;
    }
    err = gpio_direction_input (gpionr);
    if (err) {
        printk ("gpio_direction_input failed %d\n", err);
        gpio_free (gpionr);
        return -1;
    }
    rpi_irq = gpio_to_irq (gpionr);
    printk ("gpio_to_irq returned %d\n", rpi_irq);
    if (rpi_irq < 0) {
        printk ("gpio_to_irq failed %d\n", rpi_irq);
        gpio_free (gpionr);
        return -1;
    }
    err = request_irq (rpi_irq, rpi_gpio_isr,
                       IRQF_TRIGGER_RISING, devname, driver_object);
    printk ("driver_object: %p\n", driver_object);
    if (err) {
        printk ("request_irq failed with %d\n", err);
        gpio_free (gpionr);
        return -1;
    }
    printk ("gpio %d successfull configured\n", gpionr);
    return rpi_irq;
}

//*****************************************************************************************************************
/**
*interrupt pin init
*@brief inits the raspberry pin for FPGA interrupt
*/
static int
int_init (void)
{
    if (DEBUG)
        printk (KERN_DEBUG "DEBUG: Passed %s %d \n", __FUNCTION__, __LINE__);
    kthread_run (&INR_SPI_interrupt_thread, NULL, "INR_SPI_interrupt_thread");
    driver_object = cdev_alloc ();
    driver_object->owner = THIS_MODULE;
    rpi_irq_27 = config_gpio (27);
    if (rpi_irq_27 < 0) {
        printk ("Error requesting interrupt \n");
    }


    return 0;

}

static void
int_exit (void)
{

    free_irq (rpi_irq_27, driver_object);
    gpio_free (27);
    return;
}

//*****************************************************************************************************************
/**
*nibble change function for spi communication
*@brief needed because FPGA swaps nibbles
*/
void
nibbletwist (uint8_t * data, uint8_t length)
{
    uint8_t i = 0;
    uint8_t tmp;
    for (i = 0; i < length / 2; i++) {
        tmp = data[i];
        data[i] = data[length - i - 1];
        data[length - i - 1] = tmp;
    }

}

//*****************************************************************************************************************
/**
*driver intern spi read function
*@brief called internal
*/
uint32_t
RT_SPI_read (uint32_t addr)
{
    INR_SPI_autocmd (&addr, 0);
    uint8_t i = 0;
    uint8_t extend = 0;
    uint32_t data = 0;
    uint32_t tmp = addr;
    uint32_t err_data = 0;
    uint32_t err_addr =
        (RD_LOCAL << 24) + (C_BASE_ADDR_SPI_LOWER << 8) +
        C_SUB_ADDR_SPI_ACCESS_ERROR;

    if ((addr & 0xf0000000) == 0xf0000000)
        extend = 4;			// insert padding bytes for MMI read command

    nibbletwist ((uint8_t *) & addr, 4);
    if (spi_enabled) {
        down_killable (&INR_SPI_sem);
        spi_write_then_read (spi_device, &addr, sizeof (addr) + extend, &data,
                             sizeof (data));
        nibbletwist ((uint8_t *) & data, 4);
        ndelay(SPI_IPG);
        if (GET_SPI_ERROR) {
            nibbletwist ((uint8_t *) & err_addr, 4);
            spi_write_then_read (spi_device, &err_addr, sizeof (err_addr),
                                 &err_data, sizeof (err_data));
            nibbletwist ((uint8_t *) & err_data, 4);
        }
        up (&INR_SPI_sem);
    }
    if (SPI_DEBUG || err_data)
        printk (KERN_DEBUG "DEBUG: spi read 0x%08lx 0x%08lx error:0x%08lx\n", tmp,
                data, err_data);
    return data;
}

EXPORT_SYMBOL (RT_SPI_read);
//*****************************************************************************************************************
/**
*intern spi write function
*@brief called internal
*/
void
RT_SPI_write (uint32_t addr, uint32_t data)
{
    uint32_t err_data = 0;
    uint32_t err_addr =
        (RD_LOCAL << 24) + (C_BASE_ADDR_SPI_LOWER << 8) +
        C_SUB_ADDR_SPI_ACCESS_ERROR;
    INR_SPI_autocmd (&addr, 1);
    if (SPI_DEBUG)
        printk (KERN_DEBUG "DEBUG: spi write 0x%08lx 0x%08lx\n", addr, data);
    uint8_t i = 0;
    union address tmp;
    tmp.addr = addr;
    uint8_t tx_buffer[8];
    for (i = 0; i < 4; i++)
        tx_buffer[i] = tmp.bytes[3 - i];
    tmp.addr = data;
    for (i = 0; i < 4; i++)
        tx_buffer[i + 4] = tmp.bytes[3 - i];
    if (spi_enabled) {
        down_killable (&INR_SPI_sem);
        spi_write (spi_device, &tx_buffer[0], sizeof (data) * 2);
        ndelay(SPI_IPG);
        if (GET_SPI_ERROR) {
            nibbletwist ((uint8_t *) & err_addr, 4);
            spi_write_then_read (spi_device, &err_addr, sizeof (err_addr),
                                 &err_data, sizeof (err_data));
            nibbletwist ((uint8_t *) & err_data, 4);
            if (err_data)
                printk (KERN_ERR
                        "error: 0x%08lx on write to addr 0x%08lx data: 0x%08lx\n",
                        err_data, addr, data);
        }
	up (&INR_SPI_sem);
    }
}

EXPORT_SYMBOL (RT_SPI_write);
//*****************************************************************************************************************
/**
*spi read function for proc-fs
*@brief called from userpacescripts via procfs
*/
void
RT_SPI_proc_read (uint32_t addr)
{
    uint32_t err_data = 0;
    uint32_t err_addr =
        (RD_LOCAL << 24) + (C_BASE_ADDR_SPI_LOWER << 8) +
        C_SUB_ADDR_SPI_ACCESS_ERROR;
    INR_SPI_autocmd (&addr, 0);
    uint32_t tmp = addr;
    uint8_t i = 0;
    uint8_t extend = 0;
    if ((addr & 0xf0000000) == 0xf0000000)
        extend = 4;			// insert padding bytes for MMI read command
    semaphor = 1;
    nibbletwist ((uint8_t *) & addr, 4);
    if (spi_enabled) {
        down_killable (&INR_SPI_sem);
        spi_write_then_read (spi_device, &addr, sizeof (addr) + extend, &data_r,
                             sizeof (data_r));
        ndelay(SPI_IPG);
        nibbletwist ((uint8_t *) & data_r, 4);
        if (GET_SPI_ERROR) {
            nibbletwist ((uint8_t *) & err_addr, 4);
            spi_write_then_read (spi_device, &err_addr, sizeof (err_addr),
                                 &err_data, sizeof (err_data));
            nibbletwist ((uint8_t *) & err_data, 4);

        }
	up (&INR_SPI_sem);
        if (SPI_DEBUG || err_data)
            printk (KERN_DEBUG
                    "DEBUG: spi read proc 0x%08lx 0x%08lx error:0x%08lx\n", tmp,
                    data_r, err_data);
    }

}

//*****************************************************************************************************************
/**
*spi write function for proc-fs
*@brief called from userpacescripts via procfs
*/
void
RT_SPI_proc_write (uint32_t addr)
{
    uint32_t err_data = 0;
    uint32_t err_addr =
        (RD_LOCAL << 24) + (C_BASE_ADDR_SPI_LOWER << 8) +
        C_SUB_ADDR_SPI_ACCESS_ERROR;
    INR_SPI_autocmd (&addr, 1);
    if (SPI_DEBUG)
        printk (KERN_DEBUG "DEBUG: spi write proc 0x%08lx 0x%08lx\n", addr,
                data_w);
    uint8_t i = 0;
    union address tmp;
    tmp.addr = addr;
    uint8_t tx_buffer[8];
    for (i = 0; i < 4; i++)
        tx_buffer[i] = tmp.bytes[3 - i];
    tmp.addr = data_w;
    for (i = 0; i < 4; i++)
        tx_buffer[i + 4] = tmp.bytes[3 - i];
    if (spi_enabled) {
        down_killable (&INR_SPI_sem);
        spi_write (spi_device, &tx_buffer[0], sizeof (data_w) * 2);
        ndelay(SPI_IPG);
        if (GET_SPI_ERROR) {
            nibbletwist ((uint8_t *) & err_addr, 4);
            spi_write_then_read (spi_device, &err_addr, sizeof (err_addr),
                                 &err_data, sizeof (err_data));
            nibbletwist ((uint8_t *) & err_data, 4);
            if (err_data)
                printk (KERN_ERR
                        "error: 0x%08lx on write to addr 0x%08lx data: 0x%08lx\n",
                        err_data, addr, data_w);
        }
up (&INR_SPI_sem);
    }
    semaphor = 0;
}

//*****************************************************************************************************************
/**
*procfs file transfer
*@brief for FPGA programming
*/
void
RT_SPI_file_write (char *buffer, size_t size)
{
    if (spi_enabled) {
        down_killable (&INR_SPI_sem);
        spi_write (spi_device, buffer, size);
        up (&INR_SPI_sem);
    }

}

//*****************************************************************************************************************
/**
*spi device access semaphor
*@brief to handle internal and proc-fs accesses
*/
uint8_t
RT_SPI_get_semaphor ()
{
    return semaphor;
}

//*****************************************************************************************************************
/**
*set data to write from proc-fs
*@brief data and address are two different files in proc-fs
*/
void
RT_SPI_set_w_data (uint32_t data)
{
    semaphor = 1;
    data_w = data;
}

//*****************************************************************************************************************
/**
*set data to read to proc-fs
*@brief data and address are two different files in proc-fs
*/
uint32_t
RT_SPI_get_r_data ()
{
    semaphor = 0;
    return data_r;
}

//*****************************************************************************************************************
/**
*spi init function
*@brief create spi device
*/
void
RT_SPI_init ()
{
    if (spi_enabled == 0) {
        printk (KERN_DEBUG "Enable INR-RealTime-HAT SPI Module\n");
        int ret;
        int_init ();
        struct spi_master *master;

        //Register information about your slave device:
        struct spi_board_info spi_device_info = {
            .modalias = "INR-RTHAT",
            .max_speed_hz = 50000000,	//speed your device (slave) can handle
            .bus_num = MY_BUS_NUM,
            .chip_select = 0,
            .mode = SPI_MODE_0,
        };

        master = spi_busnum_to_master (spi_device_info.bus_num);
        if (!master) {
            printk ("MASTER not found.\n");
            return -ENODEV;
        }
        spi_device = spi_new_device (master, &spi_device_info);

        if (!spi_device) {
            printk ("FAILED to create slave.\n");
            return -ENODEV;
        }

        spi_device->bits_per_word = 8;

        ret = spi_setup (spi_device);

        if (ret) {
            printk ("FAILED to setup slave.\n");
            spi_unregister_device (spi_device);
            return -ENODEV;
        }

        spi_enabled = 1;

        return 0;

    } else
        return 0;
}

//*****************************************************************************************************************
/**
*spi exit
*@brief remove spi device
*/
void
RT_SPI_exit ()
{
    if (spi_enabled == 1) {
        int_exit ();
        spi_enabled = 0;
        if (spi_device) {
            spi_write (spi_device, &ch2, sizeof (ch2));
            spi_unregister_device (spi_device);
        }
    }
}
