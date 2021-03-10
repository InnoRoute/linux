/**
*@file
*@brief proc-fs functions
*@author M.Ulbricht 2021
*@copyright GNU Public License v3.
*
**/
#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/proc_fs.h>
#include <linux/openvswitch.h>
#include <linux/kthread.h>
#include <asm/signal.h>
#include <linux/semaphore.h>
#include "realtime.h"
#include "spi.h"
#include "rt_procfs.h"

#define PROCFS_MAX_SIZE		1024
static char procfs_buffer[PROCFS_MAX_SIZE];
static size_t procfs_buffer_size = 0;
static struct proc_dir_entry *reg1, *reg2, *reg3, *reg4, *reg5, *INR_proc_dir;
//*****************************************************************************************************************
/**
*  proc write function
*
*/
ssize_t
SPI_file_write (struct file *file, const char *buffer, size_t count,
		loff_t * data)
{
  procfs_buffer_size = count;
  if (procfs_buffer_size > PROCFS_MAX_SIZE)
    {
      procfs_buffer_size = PROCFS_MAX_SIZE;
    }
  if (copy_from_user (procfs_buffer, buffer, procfs_buffer_size))
    {
      return -EFAULT;
    }
  uint32_t tmp = 0;
  RT_SPI_file_write (procfs_buffer, procfs_buffer_size);
  return procfs_buffer_size;
}

//*****************************************************************************************************************
/**
*  proc print function
*
*/
static int
SPI_file_proc_show (struct seq_file *m, void *v)
{
  seq_printf (m, "0x%x\n", 0);

  return 0;
}

//*****************************************************************************************************************
/**
*  proc open function
*
*/
static int
SPI_file_proc_open (struct inode *inode, struct file *file)
{
  return single_open (file, SPI_file_proc_show, NULL);
}

static const struct file_operations SPI_file = {
  .owner = THIS_MODULE,
  .open = SPI_file_proc_open,
  .write = SPI_file_write,
  .read = seq_read,
  .llseek = seq_lseek,
  .release = single_release,
};

//*****************************************************************************************************************
/**
*  proc write function
*
*/
ssize_t
SPI_data_write (struct file *file, const char *buffer, size_t count,
		loff_t * data)
{
  procfs_buffer_size = count;
  if (procfs_buffer_size > PROCFS_MAX_SIZE)
    {
      procfs_buffer_size = PROCFS_MAX_SIZE;
    }
  if (copy_from_user (procfs_buffer, buffer, procfs_buffer_size))
    {
      return -EFAULT;
    }
  uint32_t tmp = 0;
  sscanf (procfs_buffer, "%d", &tmp);
  RT_SPI_set_w_data (tmp);
  return procfs_buffer_size;
}

//*****************************************************************************************************************
/**
*  proc print function
*
*/
static int
SPI_data_proc_show (struct seq_file *m, void *v)
{
  seq_printf (m, "0x%x\n", RT_SPI_get_r_data ());

  return 0;
}

//*****************************************************************************************************************
/**
*  proc open function
*
*/
static int
SPI_data_proc_open (struct inode *inode, struct file *file)
{
  return single_open (file, SPI_data_proc_show, NULL);
}

static const struct file_operations SPI_data = {
  .owner = THIS_MODULE,
  .open = SPI_data_proc_open,
  .write = SPI_data_write,
  .read = seq_read,
  .llseek = seq_lseek,
  .release = single_release,
};

//*****************************************************************************************************************
/**
*  proc write function
*
*/
ssize_t
SPI_write_write (struct file *file, const char *buffer, size_t count,
		 loff_t * data)
{
  procfs_buffer_size = count;
  if (procfs_buffer_size > PROCFS_MAX_SIZE)
    {
      procfs_buffer_size = PROCFS_MAX_SIZE;
    }
  if (copy_from_user (procfs_buffer, buffer, procfs_buffer_size))
    {
      return -EFAULT;
    }
  uint32_t tmp = 0;
  sscanf (procfs_buffer, "%d", &tmp);
  RT_SPI_proc_write (tmp);
  return procfs_buffer_size;
}

//*****************************************************************************************************************
/**
*  proc print function
*
*/
static int
SPI_write_proc_show (struct seq_file *m, void *v)
{
  seq_printf (m, "%i\n", RT_SPI_get_semaphor ());

  return 0;
}

//*****************************************************************************************************************
/**
*  proc open function
*
*/
static int
SPI_write_proc_open (struct inode *inode, struct file *file)
{
  return single_open (file, SPI_write_proc_show, NULL);
}

static const struct file_operations SPI_write = {
  .owner = THIS_MODULE,
  .open = SPI_write_proc_open,
  .write = SPI_write_write,
  .read = seq_read,
  .llseek = seq_lseek,
  .release = single_release,
};

//*****************************************************************************************************************
/**
*  proc write function
*
*/
ssize_t
SPI_read_write (struct file *file, const char *buffer, size_t count,
		loff_t * data)
{
  procfs_buffer_size = count;
  if (procfs_buffer_size > PROCFS_MAX_SIZE)
    {
      procfs_buffer_size = PROCFS_MAX_SIZE;
    }
  if (copy_from_user (procfs_buffer, buffer, procfs_buffer_size))
    {
      return -EFAULT;
    }
  uint32_t tmp = 0;
  sscanf (procfs_buffer, "%d", &tmp);
  RT_SPI_proc_read (tmp);
  return procfs_buffer_size;
}

//*****************************************************************************************************************
/**
*  proc print function
*
*/
static int
SPI_read_proc_show (struct seq_file *m, void *v)
{
  seq_printf (m, "%i\n", RT_SPI_get_semaphor ());

  return 0;
}

//*****************************************************************************************************************
/**
*  proc open function
*
*/
static int
SPI_read_proc_open (struct inode *inode, struct file *file)
{
  return single_open (file, SPI_read_proc_show, NULL);
}

static const struct file_operations SPI_read = {
  .owner = THIS_MODULE,
  .open = SPI_read_proc_open,
  .write = SPI_read_write,
  .read = seq_read,
  .llseek = seq_lseek,
  .release = single_release,
};

//*****************************************************************************************************************
/**
*  proc write function
*
*/
ssize_t
RT_enable_write (struct file *file, const char *buffer, size_t count,
		 loff_t * data)
{
  procfs_buffer_size = count;
  if (procfs_buffer_size > PROCFS_MAX_SIZE)
    {
      procfs_buffer_size = PROCFS_MAX_SIZE;
    }
  if (copy_from_user (procfs_buffer, buffer, procfs_buffer_size))
    {
      return -EFAULT;
    }
  uint32_t tmp = 0;
  sscanf (procfs_buffer, "%d", &tmp);
  if (tmp)
    {
      RT_enable_fkt ();
    }
  else
    {
      RT_disable_fkt ();
    }
  return procfs_buffer_size;
}

//*****************************************************************************************************************
/**
*  proc print function
*
*/
static int
RT_enable_proc_show (struct seq_file *m, void *v)
{
  seq_printf (m, "%i\n", 1234);

  return 0;
}

//*****************************************************************************************************************
/**
*  proc open function
*
*/
static int
RT_enable_proc_open (struct inode *inode, struct file *file)
{
  return single_open (file, RT_enable_proc_show, NULL);
}

static const struct file_operations RT_enable = {
  .owner = THIS_MODULE,
  .open = RT_enable_proc_open,
  .write = RT_enable_write,
  .read = seq_read,
  .llseek = seq_lseek,
  .release = single_release,
};

int
PROC_FS_init ()
{
  INR_proc_dir = proc_mkdir ("InnoRoute", NULL);
  if (!INR_proc_dir)
    {
      printk (KERN_ALERT "Error creating proc entry");
      return -ENOMEM;
    }

  reg1 = proc_create ("RT_enable", 0644, INR_proc_dir, &RT_enable);
  if (reg1 == NULL)
    {
      remove_proc_entry ("RT_enable", INR_proc_dir);
      printk (KERN_ALERT "Error: Could not initialize /proc/%s\n",
	      "RT_enable");
      return -ENOMEM;
    }
  printk (KERN_INFO "/proc/%s created\n", "RT_enable");

  reg2 = proc_create ("SPI_read", 0644, INR_proc_dir, &SPI_read);
  if (reg2 == NULL)
    {
      remove_proc_entry ("SPI_read", INR_proc_dir);
      printk (KERN_ALERT "Error: Could not initialize /proc/%s\n",
	      "SPI_read");
      return -ENOMEM;
    }
  printk (KERN_INFO "/proc/%s created\n", "SPI_read");

  reg3 = proc_create ("SPI_write", 0644, INR_proc_dir, &SPI_write);
  if (reg3 == NULL)
    {
      remove_proc_entry ("SPI_write", INR_proc_dir);
      printk (KERN_ALERT "Error: Could not initialize /proc/%s\n",
	      "SPI_write");
      return -ENOMEM;
    }
  printk (KERN_INFO "/proc/%s created\n", "SPI_write");

  reg4 = proc_create ("SPI_data", 0644, INR_proc_dir, &SPI_data);
  if (reg4 == NULL)
    {
      remove_proc_entry ("SPI_data", INR_proc_dir);
      printk (KERN_ALERT "Error: Could not initialize /proc/%s\n",
	      "SPI_data");
      return -ENOMEM;
    }
  printk (KERN_INFO "/proc/%s created\n", "SPI_data");
  reg5 = proc_create ("SPI_file", 0644, INR_proc_dir, &SPI_file);
  if (reg5 == NULL)
    {
      remove_proc_entry ("SPI_file", INR_proc_dir);
      printk (KERN_ALERT "Error: Could not initialize /proc/%s\n",
	      "SPI_file");
      return -ENOMEM;
    }
  printk (KERN_INFO "/proc/%s created\n", "SPI_file");

  return 0;
}

void
PROC_FS_exit ()
{
  remove_proc_entry ("RT_enable", INR_proc_dir);
  printk (KERN_INFO "/proc/InnoRoute/%s removed\n", "RT_enable");
  remove_proc_entry ("SPI_read", INR_proc_dir);
  printk (KERN_INFO "/proc/InnoRoute/%s removed\n", "SPI_read");
  remove_proc_entry ("SPI_write", INR_proc_dir);
  printk (KERN_INFO "/proc/InnoRoute/%s removed\n", "SPI_write");
  remove_proc_entry ("SPI_data", INR_proc_dir);
  printk (KERN_INFO "/proc/InnoRoute/%s removed\n", "SPI_data");
  remove_proc_entry ("SPI_file", INR_proc_dir);
  printk (KERN_INFO "/proc/InnoRoute/%s removed\n", "SPI_file");

  remove_proc_entry ("InnoRoute", NULL);
  printk (KERN_INFO "/proc/%s removed\n", "InnoRoute");
}
