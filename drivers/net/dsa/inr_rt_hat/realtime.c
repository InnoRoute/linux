/**
*@file
*@brief realtime functions
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
#include <uapi/linux/net_tstamp.h>
#include <linux/spinlock.h>
#include <linux/kthread.h>
#include "rt_procfs.h"
#include "spi.h"
#include "realtime.h"
#include <asm/div64.h>
DEFINE_SPINLOCK (hardwareLock);
unsigned long flags;
struct INR_TIME_TX_entry INR_TIME_vortex[INR_TIME_vortex_length];	//no, its not bigger on the inside :D
uint16_t INR_TIME_TX_vortex_current = 1;
volatile uint16_t INR_TIME_TX_vortex_lastread = 1;
static DECLARE_WAIT_QUEUE_HEAD (INR_RT_tx_ts_waittingqueu);
static DECLARE_WAIT_QUEUE_HEAD (INR_RT_tx_ts_force_waittingqueu);
static DEFINE_SPINLOCK (tx_ts_lock);
struct ptp_clock *ptp_clock;
struct ptp_clock_info ptp_caps;
uint64_t CTRLD_rate = 0x5000000;
uint64_t CTRLD_offset = 0;
uint8_t TIME_DBG_mod = 0;
uint8_t pollcount = 0;
unsigned long flags;
volatile uint8_t clock_registred = 0;
uint8_t USE_ctrl_bridge_clock_offset = 1;
uint64_t timediff = 1;
uint8_t clockjump = 0;
DEFINE_SEMAPHORE (INR_TIME_TX_transmit_interrupt_sem);
DEFINE_SEMAPHORE (INR_TIME_TX_add_sem);
//*****************************************************************************************************************
/**
*Clear Timevortex
*@brief the timevortex stores the skb's of requested tx-timestamps. If ptp restarts, this skb are not valid anymore and have to be removed.
*/
void
INR_TIME_clear_vortex ()
{
  printk (KERN_DEBUG "DEBUG: Passed %s %d \n", __FUNCTION__, __LINE__);
  uint16_t i = 0;



  if (INR_TIME_TX_vortex_current < INR_TIME_TX_vortex_lastread)
    {
      for (i = INR_TIME_TX_vortex_current; i <= INR_TIME_TX_vortex_lastread;
	   i++)
	INR_TIME_vortex[i].skb = NULL;
    }
  else
    {

      for (i = INR_TIME_TX_vortex_lastread; i <= INR_TIME_TX_vortex_current;
	   i++)
	INR_TIME_vortex[i].skb = NULL;
    }

  //wake_up_interruptible (&INR_RT_tx_ts_force_waittingqueu);
  //INR_TIME_TX_vortex_current=1;
  INR_TIME_TX_vortex_lastread=INR_TIME_TX_vortex_current;
}

EXPORT_SYMBOL (INR_TIME_clear_vortex);
//*****************************************************************************************************************
/**
*request tx_confirmation timestamps from hardware
*@brief called from interrupt
*/
void
INR_TIME_TX_transmit_interrupt (uint8_t port)
{
  if (DEBUG)
    printk (KERN_DEBUG "DEBUG: Passed %s %d \n", __FUNCTION__, __LINE__);
  //while(down_trylock(&INR_TIME_TX_transmit_interrupt_sem)); // prevent raceconditions
  down_killable (&INR_TIME_TX_transmit_interrupt_sem);
  //if(down_trylock(&INR_TIME_TX_transmit_interrupt_sem)){
  uint32_t entry_current = 1, tmp = 0, tmp2 = 0;
  uint32_t portmap = 0;
  uint32_t timestamp_L = 0;
  uint32_t timestamp_H = 0;
  uint64_t timestamp = 0;
  uint8_t last = 0;
  uint8_t i = 0;
  if (INR_TIME_enable)
    {
      while ((entry_current > 0) && (last == 0))
	{
	  entry_current =
	    INR_SPI_MMI_read ((C_BASE_ADDR_NET_LOWER << 8) +
			      C_SUB_ADDR_NET_TX_CONF_L + ((port * 4) +
							  0) * 4);
	  if (entry_current & (1 << 16))
	    last = 1;
	  entry_current &= 0xffff;
	  if (entry_current)
	    {
	      INR_TIME_TX_vortex_lastread = entry_current;
	      timestamp_L =
		INR_SPI_MMI_read ((C_BASE_ADDR_NET_LOWER << 8) +
				  C_SUB_ADDR_NET_TX_CONF_L + ((port * 4) +
							      1) * 4);
	      timestamp_H =
		INR_SPI_MMI_read ((C_BASE_ADDR_NET_LOWER << 8) +
				  C_SUB_ADDR_NET_TX_CONF_L + ((port * 4) +
							      2) * 4);
	      timestamp =
		(0x00000000ffffffff&(uint64_t) timestamp_L) | ((uint64_t) timestamp_H << 32);
	      //if (TS_DEBUG)
		if (TS_DEBUG)printk (KERN_DEBUG
			"Port: %i TX entry id: %i timestamp:0x%16llx\n", port,
			entry_current, timestamp);
	      if (INR_TIME_vortex[entry_current].used)
		{
		  unsigned long flags;
		  spin_lock_irqsave (&tx_ts_lock, flags);
		  if (!INR_TIME_vortex[entry_current].skb){
		  printk (KERN_DEBUG"error: TXtime got empty skb.\n");
		  goto unlock;
		  
		  }
		    
		  struct skb_shared_hwtstamps *skbtimestamp =
		    skb_hwtstamps (INR_TIME_vortex[entry_current].skb);
		  memset (skbtimestamp, 0,
			  sizeof (struct skb_shared_hwtstamps));

		  skbtimestamp->hwtstamp = ns_to_ktime (timestamp);	//0: insert timestamp here...

		  if (TS_DEBUG)
		    printk (KERN_DEBUG
			    "Write timestamp to skb ns:%llu, ktime:%llu\n",
			    timestamp, skbtimestamp->hwtstamp);

		  //if(INR_TIME_vortex[entry_current].skb->sk->sk_error_queue)
		  skb_tstamp_tx (INR_TIME_vortex[entry_current].skb,
				 skbtimestamp);
		  //dev_kfree_skb_any(INR_TIME_vortex[entry_current].skb);
		  INR_TIME_vortex[entry_current].used = 0;
		  INR_TIME_vortex[entry_current].skb = NULL;
		unlock:
		  spin_unlock_irqrestore (&tx_ts_lock, flags);
		}
	      else if (DEBUG)
		printk (KERN_DEBUG
			"error: TXtime got point to empty timevortex.\n");
	    }


	}
    }
  up (&INR_TIME_TX_transmit_interrupt_sem);
}

//*****************************************************************************************************************
/**
*del ptp clock device
*@brief
*/
void
INR_TIME_remove_ptp_clock ()
{

  if (clock_registred)
    ptp_clock_unregister (ptp_clock);
  clock_registred = 0;


}

//*****************************************************************************************************************
/**
*init ptp clock device
*@brief
*/
void
INR_TIME_init_ptp_clock (struct device *dev)
{
  init_interrupts ();
  if (clock_registred == 0)
    {
      snprintf (ptp_caps.name, 16, "%s", "RealTimeHAT");
      ptp_caps.owner = THIS_MODULE;
      ptp_caps.max_adj = 250000000;
      ptp_caps.n_alarm = 0;
      ptp_caps.n_ext_ts = 0;
      ptp_caps.n_per_out = 0;
      ptp_caps.pps = 1;
      ptp_caps.adjfreq = INR_TIME_ptp_adjfreq;
      ptp_caps.adjtime = INR_TIME_ptp_adjtime;
      ptp_caps.gettime64 = INR_TIME_ptp_gettime;
      ptp_caps.settime64 = INR_TIME_ptp_settime;
      ptp_caps.enable = INR_TIME_ptp_enable;

      ptp_clock = ptp_clock_register (&ptp_caps, dev);	//register always on TN0
      if (IS_ERR (ptp_clock))
	{
	  ptp_clock = NULL;
	  if (TS_DEBUG)
	    printk (KERN_DEBUG "ptp_clock_register failed\n");
	  return;
	}
      else if (TS_DEBUG)
	printk (KERN_DEBUG "registered PHC device\n");
      clock_registred = 1;
      //memset(&cc, 0, sizeof(cc));
      uint64_t BRIDGE_clock_value = 0, CTRLD_clock_value = 0;
      uint32_t BRIDGE_clock_value_L = 0, CTRLD_clock_value_L =
	0, BRIDGE_clock_value_H = 0, CTRLD_clock_value_H = 0;

      spin_lock_irqsave (&hardwareLock, flags);
      BRIDGE_clock_value_L =
	INR_SPI_MMI_read ((C_BASE_ADDR_RTC << 8) + C_SUB_ADDR_RTC_BRIDGE_LOW);
      BRIDGE_clock_value_H =
	INR_SPI_MMI_read ((C_BASE_ADDR_RTC << 8) +
			  C_SUB_ADDR_RTC_BRIDGE_HIGH);
      CTRLD_clock_value_L =
	INR_SPI_MMI_read ((C_BASE_ADDR_RTC << 8) + C_SUB_ADDR_RTC_CTRLD_LOW);
      CTRLD_clock_value_H =
	INR_SPI_MMI_read ((C_BASE_ADDR_RTC << 8) + C_SUB_ADDR_RTC_CTRLD_HIGH);
      spin_unlock_irqrestore (&hardwareLock, flags);

      BRIDGE_clock_value =
	BRIDGE_clock_value_L | ((uint64_t) BRIDGE_clock_value_H << 32);
      CTRLD_clock_value =
	CTRLD_clock_value_L | ((uint64_t) CTRLD_clock_value_H << 32);
      //cc.read = INR_TIME_cyclecounter_read;
      //cc.mask = CYCLECOUNTER_MASK(64);//trustnode count ns in 32 bit
      //cc.shift = 0;//innoroute counter runns in ns
      //cc.mult = 1;
      //timecounter_init(&tc,&cc,CTRLD_clock_value);
      INR_TIME_ptp_adjtime (NULL, 0);	//synchonize controlled and freeruning clock
    }

  uint64_t time_granularity = 5;
#ifdef C_SUB_ADDR_TM_SCHED_TAS_TICK_GRANULARITY
  time_granularity =
    INR_SPI_MMI_read ((C_BASE_ADDR_TM_SCHED_LOWER << 8) +
		      C_SUB_ADDR_TM_SCHED_TAS_TICK_GRANULARITY);
#endif
  if (time_granularity > 100)
    {
      if (TS_DEBUG)
	printk (KERN_DEBUG
		"error: Strange value %i for TAS time granularity, will be reset to 5ns",
		time_granularity);
      time_granularity = 5;	//plausicheck               
    }
  CTRLD_rate = (time_granularity << 24);
  if (TS_DEBUG)
    printk (KERN_DEBUG "Set Clock CTRL rate to 0x%llx", CTRLD_rate);

}

EXPORT_SYMBOL (INR_TIME_init_ptp_clock);
//*****************************************************************************************************************
/**
*adjust ptp frequency
*@brief clock running faster or slower
*/
static int
INR_TIME_ptp_adjfreq (struct ptp_clock_info *ptp, s32 ppb)
{
//ppb is from base frequency
  uint64_t freq = 1;

  uint32_t incval = CTRLD_rate;
  uint32_t diff = 0;
  uint8_t neg_adj = 0;

  if (ppb < 0)
    {
      neg_adj = 1;
      ppb = -ppb;
    }

  //CTRLD_rate+=ppb;
  freq *= ppb;
  freq = freq << 1;
  uint32_t rem = do_div (freq, 1000000000ULL);
  diff = (freq << 24) | ((rem) >> 3);
  incval = neg_adj ? (incval - diff) : (incval + diff);
  //incval+=3656;
  INR_SPI_MMI_write (incval,
		     (C_BASE_ADDR_RTC << 8) + C_SUB_ADDR_RTC_CTRLD_RATE);
  if (TS_DEBUG)
    printk (KERN_DEBUG
	    "PTP adjfreq called new rate:0x%lx, diff:%lu ppb:%li neg:%u\n",
	    incval, diff, ppb, neg_adj);
  return 0;

  return 1;
}

//*****************************************************************************************************************
/**
*adjust ptp clock
*@brief clock offset change
*/
static int
INR_TIME_ptp_adjtime (struct ptp_clock_info *ptp, s64 delta)
{
//this is maybe not neccesarry...

  CTRLD_offset += delta;
  clockjump = 1;
  spin_lock_irqsave (&hardwareLock, flags);
  INR_SPI_MMI_write (CTRLD_offset & 0xffffffff,
		     (C_BASE_ADDR_RTC << 8) +
		     C_SUB_ADDR_RTC_CTRLD_OFFSET_LOW);
  INR_SPI_MMI_write ((CTRLD_offset >> 32) & 0xffffffff,
		     (C_BASE_ADDR_RTC << 8) +
		     C_SUB_ADDR_RTC_CTRLD_OFFSET_HIGH);
  //INR_SPI_MMI_write(1,(C_BASE_ADDR_RTC<<8)+  C_SUB_ADDR_RTC_CLKSEL);
  spin_unlock_irqrestore (&hardwareLock, flags);
  if (TS_DEBUG)
    printk (KERN_DEBUG "PTP adjtime called new offset:%llu delta:%lli\n",
	    CTRLD_offset, delta);

//endo of maybe not neccesarry


  //timecounter_adjtime(&tc,delta);
  return 0;
}

//*****************************************************************************************************************
/**
*adjust timerequest from userspace
*@brief provide time
*/
static int
INR_TIME_ptp_gettime (struct ptp_clock_info *ptp, struct timespec64 *ts)
{
  uint64_t ns = 0, ns_l = 0, ns_h = 0;

  int64_t offset = 0;
  uint64_t BRIDGE_clock_value = 0, CTRLD_clock_value = 0;
  uint32_t BRIDGE_clock_value_L = 0, CTRLD_clock_value_L =
    0, BRIDGE_clock_value_H = 0, CTRLD_clock_value_H = 0;

  spin_lock_irqsave (&hardwareLock, flags);
  BRIDGE_clock_value_L =
    INR_SPI_MMI_read ((C_BASE_ADDR_RTC << 8) + C_SUB_ADDR_RTC_BRIDGE_LOW);
  BRIDGE_clock_value_H =
    INR_SPI_MMI_read ((C_BASE_ADDR_RTC << 8) + C_SUB_ADDR_RTC_BRIDGE_HIGH);
  CTRLD_clock_value_L =
    INR_SPI_MMI_read ((C_BASE_ADDR_RTC << 8) + C_SUB_ADDR_RTC_CTRLD_LOW);
  CTRLD_clock_value_H =
    INR_SPI_MMI_read ((C_BASE_ADDR_RTC << 8) + C_SUB_ADDR_RTC_CTRLD_HIGH);
  spin_unlock_irqrestore (&hardwareLock, flags);

  BRIDGE_clock_value =
    BRIDGE_clock_value_L | ((uint64_t) BRIDGE_clock_value_H << 32);
  CTRLD_clock_value =
    CTRLD_clock_value_L | ((uint64_t) CTRLD_clock_value_H << 32);

  //ns=timecounter_read(&tc);
  ns = CTRLD_clock_value;
  *ts = ns_to_timespec64 (ns);
  //if (TIME_DBG_mod)
  if (TS_DEBUG)
    printk (KERN_DEBUG "PTP get time called value:%lli\n", ns);
  return 0;
}

//*****************************************************************************************************************
/**
*time set from userspace
*@brief clock running faster or slower
*/
static int
INR_TIME_ptp_settime (struct ptp_clock_info *ptp, const struct timespec64 *ts)
{
  //if (TIME_DBG_mod)
  if (TS_DEBUG)
    printk (KERN_DEBUG "PTP set time called vaule:%lli\n",
	    timespec64_to_ns (ts));
  uint64_t newvalue;



  newvalue = timespec64_to_ns (ts);
  spin_lock_irqsave (&hardwareLock, flags);
  INR_SPI_MMI_write (newvalue & 0xffffffff,
		     (C_BASE_ADDR_RTC << 8) +
		     C_SUB_ADDR_RTC_CTRLD_OFFSET_LOW);
  INR_SPI_MMI_write ((newvalue >> 32) & 0xffffffff,
		     (C_BASE_ADDR_RTC << 8) +
		     C_SUB_ADDR_RTC_CTRLD_OFFSET_HIGH);
  // INR_SPI_MMI_write(1,(C_BASE_ADDR_RTC<<8)+  C_SUB_ADDR_RTC_CLKSEL);
  spin_unlock_irqrestore (&hardwareLock, flags);
  if (TS_DEBUG)
    printk (KERN_DEBUG "PTP adjtime called new value:%lli\n", newvalue);

  return 0;
}

//*****************************************************************************************************************
/**
*enable ptp clock
*@brief enable ptp clock
*/
static int
INR_TIME_ptp_enable (struct ptp_clock_info *ptp, struct ptp_clock_request *rq,
		     int on)
{

  return 0;
}

//*****************************************************************************************************************
/**
* wakeup tx confirmation request thread
*@brief function neede because waitinqueue is not global
*/
void
INR_RT_tx_ts_tread_wakeup ()
{

  wake_up_interruptible (&INR_RT_tx_ts_waittingqueu);
}

//*****************************************************************************************************************
/**
*thread for calling all tx confirmation queues
*@brief thread needed because calling spi actions from interrupt, got to sleep after one run
*/
int
INR_RT_tx_ts_thread (void *nix)
{
  DECLARE_WAITQUEUE (wait1, current);
  allow_signal (SIGKILL);
  add_wait_queue (&INR_RT_tx_ts_waittingqueu, &wait1);
  while (1)
    {
      set_current_state (TASK_INTERRUPTIBLE);
      schedule ();
      INR_TIME_TX_transmit_interrupt (0);
      INR_TIME_TX_transmit_interrupt (1);
      INR_TIME_TX_transmit_interrupt (2);

      if (signal_pending (current))
	break;			//exit on thermination

    }
  set_current_state (TASK_RUNNING);
  remove_wait_queue (&INR_RT_tx_ts_waittingqueu, &wait1);
  return 0;
}

//*****************************************************************************************************************
/**
*thread for forcing interupts
*@brief thread needed because calling spi actions from interrupt
*/
int
INR_RT_tx_ts_force_thread (void *nix)	// contains empirical thesholds..but it works ;)
{
  DECLARE_WAITQUEUE (wait2, current);
  allow_signal (SIGKILL);
  add_wait_queue (&INR_RT_tx_ts_force_waittingqueu, &wait2);
  while (1)
    {
      set_current_state (TASK_INTERRUPTIBLE);
      schedule ();
      //INR_SPI_MMI_write(0x100,(C_BASE_ADDR_SPI_LOWER<<8)+C_SUB_ADDR_SPI_INT_STATUS);// force interrupt
      if (signal_pending (current))
	break;			//exit on thermination

    }
  set_current_state (TASK_RUNNING);
  remove_wait_queue (&INR_RT_tx_ts_force_waittingqueu, &wait2);
  return 0;
}

//*****************************************************************************************************************
/**
*adds an TX entry to the wait for timestamp queue
*@brief need to keep the skb until transmit timestamp is reported
*/
uint16_t
INR_TIME_TX_add (struct sk_buff *skb)
{
  uint16_t waiting_queue_length = 0;
  if (INR_TIME_enable)
    {
      down_killable (&INR_TIME_TX_add_sem);
      INR_TIME_TX_vortex_current++;
      if (INR_TIME_TX_vortex_current == 0)
	INR_TIME_TX_vortex_current = 1;
/*        if(INR_TIME_vortex[INR_TIME_TX_vortex_current].used) { //overrun detected, clear entry*/
/*            if(INR_TIME_vortex[INR_TIME_TX_vortex_current].skb)skb_tx_timestamp(INR_TIME_vortex[INR_TIME_TX_vortex_current].skb);*/
/*        }*/
      INR_TIME_vortex[INR_TIME_TX_vortex_current].skb = skb;
      INR_TIME_vortex[INR_TIME_TX_vortex_current].used = 1;
      if (INR_TIME_TX_vortex_current < INR_TIME_TX_vortex_lastread)
	{
	  waiting_queue_length =
	    65535 - INR_TIME_TX_vortex_lastread + INR_TIME_TX_vortex_current;
	}
      else
	{
	  waiting_queue_length =
	    INR_TIME_TX_vortex_current - INR_TIME_TX_vortex_lastread;
	}
      if (waiting_queue_length > MAX_TIME_TX_vortex_queue)
	{
	printk (KERN_DEBUG "manually wakeup TX confirmation \n");
	  INR_RT_tx_ts_tread_wakeup ();	// if waiting for to much interrupts, call isr
	  //wake_up_interruptible (&INR_RT_tx_ts_force_waittingqueu);
	}
      if (DEBUG)
	printk (KERN_DEBUG
		"Net TX skb stored in timevortex at position %i waiting for %i\n",
		INR_TIME_TX_vortex_current, waiting_queue_length);
up (&INR_TIME_TX_add_sem);
      return INR_TIME_TX_vortex_current;
    }
  return 0;
}

EXPORT_SYMBOL (INR_TIME_TX_add);
//*****************************************************************************************************************
/**
*return ptp clock device information
*@brief
*/
struct ptp_clock *
INR_TIME_get_ptp_clock ()
{
  return ptp_clock;
}

EXPORT_SYMBOL (INR_TIME_get_ptp_clock);
//*****************************************************************************************************************
/**
*calculate the currect timestamp from half freerunning clock
*@brief most ts are 32 bit freerunning, but we need 64bit of controlled clock
*/
void
INR_TIME_correct_HW_timestamp (uint32_t hw_value,
			       struct INR_TIME_timestamps *ts)
{
  uint64_t offset = 0;
  uint8_t neg = 0;
  uint64_t newvalue;
  uint64_t BRIDGE_clock_value = 0, CTRLD_clock_value = 0;
  uint32_t BRIDGE_clock_value_L = 0, CTRLD_clock_value_L =
    0, BRIDGE_clock_value_H = 0, CTRLD_clock_value_H = 0;


  spin_lock_irqsave (&hardwareLock, flags);
  BRIDGE_clock_value_L =
    INR_SPI_MMI_read ((C_BASE_ADDR_RTC << 8) + C_SUB_ADDR_RTC_BRIDGE_LOW);
  BRIDGE_clock_value_H =
    INR_SPI_MMI_read ((C_BASE_ADDR_RTC << 8) + C_SUB_ADDR_RTC_BRIDGE_HIGH);
  CTRLD_clock_value_L =
    INR_SPI_MMI_read ((C_BASE_ADDR_RTC << 8) + C_SUB_ADDR_RTC_CTRLD_LOW);
  CTRLD_clock_value_H =
    INR_SPI_MMI_read ((C_BASE_ADDR_RTC << 8) + C_SUB_ADDR_RTC_CTRLD_HIGH);
  spin_unlock_irqrestore (&hardwareLock, flags);



  BRIDGE_clock_value =
    (uint64_t) BRIDGE_clock_value_L | ((uint64_t) BRIDGE_clock_value_H << 32);
  CTRLD_clock_value =
    (uint64_t) CTRLD_clock_value_L | ((uint64_t) CTRLD_clock_value_H << 32);


  if (CTRLD_clock_value < BRIDGE_clock_value)
    neg = 1;
  if (neg)
    offset = (BRIDGE_clock_value - CTRLD_clock_value);
  else
    offset = (CTRLD_clock_value - BRIDGE_clock_value);



  if ((BRIDGE_clock_value & 0xffffffff) < (uint64_t) hw_value)
    BRIDGE_clock_value -= 0x100000000;	//there was an overflow, i asume just one and not several times 4 sec
  //if((CTRLD_clock_value&0xffffffff)<(u64)hw_value)CTRLD_clock_value-=0x100000000; //there was an overflow, i asume just one and not several times 4 sec
  newvalue =
    ((BRIDGE_clock_value & 0xffffffff00000000) | (uint64_t) hw_value);
  ts->bridge = newvalue;
  if (neg)
    newvalue -= offset;
  else
    newvalue += offset;
  //if(bridgeclock)
  ts->controlled = newvalue;
  if (TS_DEBUG)
    printk (KERN_DEBUG
	    "TIME adjust value ..CTRLD_clock:%llu BRIDGE_clock:%llu negative:%u offset:%llu pkt_value:%lu ts_bridge:%llu ts_ctrld:%llu\n",
	    CTRLD_clock_value, BRIDGE_clock_value, neg, offset, hw_value,
	    ts->bridge, ts->controlled);
  //else
  //  return (((CTRLD_clock_value&0xffffffff00000000)|(u64)hw_value));
  //return timecounter_cyc2time(&tc,(u64)hw_value+abs(offset));
}

EXPORT_SYMBOL (INR_TIME_correct_HW_timestamp);
//*****************************************************************************************************************
/**
*enable function
*@brief gets called from proc-fs 
*/
void
RT_enable_fkt ()
{

  RT_SPI_init ();


}

//*****************************************************************************************************************
/**
*exit function
*@brief 
*/
void
RT_disable_fkt ()
{
  printk (KERN_DEBUG "Disable INR-RealTime-HAT SPI Module\n");
  RT_SPI_exit ();


}

//*****************************************************************************************************************
/**
*init function
*@brief 
*/
static int __init
RT_init (void)
{
  printk (KERN_DEBUG "Init INR-RealTime-HAT SPI Module\n");
  PROC_FS_init ();
  kthread_run (&INR_RT_tx_ts_thread, NULL, "INR_RT_tx_ts_thread");
  kthread_run (&INR_RT_tx_ts_force_thread, NULL, "INR_RT_tx_ts_force_thread");



  return 0;
}

module_init (RT_init);
//*****************************************************************************************************************
/**
*exit function
*@brief 
*/
static void __exit
RT_exit (void)
{
  printk (KERN_DEBUG "Remove INR-RealTime-HAT SPI Module\n");

  PROC_FS_exit ();


}

module_exit (RT_exit);

MODULE_LICENSE ("GPL");
MODULE_AUTHOR ("Marian Ulbricht");
MODULE_DESCRIPTION ("Realtime HAT driver spi driver");
