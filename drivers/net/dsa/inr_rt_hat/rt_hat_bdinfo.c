/**
*@file
*@brief Realtimehat dsa-bridge definition
*@author M.Ulbricht 2021 <ulbricht@innoroute.de>
*@copyright GNU Public License v3.
*
**/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/phy.h>
#include <net/dsa.h>

#include "rt_hat.h"

static struct rt_hat_pdata rt_hat_pdata = {
  .cd = {
	 .port_names[0] = "RT0",
	 .port_names[1] = "RT2",
	 .port_names[RT_HAT_CPU_PORT] = "cpu",
	 },
  .name = "Realtime HAT driver",
  .enabled_ports = 0x1f,
  .netdev = "eth0",
};

static const struct mdio_board_info bdinfo = {
  .bus_id = "fixed-0",
  .modalias = "rt-hat",
  .mdio_addr = 31,
  .platform_data = &rt_hat_pdata,
};

static int __init
rt_hat_bdinfo_init (void)
{
  if (DEBUG)
    printk (KERN_DEBUG "DEBUG: Passed %s %d \n", __FUNCTION__, __LINE__);
  return mdiobus_register_board_info (&bdinfo, 1);
}

arch_initcall (rt_hat_bdinfo_init) MODULE_LICENSE ("GPL");
