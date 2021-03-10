/**
*@file
*@brief bridge port definitions
*@author M.Ulbricht 2021
*@copyright GNU Public License v3.
*
**/
#define DEBUG 0
#ifndef __RT_HAT_H
#define __RT_HAT_H

struct dsa_chip_data;

struct rt_hat_pdata
{
  /* Must be first, such that dsa_register_switch() can access this
   * without gory pointer manipulations
   */
  struct dsa_chip_data cd;
  const char *name;
  unsigned int enabled_ports;
  const char *netdev;
};

#define RT_HAT_NUM_PORTS	3
#define RT_HAT_CPU_PORT (RT_HAT_NUM_PORTS-1)

#endif /* __RT_HAT_H */
