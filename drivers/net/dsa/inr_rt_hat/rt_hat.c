/**
*@file
*@brief Distributed Switch Architecture driver for InnoRoute Realtime HAT
*@author M.Ulbricht 2021 <ulbricht@innoroute.de>
*@copyright GNU Public License v3.
*
**/
#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/phy_fixed.h>
#include <linux/export.h>
#include <linux/ethtool.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/if_bridge.h>
#include <net/dsa.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/timex.h>
#include <linux/net_tstamp.h>

#include "rt_hat.h"
extern struct ptp_clock *INR_TIME_get_ptp_clock (void);
extern void INR_TIME_init_ptp_clock (struct device *dev);
extern void INR_TIME_clear_vortex (void);

struct hwtstamp_config INR_tstamp_config;


struct rt_hat_vlan {
    u16 members;
    u16 untagged;
};

struct rt_hat_mib_entry {
    char name[ETH_GSTRING_LEN];
    unsigned long val;
};

enum rt_hat_mib_counters {
    RT_HAT_PHY_READ_OK,
    RT_HAT_PHY_READ_ERR,
    RT_HAT_PHY_WRITE_OK,
    RT_HAT_PHY_WRITE_ERR,
    __RT_HAT_CNT_MAX,
};

static struct rt_hat_mib_entry rt_hat_mibs[] = {
    [RT_HAT_PHY_READ_OK] = {"phy_read_ok",},
    [RT_HAT_PHY_READ_ERR] = {"phy_read_err",},
    [RT_HAT_PHY_WRITE_OK] = {"phy_write_ok",},
    [RT_HAT_PHY_WRITE_ERR] = {"phy_write_err",},
};

struct rt_hat_port {
    struct rt_hat_mib_entry mib[__RT_HAT_CNT_MAX];
};

#define RT_HAT_VLANS	5

struct rt_hat_priv {
    struct mii_bus *bus;
    unsigned int port_base;
    struct rt_hat_vlan vlans[RT_HAT_VLANS];
    struct net_device *netdev;
    struct rt_hat_port ports[DSA_MAX_PORTS];
    u16 pvid;
};

static struct phy_device *phydevs[PHY_MAX_ADDR];

static enum dsa_tag_protocol
rt_hat_get_protocol (struct dsa_switch *ds, int port) {
    dev_dbg (ds->dev, "%s: port: %d\n", __func__, port);
    if (DEBUG)
        printk (KERN_DEBUG "DEBUG: Passed %s %d \n", __FUNCTION__, __LINE__);
    if (port == RT_HAT_CPU_PORT)
    {
        return DSA_TAG_PROTO_INR;	//DSA_TAG_PROTO_INR; DSA_TAG_PROTO_NONE
    } else
    {
        return DSA_TAG_PROTO_NONE;
    }
}

static int
rt_hat_setup (struct dsa_switch *ds)
{
    struct rt_hat_priv *ps = ds->priv;
    unsigned int i;

    for (i = 0; i < ds->num_ports; i++)
        memcpy (ps->ports[i].mib, rt_hat_mibs, sizeof (rt_hat_mibs));

    if (DEBUG)
        printk (KERN_DEBUG "DEBUG: Passed %s %d \n", __FUNCTION__, __LINE__);

    return 0;
}

static int
rt_hat_get_sset_count (struct dsa_switch *ds, int port, int sset)
{
    if (sset != ETH_SS_STATS && sset != ETH_SS_PHY_STATS)
        return 0;

    return __RT_HAT_CNT_MAX;
}

static void
rt_hat_get_strings (struct dsa_switch *ds, int port,
                    u32 stringset, uint8_t * data)
{
    struct rt_hat_priv *ps = ds->priv;
    unsigned int i;

    if (stringset != ETH_SS_STATS && stringset != ETH_SS_PHY_STATS)
        return;

    for (i = 0; i < __RT_HAT_CNT_MAX; i++)
        memcpy (data + i * ETH_GSTRING_LEN,
                ps->ports[port].mib[i].name, ETH_GSTRING_LEN);
}

static void
rt_hat_get_ethtool_stats (struct dsa_switch *ds, int port, uint64_t * data)
{
    struct rt_hat_priv *ps = ds->priv;
    unsigned int i;

    for (i = 0; i < __RT_HAT_CNT_MAX; i++)
        data[i] = ps->ports[port].mib[i].val;
}

static int
rt_hat_phy_read (struct dsa_switch *ds, int port, int regnum)
{
    struct rt_hat_priv *ps = ds->priv;
    struct mii_bus *bus = ps->bus;
    int ret;

    ret = mdiobus_read_nested (bus, ps->port_base + port, regnum);
    if (ret < 0)
        ps->ports[port].mib[RT_HAT_PHY_READ_ERR].val++;
    else
        ps->ports[port].mib[RT_HAT_PHY_READ_OK].val++;

    return ret;
}

static int
rt_hat_phy_write (struct dsa_switch *ds, int port, int regnum, u16 value)
{
    struct rt_hat_priv *ps = ds->priv;
    struct mii_bus *bus = ps->bus;
    int ret;

    ret = mdiobus_write_nested (bus, ps->port_base + port, regnum, value);
    if (ret < 0)
        ps->ports[port].mib[RT_HAT_PHY_WRITE_ERR].val++;
    else
        ps->ports[port].mib[RT_HAT_PHY_WRITE_OK].val++;

    return ret;
}

static int
rt_hat_port_bridge_join (struct dsa_switch *ds, int port,
                         struct net_device *bridge)
{
    dev_dbg (ds->dev, "%s: port: %d, bridge: %s\n",
             __func__, port, bridge->name);

    return 0;
}

static void
rt_hat_port_bridge_leave (struct dsa_switch *ds, int port,
                          struct net_device *bridge)
{
    dev_dbg (ds->dev, "%s: port: %d, bridge: %s\n",
             __func__, port, bridge->name);
}

static void
rt_hat_port_stp_state_set (struct dsa_switch *ds, int port, u8 state)
{
    dev_dbg (ds->dev, "%s: port: %d, state: %d\n", __func__, port, state);
}

static int
rt_hat_port_vlan_filtering (struct dsa_switch *ds, int port,
                            bool vlan_filtering)
{
    dev_dbg (ds->dev, "%s: port: %d, vlan_filtering: %d\n",
             __func__, port, vlan_filtering);

    return 0;
}

static int
rt_hat_port_vlan_prepare (struct dsa_switch *ds, int port,
                          const struct switchdev_obj_port_vlan *vlan)
{
    struct rt_hat_priv *ps = ds->priv;
    struct mii_bus *bus = ps->bus;

    dev_dbg (ds->dev, "%s: port: %d, vlan: %d-%d",
             __func__, port, vlan->vid_begin, vlan->vid_end);

    /* Just do a sleeping operation to make lockdep checks effective */
    mdiobus_read (bus, ps->port_base + port, MII_BMSR);

    if (vlan->vid_end > RT_HAT_VLANS)
        return -ERANGE;

    return 0;
}

static void
rt_hat_port_vlan_add (struct dsa_switch *ds, int port,
                      const struct switchdev_obj_port_vlan *vlan)
{
    bool untagged = vlan->flags & BRIDGE_VLAN_INFO_UNTAGGED;
    bool pvid = vlan->flags & BRIDGE_VLAN_INFO_PVID;
    struct rt_hat_priv *ps = ds->priv;
    struct mii_bus *bus = ps->bus;
    struct rt_hat_vlan *vl;
    u16 vid;
    if (DEBUG)
        printk (KERN_DEBUG "DEBUG: Passed %s %d \n", __FUNCTION__, __LINE__);
    /* Just do a sleeping operation to make lockdep checks effective */
    mdiobus_read (bus, ps->port_base + port, MII_BMSR);

    for (vid = vlan->vid_begin; vid <= vlan->vid_end; ++vid) {
        vl = &ps->vlans[vid];

        vl->members |= BIT (port);
        if (untagged)
            vl->untagged |= BIT (port);
        else
            vl->untagged &= ~BIT (port);

        dev_dbg (ds->dev, "%s: port: %d vlan: %d, %stagged, pvid: %d\n",
                 __func__, port, vid, untagged ? "un" : "", pvid);
    }

    if (pvid)
        ps->pvid = vid;
}

static int
rt_hat_port_vlan_del (struct dsa_switch *ds, int port,
                      const struct switchdev_obj_port_vlan *vlan)
{
    bool untagged = vlan->flags & BRIDGE_VLAN_INFO_UNTAGGED;
    struct rt_hat_priv *ps = ds->priv;
    struct mii_bus *bus = ps->bus;
    struct rt_hat_vlan *vl;
    u16 vid, pvid = ps->pvid;
    if (DEBUG)
        printk (KERN_DEBUG "DEBUG: Passed %s %d \n", __FUNCTION__, __LINE__);
    /* Just do a sleeping operation to make lockdep checks effective */
    mdiobus_read (bus, ps->port_base + port, MII_BMSR);

    for (vid = vlan->vid_begin; vid <= vlan->vid_end; ++vid) {
        vl = &ps->vlans[vid];

        vl->members &= ~BIT (port);
        if (untagged)
            vl->untagged &= ~BIT (port);

        if (pvid == vid)
            pvid = 1;

        dev_dbg (ds->dev, "%s: port: %d vlan: %d, %stagged, pvid: %d\n",
                 __func__, port, vid, untagged ? "un" : "", pvid);
    }
    ps->pvid = pvid;

    return 0;
}

int
INR_RT_hwtstamp_set (struct dsa_switch *ds, int port, struct ifreq *ifr)
{


    if (DEBUG)
        printk (KERN_DEBUG "DEBUG: Passed %s %d \n", __FUNCTION__, __LINE__);
    INR_TIME_clear_vortex ();
    return 0;

}



int
INR_RT_get_ts_info (struct dsa_switch *ds, int port,
                    struct ethtool_ts_info *info)
{
    if (DEBUG)
        printk (KERN_DEBUG "DEBUG: Passed %s %d \n", __FUNCTION__, __LINE__);
    info->so_timestamping = SOF_TIMESTAMPING_TX_SOFTWARE | SOF_TIMESTAMPING_RX_SOFTWARE | SOF_TIMESTAMPING_SOFTWARE;	// |

    info->so_timestamping |=
        SOF_TIMESTAMPING_TX_HARDWARE |
        SOF_TIMESTAMPING_RX_HARDWARE | SOF_TIMESTAMPING_RAW_HARDWARE;

    info->tx_types = 0;		// (1<<HWTSTAMP_TX_OFF) |        (1<<HWTSTAMP_TX_ON);
    info->so_timestamping |= 0;	//SOF_TIMESTAMPING_TX_HARDWARE;


    info->phc_index = ptp_clock_index (INR_TIME_get_ptp_clock ());


    info->tx_types = (1 << HWTSTAMP_TX_OFF);
    info->tx_types |= (1 << HWTSTAMP_TX_ON);

    info->rx_filters = (1 << HWTSTAMP_FILTER_NONE);
    info->rx_filters |= (1 << HWTSTAMP_FILTER_ALL);

    return 0;
}

static const struct dsa_switch_ops rt_hat_driver = {
    .get_tag_protocol = rt_hat_get_protocol,
    .setup = rt_hat_setup,
    .get_strings = rt_hat_get_strings,
    .get_ethtool_stats = rt_hat_get_ethtool_stats,
    .get_sset_count = rt_hat_get_sset_count,
    .get_ethtool_phy_stats = rt_hat_get_ethtool_stats,
    .phy_read = rt_hat_phy_read,
    .phy_write = rt_hat_phy_write,
    .port_bridge_join = rt_hat_port_bridge_join,
    .port_bridge_leave = rt_hat_port_bridge_leave,
    .port_stp_state_set = rt_hat_port_stp_state_set,
//      .port_vlan_filtering    = rt_hat_port_vlan_filtering,
//      .port_vlan_prepare      = rt_hat_port_vlan_prepare,
//      .port_vlan_add          = rt_hat_port_vlan_add,
//      .port_vlan_del          = rt_hat_port_vlan_del,
//      .port_hwtstamp_set      = mv88e6xxx_port_hwtstamp_set,
//      .port_hwtstamp_get      = mv88e6xxx_port_hwtstamp_get,
//      .port_txtstamp          = mv88e6xxx_port_txtstamp,
//      .port_rxtstamp          = mv88e6xxx_port_rxtstamp,
    .get_ts_info = INR_RT_get_ts_info,
    .port_hwtstamp_set = INR_RT_hwtstamp_set,
};

static int
rt_hat_drv_probe (struct mdio_device *mdiodev)
{
    struct rt_hat_pdata *pdata = mdiodev->dev.platform_data;
    struct rt_hat_priv *ps;
    struct dsa_switch *ds;
    if (DEBUG)
        printk (KERN_DEBUG "DEBUG: Passed %s %d \n", __FUNCTION__, __LINE__);
    if (!pdata)
        return -ENODEV;

    dev_info (&mdiodev->dev, "%s: 0x%0x\n", pdata->name, pdata->enabled_ports);

    ds = dsa_switch_alloc (&mdiodev->dev, DSA_MAX_PORTS);
    if (!ds)
        return -ENOMEM;

    ps = devm_kzalloc (&mdiodev->dev, sizeof (*ps), GFP_KERNEL);
    if (!ps)
        return -ENOMEM;

    ps->netdev = dev_get_by_name (&init_net, pdata->netdev);
    if (!ps->netdev)
        return -EPROBE_DEFER;

    pdata->cd.netdev[RT_HAT_CPU_PORT] = &ps->netdev->dev;

    ds->dev = &mdiodev->dev;
    ds->ops = &rt_hat_driver;
    ds->priv = ps;
    ps->bus = mdiodev->bus;
    ds->num_tx_queues = RTHAT_NUM_EGRESS_QUEUES;

    dev_set_drvdata (&mdiodev->dev, ds);
    INR_TIME_init_ptp_clock (&mdiodev->dev);
    return dsa_register_switch (ds);
}

static void
rt_hat_drv_remove (struct mdio_device *mdiodev)
{
    struct dsa_switch *ds = dev_get_drvdata (&mdiodev->dev);
    struct rt_hat_priv *ps = ds->priv;
    if (DEBUG)
        printk (KERN_DEBUG "DEBUG: Passed %s %d \n", __FUNCTION__, __LINE__);
    dsa_unregister_switch (ds);
    dev_put (ps->netdev);

}

static struct mdio_driver rt_hat_drv = {
    .mdiodrv.driver = {
        .name = "rt-hat",
    },
    .probe = rt_hat_drv_probe,
    .remove = rt_hat_drv_remove,
};

#define NUM_FIXED_PHYS	(RT_HAT_NUM_PORTS - 1)

static int __init
rt_hat_init (void)
{
    if (DEBUG)
        printk (KERN_DEBUG "DEBUG: Passed %s %d \n", __FUNCTION__, __LINE__);
    struct fixed_phy_status status = {
        .link = 1,
        .speed = SPEED_100,
        .duplex = DUPLEX_FULL,
    };
    unsigned int i;
    for (i = 0; i < NUM_FIXED_PHYS; i++)
        phydevs[i] = fixed_phy_register (PHY_POLL, &status, NULL);


    return mdio_driver_register (&rt_hat_drv);
}

module_init (rt_hat_init);

static void __exit
rt_hat_exit (void)
{
    unsigned int i;
    if (DEBUG)
        printk (KERN_DEBUG "DEBUG: Passed %s %d \n", __FUNCTION__, __LINE__);
    mdio_driver_unregister (&rt_hat_drv);
    for (i = 0; i < NUM_FIXED_PHYS; i++)
        if (!IS_ERR (phydevs[i]))
            fixed_phy_unregister (phydevs[i]);
}

module_exit (rt_hat_exit);

MODULE_SOFTDEP ("pre: rt_hat_bdinfo");
MODULE_LICENSE ("GPL");
MODULE_AUTHOR ("Marian Ulbricht");
MODULE_DESCRIPTION ("DSA Realtime HAT driver");
