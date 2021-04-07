/**
*@file
*@brief realtime definitions
*@author M.Ulbricht 2021
*@copyright GNU Public License v3.
*
**/
#include "tn_env.h"

#define INR_TIME_enable 1
#define TS_DEBUG 0
#define RTC_DEBUG 0

#define INR_TIME_vortex_length 65536
#define INR_TIME_base 1
#define INR_TIME_MAX_pollcount 50
#define MAX_TIME_TX_vortex_queue 3


void RT_enable_fkt (void);
void RT_disable_fkt (void);

struct INR_TIME_TX_entry
{
    struct sk_buff *skb;	 /**<store skb*/
    uint8_t used;	       /**<valid flag*/
    uint32_t timestamp;
};

struct INR_TIME_timestamps
{
    uint64_t bridge;
    uint64_t controlled;
};

//void INR_TIME_correct_HW_timestamp (uint32_t hw_value, struct INR_TIME_timestamps *ts);
uint16_t INR_TIME_TX_add (struct sk_buff *skb);
void INR_RT_tx_ts_tread_wakeup (void);
struct ptp_clock *INR_TIME_get_ptp_clock (void);
void INR_TIME_init_ptp_clock (struct device *dev);
static int INR_TIME_ptp_adjfreq (struct ptp_clock_info *ptp, s32 ppb);
static int INR_TIME_ptp_adjtime (struct ptp_clock_info *ptp, s64 delta);
static int INR_TIME_ptp_gettime (struct ptp_clock_info *ptp,
                                 struct timespec64 *ts);
static int INR_TIME_ptp_settime (struct ptp_clock_info *ptp,
                                 const struct timespec64 *ts);
static int INR_TIME_ptp_enable (struct ptp_clock_info *ptp,
                                struct ptp_clock_request *rq, int on);
void INR_TIME_clear_vortex (void);
struct ptp_clock *INR_TIME_get_ptp_clock (void);
void INR_TIME_TX_transmit_interrupt (uint8_t port);
void INR_TIME_remove_ptp_clock (void);
#define C_MAX_PORT_COUNT 4
#define C_PROC_PORTS 1
#define C_MMI_INT_HC_LOWER  0
#define C_MMI_INT_HC_UPPER 17
#define C_MMI_INT_MDIO 18
#define C_MMI_INT_RTC 19
#define C_MMI_INT_PERIPHERALS 20
#define C_MMI_INT_DEBUG 21
#define C_MMI_INT_COMMON 22
#define C_MMI_INT_ETH_SW 23
#define C_MMI_INT_CAN 24

#ifndef C_BASE_ADDR_BM
#define C_BASE_ADDR_BM 0
#define ENABLE 0
#endif

#ifndef C_SUB_ADDR_BM_MMI_INTERRUPT
#define C_SUB_ADDR_BM_MMI_INTERRUPT 0
#define ENABLE 0
#endif

#ifndef C_SUB_ADDR_MAC_SPEED
#define C_SUB_ADDR_MAC_SPEED 0
#define ENABLE 0
#endif

#ifndef C_SUB_ADDR_MDIO_INTERRUPT_EN
#define C_SUB_ADDR_MDIO_INTERRUPT_EN 0
#define ENABLE 0
#endif

#ifdef C_BASE_ADDR_NET_LOWER
#define ENABLE 1
#endif

#ifndef ENABLE
#define ENABLE 1
#endif

#define INR_HC_BASE(id) (C_BASE_ADDR_HC_ ## id )
#define INR_HC_INTERRUPT_EN(id) (INR_HC_BASE(id)<<8)+C_SUB_ADDR_HC_INTERRUPT_EN
#define INR_MDIO_set_FPGA_speed_to_PHY 1
