/**
*@file
*@brief InnoRoute Realtime-Hat DSA-tag
*@author M.Ulbricht 2021
*@copyright GNU Public License v3.
*
**/

#include <linux/etherdevice.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include "dsa_priv.h"
#include "tag_innoroute.h"
DEFINE_SPINLOCK (tx_lock);
unsigned long flags;
/* This tag length is 4 bytes, older ones were 6 bytes, we do not
 * handle them
 */


/* Tag is constructed and desconstructed using byte by byte access
 * because the tag is placed after the MAC Source Address, which does
 * not make it 4-bytes aligned, so this might cause unaligned accesses
 * on most systems where this is used.
 */

extern void INR_TIME_correct_HW_timestamp (uint32_t hw_value, struct INR_TIME_timestamps *ts);	// import from spi module
extern uint16_t INR_TIME_TX_add (struct sk_buff *skb);


//*****************************************************************************************************************
/**
*tx function lowlevel
*@brief inserting the DSA tag
*/
static struct sk_buff *
INR_tag_xmit_ll (struct sk_buff *skb,
		 struct net_device *dev, unsigned int offset)
{
  struct dsa_port *dp = dsa_slave_to_port (dev);
  u16 queue = skb_get_queue_mapping (skb);
  uint8_t is_ptp = 0;
  skb_linearize (skb);
  uint16_t ethertype = ((uint16_t) skb->data[13] << 8) + skb->data[12];
  if (INR_debug_TX)
    printk ("Ethertype:%x\n", ethertype);
  if ((ethertype == 0xf788))
    {				//is ptp packet  //||(ethertype==0x8)
      //time_queue=PTP_prio;
      is_ptp = 1;
    }
    

//spin_lock_irqsave(&tx_lock, flags);


  // from CPU
//if(DEBUG)printk(KERN_DEBUG "DEBUG: Passed %s %d por:%i\n",__FUNCTION__,__LINE__,dp->index);
  if (skb_cow_head (skb, INR_TAG_LEN) < 0){printk(KERN_ERR "tx packet dropped: cow_head\n");
  //spin_unlock_irqrestore(&tx_lock, flags);
  return NULL;}
    

  /* The Ethernet switch we are interfaced with needs packets to be at
   * least 64 bytes (including FCS) otherwise they will be discarded when
   * they enter the switch port logic. When Broadcom tags are enabled, we
   * need to make sure that packets are at least 68 bytes
   * (including FCS and tag) because the length verification is done after
   * the Broadcom tag is stripped off the ingress packet.
   *
   * Let dsa_slave_xmit() free the SKB
   */
  if (__skb_put_padto (skb, ETH_ZLEN + INR_TAG_LEN, false)){printk(KERN_ERR "tx packet dropped: padto\n");
  //spin_unlock_irqrestore(&tx_lock, flags);
  return NULL;}
    

  skb_push (skb, INR_TAG_LEN);

  if (offset)
    memmove (skb->data, skb->data + INR_TAG_LEN, offset);

  struct INR_DSA_TAG_TX *INR_tag =
    (struct INR_DSA_TAG_TX *) (skb->data + offset);

  /* Set DSA tag information
   */
  INR_tag->DSA_ETH_TYPE = INR_DSA_ETH_TYPE;
/*			switch(dp->index){*/
/*		case 0:INR_tag->INGRESS_PORT=3;break;*/
/*		case 1:INR_tag->INGRESS_PORT=4;break;*/
/*	default: INR_tag->INGRESS_PORT=0;break;*/
/*	}*/
/*	*/
/*	switch(dp->index){*/
/*		case 0:INR_tag->EGRESS_PORT=3;break;*/
/*		case 1:INR_tag->EGRESS_PORT=4;break;*/
/*	default: INR_tag->EGRESS_PORT=0;break;*/
/*	}*/


  INR_tag->INGRESS_PORT = 0x3f;
  switch (dp->index)
    {
    case 0:
      INR_tag->EGRESS_PORT = 0;
      break;
    case 1:
      INR_tag->EGRESS_PORT = 1;
      break;
    default:
      INR_tag->EGRESS_PORT = 0;
      break;
    }
  INR_tag->STREAM_Q = 0x0;	
  if(skb->priority>7)
  	INR_tag->STREAM_Q=7;
  else INR_tag->STREAM_Q=skb->priority&0x7;
  //printk("SKB_prio:%i\n",skb->priority);
  INR_tag->TX_TIMESTAMP = 0xFFFFFFFF;	//don't care'
  if (is_ptp)
    {				// if requested, ask for timestamp (skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP)||)
      INR_tag->TX_CONFIRMATION_ID = INR_TIME_TX_add (skb);
      skb_shinfo (skb)->tx_flags |= SKBTX_IN_PROGRESS;

    }
  else
    {
      INR_tag->TX_CONFIRMATION_ID = 0;

    }
  INR_tag->reserved = 0;
  if (INR_debug_TX)
    {
      printk (KERN_ERR "INR DSR TX PORT:%i  %i conf_id:%i\n",
	      INR_tag->EGRESS_PORT, dp->index, INR_tag->TX_CONFIRMATION_ID);
    }
  /* Now tell the master network device about the desired output queue
   * as well
   */
  //skb_set_queue_mapping(skb, INR_TAG_SET_PORT_QUEUE(dp->index, queue));
  skb_tx_timestamp (skb);
  
  //spin_unlock_irqrestore(&tx_lock, flags);
  return skb;
}

//*****************************************************************************************************************
/**
*rx function lowlevel
*@brief removing the DSA tag
*/
static struct sk_buff *
INR_tag_rcv_ll (struct sk_buff *skb,
		struct net_device *dev,
		struct packet_type *pt, unsigned int offset)
{
  int source_port;
  uint8_t is_ptp = 0;
  skb_linearize (skb);
//if(DEBUG)printk(KERN_DEBUG "DEBUG: Passed %s %d \n",__FUNCTION__,__LINE__);
  if (unlikely (!pskb_may_pull (skb, INR_TAG_LEN))){printk (KERN_ERR "drop may pull\n");return NULL;}
    

  struct INR_DSA_TAG_RX *INR_tag =
    (struct INR_DSA_TAG_RX *) (skb->data - offset);
  if (INR_tag->DSA_ETH_TYPE != INR_DSA_ETH_TYPE)
    {
      printk (KERN_ERR "drop non DSA packet\n");
      return NULL;
    }
/* Handle DSA tag information */
  if (INR_debug_RX)
    {
      printk (KERN_ERR "INR DSR RX PORT:%i\n", INR_tag->INGRESS_PORT);

    }
  //if(INR_tag->EGRESS_PORT!=0x3f) return NULL;
  switch (INR_tag->INGRESS_PORT)
    {
    case 0:
      source_port = 0;
      break;
    case 1:
      source_port = 1;
      break;
    default:
      source_port = 0;
      break;
    }

  skb->dev = dsa_master_find_slave (dev, 0, source_port);
  if (!skb->dev){printk (KERN_ERR "drop no master dev\n");return NULL;}
    

  /* Remove InnoRoute tag and update checksum */
  skb_pull_rcsum (skb, INR_TAG_LEN);

  skb->offload_fwd_mark = 1;
  //##timestamping
  struct INR_TIME_timestamps ts;
#if DSA_TAG_VERSION == 1
  INR_TIME_correct_HW_timestamp (INR_tag->RX_timestamp, &ts, 0);
#endif
#if DSA_TAG_VERSION == 2
  INR_TIME_correct_HW_timestamp (INR_tag->TXF_CTRL_timestamp -
				 (INR_tag->TXF_BRIDGE_timestamp -
				  INR_tag->RX_timestamp), &ts, 0);
#endif
  struct skb_shared_hwtstamps *skbtimestamp = skb_hwtstamps (skb);
  memset (skbtimestamp, 0, sizeof (struct skb_shared_hwtstamps));
#if DSA_TAG_VERSION == 1
  skbtimestamp->hwtstamp = ns_to_ktime ((u64) ts.controlled);
#endif
#if DSA_TAG_VERSION == 2
  skbtimestamp->hwtstamp = ns_to_ktime ((u64) ts.controlled);
#endif
#if DSA_TAG_VERSION == 3
  skbtimestamp->hwtstamp =
    ns_to_ktime ((u64) INR_tag->CTL_timestamp -
		 ((u64) INR_tag->BRIDGE_timestamp -
		  (u64) INR_tag->RX_timestamp));
#endif
  if (INR_debug_RX)
    printk (KERN_ERR
	    "RX: skb_ts:%llu CTL_timestamp:%llu BRIDGE_timestamp:%llu RX_timestamp:%llu\n",
	    skbtimestamp->hwtstamp, (u64) INR_tag->CTL_timestamp,
	    (u64) INR_tag->BRIDGE_timestamp, (u64) INR_tag->RX_timestamp);
  __net_timestamp (skb);

  return skb;
}

//*****************************************************************************************************************
/**
*tx function highlevel
*@brief call lowlevel
*/
static struct sk_buff *
INR_tag_xmit (struct sk_buff *skb, struct net_device *dev)
{
//if(DEBUG)printk(KERN_DEBUG "DEBUG: Passed %s %d \n",__FUNCTION__,__LINE__);
  /* Build the tag after the MAC Source Address */
  return INR_tag_xmit_ll (skb, dev, 2 * ETH_ALEN);
}

//*****************************************************************************************************************
/**
*rx function highlevel
*@brief call lowlevel
*/
static struct sk_buff *
INR_tag_rcv (struct sk_buff *skb, struct net_device *dev,
	     struct packet_type *pt)
{
  struct sk_buff *nskb;
//if(DEBUG)printk(KERN_DEBUG "DEBUG: Passed %s %d \n",__FUNCTION__,__LINE__);
  // skb->data points to the EtherType, the tag is right before it 
  nskb = INR_tag_rcv_ll (skb, dev, pt, 2);
  if (!nskb)
    return nskb;

  // Move the Ethernet DA and SA 
  memmove (nskb->data - ETH_HLEN, nskb->data - ETH_HLEN - INR_TAG_LEN,
	   2 * ETH_ALEN);

  return nskb;
}

static const struct dsa_device_ops INR_netdev_ops = {
  .name = "INR",
  .proto = DSA_TAG_PROTO_INR,
  .xmit = INR_tag_xmit,
  .rcv = INR_tag_rcv,
  .overhead = INR_TAG_LEN,
};

DSA_TAG_DRIVER (INR_netdev_ops);
MODULE_ALIAS_DSA_TAG_DRIVER (DSA_TAG_PROTO_INR);




static struct dsa_tag_driver *dsa_tag_driver_array[] = {

  &DSA_TAG_DRIVER_NAME (INR_netdev_ops),


};

module_dsa_tag_drivers (dsa_tag_driver_array);
MODULE_SOFTDEP ("pre: INR_spi");
MODULE_LICENSE ("GPL");
