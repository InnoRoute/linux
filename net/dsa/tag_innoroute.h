/**
*@file
*@brief InnoRoute Realtime-Hat DSA-tag definitions
*@author M.Ulbricht 2021
*@copyright GNU Public License v3.
*
**/
#define INR_DSA_ETH_TYPE 0x813E
#define INR_debug_RX 0
#define INR_debug_TX 0
#define DEBUG 1
#define DSA_TAG_VERSION 3


struct INR_TIME_timestamps
{
  uint64_t bridge;
  uint64_t controlled;
};



#if DSA_TAG_VERSION == 1
#define INR_TAG_LEN	16
struct INR_DSA_TAG_RX		// to cpu
{

  uint16_t DSA_ETH_TYPE;
  uint8_t INGRESS_PORT:6;
  uint8_t EGRESS_PORT:6;
  uint8_t PROC_RSN:3;
  uint8_t BAD_0:1;
  uint64_t RX_timestamp;
  uint32_t reserved;

} __attribute__((__packed__, scalar_storage_order ("big-endian")));
struct INR_DSA_TAG_TX		//from CPU
{

  uint32_t DSA_ETH_TYPE:16;
  uint32_t INGRESS_PORT:6;
  uint32_t EGRESS_PORT:6;
  uint32_t STREAM_Q:4;
  uint32_t TX_TIMESTAMP;
  uint32_t TX_CONFIRMATION_ID;
  uint32_t reserved;

} __attribute__((__packed__, scalar_storage_order ("big-endian")));
#endif
#if DSA_TAG_VERSION == 2
#define INR_TAG_LEN	16
struct INR_DSA_TAG_RX		// to cpu
{

  uint16_t DSA_ETH_TYPE;
  uint8_t INGRESS_PORT:6;
  uint8_t EGRESS_PORT:6;
  uint8_t PROC_RSN:3;
  uint8_t BAD_0:1;
  uint32_t TXF_BRIDGE_timestamp;
  uint32_t RX_timestamp;
  uint32_t TXF_CTRL_timestamp;

} __attribute__((__packed__, scalar_storage_order ("big-endian")));
struct INR_DSA_TAG_TX		//from CPU
{

  uint32_t DSA_ETH_TYPE:16;
  uint32_t INGRESS_PORT:6;
  uint32_t EGRESS_PORT:6;
  uint32_t STREAM_Q:4;
  uint32_t TX_TIMESTAMP;
  uint32_t TX_CONFIRMATION_ID;
  uint32_t reserved;

} __attribute__((__packed__, scalar_storage_order ("big-endian")));
#endif

#if DSA_TAG_VERSION == 3
#define INR_TAG_LEN	24
struct INR_DSA_TAG_RX		// to cpu
{

  uint16_t DSA_ETH_TYPE;
  uint8_t INGRESS_PORT:6;
  uint8_t EGRESS_PORT:6;
  uint8_t PROC_RSN:3;
  uint8_t BAD_0:1;
  uint32_t BRIDGE_timestamp;
  uint32_t RX_timestamp;
  uint64_t CTL_timestamp;
  uint32_t reserved3;


} __attribute__((__packed__, scalar_storage_order ("big-endian")));
struct INR_DSA_TAG_TX		//from CPU
{

  uint32_t DSA_ETH_TYPE:16;
  uint32_t INGRESS_PORT:6;
  uint32_t EGRESS_PORT:6;
  uint32_t STREAM_Q:4;
  uint32_t TX_TIMESTAMP;
  uint32_t TX_CONFIRMATION_ID;
  uint32_t reserved;

} __attribute__((__packed__, scalar_storage_order ("big-endian")));
#endif
#if DSA_TAG_VERSION == 4
#define INR_TAG_LEN	24
struct INR_DSA_TAG_RX		// to cpu
{

  uint32_t DSA_ETH_TYPE:16;
  uint32_t INGRESS_PORT:5;
  uint32_t EGRESS_PORT:5;
  uint32_t PROC_RSN:5;
  uint32_t BAD_0:1;
  uint32_t BRIDGE_timestamp;
  uint32_t RX_timestamp;
  uint64_t CTL_timestamp;
  uint32_t reserved3;


} __attribute__((__packed__, scalar_storage_order ("big-endian")));
struct INR_DSA_TAG_TX		//from CPU
{

  uint32_t DSA_ETH_TYPE:16;
  uint32_t INGRESS_PORT:5;
  uint32_t EGRESS_PORT:5;
  uint32_t STREAM_Q:5;
  uint32_t DELAY_pkt:1;
  uint32_t TX_TIMESTAMP;
  uint32_t TX_CONFIRMATION_ID;
  uint32_t reserved;

} __attribute__((__packed__, scalar_storage_order ("big-endian")));
#endif
