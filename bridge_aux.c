/**
 * \file bridge_aux.c
 *         Routines to interface between Ethernet and 6LowPan, copied from
 *         sicslowmac.c
 *
 */

/* Copyright (c) 2008 by:
 * Colin O'Flynn coflynn@newae.com
 * Eric Gnoske egnoske@gmail.com
 * Blake Leverett bleverett@gmail.com
 * Mike Vidales mavida404@gmail.com
 * Kevin Brown kbrown3@uccs.edu
 * Nate Bohlmann nate@elfwerks.com
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in
 * the documentation and/or other materials provided with the
 * distribution.
 * * Neither the name of the copyright holders nor the names of
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
   \par Ethernet to 6LowPan Address Translation

   It should be obvious that since 802.15.4 addresses are 8 
   bytes, and 802.3 addresses are 6 bytes, some form of 
   address translation is needed. These routines provide this

   \par 802.3 Address Formats

   802.3 MAC addresses used here have this form:

   \verbatim
   +----+----+----+----+----+----+----+----+
   +    +    +    +    +    + TR + GL + MU +
   +----+----+----+----+----+----+----+----+
   \endverbatim


   It can be seen this is like a normal ethernet MAC address, 
   with GL being the Global/Local bit, and MU being the
   Multicast/Unicast bit.

   The addition is the 'TR' bit, which if set indicates that
   the address must be translated when going between 802.15.4
   and 802.3. 

   \par Address Translation

   If the TRANSLATE (TR) bit is CLEAR, this means the 5th and
   4th LSBytes of the 802.15.4 address are fffe, aka the address
   has the hexidecial form:
   
   xxxxxxfffexxxxxx

   \note
   You should always aim to set the 802.15.4 addresses
   of the devices on your network to ones that will
   satisfy this requirement. Some examples are:
   \note
   0x02 23 42 ff fe 73 92 28
   \note
   0x82 00 82 ff fe cd ee 22

   \note
   So the most significant octets MUST 
   have bit 0 CLEAR, bit 1 SET, and bit 2 CLEAR. The remaining
   bits in this octet can be anything.

   If the TRANSLATE bit is SET, this means the address on the 
   802.3 side does not directly convert to an 802.15.4 address.
   To translate it, the remainder of the octet is used as an
   index in a look-up table. This look-up table simply stores
   the 4th, 5th, and 8th octet of the 802.15.4 address, and attaches
   them to the remaining 5 bytes of the 802.3 address.

   In this way there can be 32 different 802.15.4 'prefixes', 
   requiring only 96 bytes of RAM in a storage table on the
   802.3 to 802.15.4 bridge. 
  
   Mulitcast addresses on 802.3 are mapped to broadcast addresses on
   802.15.4 and vis-versa. Since IPv6 does not use 802.3 broadcast, 
   this code will drop all 802.3 broadcast packets. They are most
   likely something unwanted, such as IPv4 packets that snuck in.

   \par Notes on how addresses are stored

   An 802.15.4 address will be reported for example as:

   0x8877665544332211

   Stored in the array as passed to these functions, it will be:
   \verbatim
   array[0] = 0x88;
   array[1] = 0x77;
   array[2] = 0x66;
   etc.
   \endverbatim

   An 802.3 address will be reported for example as:
   02:43:53:35:45:45

   Stored in the array as passed to these functions, it will be:
   \verbatim
   array[0] = 0x02;
   array[1] = 0x43;
   array[2] = 0x53;
   array[3] = 0x35
   etc.
   \endverbatim
*/

#include "net/uip.h"
#include "uip_arp.h" //For ethernet header structure
#include "net/rime.h"
#include "sicslowpan.h"
#include "gdb2.h"

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define ETHBUF(x)    ((struct uip_eth_hdr *)x)

typedef enum {
  ll_802154_type,
  ll_8023_type
} __attribute((__packed__)) lltype_t;

static bool    mac_createSicslowpanLongAddr(uint8_t * ethernet, uip_lladdr_t * lowpan);
static uint8_t mac_createEthernetAddr(uint8_t * ethernet, uip_lladdr_t * lowpan);
static int8_t  mac_translateIPLinkLayer(lltype_t target);
static int8_t  mac_translateIcmpLinkLayer(lltype_t target);

// Location of TRANSLATE (TR) bit in Ethernet address
#define TRANSLATE_BIT_MASK (1<<2) 
// Location of LOCAL (GL) bit in Ethernet address
#define LOCAL_BIT_MASK     (1<<1)
// Location of MULTICAST (MU) bit in Ethernet address
#define MULTICAST_BIT_MASK (1<<0)

#define PREFIX_BUFFER_SIZE 32

uint8_t prefixCounter;
uint8_t prefixBuffer[PREFIX_BUFFER_SIZE][3];

/**
 * \brief   Take a packet received over the ethernet link, and send it
 * out over 802.15.4
 *
 * received ethernet frame will be in uip_buf
 */
static uip_lladdr_t*
mac_ethernetToLowpan()
{
  static uip_lladdr_t destAddr;

  // If not IPv6 we don't do anything
  if (ETHBUF(uip_buf)->type != HTONS(UIP_ETHTYPE_IPV6))
  {
    uip_log("bridge: unknown mac type\n");
    return NULL;
  }

  /* IPv6 uses 33-33-xx-xx-xx-xx prefix for multicast ND stuff */
  if ( (ETHBUF(uip_buf)->dest.addr[0] == 0x33) &&
       (ETHBUF(uip_buf)->dest.addr[1] == 0x33) )
  {
    // set destination address to broadcast
    memcpy(&destAddr, &rimeaddr_null, sizeof(destAddr));
  }
  else if (ETHBUF(uip_buf)->dest.addr[0] == 0xFF &&
           ETHBUF(uip_buf)->dest.addr[1] == 0xFF &&
           ETHBUF(uip_buf)->dest.addr[2] == 0xFF &&
           ETHBUF(uip_buf)->dest.addr[3] == 0xFF &&
           ETHBUF(uip_buf)->dest.addr[4] == 0xFF &&
           ETHBUF(uip_buf)->dest.addr[5] == 0xFF )
  {
    /* IPv6 does not use broadcast addresses, hence this should not happen */
    uip_log("bridge: ipv6 mac broadcast\n");
    return NULL;
  }
  else
  {
    // Check this returns OK
    if (!mac_createSicslowpanLongAddr( ETHBUF(uip_buf)->dest.addr, &destAddr))
    {
      uip_log("bridge: addr translation failed\n");
      return NULL; // translation failed
    }
  }

  // Remove header from length before passing onward
  // Some IP packets have link layer in them, need to change them around!
  uip_len -= UIP_LLH_LEN;
  mac_translateIPLinkLayer(ll_802154_type);

  return &destAddr;
}


/**
 * \brief Take a packet received over the 802.15.4 link, and send it
 * out over ethernet, performing any translations needed.
 *
 * ieee802.15.4 data will be delivered in rime buffer
 * ip data is in uip_buf, after ethernet header if UIP is configured correctly,
 * uip_buf will be used for ethernet frame
 */
#if UIP_LLH_LEN!=14 //sizeof(uip_eth_hdr)
# error "uip misconfigured for bridging"
#endif
void
mac_LowpanToEthernet(void)
{
  //Setup generic ethernet stuff
  ETHBUF(uip_buf)->type = htons(UIP_ETHTYPE_IPV6);

  //Check for broadcast message
  if(rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER), &rimeaddr_null)) {
    ETHBUF(uip_buf)->dest.addr[0] = 0x33;
    ETHBUF(uip_buf)->dest.addr[1] = 0x33;
    ETHBUF(uip_buf)->dest.addr[2] = UIP_IP_BUF->destipaddr.u8[12];
    ETHBUF(uip_buf)->dest.addr[3] = UIP_IP_BUF->destipaddr.u8[13];
    ETHBUF(uip_buf)->dest.addr[4] = UIP_IP_BUF->destipaddr.u8[14];
    ETHBUF(uip_buf)->dest.addr[5] = UIP_IP_BUF->destipaddr.u8[15];
  } else {
    //Otherwise we have a real address
    mac_createEthernetAddr((uint8_t *) ETHBUF(uip_buf)->dest.addr,
                           (uip_lladdr_t *)packetbuf_addr(PACKETBUF_ADDR_RECEIVER));
  }

  mac_createEthernetAddr((uint8_t *) ETHBUF(uip_buf)->src.addr,
                         (uip_lladdr_t *)packetbuf_addr(PACKETBUF_ADDR_SENDER));

  // Some IP packets have link layer in them, need to change them around!
  mac_translateIPLinkLayer(ll_8023_type);

  uip_len += UIP_LLH_LEN;
}

/**
 * \brief Translate IP packet's possible link-layer addresses, passing
 *        the message to the appropriate higher level function for this
 *        packet (aka: ICMP)
 * \param target The target we want to end up with - either ll_8023_type
 *        for ethernet, or ll_802154_type for 802.15.4
 * \return    Returns how successful the translation was
 * \retval 0  Addresses, if present, were translated.
 * \retval <0 Negative return values indicate various errors, as defined
 *            by the higher level function.
 */
static int8_t
mac_translateIPLinkLayer(lltype_t target)
{
  if (UIP_IP_BUF->proto == UIP_PROTO_ICMP6)
    return mac_translateIcmpLinkLayer(target);

  return 0;
}

#include "net/uip-icmp6.h"
#include "net/uip-nd6.h"

typedef struct {
  uint8_t type;
  uint8_t length;
  uint8_t data[16];
} icmp_opts_t;

#define UIP_ICMP_BUF     ((struct uip_icmp_hdr *)&uip_buf[UIP_LLH_LEN + UIP_IPH_LEN])
#define UIP_ICMP_OPTS(x) ((icmp_opts_t *)&uip_buf[UIP_LLH_LEN + UIP_IPH_LEN + x])

void slide(uint8_t * data, uint8_t length, int16_t slide);

/**
 * \brief Translate the link-layer (L2) addresses in an ICMP packet.
 *        This will just be NA/NS/RA/RS packets currently.
 * \param target The target we want to end up with - either ll_8023_type
 *        for ethernet, or ll_802154_type for 802.15.4
 * \return       Returns how successful the translation was
 * \retval 0     Addresses, if present, were translated.
 * \retval -1    ICMP message was unknown type, nothing done.
 * \retval -2    ICMP Length does not make sense?
 * \retval -3    Unknown 'target' type
 */
static int8_t
mac_translateIcmpLinkLayer(lltype_t target)
{
  uint16_t icmp_opt_offset=0,
           sizechange,
           iplen;
  int16_t  len = UIP_IP_BUF->len[1] | (UIP_IP_BUF->len[0] << 8);
  uint8_t  i,
           llbuf[6];

  // Figure out offset to start of options
  switch(UIP_ICMP_BUF->type) {
    case ICMP6_NS:
    case ICMP6_NA:
      icmp_opt_offset = 24;
      break;

    case ICMP6_RS:
      icmp_opt_offset = 8;
      break;

    case ICMP6_RA:
      icmp_opt_offset = 16;
      break;

    case ICMP6_REDIRECT:
      icmp_opt_offset = 40;
      break;

    /** Things without link-layer */
    case ICMP6_DST_UNREACH:
    case ICMP6_PACKET_TOO_BIG:
    case ICMP6_TIME_EXCEEDED:
    case ICMP6_PARAM_PROB:
    case ICMP6_ECHO_REQUEST:
    case ICMP6_ECHO_REPLY:
      return 0;
      break;

    default:
      return -1;
  }

  // Figure out length of options
  len -= icmp_opt_offset;

  // Sanity check
  if (len < 8) return -2;

  // While we have options to do...
  while (len >= 8)
  {
    // If we have one of these, we have something useful!
    if (((UIP_ICMP_OPTS(icmp_opt_offset)->type) == UIP_ND6_OPT_SLLAO) ||
        ((UIP_ICMP_OPTS(icmp_opt_offset)->type) == UIP_ND6_OPT_TLLAO) )
    {
      /* Shrinking the buffer may thrash things, so we store the old
         link-layer address */
      for(i=0; i<(UIP_ICMP_OPTS(icmp_opt_offset)->length*8 - 2); i++)
      {
        llbuf[i] = UIP_ICMP_OPTS(icmp_opt_offset)->data[i];
      }

      // Shrink/grow buffer as needed
      if (target == ll_802154_type)
      {
        // Current is 802.3, Hence current link-layer option is 6 extra bytes
        sizechange = 8;
        slide(UIP_ICMP_OPTS(icmp_opt_offset)->data + 6, len - 6, sizechange);
      }
      else if (target == ll_8023_type)
      {
        /* Current is 802.15.4, Hence current link-layer option is 14 extra
         * bytes.
         * (Actual LL is 8 bytes, but total option length is in multiples of
         * 8 Bytes, hence 8 + 2 = 10. Closest is 16 bytes, then 16 bytes for
         * total optional length - 2 bytes for type + length leaves 14 )
         */
        sizechange = -8;
        slide(UIP_ICMP_OPTS(icmp_opt_offset)->data + 14, len - 14, sizechange);
      }
      else
      {
        return -3; //Uh-oh!
      }

      // Translate addresses
      if (target == ll_802154_type)
        mac_createSicslowpanLongAddr(llbuf, (uip_lladdr_t *)UIP_ICMP_OPTS(icmp_opt_offset)->data);
      else
        mac_createEthernetAddr(UIP_ICMP_OPTS(icmp_opt_offset)->data, (uip_lladdr_t *)llbuf);

      // adjust the length
      if (target == ll_802154_type)
        UIP_ICMP_OPTS(icmp_opt_offset)->length = 2;
      else
        UIP_ICMP_OPTS(icmp_opt_offset)->length = 1;

      // Adjust the IP header length, as well as uIP length
      iplen = UIP_IP_BUF->len[1] | (UIP_IP_BUF->len[0]<<8);
      iplen += sizechange;
      len += sizechange;

      UIP_IP_BUF->len[1] = (uint8_t)iplen;
      UIP_IP_BUF->len[0] = (uint8_t)(iplen >> 8);

      uip_len += sizechange;

      // We broke ICMP checksum, be sure to fix that
      UIP_ICMP_BUF->icmpchksum = 0;
      UIP_ICMP_BUF->icmpchksum = ~uip_icmp6chksum();

      // Finally set up next run in while loop
      len -= 8 * UIP_ICMP_OPTS(icmp_opt_offset)->length;
      icmp_opt_offset += 8 * UIP_ICMP_OPTS(icmp_opt_offset)->length;
    }
    else
    {
      // Not an option we care about, ignore it
      len -= 8 * UIP_ICMP_OPTS(icmp_opt_offset)->length;

      // This shouldn't happen!
      if (UIP_ICMP_OPTS(icmp_opt_offset)->length == 0)
        len = 0;

      icmp_opt_offset += 8 * UIP_ICMP_OPTS(icmp_opt_offset)->length;
    }
  }

  return 0;
}


/**
 * \brief Create a 802.15.4 long address from a 802.3 address
 * \param ethernet   Pointer to ethernet address
 * \param lowpan     Pointer to 802.15.4 address
 */
bool
mac_createSicslowpanLongAddr(uint8_t * ethernet, uip_lladdr_t * lowpan)
{
  uint8_t index;

  // Special case - direct translation for our own address, which we detect by
  // a special 802.3 mac address.
  //  TODO: need a more robust way to achieve this
  if (ethernet[0] == 0x00 /*0x06*/      &&
      ethernet[1] == uip_lladdr.addr[1] &&
      ethernet[2] == uip_lladdr.addr[2] &&
      ethernet[3] == uip_lladdr.addr[5] &&
      ethernet[4] == uip_lladdr.addr[6] &&
      ethernet[5] == uip_lladdr.addr[7])
  {
      memcpy(lowpan, uip_lladdr.addr, sizeof(uip_lladdr.addr));
      return true;
  }

  // Check if translate bit is set, hence we have to look up the prefix
  if (ethernet[0] & TRANSLATE_BIT_MASK)
  {
    // Get top bits
    index = ethernet[0] >> 3;

    // Check this is plausible...
    if (index >= prefixCounter)
    {
      return false;
    }

    // Copy over prefix
    lowpan->addr[0] = prefixBuffer[index][0];
    lowpan->addr[3] = prefixBuffer[index][1];
    lowpan->addr[4] = prefixBuffer[index][2];

    // Bit is clear
    // so we copy all six
  }
  else
  {
    lowpan->addr[0] = ethernet[0];
    lowpan->addr[3] = 0xff;
    lowpan->addr[4] = 0xfe;
  }

  // Copy over remaining five bytes
  lowpan->addr[1] = ethernet[1];
  lowpan->addr[2] = ethernet[2];
  lowpan->addr[5] = ethernet[3];
  lowpan->addr[6] = ethernet[4];
  lowpan->addr[7] = ethernet[5];

  return true;
}


/**
 * \brief Create a 802.3 address from a 802.15.4 long address
 * \param ethernet   Pointer to ethernet address
 * \param lowpan     Pointer to 802.15.4 address
 */
uint8_t mac_createEthernetAddr(uint8_t * ethernet, uip_lladdr_t * lowpan)
{
  uint8_t index=0,i;

  // Special case - direct translation for our own address.
  //  TODO: need a more robust way to achieve this
  if (memcmp(lowpan, uip_lladdr.addr, sizeof(uip_lladdr.addr))==0)
  {
      ethernet[0] = 0x00 /*TRANSLATE_BIT_MASK|LOCAL_BIT_MASK*/;
      ethernet[1] = uip_lladdr.addr[1];
      ethernet[2] = uip_lladdr.addr[2];
      ethernet[3] = uip_lladdr.addr[5];
      ethernet[4] = uip_lladdr.addr[6];
      ethernet[5] = uip_lladdr.addr[7];
      return true;
  }

  //Check if we need to do anything:
  if ((lowpan->addr[3] == 0xff) && (lowpan->addr[4] == 0xfe) &&
      ((lowpan->addr[0] & TRANSLATE_BIT_MASK) == 0) &&
      ((lowpan->addr[0] & MULTICAST_BIT_MASK) == 0) &&
      (lowpan->addr[0] & LOCAL_BIT_MASK))
  {
      /** Nope: just copy over 6 bytes **/
      ethernet[0] = lowpan->addr[0];
      ethernet[1] = lowpan->addr[1];
      ethernet[2] = lowpan->addr[2];
      ethernet[3] = lowpan->addr[5];
      ethernet[4] = lowpan->addr[6];
      ethernet[5] = lowpan->addr[7];

  }
  else
  {
    /** Yes: need to store prefix **/
    for (i=0; i<prefixCounter; i++)
    {
      //Check the current prefix - if it fails, check next one
      if ((lowpan->addr[0] == prefixBuffer[i][0]) &&
          (lowpan->addr[3] == prefixBuffer[i][1]) &&
          (lowpan->addr[4] == prefixBuffer[i][2]))
      {
            break;
      }
    }

    index = i;

    //Deal with overflow, iron-fist style
    if (index >= PREFIX_BUFFER_SIZE)
    {
      index = 0;
      prefixCounter = PREFIX_BUFFER_SIZE;
    }
    else
    {
      //Are we making a new one?
      if (index == prefixCounter)
      {
        prefixCounter++;
      }
    }

    //Copy the prefix over, no matter if we have a new or old one
    prefixBuffer[index][0] = lowpan->addr[0];
    prefixBuffer[index][1] = lowpan->addr[3];
    prefixBuffer[index][2] = lowpan->addr[4];

    //Create ethernet MAC address now
    ethernet[1] = lowpan->addr[1];
    ethernet[2] = lowpan->addr[2];
    ethernet[3] = lowpan->addr[5];
    ethernet[4] = lowpan->addr[6];
    ethernet[5] = lowpan->addr[7];
    ethernet[0] = TRANSLATE_BIT_MASK | LOCAL_BIT_MASK | (index << 3);
  }

  return 1;
}


/**
 * \brief        Slide the pointed to memory up a certain amount,
 *               growing/shrinking a buffer
 * \param data   Pointer to start of data buffer
 * \param length Length of the data buffer
 * \param slide  How many bytes to slide the buffer up in memory (if +) or
 *               down in memory (if -)
 */
void slide(uint8_t * data, uint8_t length, int16_t slide)
{
  //Sanity checks
  if (!length) return;
  if (!slide) return;

  uint8_t i = 0;

  while(length) {
    length--;

    //If we are sliding up, we do from the top of the buffer down
    if (slide > 0) {
      *(data + length + slide) = *(data + length);  

      //If we are sliding down, we do from the bottom of the buffer up
    } else {
      *(data + slide + i) = *(data + i);
    }
    
    i++;
  }
}

/** @} */
