/*
 * Copyright (c) 2008-2010
 * Telecooperation Office (TecO), Universitaet Karlsruhe (TH), Germany.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * 3. Neither the name of the Universitaet Karlsruhe (TH) nor the names
 *    of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author(s): Philipp Scholl <scholl@teco.edu>
 */

#include "contiki-net.h"
#include "mac.h"
#include "ieee802.h"
#include "bridge_aux.c"
#include "net/netstack.h"
#include "net/cdc_dev.h"

#define UIP_PROTO_UDPLITE 0x88
#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define UIP_UDP_BUF  ((struct uip_udp_hdr *)&uip_buf[uip_l2_l3_hdr_len])

static volatile u8_t last_rssi = 14;

/* rssi udplite hack, only the rssi value of the last packet
 * delivered to lowpan2eth will be added as rssi value, so we only
 * store one. */
static void
mac_udp_rssicb(const rimeaddr_t *from, u8_t rssi)
{
  last_rssi = rssi;
}

u8_t (*wpan_send)(uip_lladdr_t*) = NULL;
u8_t (*other_send)() = NULL;

void
eth2lowpan()
{
  uip_lladdr_t *destaddr;
  destaddr = mac_ethernetToLowpan();

  if(destaddr!=NULL)
    wpan_send(destaddr);
  else
    uip_log("bridge: translation failed\n");
}

void
lowpan2eth()
{
  /* special hack to put rssi values into udp packets transforming them
   * into udplite packets. */
  if (UIP_IP_BUF->proto == UIP_PROTO_UDP &&
      uip_len < sizeof(uip_buf))
  {
    u16_t iplen;

    /* change protocol field to udplite */
    UIP_IP_BUF->proto = UIP_PROTO_UDPLITE;

    /* change udp checksum for changed data */
    UIP_UDP_BUF->udpchksum = ~htons( ntohs(~UIP_UDP_BUF->udpchksum)
                                   + 1 + (UIP_PROTO_UDPLITE - UIP_PROTO_UDP));

    /* add rssi value */
    uip_buf[UIP_LLH_LEN+uip_len] = last_rssi;

    /* change ip header length */
    iplen = HTONS(*((u16_t*) UIP_IP_BUF->len));
    iplen += 1;
    *((u16_t*) UIP_IP_BUF->len) = HTONS(iplen);

    /* tell uip that a byte has been added */
    uip_len += 1;
  }

  mac_LowpanToEthernet();
  other_send();
}

PROCESS(ethbr_process, "Ethernet Bridge Process");
PROCESS_THREAD(ethbr_process, ev, data)
{
  PROCESS_BEGIN();
  uip_log("bridge: starting\n");

  /* hack for getting lqi values attached to every udp packet using udplite */
  ieee_register_lqi_callback(mac_udp_rssicb);

  /* rewire wpan interface to lowpan2eth() and other interface to eth2lowpan(),
   * this is ulgy I know */
  sicslowpan_tcpip_input = lowpan2eth;
  wpan_send = sicslowpan_output;

  cdc_tcpip_input = eth2lowpan;
  other_send = cdc_output;

  /* wait until killed then rewire network layer to tcpip layer */
  PROCESS_YIELD_UNTIL(ev==PROCESS_EVENT_EXIT);
  uip_log("bridge: exiting\n");

  sicslowpan_tcpip_input = tcpip_input;
  cdc_tcpip_input = tcpip_input;

  PROCESS_END();
}
