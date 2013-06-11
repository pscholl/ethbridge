/*
 * Copyright (c) 2011
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

#include "contiki.h"
#include "contiki-net.h"
#include "dev/slip.h"
#include "dev/leds.h"
#include "mac.h"
#include "ieee802.h"
#include "bridge_aux.h"
#include "sicslowpan.h"
#include "net/netstack.h"

PROCESS(slipbr_process, "Serial Line IP Bridge process");
PROCESS(test_process, "Test");
AUTOSTART_PROCESSES(&slipbr_process, &test_process);

void
lowpan2eth()
{
  mac_LowpanToEthernet();
  slip_send();
}

#define ETHBUF(x)    ((struct uip_eth_hdr *)x)

void
eth2lowpan()
{
  uip_lladdr_t *destaddr;
  size_t i=0;

  destaddr = mac_ethernetToLowpan();

  if(destaddr!=NULL)
    sicslowpan_output(destaddr);
  else {
    uip_log("bridge: translation failed\n");

    printf("\n\n");
    printf("len: %d\n", uip_len);
    printf("ethbuf: %d\n", ETHBUF(uip_buf)->type);
    for (i=0; i<30; i++)
      printf("0x%x ", uip_buf[i]);
    printf("\n");
  }
}

PROCESS_THREAD(slipbr_process, ev, data)
{
  int i;

  PROCESS_BEGIN();
  uip_log("bridge: starting\n");

  /* DO NOT DELETE THESE LINES, thery are needed for SLIP operation, i.e.
   * letting the atmega know the mac address */
  printf("mac: ");
  for (i=0; i<sizeof(uip_lladdr); i++)
    uart0_writeb(uip_lladdr.addr[i]);

  /* rewire wpan interface to lowpan2eth() and other interface to eth2lowpan(),
   * this is ulgy I know */
  process_exit(&tcpip_process);
  sicslowpan_tcpip_input = lowpan2eth;
  slip_set_input_callback(eth2lowpan);
  slip_arch_init(1000000);
  process_start(&slip_process, NULL);

  /* wait until killed then rewire network layer to tcpip layer */
  PROCESS_YIELD_UNTIL(ev==PROCESS_EVENT_EXIT);
  uip_log("bridge: exiting\n");

  process_start(&tcpip_process, NULL);
  sicslowpan_tcpip_input = tcpip_input;

  PROCESS_END();
}

//lässt leds blinken wenn über Port 10000 udp Nachricht kommt -> nur wenn rndis aus
PROCESS_THREAD(test_process, ev, data)
{
  static struct uip_udp_conn *udp;

  PROCESS_BEGIN();

  udp = udp_new(NULL, UIP_HTONS(10000), NULL);
  uip_udp_bind(udp, UIP_HTONS(10000));

  while(1)
  {
    PROCESS_YIELD_UNTIL(ev == tcpip_event);
    leds_toggle(LEDS_ALL);
  }

  PROCESS_END();
}
