/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* lwIP includes. */
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include <lwip/stats.h>
#include <lwip/snmp.h>
#include "netif/etharp.h"

//#include "r2c2.h"
#include "timer.h"
#include "enc28j60.h"


#define IFNAME0 							'e'
#define IFNAME1 							'n'

#define netifMTU                          ( 1500 )
#define netifINTERFACE_TASK_STACK_SIZE		( 350 )
#define netifINTERFACE_TASK_PRIORITY      ( configMAX_PRIORITIES - 1 )
#define netifBUFFER_WAIT_ATTEMPTS         10
#define netifBUFFER_WAIT_DELAY            (10 / portTICK_RATE_MS)

// The time to block waiting for input. 
#define enc28j60BLOCK_TIME_WAITING_FOR_INPUT	( ( portTickType ) 100 )


// 
struct ethernetif 
{
  struct eth_addr *ethaddr;
  /* Add whatever per-interface state that is needed here. */
};

// Receive buffer
static uint8_t rxtxBuf[ 1520 ];

// The semaphore used by the ISR to wake the lwIP task. 
static xSemaphoreHandle s_xSemaphore = NULL;


// Forward declarations. 
// Private functions
//static uint8_t *pcGetNextBuffer( void );

static void       ethernetif_input( void *pParams );

static void       low_level_init( struct netif *netif );
static err_t      low_level_output( struct netif *netif, struct pbuf *p );
static struct pbuf *low_level_input( struct netif *netif );

// ==================================================================
// glue
static void enc28j60_MACInit( struct netif *pnetif )
{
  enc28j60SpiInit();

  enc28j60Init (pnetif->hwaddr);

  enc28j60clkout(2); // change clkout from 6.25MHz to 12.5MHz
  delay_ms(10);

  // flash leds on
   // 0x880 is PHLCON LEDB=on, LEDA=on
   // enc28j60PhyWrite(PHLCON,0b0011 1000 1000 00 00);
   enc28j60PhyWrite(PHLCON,0x3880);
   delay_ms(100);

   // 0x990 is PHLCON LEDB=off, LEDA=off
   // enc28j60PhyWrite(PHLCON,0b0011 1001 1001 00 00);
   enc28j60PhyWrite(PHLCON,0x3990);

  // 0x476 is PHLCON LEDA=links status, LEDB=receive/transmit
  // enc28j60PhyWrite(PHLCON,0b0011 0100 0111 01 10);
  enc28j60PhyWrite(PHLCON,0x3476);

}

bool enc28j60_MACTransmit( uint8_t *pBuf, uint16_t len )
{
}

uint16_t enc28j60_MACGetRxPacket( uint8_t *pBuf )
{
}
// ==================================================================

/*!
///////////////////////////////////////////////////////////////////////////////
//  low_level_init
//
//  In this function, the hardware should be initialized.
//  Called from ethernetif_init().
//
//  @param netif the already initialized lwip network interface structure
//        for this ethernetif
//
*/
 
static void low_level_init( struct netif *pnetif )
{
	// set MAC hardware address length 
	pnetif->hwaddr_len = ETHARP_HWADDR_LEN;

	// set MAC hardware address 
	//MACStrToBin( DEFAULT_MAC_ADDR, pnetif->hwaddr );
	pnetif->hwaddr[ 0 ] = 0x00;
	pnetif->hwaddr[ 1 ] = 0x04;
	pnetif->hwaddr[ 2 ] = 0xa3;
	pnetif->hwaddr[ 3 ] = 0x00;
	pnetif->hwaddr[ 4 ] = 0x00;
	pnetif->hwaddr[ 5 ] = 0x02;

	// maximum transfer unit 
	pnetif->mtu = netifMTU;

	// device capabilities.
	// don't set NETIF_FLAG_ETHARP if this device is not an ethernet one
	pnetif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;

	// Do whatever else is needed to initialize interface. 
	
	if ( s_xSemaphore == NULL )
	{
		vSemaphoreCreateBinary( s_xSemaphore );
		xSemaphoreTake( s_xSemaphore,  0);
	}

	// Initialise the MAC. 
	enc28j60_MACInit( pnetif );
		
	pnetif->flags |= NETIF_FLAG_LINK_UP;

	// Create the task that handles the EMAC. 
	sys_thread_new( ( signed portCHAR *)"eth", 
                        ethernetif_input, 
                        (void *)pnetif, 
                        netifINTERFACE_TASK_STACK_SIZE, 
                        netifINTERFACE_TASK_PRIORITY );
}	



/*! 
///////////////////////////////////////////////////////////////////////////////
// low_level_output
//
//  This function should do the actual transmission of the packet. The packet is
// contained in the pbuf that is passed to the function. This pbuf
// might be chained.
//
// @param netif the lwip network interface structure for this ethernetif
// @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
// @return ERR_OK if the packet could be sent
//         an err_t value if the packet couldn't be sent
//
// @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
//       strange results. You might consider waiting for space in the DMA queue
//       to become availale since the stack doesn't retry to send a packet
//       dropped because of memory failure (except for the TCP timers).
//
*/

static err_t low_level_output( struct netif *netif, struct pbuf *p )
{
    struct pbuf *q;
    uint32_t len = 0;

#if ETH_PAD_SIZE
    pbuf_header( p, -ETH_PAD_SIZE );    // drop the padding word 
#endif
		 
    {
		for ( q = p; q != NULL; q = q->next ) 
    	{	
			// Send the data from the pbuf to the interface, one pbuf at a
			// time. The size of the data in each pbuf is kept in the ->len
		    // variable.
			
			vTaskSuspendAll();
			memcpy( ( &rxtxBuf[len] ), (u8_t*)q->payload, q->len );
			xTaskResumeAll();
			len += q->len;
		}
	}

	enc28j60_MACTransmit( rxtxBuf, len );
			
#if ETH_PAD_SIZE
    pbuf_header( p, ETH_PAD_SIZE );	    // reclaim the padding word 
#endif

    LINK_STATS_INC( link.xmit );

    return ERR_OK;
}

/*! 
///////////////////////////////////////////////////////////////////////////////
// low_level_input
//
//  Should allocate a pbuf and transfer the bytes of the incoming
//  packet from the interface into the pbuf.
//
// @param netif the lwip network interface structure for this ethernetif
// @return a pbuf filled with the received packet (including MAC header)
//         NULL on memory error
//
*/
 
static struct pbuf *low_level_input( struct netif *netif )
{
    struct pbuf *p, *q;
    uint16_t len, l;

    len = 0;
    p = NULL;
	

    // Get packet if any
    len = enc28j60_MACGetRxPacket( rxtxBuf );
    
    if ( len )
    {
#if ETH_PAD_SIZE
        len += ETH_PAD_SIZE;	            // allow room for Ethernet padding 
#endif

        // We allocate a pbuf chain of pbufs from the pool. 
        p = pbuf_alloc( PBUF_RAW, len, PBUF_POOL) ;

        if ( p != NULL ) 
        {

#if ETH_PAD_SIZE
          pbuf_header( p, -ETH_PAD_SIZE );	// drop the padding word 
#endif

          // We iterate over the pbuf chain until we have read the entire
          // packet into the pbuf. 
          l = 0;
          for ( q = p; q != NULL; q = q->next ) 
          {
            // Read enough bytes to fill this pbuf in the chain. The
            // available data in the pbuf is given by the q->len
            // variable. 
            vTaskSuspendAll();
            memcpy( (uint8_t*)q->payload, (rxtxBuf + l), q->len );				
            xTaskResumeAll();
            l += q->len;
          }

#if ETH_PAD_SIZE
          pbuf_header(p, ETH_PAD_SIZE);   // reclaim the padding word 
#endif

          LINK_STATS_INC(link.recv);

        } 
        else 
        {
          LINK_STATS_INC (link.memerr );
          LINK_STATS_INC( link.drop );
        } 
    } 
			
    return p;
}



///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////
// ethernetif_init
//
// Should be called at the beginning of the program to set up the
// network interface. It calls the function low_level_init() to do the
// actual setup of the hardware.
//
// This function should be passed as a parameter to netif_add().
//
// @param netif the lwip network interface structure for this ethernetif
// @return ERR_OK if the loopif is initialized
//         ERR_MEM if private data couldn't be allocated
//         any other err_t on error
//

 
err_t ethernetif_init( struct netif *netif )
{
	struct ethernetif *ethernetif;
	
	LWIP_ASSERT( "netif != NULL", ( netif != NULL ) );

	ethernetif = mem_malloc( sizeof(struct ethernetif ) );

	if ( ethernetif == NULL )
	{
		LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_init: out of memory\n"));
		return ERR_MEM;
	}

#if LWIP_NETIF_HOSTNAME
	/* Initialize interface hostname */
	netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */
	
	/*
	 * Initialize the snmp variables and counters inside the struct netif.
	 * The last argument should be replaced with your link speed, in units
	 * of bits per second.
	 */
	NETIF_INIT_SNMP( netif, snmp_ifType_ethernet_csmacd, 100 );	

	netif->state = ethernetif;
	netif->name[0] = IFNAME0;
	netif->name[1] = IFNAME1;
	
	/* We directly use etharp_output() here to save a function call.
	 * You can instead declare your own function an call etharp_output()
	 * from it if you have to do some checks before sending (e.g. if link
	 * is available...)
	 */	
	netif->output = etharp_output;
	netif->linkoutput = low_level_output;

	ethernetif->ethaddr = (struct eth_addr *)&( netif->hwaddr[ 0 ] );

	low_level_init( netif );
	
	return ERR_OK;
}



/*!
///////////////////////////////////////////////////////////////////////////////
//  ethernetif_input
//
//  This function is a task which waits for input from the ethernet device.
//
//  It uses the function low_level_input() that
//  should handle the actual reception of bytes from the network
//  interface. Then the type of the received packet is determined and
//  the appropriate input function is called.
//
//  @param pParams a *netif pointer to the lwip network interface structure for this ethernetif
//
*/

static void ethernetif_input( void *pParams )
{
    struct netif *netif;
    struct ethernetif *ethernetif;
    struct eth_hdr *ethhdr;
    struct pbuf *p;

    netif = (struct netif*) pParams;
    ethernetif = netif->state;
	
    for( ;; )
    {
      do
      {
        // move received packet into a new pbuf
        p = low_level_input( netif );
			
        if ( p == NULL )
        {
          // No packet could be read.  Wait for an interrupt to tell us
          // there is more data available.
          xSemaphoreTake( s_xSemaphore, enc28j60BLOCK_TIME_WAITING_FOR_INPUT );
        }
		
      } while( p == NULL );

      // points to packet payload, which starts with an Ethernet header 
      ethhdr = p->payload;

      switch ( htons( ethhdr->type ) ) 
      {
			
        //  IP or ARP packet
		    case ETHTYPE_IP:
		    case ETHTYPE_ARP:
	
			    // full packet send to tcpip_thread to process 

			    if ( netif->input( p, netif ) != ERR_OK)
			    {
				    LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
				    pbuf_free( p );
				    p = NULL;
			    }
			    break;
				
		    default:
			    pbuf_free( p );
			    p = NULL;
		    	break;
		}
	}
}

/*!
///////////////////////////////////////////////////////////////////////////////
// ethernetif_output
//
// This function is called by the TCP/IP stack when an IP packet
// should be sent. It calls the function called low_level_output() to
// do the actual transmission of the packet.
//
*/

static err_t ethernetif_output( struct netif *netif, 
                    struct pbuf *p,
                    struct ip_addr *ipaddr )
{
    // resolve hardware address, then send (or queue) packet 
    return etharp_output( netif, p, ipaddr );
}

#if 0
// not required ? - rmc

///////////////////////////////////////////////////////////////////////////////
// arp_timer
//

static void arp_timer(void *arg)
{
    etharp_tmr();
    sys_timeout( ARP_TMR_INTERVAL, arp_timer, NULL );
}
#endif 
