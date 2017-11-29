/*
 * FreeRTOS+TCP Labs Build 160919 (C) 2016 Real Time Engineers ltd.
 * Authors include Hein Tibosch and Richard Barry
 *
 *******************************************************************************
 ***** NOTE ******* NOTE ******* NOTE ******* NOTE ******* NOTE ******* NOTE ***
 ***                                                                         ***
 ***                                                                         ***
 ***   FREERTOS+TCP IS STILL IN THE LAB (mainly because the FTP and HTTP     ***
 ***   demos have a dependency on FreeRTOS+FAT, which is only in the Labs    ***
 ***   download):                                                            ***
 ***                                                                         ***
 ***   FreeRTOS+TCP is functional and has been used in commercial products   ***
 ***   for some time.  Be aware however that we are still refining its       ***
 ***   design, the source code does not yet quite conform to the strict      ***
 ***   coding and style standards mandated by Real Time Engineers ltd., and  ***
 ***   the documentation and testing is not necessarily complete.            ***
 ***                                                                         ***
 ***   PLEASE REPORT EXPERIENCES USING THE SUPPORT RESOURCES FOUND ON THE    ***
 ***   URL: http://www.FreeRTOS.org/contact  Active early adopters may, at   ***
 ***   the sole discretion of Real Time Engineers Ltd., be offered versions  ***
 ***   under a license other than that described below.                      ***
 ***                                                                         ***
 ***                                                                         ***
 ***** NOTE ******* NOTE ******* NOTE ******* NOTE ******* NOTE ******* NOTE ***
 *******************************************************************************
 *
 * FreeRTOS+TCP can be used under two different free open source licenses.  The
 * license that applies is dependent on the processor on which FreeRTOS+TCP is
 * executed, as follows:
 *
 * If FreeRTOS+TCP is executed on one of the processors listed under the Special
 * License Arrangements heading of the FreeRTOS+TCP license information web
 * page, then it can be used under the terms of the FreeRTOS Open Source
 * License.  If FreeRTOS+TCP is used on any other processor, then it can be used
 * under the terms of the GNU General Public License V2.  Links to the relevant
 * licenses follow:
 *
 * The FreeRTOS+TCP License Information Page: http://www.FreeRTOS.org/tcp_license
 * The FreeRTOS Open Source License: http://www.FreeRTOS.org/license
 * The GNU General Public License Version 2: http://www.FreeRTOS.org/gpl-2.0.txt
 *
 * FreeRTOS+TCP is distributed in the hope that it will be useful.  You cannot
 * use FreeRTOS+TCP unless you agree that you use the software 'as is'.
 * FreeRTOS+TCP is provided WITHOUT ANY WARRANTY; without even the implied
 * warranties of NON-INFRINGEMENT, MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. Real Time Engineers Ltd. disclaims all conditions and terms, be they
 * implied, expressed, or statutory.
 *
 * 1 tab == 4 spaces!
 *
 * http://www.FreeRTOS.org
 * http://www.FreeRTOS.org/plus
 * http://www.FreeRTOS.org/labs
 *
 */

/* SAMV71 (Cortex M7) network interface, 
 * intended for use with KSZ8061 Ethernet PHY
 *
 * - adapted from SAM4E (Cortex M3/4) example interface 
 */

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* FreeRTOS+TCP includes. */
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"
#include "FreeRTOS_IP_Private.h"
#include "NetworkBufferManagement.h"
#include "NetworkInterface.h"

/* Some files from the Atmel Software Framework */
#include <sysclk.h>
#include "gmac.h" /* There is also a component/gmac.h in the atmel pack */
#include "gmac_raw_2.h"
#include "ethernet_phy.h"

// Settings for SAMV71
#define BMSR_REGISTER    GMII_BMSR
#define BMSR_LINK_STATUS GMII_LINK_STATUS

/**************************************************************************
 * NOTE: Before calling FreeRTOS_IPInit(), the following need to be called:
 **************************************************************************
 sysclk_init() // with SYSCLK_PRES_2
 board_init()  // with MPU_HAS_NOCACHE_REGION defined and large enough to 
               // contain net buffers & descriptors, or with caching off
 pmc_enable_periph_clk(ID_GMAC);
 **************************************************************************/

#ifndef	PHY_LS_HIGH_CHECK_TIME_MS
	/* Check if the LinkSStatus in the PHY is still high after 10 seconds of not
	receiving packets. */
	#define PHY_LS_HIGH_CHECK_TIME_MS	10000
#endif

#ifndef	PHY_LS_LOW_CHECK_TIME_MS
	/* Check if the LinkSStatus in the PHY is still low every second. */
	#define PHY_LS_LOW_CHECK_TIME_MS	1000
#endif

/* Interrupt events to process.  Currently only the Rx event is processed
although code for other events is included to allow for possible future
expansion. */
#define EMAC_IF_RX_EVENT        1UL
#define EMAC_IF_TX_EVENT        2UL
#define EMAC_IF_ERR_EVENT       4UL
#define EMAC_IF_ALL_EVENT       ( EMAC_IF_RX_EVENT | EMAC_IF_TX_EVENT | EMAC_IF_ERR_EVENT )

#define ETHERNET_CONF_PHY_ADDR  BOARD_GMAC_PHY_ADDR

#define HZ_PER_MHZ				( 1000000UL )

#ifndef	EMAC_MAX_BLOCK_TIME_MS
	#define	EMAC_MAX_BLOCK_TIME_MS	100ul
#endif

/* Default the size of the stack used by the EMAC deferred handler task to 4x
the size of the stack used by the idle task - but allow this to be overridden in
FreeRTOSConfig.h as configMINIMAL_STACK_SIZE is a user definable constant. */
#ifndef configEMAC_TASK_STACK_SIZE
	#define configEMAC_TASK_STACK_SIZE ( 4 * configMINIMAL_STACK_SIZE )
#endif

/*-----------------------------------------------------------*/

/*
 * Wait a fixed time for the link status to indicate the network is up.
 */
static BaseType_t xGMACWaitLS( TickType_t xMaxTime );

#if( ipconfigDRIVER_INCLUDED_TX_IP_CHECKSUM == 1 ) && ( ipconfigHAS_TX_CRC_OFFLOADING == 0 )
	void vGMACGenerateChecksum( uint8_t *apBuffer );
#endif

/*
 * Called from the ASF GMAC driver.
 */
static void prvRxCallback( uint32_t ulStatus );

/*
 * A deferred interrupt handler task that processes GMAC interrupts.
 */
static void prvEMACHandlerTask( void *pvParameters );

/*
 * Initialize the ASF GMAC driver.
 */
static BaseType_t prvGMACInit( void );

/*
 * Try to obtain an Rx packet from the hardware.
 */
static uint32_t prvEMACRxPoll( void );

/*-----------------------------------------------------------*/

/* Bit map of outstanding ETH interrupt events for processing.  Currently only
the Rx interrupt is handled, although code is included for other events to
enable future expansion. */
static volatile uint32_t ulISREvents = EMAC_IF_RX_EVENT; // Process incoming packets upon startup

/* A copy of PHY register 1: 'GMII_BMSR' */
static uint32_t ulPHYLinkStatus = 0;
static volatile BaseType_t xGMACSwitchRequired;

/* ethernet_phy_addr: the address of the PHY in use.
JB: Needed to modify ethernet_phy_init() in ethernet_phy.c to obtain this, 
as that function may previously have changed it without our knowledge */
static uint8_t ethernet_phy_addr;

/* LLMNR multicast address; a second MAC address to listen on */
#if ipconfigUSE_LLMNR == 1
static const uint8_t llmnr_mac_address[] = { 0x01, 0x00, 0x5E, 0x00, 0x00, 0xFC };
#endif

/* The GMAC object as defined by the ASF drivers. */
static gmac_device_t gs_gmac_dev;

/* JB Addn: GMAC priority queue index; always use 0 for no priority queue. 
(see http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-44003-32-bit-Cortex-M7-Microcontroller-SAM-V71Q-SAM-V71N-SAM-V71J_Datasheet.pdf section 39.6.3.9) */
#define QUEUE_INDEX GMAC_QUE_0

/* Holds the handle of the task used as a deferred interrupt processor.  The
handle is used so direct notifications can be sent to the task for all EMAC/DMA
related interrupts. */
TaskHandle_t xEMACTaskHandle = NULL;


/*-----------------------------------------------------------*/

/*
 * GMAC interrupt handler.
 */
void GMAC_Handler(void)
{
	xGMACSwitchRequired = pdFALSE;

	/* gmac_handler() may call prvRxCallback() which may change
	the value of xGMACSwitchRequired. */
	gmac_handler( &gs_gmac_dev, QUEUE_INDEX );

	if( xGMACSwitchRequired != pdFALSE )
	{
		portEND_SWITCHING_ISR( xGMACSwitchRequired );
	}
}
/*-----------------------------------------------------------*/

/* Rx interrupt callback, with status from Recieve Status Register */
static void prvRxCallback( uint32_t ulStatus )
{
	if( ( ( ulStatus & GMAC_RSR_REC ) != 0 ) && ( xEMACTaskHandle != NULL ) )
	{
		/* let the prvEMACHandlerTask know that there was an RX event. */
		ulISREvents |= EMAC_IF_RX_EVENT;
		/* Only an RX interrupt can wakeup prvEMACHandlerTask. */
		vTaskNotifyGiveFromISR( xEMACTaskHandle, ( BaseType_t * ) &xGMACSwitchRequired );
	}
	/* ignore packets with errors, or when packets could not be received */
}
/*-----------------------------------------------------------*/

BaseType_t xNetworkInterfaceInitialise( void )
{
const TickType_t x5_Seconds = 5000UL;

	if( xEMACTaskHandle == NULL )
	{
		if (prvGMACInit() != pdPASS) {
			FreeRTOS_printf( ("prvGMACInit failed. Link may be disconnected or down. Will retry.\n") );
			return pdFAIL;
		}

		/* Wait at most 5 seconds for a Link Status in the PHY. */
		xGMACWaitLS( pdMS_TO_TICKS( x5_Seconds ) );

		/* The handler task is created at the highest possible priority to
		ensure the interrupt handler can return directly to it. */
		xTaskCreate( prvEMACHandlerTask, "EMAC", configEMAC_TASK_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, &xEMACTaskHandle );
		configASSERT( xEMACTaskHandle );
	}
	/* When returning non-zero, the stack will become active and
    start DHCP (in configured) */
	return ( ulPHYLinkStatus & BMSR_LINK_STATUS ) != 0;
}
/*-----------------------------------------------------------*/

BaseType_t xGetPhyLinkStatus( void )
{
BaseType_t xResult;

	/* This function returns true if the Link Status in the PHY is high. */
	if( ( ulPHYLinkStatus & BMSR_LINK_STATUS ) != 0 )
	{
		xResult = pdTRUE;
	}
	else
	{
		xResult = pdFALSE;
	}

	return xResult;
}
/*-----------------------------------------------------------*/

BaseType_t xNetworkInterfaceOutput( NetworkBufferDescriptor_t * const pxDescriptor, BaseType_t bReleaseAfterSend )
{
uint32_t uiResult;

	if( ( ulPHYLinkStatus & BMSR_LINK_STATUS ) != 0 )
	{
		/* Not interested in a call-back after TX. */
		uiResult = gmac_dev_write( &gs_gmac_dev, QUEUE_INDEX, (void *)pxDescriptor->pucEthernetBuffer, pxDescriptor->xDataLength, NULL );
		if (uiResult == GMAC_OK)
			iptraceNETWORK_INTERFACE_TRANSMIT();
		else
			FreeRTOS_printf(("Unsuccessful packet send\n"));
	}

	if( bReleaseAfterSend != pdFALSE )
	{
		vReleaseNetworkBufferAndDescriptor( pxDescriptor );
	}
	return pdTRUE;
}
/*-----------------------------------------------------------*/

static BaseType_t prvGMACInit( void )
{
gmac_options_t gmac_option;
uint32_t mck_hz;
uint8_t result;

	/* Set GMAC options to copy all frames and support broadcast */
	/* Use primary mac address ipLOCAL_MAC_ADDRESS */
	memset( &gmac_option, '\0', sizeof( gmac_option ) );
	gmac_option.uc_copy_all_frame = 0;
	gmac_option.uc_no_broadcast = 0;
	memcpy( gmac_option.uc_mac_addr, ipLOCAL_MAC_ADDRESS, sizeof( gmac_option.uc_mac_addr ) );
	gs_gmac_dev.p_hw = GMAC;

	/* Initialize GMAC device */
	gmac_dev_init( GMAC, &gs_gmac_dev, &gmac_option );
	
	/* Enable interrupt service for GMAC */
	NVIC_SetPriority( GMAC_IRQn, configMAC_INTERRUPT_PRIORITY);
	NVIC_EnableIRQ( GMAC_IRQn );
	/* On interrupt, device-registered peripheral handler GMAC_Handler(), defined above, will be called */

	/* Get master clock (MCK) speed; must not exceed 240Mhz for GMAC Management Data Clock (MDC) to 
	 * conform to 802.3 spec. (CPU Clock may need to be downscaled to work [see CONFIG_SYSCLK_PRES]) */
	mck_hz = sysclk_get_peripheral_hz();
	
	/* Initialize the Ethernet PHY and store its address for later use */
	/* JB: NOTE, ethernet_phy_init from ASF was rewritten to return the PHY address */
	ethernet_phy_addr = 
		ethernet_phy_init( GMAC, ETHERNET_CONF_PHY_ADDR, mck_hz );
	configASSERT(ethernet_phy_addr != GMAC_INVALID);
	
	/* Link will be established based upon auto-negotiation if possible */
	result = ethernet_phy_auto_negotiate( GMAC, ethernet_phy_addr );
	if (result == GMAC_TIMEOUT) {
		/* Auto-negotiate timed out, apply default settings if link is up */
		if (ethernet_phy_set_link( GMAC, ethernet_phy_addr, 1 ) != GMAC_OK) {
			return pdFAIL; /* Link is down */
		}
	} else if (result != GMAC_OK) {
		return pdFAIL;
	}

	/* The GMAC driver will call a hook prvRxCallback() on receiving eth frames, which
	in turn will wake-up the task by calling vTaskNotifyGiveFromISR() */
	gmac_dev_set_rx_callback( &gs_gmac_dev, QUEUE_INDEX, prvRxCallback );

#if ipconfigUSE_LLMNR == 1
	/* In gmac_dev_init, we set the primary MAC address, here we set a second multicast address used 
	to provide Link Local Multicast Name Resolution (LLMNR) . */
	gmac_set_address( GMAC, 1, (uint8_t*)llmnr_mac_address );
#endif

	return pdPASS;
}
/*-----------------------------------------------------------*/

static inline unsigned long ulReadMDIO( unsigned /*short*/ usAddress )
{
uint32_t ulValue, ulReturn;
int rc;

	gmac_enable_management( GMAC, 1 );
	rc = gmac_phy_read( GMAC, ethernet_phy_addr, usAddress, &ulValue );
	gmac_enable_management( GMAC, 0 );
	if( rc == GMAC_OK )
	{
		ulReturn = ulValue;
	}
	else
	{
		ulReturn = 0UL;
	}

	return ulReturn;
}
/*-----------------------------------------------------------*/

/* Wait up to xMaxTime for the link to go up */
static BaseType_t xGMACWaitLS( TickType_t xMaxTime )
{
TickType_t xStartTime = xTaskGetTickCount();
TickType_t xEndTime;
BaseType_t xReturn;
const TickType_t xShortTime = pdMS_TO_TICKS( 100UL );

	for( ;; )
	{
		xEndTime = xTaskGetTickCount();

		if( ( xEndTime - xStartTime ) > xMaxTime )
		{
			/* Waited more than xMaxTime, return. */
			xReturn = pdFALSE;
			break;
		}

		/* Check the link status again. */
		ulPHYLinkStatus = ulReadMDIO( BMSR_REGISTER );

		if( ( ulPHYLinkStatus & BMSR_LINK_STATUS ) != 0 )
		{
			/* Link is up - return. */
			xReturn = pdTRUE;
			break;
		}

		/* Link is down - wait in the Blocked state for a short while (to allow
		other tasks to execute) before checking again. */
		vTaskDelay( xShortTime );
	}

	FreeRTOS_printf( ( "xGMACWaitLS: %ld (PHY %d) freq %lu Mz\n",
		xReturn,
		ethernet_phy_addr,
		sysclk_get_cpu_hz() / HZ_PER_MHZ ) );

	return xReturn;
}

/*-----------------------------------------------------------*/
/* In our driver ipconfigHAS_TX_CRC_OFFLOADING should always be set to 1, 
   unless checksum offloading is found to be failing */
#if ( ipconfigDRIVER_INCLUDED_TX_IP_CHECKSUM == 1 ) && ( ipconfigHAS_TX_CRC_OFFLOADING == 0 )

	void vGMACGenerateChecksum( uint8_t *apBuffer )
	{
	ProtocolPacket_t *xProtPacket = (ProtocolPacket_t *)apBuffer;
	IPHeader_t *pxIPHeader;

		if ( xProtPacket->xTCPPacket.xEthernetHeader.usFrameType == ipIPv4_FRAME_TYPE )
		{
			pxIPHeader = &(xProtPacket->xTCPPacket.xIPHeader);

			/* Calculate the IP header checksum. */
			pxIPHeader->usHeaderChecksum = 0x00;
			pxIPHeader->usHeaderChecksum = usGenerateChecksum( 0, ( uint8_t * ) &( pxIPHeader->ucVersionHeaderLength ), ipSIZE_OF_IPv4_HEADER );
			pxIPHeader->usHeaderChecksum = ~FreeRTOS_htons( pxIPHeader->usHeaderChecksum );

			/* Calculate & insert the UDP/TCP/ICMP/IGMP checksum for an outgoing packet. */
			usGenerateProtocolChecksum( ( uint8_t * ) apBuffer, pdTRUE );
		}
	}

#endif
/*-----------------------------------------------------------*/

static uint32_t prvEMACRxPoll( void )
{
unsigned char *pucUseBuffer;
uint32_t ulReceiveCount, ulResult, ulReturnValue = 0;
static NetworkBufferDescriptor_t *pxNextNetworkBufferDescriptor = NULL;
const UBaseType_t xMinDescriptorsToLeave = 2UL;
const TickType_t xBlockTime = pdMS_TO_TICKS( 100UL );
static IPStackEvent_t xRxEvent = { eNetworkRxEvent, NULL };

	for( ;; )
	{
		/* If pxNextNetworkBufferDescriptor was not left pointing at a valid
		descriptor then allocate one now. */
		if( ( pxNextNetworkBufferDescriptor == NULL ) && ( uxGetNumberOfFreeNetworkBuffers() > xMinDescriptorsToLeave ) )
		{
			pxNextNetworkBufferDescriptor = pxGetNetworkBufferWithDescriptor( ipTOTAL_ETHERNET_FRAME_SIZE, xBlockTime );
		}

		if( pxNextNetworkBufferDescriptor != NULL )
		{
			/* Point pucUseBuffer to the buffer pointed to by the descriptor. */
			pucUseBuffer = ( unsigned char* ) ( pxNextNetworkBufferDescriptor->pucEthernetBuffer - ipconfigPACKET_FILLER_SIZE );
		}
		else
		{
			/* As long as pxNextNetworkBufferDescriptor is NULL, the incoming
			messages will be flushed and ignored. */
			pucUseBuffer = NULL;
		}

		/* Read the next packet from the hardware into pucUseBuffer. */
		ulResult = gmac_dev_read( &gs_gmac_dev, QUEUE_INDEX, pucUseBuffer, ipTOTAL_ETHERNET_FRAME_SIZE, &ulReceiveCount );

		if( ( ulResult != GMAC_OK ) || ( ulReceiveCount == 0 ) )
		{
			/* No data from the hardware. */
			break;
		}

		if( pxNextNetworkBufferDescriptor == NULL )
		{
			/* Data was read from the hardware, but no descriptor was available
			for it, so it will be dropped. Should not get here because pucUseBuffer==null 
			should indicate to driver to only free buffers */
			iptraceETHERNET_RX_EVENT_LOST();
			continue;
		}

		iptraceNETWORK_INTERFACE_RECEIVE();
		pxNextNetworkBufferDescriptor->xDataLength = ( size_t ) ulReceiveCount;
		xRxEvent.pvData = ( void * ) pxNextNetworkBufferDescriptor;

		/* Send the descriptor to the IP task for processing. */
		if( xSendEventStructToIPTask( &xRxEvent, xBlockTime ) != pdTRUE )
		{
			/* The buffer could not be sent to the stack so must be released
			again. */
			vReleaseNetworkBufferAndDescriptor( pxNextNetworkBufferDescriptor );
			iptraceETHERNET_RX_EVENT_LOST();
			FreeRTOS_printf( ( "prvEMACRxPoll: Can not queue return packet!\n" ) );
		}

		/* Now the buffer has either been passed to the IP-task,
		or it has been released in the code above. */
		pxNextNetworkBufferDescriptor = NULL;
		ulReturnValue++;
	}

	return ulReturnValue;
}
/*-----------------------------------------------------------*/


/* Main function for EMAC task */
static void prvEMACHandlerTask( void *pvParameters )
{
TimeOut_t xPhyTime;
TickType_t xPhyRemTime;
UBaseType_t uxLastBufferCount = 0;
UBaseType_t uxCurrentCount;
#if( ipconfigCHECK_IP_QUEUE_SPACE != 0 )
	UBaseType_t uxLastMinQueueSpace = 0;
#endif
BaseType_t xResult = 0;
uint32_t xStatus;
const TickType_t ulMaxBlockTime = pdMS_TO_TICKS( EMAC_MAX_BLOCK_TIME_MS );

	/* Remove compiler warnings about unused parameters. */
	( void ) pvParameters;

	configASSERT( xEMACTaskHandle );

	vTaskSetTimeOutState( &xPhyTime );
	xPhyRemTime = pdMS_TO_TICKS( PHY_LS_LOW_CHECK_TIME_MS );

	for( ;; )
	{
		uxCurrentCount = uxGetNumberOfFreeNetworkBuffers();
		if( uxLastBufferCount != uxCurrentCount )
		{
			/* The logging produced below may be helpful
			while tuning +TCP: see how many buffers are in use. */
			FreeRTOS_debug_printf( ( "Network buffers: %lu lowest %lu\n",
				uxCurrentCount, uxGetMinimumFreeNetworkBuffers() ) );
		}
		uxLastBufferCount = uxCurrentCount;

		#if( ipconfigCHECK_IP_QUEUE_SPACE != 0 )
		{
			uxCurrentCount = uxGetMinimumIPQueueSpace();
			if( uxLastMinQueueSpace != uxCurrentCount )
			{
				/* The logging produced below may be helpful
				while tuning +TCP: see how many buffers are in use. */
				uxLastMinQueueSpace = uxCurrentCount;
				FreeRTOS_debug_printf( ( "Queue space: lowest %lu\n", uxCurrentCount ) );
			}
		}
		#endif /* ipconfigCHECK_IP_QUEUE_SPACE */

		if( ( ulISREvents & EMAC_IF_ALL_EVENT ) == 0 )
		{
			/* No events to process now, wait for the next notification from prvprvRxCallback() */
			ulTaskNotifyTake( pdFALSE, ulMaxBlockTime );
		}

		if( ( ulISREvents & EMAC_IF_RX_EVENT ) != 0 )
		{
			ulISREvents &= ~EMAC_IF_RX_EVENT;

			/* Wait for the EMAC interrupt to indicate that another packet has been
			received. */
			xResult = prvEMACRxPoll();
		}

		if( ( ulISREvents & EMAC_IF_TX_EVENT ) != 0 )
		{
			/* Future extension: code to release TX buffers if zero-copy is used. */
			ulISREvents &= ~EMAC_IF_TX_EVENT;
		}

		if( ( ulISREvents & EMAC_IF_ERR_EVENT ) != 0 )
		{
			/* Future extension: logging about errors that occurred. 
			   Statistics about errors are already kept at the GMAC driver level however. */
			ulISREvents &= ~EMAC_IF_ERR_EVENT;
		}

		if( xResult > 0 )
		{
			/* A packet was received. No need to check for the PHY status now,
			but set a timer to check it later on. */
			vTaskSetTimeOutState( &xPhyTime );
			xPhyRemTime = pdMS_TO_TICKS( PHY_LS_HIGH_CHECK_TIME_MS );
			xResult = 0;
		}
		else if( xTaskCheckForTimeOut( &xPhyTime, &xPhyRemTime ) != pdFALSE )
		{
			/* Check the link status again. */
			xStatus = ulReadMDIO( BMSR_REGISTER );

			if( ( ulPHYLinkStatus & BMSR_LINK_STATUS ) != ( xStatus & BMSR_LINK_STATUS ) )
			{
				ulPHYLinkStatus = xStatus;
				FreeRTOS_printf( ( "prvEMACHandlerTask: PHY LS now %d\n", ( ulPHYLinkStatus & BMSR_LINK_STATUS ) != 0 ) );
				// Handling link going down and coming back up is unsupported...
				//if( ( ulPHYLinkStatus & BMSR_LINK_STATUS ) == 0 ) {
					//FreeRTOS_NetworkDown();
				//}
			}

			vTaskSetTimeOutState( &xPhyTime );
			if( ( ulPHYLinkStatus & BMSR_LINK_STATUS ) != 0 )
			{
				xPhyRemTime = pdMS_TO_TICKS( PHY_LS_HIGH_CHECK_TIME_MS );
			}
			else
			{
				xPhyRemTime = pdMS_TO_TICKS( PHY_LS_LOW_CHECK_TIME_MS );
			}
		}
	}
}
/*-----------------------------------------------------------*/
