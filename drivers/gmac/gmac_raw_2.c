/**
 * \file
 *
 * \brief GMAC (Ethernet MAC) driver for SAM.
 *
 * Copyright (c) 2015-2016 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 * */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#ifdef __FREERTOS__
#include <FreeRTOS.h>
#include <task.h>
#include <FreeRTOSIPConfig.h>
#endif

#include "compiler.h"
#include "gmac.h"
#include "gmac_raw_2.h"
#include <string.h>
#include "conf_eth.h"

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

/**
 * \defgroup gmac_group Ethernet Media Access Controller
 *
 * See \ref gmac_quickstart.
 *
 * Driver for the GMAC (Ethernet Media Access Controller).
 * This file contains basic functions for the GMAC, with support for all modes, settings
 * and clock speeds.
 *
 * \section dependencies Dependencies
 * This driver does not depend on other modules.
 *
 * @{
 */

/* GMAC Send/Receive (TX/RX) descriptors and buffers must be placed in non-cacheable portion of RAM */
/** TX descriptor lists */
COMPILER_ALIGNED(8)
__attribute__ ((section(".ram_nocache"))) static gmac_tx_descriptor_t gs_tx_desc_null, gs_tx_desc[GMAC_TX_BUFFERS];
/** RX descriptors lists */
COMPILER_ALIGNED(8)
__attribute__ ((section(".ram_nocache"))) static gmac_rx_descriptor_t gs_rx_desc_null, gs_rx_desc[GMAC_RX_BUFFERS];

/** Send Buffer. Section 3.6 of AMBA 2.0 spec states that burst should not cross the
 * 1K Boundaries. Receive buffer manager write operations are burst of 8 words => 4 least significant bits
 * of the address shall be set to 0.
 */
COMPILER_ALIGNED(16)
__attribute__ ((section(".ram_nocache"))) static uint8_t gs_uc_tx_buffer[GMAC_TX_BUFFERS * GMAC_TX_UNITSIZE];
/** Receive Buffer */
COMPILER_ALIGNED(16)
__attribute__ ((section(".ram_nocache"))) static uint8_t gs_uc_rx_buffer[GMAC_RX_BUFFERS * GMAC_RX_UNITSIZE];

/** TX callback lists */
static gmac_dev_tx_cb_t gs_tx_callback[GMAC_TX_BUFFERS];

/* JB Note: increased size from 16- to 32-bits, because it's more efficient */
/**
 * GMAC device memory management struct.
 */
typedef struct gmac_dev_mem {
	/* Pointer to allocated buffer for RX. The address should be 8-byte aligned
	and the size should be GMAC_RX_UNITSIZE * GMAC_RX_BUFFERS. */
	uint8_t *p_rx_buffer;
	/* Pointer to allocated RX descriptor list. */
	gmac_rx_descriptor_t *p_rx_dscr;
	/* RX size, in number of registered units (RX descriptors). */
	uint32_t ul_rx_size;
	/* Pointer to allocated buffer for TX. The address should be 8-byte aligned
	and the size should be GMAC_TX_UNITSIZE * GMAC_TX_BUFFERS. */
	uint8_t *p_tx_buffer;
	/* Pointer to allocated TX descriptor list. */
	gmac_tx_descriptor_t *p_tx_dscr;
	/* TX size, in number of registered units (TX descriptors). */
	uint32_t ul_tx_size;
} gmac_dev_mem_t;

/** Return count in buffer */
#define CIRC_CNT(head,tail,size) (((head) - (tail)) % (size))

/*
 * Return space available, from 0 to size-1.
 * Always leave one free char as a completely full buffer that has (head == tail),
 * which is the same as empty.
 */
#define CIRC_SPACE(head,tail,size) CIRC_CNT((tail),((head)+1),(size))

/** Circular buffer is empty ? */
#define CIRC_EMPTY(head, tail)     (head == tail)
/** Clear circular buffer */
#define CIRC_CLEAR(head, tail)     (head = tail = 0)

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x)  (int)( sizeof(x) / sizeof(x)[0] )
#endif

/** Increment head or tail */
static void circ_inc(uint32_t *headortail, uint32_t ul_size)
{
    (*headortail)++;
    if( (*headortail) >= ul_size ) {
        (*headortail) = 0;
    }
}

/**
 * \brief Disable transfer, reset registers and descriptor lists.
 *
 * \param p_dev Pointer to GMAC driver instance.
 *
 */
static void gmac_reset_tx_mem(gmac_device_t* p_dev, gmac_quelist_t queue_idx)
{
	Gmac *p_hw = p_dev->p_hw;
	uint8_t *p_tx_buff = p_dev->gmac_queue_list[queue_idx].p_tx_buffer;
	gmac_tx_descriptor_t *p_td = p_dev->gmac_queue_list[queue_idx].p_tx_dscr;

	uint32_t ul_index;
	uint32_t ul_address;

	/* Disable TX */
	gmac_enable_transmit(p_hw, 0);

	/* Set up the TX descriptors */
	CIRC_CLEAR(p_dev->gmac_queue_list[queue_idx].ul_tx_head, p_dev->gmac_queue_list[queue_idx].ul_tx_tail);
	for (ul_index = 0; ul_index < p_dev->gmac_queue_list[queue_idx].ul_tx_list_size; ul_index++) {
		ul_address = (uint32_t) (&(p_tx_buff[ul_index * GMAC_TX_UNITSIZE]));
		p_td[ul_index].addr = ul_address;
		p_td[ul_index].status.val = GMAC_TXD_USED;
	}
	p_td[p_dev->gmac_queue_list[queue_idx].ul_tx_list_size - 1].status.val =
			GMAC_TXD_USED | GMAC_TXD_WRAP;

	/* Set transmit buffer queue */
	if(queue_idx == GMAC_QUE_0) {
		gmac_set_tx_queue(p_hw, (uint32_t) p_td);
	} else {
		gmac_set_tx_priority_queue(p_hw, (uint32_t) p_td, queue_idx);
	}
}

/**
 * \brief Disable receiver, reset registers and descriptor list.
 *
 * \param p_dev Pointer to GMAC Driver instance.
 */
static void gmac_reset_rx_mem(gmac_device_t* p_dev, gmac_quelist_t queue_idx)
{
	Gmac *p_hw = p_dev->p_hw;
	uint8_t *p_rx_buff = p_dev->gmac_queue_list[queue_idx].p_rx_buffer;
	gmac_rx_descriptor_t *pRd = p_dev->gmac_queue_list[queue_idx].p_rx_dscr;

	uint32_t ul_index;
	uint32_t ul_address;

	/* Disable RX */
	gmac_enable_receive(p_hw, 0);

	/* Set up the RX descriptors */
	p_dev->gmac_queue_list[queue_idx].ul_rx_idx = 0;
	for (ul_index = 0; ul_index < p_dev->gmac_queue_list[queue_idx].ul_rx_list_size; ul_index++) {
		ul_address = (uint32_t) (&(p_rx_buff[ul_index * GMAC_RX_UNITSIZE]));
		pRd[ul_index].addr.val = ul_address & GMAC_RXD_ADDR_MASK;
		pRd[ul_index].status.val = 0;
	}
	pRd[p_dev->gmac_queue_list[queue_idx].ul_rx_list_size - 1].addr.val |= GMAC_RXD_WRAP;

	/* Set receive buffer queue */
	if(queue_idx == GMAC_QUE_0) {
		gmac_set_rx_queue(p_hw, (uint32_t) pRd);
	} else {
		gmac_set_rx_priority_queue(p_hw, (uint32_t) pRd, queue_idx);
	}
}

/**
 * \brief Initialize the allocated buffer lists for GMAC driver to transfer data.
 * Must be invoked after gmac_dev_init() but before RX/TX starts.
 *
 * \note If input address is not 8-byte aligned, the address is automatically
 *       adjusted and the list size is reduced by one.
 *
 * \param p_gmac Pointer to GMAC instance.
 * \param p_gmac_dev Pointer to GMAC device instance.
 * \param p_dev_mm Pointer to the GMAC memory management control block.
 * \param p_tx_cb Pointer to allocated TX callback list.
 *
 * \return GMAC_OK or GMAC_PARAM.
 */
static uint8_t gmac_init_mem(gmac_device_t* p_gmac_dev, gmac_quelist_t queue_idx,
		gmac_dev_mem_t* p_dev_mm,
		gmac_dev_tx_cb_t* p_tx_cb)
{
	if (p_dev_mm->ul_rx_size <= 1 || p_dev_mm->ul_tx_size <= 1 || p_tx_cb == NULL) {
		return GMAC_PARAM;
	}

	gmac_queue_t* p_gmac_queue = &p_gmac_dev->gmac_queue_list[queue_idx];

	/* Assign RX buffers */
	if (((uint32_t) p_dev_mm->p_rx_buffer & 0x7)
			|| ((uint32_t) p_dev_mm->p_rx_dscr & 0x7)) {
		p_dev_mm->ul_rx_size--;
	}
	p_gmac_queue->p_rx_buffer =
			(uint8_t *) ((uint32_t) p_dev_mm->p_rx_buffer & 0xFFFFFFF8);
	p_gmac_queue->p_rx_dscr =
			(gmac_rx_descriptor_t *) ((uint32_t) p_dev_mm->p_rx_dscr
			& 0xFFFFFFF8);
	p_gmac_queue->ul_rx_list_size = p_dev_mm->ul_rx_size;

	/* Assign TX buffers */
	if (((uint32_t) p_dev_mm->p_tx_buffer & 0x7)
			|| ((uint32_t) p_dev_mm->p_tx_dscr & 0x7)) {
		p_dev_mm->ul_tx_size--;
	}
	p_gmac_queue->p_tx_buffer =
			(uint8_t *) ((uint32_t) p_dev_mm->p_tx_buffer & 0xFFFFFFF8);
	p_gmac_queue->p_tx_dscr =
			(gmac_tx_descriptor_t *) ((uint32_t) p_dev_mm->p_tx_dscr
			& 0xFFFFFFF8);
	p_gmac_queue->ul_tx_list_size = p_dev_mm->ul_tx_size;
	p_gmac_queue->func_tx_cb_list = p_tx_cb;

	/* Reset TX & RX */
	gmac_reset_rx_mem(p_gmac_dev, queue_idx);
	gmac_reset_tx_mem(p_gmac_dev, queue_idx);

	return GMAC_OK;
}

static void gmac_init_queue(Gmac* p_gmac, gmac_device_t* p_gmac_dev)
{
	gmac_dev_mem_t gmac_dev_mm;

	/* Clear interrupts */
	gmac_get_priority_interrupt_status(p_gmac, GMAC_QUE_2);
	gmac_get_priority_interrupt_status(p_gmac, GMAC_QUE_1);

	/* Set Tx Priority */
	gs_tx_desc_null.addr = (uint32_t)0xFFFFFFFF;
	gs_tx_desc_null.status.val = GMAC_TXD_WRAP | GMAC_TXD_USED;
	gmac_set_tx_priority_queue(p_gmac, (uint32_t)&gs_tx_desc_null, GMAC_QUE_2);
	gmac_set_tx_priority_queue(p_gmac, (uint32_t)&gs_tx_desc_null, GMAC_QUE_1);
	
	/* Set Rx Priority */
	gs_rx_desc_null.addr.val = (uint32_t)0xFFFFFFFF & GMAC_RXD_ADDR_MASK;
	gs_rx_desc_null.addr.val |= GMAC_RXD_WRAP;
	gs_rx_desc_null.status.val = 0;
	gmac_set_rx_priority_queue(p_gmac, (uint32_t)&gs_rx_desc_null, GMAC_QUE_2);
	gmac_set_rx_priority_queue(p_gmac, (uint32_t)&gs_rx_desc_null, GMAC_QUE_1);

	/* Clear interrupts */
	gmac_get_interrupt_status(p_gmac);

	/* Fill in GMAC device memory management */
	gmac_dev_mm.p_rx_buffer = gs_uc_rx_buffer;
	gmac_dev_mm.p_rx_dscr = gs_rx_desc;
	gmac_dev_mm.ul_rx_size = GMAC_RX_BUFFERS;

	gmac_dev_mm.p_tx_buffer = gs_uc_tx_buffer;
	gmac_dev_mm.p_tx_dscr = gs_tx_desc;
	gmac_dev_mm.ul_tx_size = GMAC_TX_BUFFERS;

	gmac_init_mem(p_gmac_dev, GMAC_QUE_0, &gmac_dev_mm, gs_tx_callback);

	/* Enable Rx and Tx, plus the statistics register */
	gmac_enable_transmit(p_gmac, true);
	gmac_enable_receive(p_gmac, true);
	gmac_enable_statistics_write(p_gmac, true);

	/* Set up the interrupts for transmission and errors */
	gmac_enable_interrupt(p_gmac,
		GMAC_IER_RLEX  | /* Enable retry limit exceeded interrupt. */
		GMAC_IER_RCOMP | /* Enable receive complete interrupt. */
		GMAC_IER_RXUBR | /* Enable receive used bit read interrupt. */
		GMAC_IER_ROVR  | /* Enable receive overrun interrupt. */
		GMAC_IER_TCOMP | /* Enable transmit complete interrupt. */
		/* JB TODO: the following 5 are not checked explicitly in handler */
		GMAC_IER_TUR   | /* Enable transmit underrun interrupt. */
		GMAC_IER_TFC   | /* Enable transmit buffers exhausted in mid-frame interrupt. */
		GMAC_IER_HRESP | /* Enable Hresp not OK interrupt. */
		GMAC_IER_PFNZ  | /* Enable pause frame received interrupt. */
		GMAC_IER_PTZ);   /* Enable pause time zero interrupt. */
}

/**
 * \brief Initialize the GMAC driver.
 *
 * \param p_gmac   Pointer to the GMAC instance.
 * \param p_gmac_dev Pointer to the GMAC device instance.
 * \param p_opt GMAC configure options.
 */
void gmac_dev_init(Gmac* p_gmac, gmac_device_t* p_gmac_dev,
		gmac_options_t* p_opt)
{
	/* Disable TX & RX and more */
	gmac_network_control(p_gmac, 0);
	gmac_disable_interrupt(p_gmac, ~0u);

	gmac_clear_statistics(p_gmac);

	/* Clear all status bits in the receive status register. */
	gmac_clear_rx_status(p_gmac, GMAC_RSR_RXOVR | GMAC_RSR_REC | GMAC_RSR_BNA
			| GMAC_RSR_HNO);

	/* Clear all status bits in the transmit status register */
	gmac_clear_tx_status(p_gmac, GMAC_TSR_UBR | GMAC_TSR_COL | GMAC_TSR_RLE
            | GMAC_TSR_TXGO | GMAC_TSR_TFC | GMAC_TSR_TXCOMP | GMAC_TSR_HRESP );

#if !defined(ETHERNET_CONF_DATA_OFFSET)
	/*  Receive Buffer Offset
	 * Indicates the number of bytes by which the received data
	 * is offset from the start of the receive buffer
	 * which can be handy for alignment reasons */
	/* Note: FreeRTOS+TCP wants to have this offset set to 2 bytes */
	#error ETHERNET_CONF_DATA_OFFSET not defined, assuming 0
#endif

	/* Set GMAC parameters */
	/* Note: 100/10Mbps, duplex, and clock speed are set in ethernet_phy code */
	gmac_set_config(p_gmac, 
			(gmac_get_config(p_gmac) & ~GMAC_NCFGR_RXBUFO_Msk) |
			GMAC_NCFGR_RXBUFO( ETHERNET_CONF_DATA_OFFSET ) | /* start offset from receive buffer */
			GMAC_NCFGR_DBW(0) | /* Data bus width */
			GMAC_NCFGR_MAXFS | /* 1536 Maximum frame size (not 1518) */
			GMAC_NCFGR_RFCS | /*  Remove FCS, frame check sequence (last 4 bytes) */
			GMAC_NCFGR_PEN | /* Pause Enable */
			GMAC_NCFGR_RXCOEN /* Receive checksum offload enabled - discard frames w/ bad IP/TCP/UDP checksums */
			);
	/* Enable/disable the copy of all data into buffers and
	 * ignore/accept broadcasts, according to preferences */
	gmac_enable_copy_all(p_gmac, p_opt->uc_copy_all_frame);
	gmac_disable_broadcast(p_gmac, p_opt->uc_no_broadcast);
		
	/* Enable Transmitter Checksum Generation Offload to calculate & 
	 * substitute checksums for transmit frames; 
	 * Increase DMA burst speeds to 16 bytes from 4 bytes */
	gmac_set_dma_config(p_gmac,
			gmac_get_dma_config(p_gmac) | GMAC_DCFGR_TXCOEN | GMAC_DCFGR_FBLDO_INCR16);

	/* Enable partial store and forward behavior for Rx; this may slightly reduce 
	   latencies but has other disadvantages; prefer to leave off for simplicity */
	/* gmac_enable_rx_psf(p_gmac, true); */
	
	/* Could change default size of each Rx buffer from GMAC_RX_UNITSIZE (default of 128 bytes): 
	gmac_set_rx_bufsize(param is multiplied by 64 for buffer size); */

	gmac_init_queue(p_gmac, p_gmac_dev);

	gmac_set_address(p_gmac, 0, p_opt->uc_mac_addr);
}

/**
 * \brief Return the number of RX buffer full.
 *
 * \param p_gmac_dev Pointer to the GMAC device instance.
 *
 * \return The number of RX buffer full.
 */
uint32_t gmac_dev_rx_buf_used(gmac_device_t* p_gmac_dev, gmac_quelist_t queue_idx)
{
	uint32_t used = 0;

	for (uint32_t i = 0; i < p_gmac_dev->gmac_queue_list[queue_idx].ul_rx_list_size; ++i) {
		if (p_gmac_dev->gmac_queue_list[queue_idx].p_rx_dscr[i].addr.val & GMAC_RXD_OWNERSHIP)
			used += 1;
	}

	return used;
}

/**
 * \brief Frames can be read from the GMAC in multiple sections.
 *
 * Returns > 0 (data length) if a complete frame is available
 * It also it cleans up incomplete older frames
 */
static uint32_t gmac_dev_poll(gmac_device_t* p_gmac_dev, gmac_quelist_t queue_idx)
{
	uint32_t ulReturn = 0;

	gmac_queue_t* p_gmac_queue = &p_gmac_dev->gmac_queue_list[queue_idx];
	uint32_t ulIndex = p_gmac_queue->ul_rx_idx;
	uint32_t ulStartIndex = ulIndex;
	/* Received RX descriptor */
	gmac_rx_descriptor_t *pxHead = &p_gmac_queue->p_rx_dscr[ulIndex];

	/* Discard any incomplete frames */
	while ((pxHead->addr.val & GMAC_RXD_OWNERSHIP) != 0 &&
			(pxHead->status.val & GMAC_RXD_SOF) == 0) {
		pxHead->addr.val &= ~(GMAC_RXD_OWNERSHIP); /* Setting to zero frees it for the GMAC to write to */
		circ_inc(&ulIndex, p_gmac_queue->ul_rx_list_size);
		configASSERT(ulIndex != ulStartIndex); // GMAC_RX_ERROR, we've circled around; shuoldn't happen since we're unsetting ownership

		pxHead = &p_gmac_queue->p_rx_dscr[ulIndex];
		p_gmac_queue->ul_rx_idx = ulIndex;
		#if( GMAC_STATS != 0 )
		{
			gmacStats.incompCount++;
		}
		#endif
	}

	while ((pxHead->addr.val & GMAC_RXD_OWNERSHIP) != 0) {
		if ((pxHead->status.val & GMAC_RXD_EOF) != 0) {
			/* Here a complete frame has been seen with SOF and EOF */
			ulReturn = pxHead->status.bm.b_len;
			break;
		}
		circ_inc(&ulIndex, p_gmac_queue->ul_rx_list_size);
		configASSERT(ulIndex != ulStartIndex); // GMAC_RX_ERROR, we've circled around; shouldn't happen since we're unsetting ownership
		
		pxHead = &p_gmac_queue->p_rx_dscr[ulIndex];
		if ((pxHead->addr.val & GMAC_RXD_OWNERSHIP) == 0) {
			/* CPU is not the owner (yet) */
			break;
		}
		if ((pxHead->status.val & GMAC_RXD_SOF) != 0) {
			/* Strange, we found a new Start Of Frame;
			 * discard previous segments */
			uint32_t ulPrev = p_gmac_queue->ul_rx_idx;
			pxHead = &p_gmac_queue->p_rx_dscr[ulPrev];
			do {
				pxHead->addr.val &= ~(GMAC_RXD_OWNERSHIP);
				circ_inc(&ulPrev, p_gmac_queue->ul_rx_list_size);
				pxHead = &p_gmac_queue->p_rx_dscr[ulPrev];
				#if( GMAC_STATS != 0 )
				{
					gmacStats.truncCount++;
				}
				#endif
			} while (ulPrev != ulIndex);
			p_gmac_queue->ul_rx_idx = ulIndex;
		}
	}
	return ulReturn;
}

/**
 * \brief Frames can be read from the GMAC in multiple sections.
 * Read ul_frame_size bytes from the GMAC receive buffers to p_frame.
 * p_rcv_size is the size of the entire frame.  Generally gmac_dev_read
 * will be repeatedly called until the sum of all the p_rcv_size equals
 * the value of ul_frame_size.
 *
 * \param p_gmac_dev Pointer to the GMAC device instance.
 * \param p_frame Address of the frame buffer. If NULL, discard frames to free space.
 * \param ul_frame_size  Length of the frame.
 * \param p_rcv_size   Received frame size.
 *
 * \return GMAC_OK if receiving frame successfully, otherwise failed.
 */
uint32_t gmac_dev_read(gmac_device_t* p_gmac_dev, gmac_quelist_t queue_idx, 
		uint8_t* p_frame, uint32_t ul_frame_size, uint32_t* p_rcv_size)
{
	uint32_t nextIdx;	/* A copy of the Rx-index 'ul_rx_idx' */
	uint32_t bytesLeft = gmac_dev_poll(p_gmac_dev, queue_idx);
	gmac_queue_t* p_gmac_queue = &p_gmac_dev->gmac_queue_list[queue_idx];
	gmac_rx_descriptor_t *pxHead;

	if (bytesLeft == 0 )
	{
		return GMAC_RX_NO_DATA; /* or other error */
	}

	/* gmac_dev_poll has confirmed that there is a complete frame at
	 * the current position 'ul_rx_idx'
	 */
	nextIdx = p_gmac_queue->ul_rx_idx;

	/* Read +2 bytes because buffers are aligned at -2 bytes */
	bytesLeft = min( bytesLeft + 2, ul_frame_size );

	/* The frame will be copied in 1 or 2 memcpy's */
	if( ( p_frame != NULL ) && ( bytesLeft != 0 ) )
	{
		const uint8_t *source;
		uint32_t left;
		uint32_t toCopy;

		source = p_gmac_queue->p_rx_buffer + nextIdx * GMAC_RX_UNITSIZE;
		left = bytesLeft;
		toCopy = ( p_gmac_queue->ul_rx_list_size - nextIdx ) * GMAC_RX_UNITSIZE;
		if(toCopy > left )
		{
			toCopy = left;
		}
		memcpy (p_frame, source, toCopy);
		left -= toCopy;

		if( left != 0ul )
		{
			/* circular buffer; copy first portion of buffer to second portion of frame */
			memcpy (p_frame + toCopy, (void*)p_gmac_queue->p_rx_buffer, left);
		}
	}

	/* Discard the frame segments we have just copied, or if p_frame == NULL */
	do
	{
		pxHead = &p_gmac_queue->p_rx_dscr[nextIdx];
		pxHead->addr.val &= ~(GMAC_RXD_OWNERSHIP);
		circ_inc(&nextIdx, p_gmac_queue->ul_rx_list_size);
	} while ((pxHead->status.val & GMAC_RXD_EOF) == 0);

	p_gmac_queue->ul_rx_idx = nextIdx;
	*p_rcv_size = bytesLeft;
	#if( GMAC_STATS != 0 )
	if (p_frame != NULL) {
		gmacStats.recvCount++;
	}
	else {
		gmacStats.rignoCount++;
	}
	#endif // GMAC_STATS !=0

	return GMAC_OK;
}

/**
 * \brief Return the number of TX buffer waiting for transfer.
 *
 * \param p_gmac_dev Pointer to the GMAC device instance.
 *
 * \return The number of TX buffer used.
 */
uint32_t gmac_dev_tx_buf_used(gmac_device_t* p_gmac_dev, gmac_quelist_t queue_idx)
{
	uint32_t used = 0;

	for (uint32_t i = 0; i < p_gmac_dev->gmac_queue_list[queue_idx].ul_tx_list_size; ++i) {
		if ((p_gmac_dev->gmac_queue_list[queue_idx].p_tx_dscr[i].status.val & GMAC_TXD_USED) == 0)
			used += 1;
	}

	return used;
}

/* Above, we set TXCOEN on, so CRCs are calculated and inserted by the GMAC. 
As such we disable it here. The code is left because some GMACs do not 
support TXCOEN, despite claims to the contrary. */
/* ipconfigHAS_TX_CRC_OFFLOADING should be set to 1 in FreeRTOSIPConfig.h */

#if ( ipconfigHAS_TX_CRC_OFFLOADING == 0 )
extern void vGMACGenerateChecksum( uint8_t *apBuffer );
#endif
/**
 * \brief Send ulLength bytes from pcFrom. This copies the buffer to one of the
 * GMAC Tx buffers, and then indicates to the GMAC that the buffer is ready.
 * If lEndOfFrame is true then the data being copied is the end of the frame
 * and the frame can be transmitted.
 *
 * \param p_gmac_dev Pointer to the GMAC device instance.
 * \param p_buffer       Pointer to the data buffer.
 * \param ul_size    Length of the frame.
 * \param func_tx_cb  Transmit callback function (NULL for none)
 *
 * \return Length sent.
 */
uint32_t gmac_dev_write(gmac_device_t* p_gmac_dev, gmac_quelist_t queue_idx, void *p_buffer,
		uint32_t ul_size, gmac_dev_tx_cb_t func_tx_cb)
{
	volatile gmac_tx_descriptor_t *p_tx_td;
	volatile gmac_dev_tx_cb_t *p_func_tx_cb;

	Gmac *p_hw = p_gmac_dev->p_hw;
	gmac_queue_t* p_gmac_queue = &p_gmac_dev->gmac_queue_list[queue_idx];

	/* Check parameter */
	if (ul_size > GMAC_TX_UNITSIZE) {
		return GMAC_PARAM;
	}

	/* Pointers to the current transmit descriptor */
	p_tx_td = &p_gmac_queue->p_tx_dscr[p_gmac_queue->ul_tx_head];

	/* If no free TxTd, buffer can't be sent, schedule the wakeup callback */
	if ( CIRC_SPACE(p_gmac_queue->ul_tx_head, p_gmac_queue->ul_tx_tail,
					p_gmac_queue->ul_tx_list_size) == 0 &&
		 (p_tx_td[p_gmac_queue->ul_tx_head].status.val & GMAC_TXD_USED) )
	{
		#if( GMAC_STATS != 0 )
		{
			gmacStats.sbusyCount++;
		}
		#endif // GMAC_STATS !=0
		return GMAC_TX_BUSY;
	}

	/* Pointers to the current Tx callback */
	p_func_tx_cb = &p_gmac_queue->func_tx_cb_list[p_gmac_queue->ul_tx_head];

	/* Set up/copy data to transmission buffer */
	if (p_buffer && ul_size) {
		/* Driver manages the ring buffer */
		memcpy((void *)p_tx_td->addr, p_buffer, ul_size);
#if ( ipconfigHAS_TX_CRC_OFFLOADING == 0 )
		/* calculate & insert checksum */
		vGMACGenerateChecksum(( uint8_t * ) p_tx_td->addr);
#endif
	}

	/* Tx callback */
	*p_func_tx_cb = func_tx_cb;

	/* Update transmit descriptor status */

	/* The buffer size defined as the length of an ethernet frame,
	   so it's always the last buffer of the frame. */
	p_tx_td->status.val = (ul_size & GMAC_TXD_LEN_MASK) | GMAC_TXD_LAST;
	if (p_gmac_queue->ul_tx_head == p_gmac_queue->ul_tx_list_size - 1) {
		p_tx_td->status.val |= GMAC_TXD_WRAP;
	}

	circ_inc(&p_gmac_queue->ul_tx_head, p_gmac_queue->ul_tx_list_size);

	/* Now start to transmit if it is still not done */
	gmac_start_transmission(p_hw);

	#if( GMAC_STATS != 0 )
	{
		gmacStats.sendCount++;
	}
	#endif // GMAC_STATS !=0

	return GMAC_OK;
}

uint32_t gmac_dev_write_nocopy(gmac_device_t* p_gmac_dev, gmac_quelist_t queue_idx,
		uint32_t ul_size, gmac_dev_tx_cb_t func_tx_cb)
{
	volatile gmac_tx_descriptor_t *p_tx_td;
	volatile gmac_dev_tx_cb_t *p_func_tx_cb;

	Gmac *p_hw = p_gmac_dev->p_hw;
	gmac_queue_t* p_gmac_queue = &p_gmac_dev->gmac_queue_list[queue_idx];

	/* Check parameter */
	if (ul_size > GMAC_TX_UNITSIZE) {
		return GMAC_PARAM;
	}

	/* Pointers to the current transmit descriptor */
	p_tx_td = &p_gmac_queue->p_tx_dscr[p_gmac_queue->ul_tx_head];

	/* Pointers to the current Tx callback */
	p_func_tx_cb = &p_gmac_queue->func_tx_cb_list[p_gmac_queue->ul_tx_head];

	/* Tx callback */
	*p_func_tx_cb = func_tx_cb;

	/* Update transmit descriptor status */

	/* The buffer size defined as the length of an ethernet frame,
	   so it's always the last buffer of the frame. */
	p_tx_td->status.val = (ul_size & GMAC_TXD_LEN_MASK) | GMAC_TXD_LAST;
	if (p_gmac_queue->ul_tx_head == p_gmac_queue->ul_tx_list_size - 1) {
		p_tx_td->status.val |= GMAC_TXD_WRAP;
	}

	circ_inc(&p_gmac_queue->ul_tx_head, p_gmac_queue->ul_tx_list_size);

	/* Now start to transmit if it is still not done */
	gmac_start_transmission(p_hw);

	return GMAC_OK;
}

uint8_t *gmac_dev_get_tx_buffer(gmac_device_t* p_gmac_dev, gmac_quelist_t queue_idx)
{
	volatile gmac_tx_descriptor_t *p_tx_td;

	gmac_queue_t* p_gmac_queue = &p_gmac_dev->gmac_queue_list[queue_idx];
	/* Pointers to the current transmit descriptor */
	p_tx_td = &p_gmac_queue->p_tx_dscr[p_gmac_queue->ul_tx_head];

	/* If no free TxTd, forget it */
	if (CIRC_SPACE(p_gmac_queue->ul_tx_head, p_gmac_queue->ul_tx_tail,
					p_gmac_queue->ul_tx_list_size) == 0) {
		if (p_tx_td[p_gmac_queue->ul_tx_head].status.val & GMAC_TXD_USED)
			return 0;
	}

	return (uint8_t *)p_tx_td->addr;
}


/**
 * \brief Get current load of transmit.
 *
 * \param p_gmac_dev Pointer to the GMAC device instance.
 *
 * \return Current load of transmit.
 */
uint32_t gmac_dev_get_tx_load(gmac_device_t* p_gmac_dev, gmac_quelist_t queue_idx)
{
	uint32_t ul_head = p_gmac_dev->gmac_queue_list[queue_idx].ul_tx_head;
	uint32_t ul_tail = p_gmac_dev->gmac_queue_list[queue_idx].ul_tx_tail;
	return CIRC_CNT(ul_head, ul_tail, p_gmac_dev->gmac_queue_list[queue_idx].ul_tx_list_size);
}

/**
 * \brief Register/Clear RX callback. Callback will be invoked after the next received
 * frame.
 *
 * When gmac_dev_read() returns GMAC_RX_NULL, the application task calls
 * gmac_dev_set_rx_callback() to register func_rx_cb() callback and enters suspend state.
 * The callback is in charge to resume the task once a new frame has been
 * received. The next time gmac_dev_read() is called, it will be successful.
 *
 * This function is usually invoked from the RX callback itself with NULL
 * callback, to unregister. Once the callback has resumed the application task,
 * there is no need to invoke the callback again.
 *
 * \param p_gmac_dev Pointer to the GMAC device instance.
 * \param func_tx_cb  Receive callback function.
 */
void gmac_dev_set_rx_callback(gmac_device_t* p_gmac_dev, gmac_quelist_t queue_idx,
		gmac_dev_tx_cb_t func_rx_cb)
{
	Gmac *p_hw = p_gmac_dev->p_hw;

	if (func_rx_cb == NULL) {
		if(queue_idx == GMAC_QUE_0) {
			gmac_disable_interrupt(p_hw, GMAC_IDR_RCOMP);
		} else {
			gmac_disable_priority_interrupt(p_hw, GMAC_IER_RCOMP, queue_idx);
		}
		p_gmac_dev->gmac_queue_list[queue_idx].func_rx_cb = NULL;
	} else {
		p_gmac_dev->gmac_queue_list[queue_idx].func_rx_cb = func_rx_cb;
		if(queue_idx == GMAC_QUE_0) {
			gmac_enable_interrupt(p_hw, GMAC_IER_RCOMP);
		} else {
			gmac_enable_priority_interrupt(p_hw, GMAC_IER_RCOMP, queue_idx);
		}
	}
}

/**
 *  \brief Register/Clear TX wakeup callback.
 *
 * When gmac_dev_write() returns GMAC_TX_BUSY (all transmit descriptor busy), the application
 * task calls gmac_dev_set_tx_wakeup_callback() to register func_wakeup() callback and
 * enters suspend state. The callback is in charge to resume the task once
 * several transmit descriptors have been released. The next time gmac_dev_write() will be called,
 * it shall be successful.
 *
 * This function is usually invoked with NULL callback from the TX wakeup
 * callback itself, to unregister. Once the callback has resumed the
 * application task, there is no need to invoke the callback again.
 *
 * \param p_gmac_dev   Pointer to GMAC device instance.
 * \param func_wakeup    Pointer to wakeup callback function.
 * \param uc_threshold Number of free transmit descriptor before wakeup callback invoked.
 *
 * \return GMAC_OK, GMAC_PARAM on parameter error.
 */
uint8_t gmac_dev_set_tx_wakeup_callback(gmac_device_t* p_gmac_dev, gmac_quelist_t queue_idx,
		gmac_dev_wakeup_cb_t func_wakeup_cb, uint8_t uc_threshold)
{
	if (func_wakeup_cb == NULL) {
		p_gmac_dev->gmac_queue_list[queue_idx].func_wakeup_cb = NULL;
	} else {
		if (uc_threshold <= p_gmac_dev->gmac_queue_list[queue_idx].ul_tx_list_size) {
			p_gmac_dev->gmac_queue_list[queue_idx].func_wakeup_cb = func_wakeup_cb;
			p_gmac_dev->gmac_queue_list[queue_idx].uc_wakeup_threshold = uc_threshold;
		} else {
			return GMAC_PARAM;
		}
	}

	return GMAC_OK;
}


/**
 * \brief Reset TX & RX queue & statistics.
 *
 * \param p_gmac_dev   Pointer to GMAC device instance.
 */
void gmac_dev_reset(gmac_device_t* p_gmac_dev, gmac_quelist_t queue_idx)
{
	Gmac *p_hw = p_gmac_dev->p_hw;

	gmac_reset_rx_mem(p_gmac_dev, queue_idx);
	gmac_reset_tx_mem(p_gmac_dev, queue_idx);
	gmac_network_control(p_hw, GMAC_NCR_TXEN | GMAC_NCR_RXEN
			| GMAC_NCR_WESTAT | GMAC_NCR_CLRSTAT);
}

void gmac_dev_halt(Gmac* p_gmac);

void gmac_dev_halt(Gmac* p_gmac)
{
	gmac_network_control(p_gmac, GMAC_NCR_WESTAT | GMAC_NCR_CLRSTAT);
	gmac_disable_interrupt(p_gmac, ~0u);
}

#if( GMAC_STATS != 0 )
/* definition here */
struct SGmacStats gmacStats;

extern void vConsolePrintf( const char *pcFormat, ... );

void gmac_show_irq_counts ()
{
	int index;
	for (index = 0; index < ARRAY_SIZE(intPairs); index++) {
		if (gmacStats.intStatus[intPairs[index].index]) {
			vConsolePrintf("%s : %6u\n", intPairs[index].name, gmacStats.intStatus[intPairs[index].index]);
		}
	}
}
#endif

/**
 * \brief GMAC Interrupt handler.
 *
 * \param p_gmac_dev   Pointer to GMAC device instance.
 */
void gmac_handler(gmac_device_t* p_gmac_dev, gmac_quelist_t queue_idx)
{
	Gmac *p_hw = p_gmac_dev->p_hw;

	gmac_tx_descriptor_t *p_tx_td;
	gmac_dev_tx_cb_t *p_tx_cb = NULL;
	/* volatile */ uint32_t ul_isr;
	/* volatile */ uint32_t ul_rsr;
	/* volatile */ uint32_t ul_tsr;

	uint32_t ul_tx_status_flag;

	gmac_queue_t* p_gmac_queue = &p_gmac_dev->gmac_queue_list[queue_idx];

	if(queue_idx == GMAC_QUE_0) {
		ul_isr = gmac_get_interrupt_status(p_hw);
	} else {
		ul_isr = gmac_get_priority_interrupt_status(p_hw, queue_idx);
	}
	ul_rsr = gmac_get_rx_status(p_hw);
	ul_tsr = gmac_get_tx_status(p_hw);

	#if( GMAC_STATS != 0 )
	{
		int index;
		for (index = 0; index < ARRAY_SIZE(intPairs); index++) {
			if (ul_isr & intPairs[index].mask)
				gmacStats.intStatus[intPairs[index].index]++;
		}
	}
	#endif /* GMAC_STATS != 0 */

	ul_isr &= ~(gmac_get_interrupt_mask(p_hw) | 0xF8030300);

	/* RX packet */
	if ((ul_isr & GMAC_ISR_RCOMP) || (ul_rsr & (GMAC_RSR_REC|GMAC_RSR_RXOVR|GMAC_RSR_BNA|GMAC_RSR_HNO))) {
		/* Clear status */
		gmac_clear_rx_status(p_hw, ul_rsr);

		if (ul_isr & GMAC_ISR_RCOMP)
			ul_rsr |= GMAC_RSR_REC;
		
#if( GMAC_STATS != 0 )
		{
			if (ul_rsr & GMAC_RSR_BNA) {
				gmacStats.bnaCount++;
			}
			if (ul_rsr & GMAC_RSR_RXOVR) {
				gmacStats.rovrCount++;
			}
			if (ul_rsr & GMAC_RSR_HNO) {
				gmacStats.hnoCount++;
			}
		}
#endif // GMAC_STATS !=0

		/* Invoke callbacks */
		if (p_gmac_queue->func_rx_cb) {
			p_gmac_queue->func_rx_cb(ul_rsr);
		}
	}

	/* TX packet */
	if ((ul_isr & GMAC_ISR_TCOMP) || 
		(ul_tsr & (GMAC_TSR_TXCOMP|GMAC_TSR_COL|GMAC_TSR_RLE|GMAC_TSR_TFC|GMAC_TSR_HRESP))) {
		ul_tx_status_flag = GMAC_TSR_TXCOMP;

		/* Check Retry Limit Exceeded (RLE) */
		if (ul_tsr & GMAC_TSR_RLE) {
			/* Status RLE & Number of discarded buffers */
			ul_tx_status_flag = GMAC_TSR_RLE | 
				CIRC_CNT(p_gmac_queue->ul_tx_head, p_gmac_queue->ul_tx_tail, p_gmac_queue->ul_tx_list_size);
			p_tx_cb = &p_gmac_queue->func_tx_cb_list[p_gmac_queue->ul_tx_tail];
			gmac_reset_tx_mem(p_gmac_dev, queue_idx);
			gmac_enable_transmit(p_hw, 1);
#if( GMAC_STATS != 0 )
			gmacStats.srleCount++;
#endif
		}
		/* Check COL */
		if (ul_tsr & GMAC_TSR_COL) {
			ul_tx_status_flag |= GMAC_TSR_COL;
#if( GMAC_STATS != 0 )
			gmacStats.scolCount++;
#endif
		}
#if( GMAC_STATS != 0 )
		{
			if (ul_tsr & GMAC_TSR_TFC) {
				gmacStats.tfcCount++;
			}
			if (ul_tsr & GMAC_TSR_HRESP) {
				gmacStats.shnoCount++;
			}
		}
#endif

		/* Clear status */
		gmac_clear_tx_status(p_hw, ul_tx_status_flag);

		if (!CIRC_EMPTY(p_gmac_queue->ul_tx_head, p_gmac_queue->ul_tx_tail)) {
			/* Check the buffers */
			do {
				p_tx_td = &p_gmac_queue->p_tx_dscr[p_gmac_queue->ul_tx_tail];
				p_tx_cb = &p_gmac_queue->func_tx_cb_list[p_gmac_queue->ul_tx_tail];
				/* Any error? Exit if buffer has not been sent yet */
				if ((p_tx_td->status.val & GMAC_TXD_USED) == 0) {
					break;
				}

				/* Notify upper layer that a packet has been sent */
				if (*p_tx_cb) { /* Callback fn not NULL */
					(*p_tx_cb) (ul_tx_status_flag);
				}

				circ_inc(&p_gmac_queue->ul_tx_tail, p_gmac_queue->ul_tx_list_size);
			} while (CIRC_CNT(p_gmac_queue->ul_tx_head, p_gmac_queue->ul_tx_tail,
							  p_gmac_queue->ul_tx_list_size));
		}

		if (ul_tsr & GMAC_TSR_RLE) {
			/* Notify upper layer RLE */
			if (*p_tx_cb) {
				(*p_tx_cb) (ul_tx_status_flag);
			}
		}

		/* If a wakeup has been scheduled, notify upper layer that it can
		   send other packets, and the sending will be successful. */
		if ((CIRC_SPACE(p_gmac_queue->ul_tx_head, p_gmac_queue->ul_tx_tail,
				p_gmac_queue->ul_tx_list_size) >= p_gmac_queue->uc_wakeup_threshold)
				&& p_gmac_queue->func_wakeup_cb) {
			p_gmac_queue->func_wakeup_cb();
		}
	}
}

//@}

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond
