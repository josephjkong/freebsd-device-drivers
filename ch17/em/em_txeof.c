/* $Id: em_txeof.c,v 1.3 2012/04/13 10:15:13 ghost Exp $ */

/*-
 * Copyright (c) 2001-2011 Intel Corporation.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Intel Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

static bool
em_txeof(struct tx_ring *txr)
{
	struct adapter *adapter = txr->adapter;
	struct ifnet *ifp = adapter->ifp;
	struct e1000_tx_desc *tx_desc, *eop_desc;
	struct em_buffer *tx_buffer;
	int processed, first, last, done;

	EM_TX_LOCK_ASSERT(txr);

	if (txr->tx_avail == adapter->num_tx_desc) {
		txr->queue_status = EM_QUEUE_IDLE;
		return (FALSE);
	}

	processed = 0;
	first = txr->next_to_clean;
	tx_desc = &txr->tx_base[first];
	tx_buffer = &txr->tx_buffers[first];
	last = tx_buffer->next_eop;
	eop_desc = &txr->tx_base[last];

	if (++last == adapter->num_tx_desc)
		last = 0;
	done = last;

	bus_dmamap_sync(txr->txdma.dma_tag, txr->txdma.dma_map,
	    BUS_DMASYNC_POSTREAD);

	while (eop_desc->upper.fields.status & E1000_TXD_STAT_DD) {
		while (first != done) {
			tx_desc->upper.data = 0;
			tx_desc->lower.data = 0;
			tx_desc->buffer_addr = 0;
			++txr->tx_avail;
			++processed;

			if (tx_buffer->m_head) {
				bus_dmamap_unload(txr->txtag,
				    tx_buffer->map);
				m_freem(tx_buffer->m_head);
				tx_buffer->m_head = NULL;
			}

			tx_buffer->next_eop = -1;
			txr->watchdog_time = ticks;

			if (++first == adapter->num_tx_desc)
				first = 0;
			tx_buffer = &txr->tx_buffers[first];
			tx_desc = &txr->tx_base[first];
		}

		++ifp->if_opackets;

		last = tx_buffer->next_eop;
		if (last != -1) {
			eop_desc = &txr->tx_base[last];
			if (++last == adapter->num_tx_desc)
				last = 0;
			done = last;
		} else
			break;
	}

	bus_dmamap_sync(txr->txdma.dma_tag, txr->txdma.dma_map,
	    BUS_DMASYNC_PREWRITE);

	txr->next_to_clean = first;

	if (!processed && ((ticks - txr->watchdog_time) > EM_WATCHDOG))
		txr->queue_status = EM_QUEUE_HUNG;

	if (txr->tx_avail > EM_MAX_SCATTER)
		ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;

	if (txr->tx_avail == adapter->num_tx_desc) {
		txr->queue_status = EM_QUEUE_IDLE;
		return (FALSE);
	}

	return (TRUE);
}
