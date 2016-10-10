/* $Id: em_setup_interface.c,v 1.4 2012/04/13 10:01:20 ghost Exp $ */

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

static int
em_setup_interface(device_t dev, struct adapter *adapter)
{
	struct ifnet *ifp;

	ifp = adapter->ifp = if_alloc(IFT_ETHER);
	if (ifp == NULL) {
		device_printf(dev, "cannot allocate ifnet structure\n");
		return (-1);
	}

	if_initname(ifp, device_get_name(dev), device_get_unit(dev));
	ifp->if_mtu = ETHERMTU;
	ifp->if_init = em_init;
	ifp->if_softc = adapter;
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_ioctl = em_ioctl;
	ifp->if_start = em_start;
	IFQ_SET_MAXLEN(&ifp->if_snd, adapter->num_tx_desc - 1);
	ifp->if_snd.ifq_drv_maxlen = adapter->num_tx_desc - 1;
	IFQ_SET_READY(&ifp->if_snd);

	ether_ifattach(ifp, adapter->hw.mac.addr);

	ifp->if_capabilities = ifp->if_capenable = 0;

	/* Enable checksum offload. */
	ifp->if_capabilities |= IFCAP_HWCSUM | IFCAP_VLAN_HWCSUM;
	ifp->if_capenable |= IFCAP_HWCSUM | IFCAP_VLAN_HWCSUM;

	/* Enable TCP segmentation offload. */
	ifp->if_capabilities |= IFCAP_TSO4;
	ifp->if_capenable |= IFCAP_TSO4;

	/* Enable VLAN support. */
	ifp->if_data.ifi_hdrlen = sizeof(struct ether_vlan_header);
	ifp->if_capabilities |= IFCAP_VLAN_HWTAGGING | IFCAP_VLAN_MTU;
	ifp->if_capenable |= IFCAP_VLAN_HWTAGGING | IFCAP_VLAN_MTU;

	/* Interface can filter VLAN tags. */
	ifp->if_capabilities |= IFCAP_VLAN_HWFILTER;

#ifdef DEVICE_POLLING
	ifp->if_capabilities |= IFCAP_POLLING;
#endif

	/* Enable Wake-on-LAN (WOL) via magic packet? */
	if (adapter->wol) {
		ifp->if_capabilities |= IFCAP_WOL;
		ifp->if_capenable |= IFCAP_WOL_MAGIC;
	}

	ifmedia_init(&adapter->media, IFM_IMASK, em_media_change,
	    em_media_status);

	if ((adapter->hw.phy.media_type == e1000_media_type_fiber) ||
	    (adapter->hw.phy.media_type == e1000_media_type_internal_serdes))
	{
		u_char fiber_type = IFM_1000_SX;

		ifmedia_add(&adapter->media,
		    IFM_ETHER | fiber_type, 0, NULL);
		ifmedia_add(&adapter->media,
		    IFM_ETHER | fiber_type | IFM_FDX, 0, NULL);
	} else {
		ifmedia_add(&adapter->media,
		    IFM_ETHER | IFM_10_T, 0, NULL);
		ifmedia_add(&adapter->media,
		    IFM_ETHER | IFM_10_T | IFM_FDX, 0, NULL);
		ifmedia_add(&adapter->media,
		    IFM_ETHER | IFM_100_TX, 0, NULL);
		ifmedia_add(&adapter->media,
		    IFM_ETHER | IFM_100_TX | IFM_FDX, 0, NULL);

		if (adapter->hw.phy.type != e1000_phy_ife) {
			ifmedia_add(&adapter->media,
			    IFM_ETHER | IFM_1000_T, 0, NULL);
			ifmedia_add(&adapter->media,
			    IFM_ETHER | IFM_1000_T | IFM_FDX, 0, NULL);
		}
	}

	ifmedia_add(&adapter->media, IFM_ETHER | IFM_AUTO, 0, NULL);
	ifmedia_set(&adapter->media, IFM_ETHER | IFM_AUTO);

	return (0);
}
