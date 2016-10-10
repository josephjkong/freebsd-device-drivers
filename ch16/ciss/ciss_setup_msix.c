/* $Id: ciss_setup_msix.c,v 1.3 2012/04/12 19:05:31 ghost Exp $ */

/*-
 * Copyright (c) 2001 Michael Smith.
 * Copyright (c) 2004 Paul Saab.
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
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

static int
ciss_setup_msix(struct ciss_softc *sc)
{
	int i, count, error;

	i = ciss_lookup(sc->ciss_dev);
	if (ciss_vendor_data[i].flags & CISS_BOARD_NOMSI)
		return (EINVAL);

	count = pci_msix_count(sc->ciss_dev);
	if (count < CISS_MSI_COUNT) {
		count = pci_msi_count(sc->ciss_dev);
		if (count < CISS_MSI_COUNT)
			return (EINVAL);
	}

	count = MIN(count, CISS_MSI_COUNT);
	error = pci_alloc_msix(sc->ciss_dev, &count);
	if (error) {
		error = pci_alloc_msi(sc->ciss_dev, &count);
		if (error)
			return (EINVAL);
	}

	sc->ciss_msi = count;
	for (i = 0; i < count; i++)
		sc->ciss_irq_rid[i] = i + 1;

	return (0);
}
