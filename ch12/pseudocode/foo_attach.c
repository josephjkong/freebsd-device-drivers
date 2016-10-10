/* $Id: foo_attach.c,v 1.4 2012/04/12 17:25:54 ghost Exp $ */

/*-
 * Copyright (c) 2012 Joseph Kong.
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
 * 3. Neither the name of the author nor the names of any co-contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
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
foo_attach(device_t dev)
{
	struct foo_softc *sc = device_get_softc(dev);
	int error;

	if (bus_dma_tag_create(bus_get_dma_tag(dev),	/* parent	*/
			       1,			/* alignment	*/
			       0,			/* boundary	*/
			       BUS_SPACE_MAXADDR,	/* lowaddr	*/
			       BUS_SPACE_MAXADDR,	/* highaddr	*/
			       NULL,			/* filter	*/
			       NULL,			/* filterarg	*/
			       BUS_SPACE_MAXSIZE_32BIT,	/* maxsize	*/
			       BUS_SPACE_UNRESTRICTED,	/* nsegments	*/
			       BUS_SPACE_MAXSIZE_32BIT,	/* maxsegsize	*/
			       0,			/* flags	*/
			       NULL,			/* lockfunc	*/
			       NULL,			/* lockfuncarg	*/
			       &sc->foo_parent_dma_tag)) {
		device_printf(dev, "Cannot allocate parent DMA tag!\n");
		return (ENOMEM);
	}

	if (bus_dma_tag_create(sc->foo_parent_dma_tag,	/* parent	*/
			       1,			/* alignment	*/
			       0,			/* boundary	*/
			       BUS_SPACE_MAXADDR,	/* lowaddr	*/
			       BUS_SPACE_MAXADDR,	/* highaddr	*/
			       NULL,			/* filter	*/
			       NULL,			/* filterarg	*/
			       MAX_BAZ_SIZE,		/* maxsize	*/
			       MAX_BAZ_SCATTER,		/* nsegments	*/
			       BUS_SPACE_MAXSIZE_32BIT,	/* maxsegsize	*/
			       0,			/* flags	*/
			       NULL,			/* lockfunc	*/
			       NULL,			/* lockfuncarg	*/
			       &sc->foo_baz_dma_tag)) {
		device_printf(dev, "Cannot allocate baz DMA tag!\n");
		return (ENOMEM);
	}

	if (bus_dmamap_create(sc->foo_baz_dma_tag,	/* DMA tag	*/
			      0,			/* flags	*/
			      &sc->foo_baz_dma_map)) {
		device_printf(dev, "Cannot allocate baz DMA map!\n");
		return (ENOMEM);
	}

	bzero(sc->foo_baz_buf, BAZ_BUF_SIZE);

	error = bus_dmamap_load(sc->foo_baz_dma_tag,	/* DMA tag	*/
				sc->foo_baz_dma_map,	/* DMA map	*/
				sc->foo_baz_buf,	/* buffer	*/
				BAZ_BUF_SIZE,		/* buffersize	*/
				foo_callback,		/* callback	*/
				&sc->foo_baz_busaddr,	/* callbackarg	*/
				BUS_DMA_NOWAIT);	/* flags	*/
	if (error || sc->foo_baz_busaddr == 0) {
		device_printf(dev, "Cannot map baz DMA memory!\n");
		return (ENOMEM);
	}

...
}
