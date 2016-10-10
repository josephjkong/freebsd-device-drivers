/* $Id: foo_pci.c,v 1.4 2012/04/12 16:03:59 ghost Exp $ */

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

#include <sys/param.h>
#include <sys/module.h>
#include <sys/kernel.h>
#include <sys/systm.h>

#include <sys/conf.h>
#include <sys/uio.h>
#include <sys/bus.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

struct foo_pci_softc {
	device_t	device;
	struct cdev	*cdev;
};

static d_open_t		foo_pci_open;
static d_close_t	foo_pci_close;
static d_read_t		foo_pci_read;
static d_write_t	foo_pci_write;

static struct cdevsw foo_pci_cdevsw = {
	.d_version =	D_VERSION,
	.d_open =	foo_pci_open,
	.d_close =	foo_pci_close,
	.d_read =	foo_pci_read,
	.d_write =	foo_pci_write,
	.d_name =	"foo_pci"
};

static devclass_t foo_pci_devclass;

static int
foo_pci_open(struct cdev *dev, int oflags, int devtype, struct thread *td)
{
	struct foo_pci_softc *sc;

	sc = dev->si_drv1;
	device_printf(sc->device, "opened successfully\n");
	return (0);
}

static int
foo_pci_close(struct cdev *dev, int fflag, int devtype, struct thread *td)
{
	struct foo_pci_softc *sc;

	sc = dev->si_drv1;
	device_printf(sc->device, "closed\n");
	return (0);
}

static int
foo_pci_read(struct cdev *dev, struct uio *uio, int ioflag)
{
	struct foo_pci_softc *sc;

	sc = dev->si_drv1;
	device_printf(sc->device, "read request = %dB\n", uio->uio_resid);
	return (0);
}

static int
foo_pci_write(struct cdev *dev, struct uio *uio, int ioflag)
{
	struct foo_pci_softc *sc;

	sc = dev->si_drv1;
	device_printf(sc->device, "write request = %dB\n", uio->uio_resid);
	return (0);
}

static struct _pcsid {
	uint32_t	type;
	const char	*desc;
} pci_ids[] = {
	{ 0x1234abcd, "RED PCI Widget" },
	{ 0x4321fedc, "BLU PCI Widget" },
	{ 0x00000000, NULL }
};

static int
foo_pci_probe(device_t dev)
{
	uint32_t type = pci_get_devid(dev);
	struct _pcsid *ep = pci_ids;

	while (ep->type && ep->type != type)
		ep++;
	if (ep->desc) {
		device_set_desc(dev, ep->desc);
		return (BUS_PROBE_DEFAULT);
	}

	return (ENXIO);
}

static int
foo_pci_attach(device_t dev)
{
	struct foo_pci_softc *sc = device_get_softc(dev);
	int unit = device_get_unit(dev);

	sc->device = dev;
	sc->cdev = make_dev(&foo_pci_cdevsw, unit, UID_ROOT, GID_WHEEL,
	    0600, "foo_pci%d", unit);
	sc->cdev->si_drv1 = sc;

	return (0);
}

static int
foo_pci_detach(device_t dev)
{
	struct foo_pci_softc *sc = device_get_softc(dev);

	destroy_dev(sc->cdev);
	return (0);
}

static device_method_t foo_pci_methods[] = {
	/* Device interface. */
	DEVMETHOD(device_probe,		foo_pci_probe),
	DEVMETHOD(device_attach,	foo_pci_attach),
	DEVMETHOD(device_detach,	foo_pci_detach),
	{ 0, 0 }
};

static driver_t foo_pci_driver = {
	"foo_pci",
	foo_pci_methods,
	sizeof(struct foo_pci_softc)
};

DRIVER_MODULE(foo_pci, pci, foo_pci_driver, foo_pci_devclass, 0, 0);
