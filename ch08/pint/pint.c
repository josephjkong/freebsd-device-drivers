/* $Id: pint.c,v 1.7 2012/04/12 16:06:18 ghost Exp $ */

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
#include <sys/malloc.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/ppbus/ppbconf.h>
#include "ppbus_if.h"
#include <dev/ppbus/ppbio.h>

#define PINT_NAME		"pint"
#define BUFFER_SIZE		256

struct pint_data {
	int			sc_irq_rid;
	struct resource	       *sc_irq_resource;
	void		       *sc_irq_cookie;
	device_t		sc_device;
	struct cdev	       *sc_cdev;
	short			sc_state;
#define PINT_OPEN		0x01
	char		       *sc_buffer;
	int			sc_length;
};

static d_open_t			pint_open;
static d_close_t		pint_close;
static d_read_t			pint_read;
static d_write_t		pint_write;

static struct cdevsw pint_cdevsw = {
	.d_version =		D_VERSION,
	.d_open =		pint_open,
	.d_close =		pint_close,
	.d_read =		pint_read,
	.d_write =		pint_write,
	.d_name =		PINT_NAME
};

static devclass_t pint_devclass;

static int
pint_open(struct cdev *dev, int oflags, int devtype, struct thread *td)
{
	struct pint_data *sc = dev->si_drv1;
	device_t pint_device = sc->sc_device;
	device_t ppbus = device_get_parent(pint_device);
	int error;

	ppb_lock(ppbus);

	if (sc->sc_state) {
		ppb_unlock(ppbus);
		return (EBUSY);
	} else
		sc->sc_state |= PINT_OPEN;

	error = ppb_request_bus(ppbus, pint_device, PPB_WAIT | PPB_INTR);
	if (error) {
		sc->sc_state = 0;
		ppb_unlock(ppbus);
		return (error);
	}

	ppb_wctr(ppbus, 0);
	ppb_wctr(ppbus, IRQENABLE);

	ppb_unlock(ppbus);
	return (0);
}

static int
pint_close(struct cdev *dev, int fflag, int devtype, struct thread *td)
{
	struct pint_data *sc = dev->si_drv1;
	device_t pint_device = sc->sc_device;
	device_t ppbus = device_get_parent(pint_device);

	ppb_lock(ppbus);

	ppb_wctr(ppbus, 0);
	ppb_release_bus(ppbus, pint_device);
	sc->sc_state = 0;

	ppb_unlock(ppbus);
	return (0);
}

static int
pint_write(struct cdev *dev, struct uio *uio, int ioflag)
{
	struct pint_data *sc = dev->si_drv1;
	device_t pint_device = sc->sc_device;
	int amount, error = 0;

	amount = MIN(uio->uio_resid,
	    (BUFFER_SIZE - 1 - uio->uio_offset > 0) ?
	     BUFFER_SIZE - 1 - uio->uio_offset : 0);
	if (amount == 0)
		return (error);

	error = uiomove(sc->sc_buffer, amount, uio);
	if (error) {
		device_printf(pint_device, "write failed\n");
		return (error);
	}

	sc->sc_buffer[amount] = '\0';
	sc->sc_length = amount;

	return (error);
}

static int
pint_read(struct cdev *dev, struct uio *uio, int ioflag)
{
	struct pint_data *sc = dev->si_drv1;
	device_t pint_device = sc->sc_device;
	device_t ppbus = device_get_parent(pint_device);
	int amount, error = 0;

	ppb_lock(ppbus);
	error = ppb_sleep(ppbus, pint_device, PPBPRI | PCATCH, PINT_NAME, 0);
	ppb_unlock(ppbus);
	if (error)
		return (error);

	amount = MIN(uio->uio_resid,
	    (sc->sc_length - uio->uio_offset > 0) ?
	     sc->sc_length - uio->uio_offset : 0);

	error = uiomove(sc->sc_buffer + uio->uio_offset, amount, uio);
	if (error)
		device_printf(pint_device, "read failed\n");

	return (error);
}

static void
pint_intr(void *arg)
{
	struct pint_data *sc = arg;
	device_t pint_device = sc->sc_device;

#ifdef INVARIANTS
	device_t ppbus = device_get_parent(pint_device);
	ppb_assert_locked(ppbus);
#endif

	wakeup(pint_device);
}

static void
pint_identify(driver_t *driver, device_t parent)
{
	device_t dev;

	dev = device_find_child(parent, PINT_NAME, -1);
	if (!dev)
		BUS_ADD_CHILD(parent, 0, PINT_NAME, -1);
}

static int
pint_probe(device_t dev)
{
	/* probe() is always OK. */
	device_set_desc(dev, "Interrupt Handler Example");

	return (BUS_PROBE_SPECIFIC);
}

static int
pint_attach(device_t dev)
{
	struct pint_data *sc = device_get_softc(dev);
	int error, unit = device_get_unit(dev);

	/* Declare our interrupt handler. */
	sc->sc_irq_rid = 0;
	sc->sc_irq_resource = bus_alloc_resource_any(dev, SYS_RES_IRQ,
	    &sc->sc_irq_rid, RF_ACTIVE | RF_SHAREABLE);

	/* Interrupts are mandatory. */
	if (!sc->sc_irq_resource) {
		device_printf(dev,
		    "unable to allocate interrupt resource\n");
		return (ENXIO);
	}

	/* Register our interrupt handler. */
	error = bus_setup_intr(dev, sc->sc_irq_resource,
	    INTR_TYPE_TTY | INTR_MPSAFE, NULL, pint_intr,
	    sc, &sc->sc_irq_cookie);
	if (error) {
		bus_release_resource(dev, SYS_RES_IRQ, sc->sc_irq_rid,
		    sc->sc_irq_resource);
		device_printf(dev, "unable to register interrupt handler\n");
		return (error);
	}

	sc->sc_buffer = malloc(BUFFER_SIZE, M_DEVBUF, M_WAITOK);

	sc->sc_device = dev;
	sc->sc_cdev = make_dev(&pint_cdevsw, unit, UID_ROOT, GID_WHEEL, 0600,
	    PINT_NAME "%d", unit);
	sc->sc_cdev->si_drv1 = sc;

	return (0);
}

static int
pint_detach(device_t dev)
{
	struct pint_data *sc = device_get_softc(dev);

	destroy_dev(sc->sc_cdev);

	bus_teardown_intr(dev, sc->sc_irq_resource, sc->sc_irq_cookie);
	bus_release_resource(dev, SYS_RES_IRQ, sc->sc_irq_rid,
	    sc->sc_irq_resource);

	free(sc->sc_buffer, M_DEVBUF);

	return (0);
}

static device_method_t pint_methods[] = {
	/* Device interface. */
	DEVMETHOD(device_identify,	pint_identify),
	DEVMETHOD(device_probe,		pint_probe),
	DEVMETHOD(device_attach,	pint_attach),
	DEVMETHOD(device_detach,	pint_detach),
	{ 0, 0 }
};

static driver_t pint_driver = {
	PINT_NAME,
	pint_methods,
	sizeof(struct pint_data)
};

DRIVER_MODULE(pint, ppbus, pint_driver, pint_devclass, 0, 0);
MODULE_DEPEND(pint, ppbus, 1, 1, 1);
