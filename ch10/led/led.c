/* $Id: led.c,v 1.6 2012/04/12 17:03:52 ghost Exp $ */

/*-
 * Copyright (c) 2000 M. Warner Losh.
 * All rights reserved.
 *
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <imp@FreeBSD.ORG> wrote this file.  As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.   M. Warner Losh
 * ----------------------------------------------------------------------------
 */

#include <sys/param.h>
#include <sys/module.h>
#include <sys/kernel.h>
#include <sys/systm.h>

#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/uio.h>
#include <sys/lock.h>
#include <sys/mutex.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#define LED_IO_ADDR		0x404c
#define LED_NUM			2

struct led_softc {
	int			sc_io_rid;
	struct resource	       *sc_io_resource;
	struct cdev	       *sc_cdev0;
	struct cdev	       *sc_cdev1;
	u_int32_t		sc_open_mask;
	u_int32_t		sc_read_mask;
	struct mtx		sc_mutex;
};

static devclass_t led_devclass;

static d_open_t			led_open;
static d_close_t		led_close;
static d_read_t			led_read;
static d_write_t		led_write;

static struct cdevsw led_cdevsw = {
	.d_version =		D_VERSION,
	.d_open =		led_open,
	.d_close =		led_close,
	.d_read =		led_read,
	.d_write =		led_write,
	.d_name =		"led"
};

static int
led_open(struct cdev *dev, int oflags, int devtype, struct thread *td)
{
	int led = dev2unit(dev) & 0xff;
	struct led_softc *sc = dev->si_drv1;

	if (led >= LED_NUM)
		return (ENXIO);

	mtx_lock(&sc->sc_mutex);
	if (sc->sc_open_mask & (1 << led)) {
		mtx_unlock(&sc->sc_mutex);
		return (EBUSY);
	}
	sc->sc_open_mask |= 1 << led;
	sc->sc_read_mask |= 1 << led;
	mtx_unlock(&sc->sc_mutex);

	return (0);
}

static int
led_close(struct cdev *dev, int fflag, int devtype, struct thread *td)
{
	int led = dev2unit(dev) & 0xff;
	struct led_softc *sc = dev->si_drv1;

	if (led >= LED_NUM)
		return (ENXIO);

	mtx_lock(&sc->sc_mutex);
	sc->sc_open_mask &= ~(1 << led);
	mtx_unlock(&sc->sc_mutex);

	return (0);
}

static int
led_read(struct cdev *dev, struct uio *uio, int ioflag)
{
	int led = dev2unit(dev) & 0xff;
	struct led_softc *sc = dev->si_drv1;
	u_int8_t ch;
	int error;

	if (led >= LED_NUM)
		return (ENXIO);

	mtx_lock(&sc->sc_mutex);
	/* No error EOF condition. */
	if (!(sc->sc_read_mask & (1 << led))) {
		mtx_unlock(&sc->sc_mutex);
		return (0);
	}
	sc->sc_read_mask &= ~(1 << led);
	mtx_unlock(&sc->sc_mutex);

	ch = bus_read_1(sc->sc_io_resource, 0);
	if (ch & (1 << led))
		ch = '1';
	else
		ch = '0';

	error = uiomove(&ch, 1, uio);
	return (error);
}

static int
led_write(struct cdev *dev, struct uio *uio, int ioflag)
{
	int led = dev2unit(dev) & 0xff;
	struct led_softc *sc = dev->si_drv1;
	u_int8_t ch;
	u_int8_t old;
	int error;

	if (led >= LED_NUM)
		return (ENXIO);

	error = uiomove(&ch, 1, uio);
	if (error)
		return (error);

	old = bus_read_1(sc->sc_io_resource, 0);
	if (ch & 1)
		old |= (1 << led);
	else
		old &= ~(1 << led);

	bus_write_1(sc->sc_io_resource, 0, old);

	return (error);
}

static void
led_identify(driver_t *driver, device_t parent)
{
	device_t child;

	child = device_find_child(parent, "led", -1);
	if (!child) {
		child = BUS_ADD_CHILD(parent, 0, "led", -1);
		bus_set_resource(child, SYS_RES_IOPORT, 0, LED_IO_ADDR, 1);
	}
}

static int
led_probe(device_t dev)
{
	if (!bus_get_resource_start(dev, SYS_RES_IOPORT, 0))
		return (ENXIO);

	device_set_desc(dev, "I/O Port Example");
	return (BUS_PROBE_SPECIFIC);
}

static int
led_attach(device_t dev)
{
	struct led_softc *sc = device_get_softc(dev);

	sc->sc_io_rid = 0;
	sc->sc_io_resource = bus_alloc_resource_any(dev, SYS_RES_IOPORT,
	    &sc->sc_io_rid, RF_ACTIVE);
	if (!sc->sc_io_resource) {
		device_printf(dev, "unable to allocate resource\n");
		return (ENXIO);
	}

	sc->sc_open_mask = 0;
	sc->sc_read_mask = 0;
	mtx_init(&sc->sc_mutex, "led", NULL, MTX_DEF);

	sc->sc_cdev0 = make_dev(&led_cdevsw, 0, UID_ROOT, GID_WHEEL, 0644,
	    "led0");
	sc->sc_cdev1 = make_dev(&led_cdevsw, 1, UID_ROOT, GID_WHEEL, 0644,
	    "led1");
	sc->sc_cdev0->si_drv1 = sc;
	sc->sc_cdev1->si_drv1 = sc;

	return (0);
}

static int
led_detach(device_t dev)
{
	struct led_softc *sc = device_get_softc(dev);

	destroy_dev(sc->sc_cdev0);
	destroy_dev(sc->sc_cdev1);

	mtx_destroy(&sc->sc_mutex);

	bus_release_resource(dev, SYS_RES_IOPORT, sc->sc_io_rid,
	    sc->sc_io_resource);

	return (0);
}

static device_method_t led_methods[] = {
	/* Device interface. */
	DEVMETHOD(device_identify,	led_identify),
	DEVMETHOD(device_probe,		led_probe),
	DEVMETHOD(device_attach,	led_attach),
	DEVMETHOD(device_detach,	led_detach),
	{ 0, 0 }
};

static driver_t led_driver = {
	"led",
	led_methods,
	sizeof(struct led_softc)
};

DRIVER_MODULE(led, isa, led_driver, led_devclass, 0, 0);
