/* $Id: lpt.c,v 1.21 2012/04/12 16:34:38 ghost Exp $ */

/*-
 * Copyright (c) 1990 William F. Jolitz, TeleMuse
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
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This software is a component of "386BSD" developed by
 *	William F. Jolitz, TeleMuse.
 * 4. Neither the name of the developer nor the name "386BSD"
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS A COMPONENT OF 386BSD DEVELOPED BY WILLIAM F. JOLITZ
 * AND IS INTENDED FOR RESEARCH AND EDUCATIONAL PURPOSES ONLY. THIS
 * SOFTWARE SHOULD NOT BE CONSIDERED TO BE A COMMERCIAL PRODUCT.
 * THE DEVELOPER URGES THAT USERS WHO REQUIRE A COMMERCIAL PRODUCT
 * NOT MAKE USE OF THIS WORK.
 *
 * FOR USERS WHO WISH TO UNDERSTAND THE 386BSD SYSTEM DEVELOPED
 * BY WILLIAM F. JOLITZ, WE RECOMMEND THE USER STUDY WRITTEN
 * REFERENCES SUCH AS THE  "PORTING UNIX TO THE 386" SERIES
 * (BEGINNING JANUARY 1991 "DR. DOBBS JOURNAL", USA AND BEGINNING
 * JUNE 1991 "UNIX MAGAZIN", GERMANY) BY WILLIAM F. JOLITZ AND
 * LYNNE GREER JOLITZ, AS WELL AS OTHER BOOKS ON UNIX AND THE
 * ON-LINE 386BSD USER MANUAL BEFORE USE. A BOOK DISCUSSING THE INTERNALS
 * OF 386BSD ENTITLED "386BSD FROM THE INSIDE OUT" WILL BE AVAILABLE LATE
 * 1992.
 *
 * THIS SOFTWARE IS PROVIDED BY THE DEVELOPER ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE DEVELOPER BE LIABLE
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
#include <sys/syslog.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/ppbus/ppbconf.h>
#include "ppbus_if.h"
#include <dev/ppbus/ppbio.h>
#include <dev/ppbus/ppb_1284.h>

#include <dev/ppbus/lpt.h>
#include <dev/ppbus/lptio.h>

#define LPT_NAME	"lpt"		/* official driver name.	*/
#define LPT_INIT_READY	4		/* wait up to 4 seconds.	*/
#define LPT_PRI		(PZERO + 8)	/* priority.			*/
#define BUF_SIZE	1024		/* sc_buf size.			*/
#define BUF_STAT_SIZE	32		/* sc_buf_stat size.		*/

struct lpt_data {
	short			sc_state;
	char			sc_primed;
	struct callout		sc_callout;
	u_char			sc_ticks;
	int			sc_irq_rid;
	struct resource	       *sc_irq_resource;
	void		       *sc_irq_cookie;
	u_short			sc_irq_status;
	void		       *sc_buf;
	void		       *sc_buf_stat;
	char		       *sc_cp;
	device_t		sc_dev;
	struct cdev	       *sc_cdev;
	struct cdev	       *sc_cdev_bypass;
	char			sc_flags;
	u_char			sc_control;
	short			sc_transfer_count;
};

/* bits for sc_state. */
#define LP_OPEN		(1 << 0)	/* device is open.		*/
#define LP_ERROR	(1 << 2)	/* error received from printer.	*/
#define LP_BUSY		(1 << 3)	/* printer is busy writing.	*/
#define LP_TIMEOUT	(1 << 5)	/* timeout enabled.		*/
#define LP_INIT		(1 << 6)	/* initializing in lpt_open.	*/
#define LP_INTERRUPTED	(1 << 7)	/* write call was interrupted.	*/
#define LP_HAVEBUS	(1 << 8)	/* driver owns the bus.		*/

/* bits for sc_ticks. */
#define LP_TOUT_INIT	10		/* initial timeout: 1/10 sec.	*/
#define LP_TOUT_MAX	1		/* max timeout: 1/1 sec.	*/

/* bits for sc_irq_status. */
#define LP_HAS_IRQ	0x01		/* we have an IRQ available.	*/
#define LP_USE_IRQ	0x02		/* our IRQ is in use.		*/
#define LP_ENABLE_IRQ	0x04		/* enable our IRQ on open.	*/
#define LP_ENABLE_EXT	0x10		/* enable extended mode.	*/

/* bits for sc_flags. */
#define LP_NO_PRIME	0x10		/* don't prime the printer.	*/
#define LP_PRIME_OPEN	0x20		/* prime on every open.		*/
#define LP_AUTO_LF	0x40		/* automatic line feed.		*/
#define LP_BYPASS	0x80		/* bypass printer ready checks.	*/

/* masks to interrogate printer status. */
#define LP_READY_MASK	(LPS_NERR | LPS_SEL | LPS_OUT | LPS_NBSY)
#define LP_READY	(LPS_NERR | LPS_SEL |           LPS_NBSY)

/* used in polling code. */
#define LPS_INVERT	(LPS_NERR | LPS_SEL |           LPS_NACK | LPS_NBSY)
#define LPS_MASK	(LPS_NERR | LPS_SEL | LPS_OUT | LPS_NACK | LPS_NBSY)
#define NOT_READY(bus)	((ppb_rstr(bus) ^ LPS_INVERT) & LPS_MASK)
#define MAX_SPIN	20		/* wait up to 20 usec.		*/
#define MAX_SLEEP	(hz * 5)	/* timeout while waiting.	*/

static d_open_t			lpt_open;
static d_close_t		lpt_close;
static d_read_t			lpt_read;
static d_write_t		lpt_write;
static d_ioctl_t		lpt_ioctl;

static struct cdevsw lpt_cdevsw = {
	.d_version =		D_VERSION,
	.d_open =		lpt_open,
	.d_close =		lpt_close,
	.d_read =		lpt_read,
	.d_write =		lpt_write,
	.d_ioctl =		lpt_ioctl,
	.d_name =		LPT_NAME
};

static devclass_t lpt_devclass;

static void
lpt_identify(driver_t *driver, device_t parent)
{
	device_t dev;

	dev = device_find_child(parent, LPT_NAME, -1);
	if (!dev)
		BUS_ADD_CHILD(parent, 0, LPT_NAME, -1);
}

static int
lpt_request_ppbus(device_t dev, int how)
{
	device_t ppbus = device_get_parent(dev);
	struct lpt_data *sc = device_get_softc(dev);
	int error;

	ppb_assert_locked(ppbus);

	if (sc->sc_state & LP_HAVEBUS)
		return (0);

	error = ppb_request_bus(ppbus, dev, how);
	if (!error)
		sc->sc_state |= LP_HAVEBUS;

	return (error);
}

static int
lpt_release_ppbus(device_t dev)
{
	device_t ppbus = device_get_parent(dev);
	struct lpt_data *sc = device_get_softc(dev);
	int error = 0;

	ppb_assert_locked(ppbus);

	if (sc->sc_state & LP_HAVEBUS) {
		error = ppb_release_bus(ppbus, dev);
		if (!error)
			sc->sc_state &= ~LP_HAVEBUS;
	}

	return (error);
}

static int
lpt_port_test(device_t ppbus, u_char data, u_char mask)
{
	int temp, timeout = 10000;

	data &= mask;
	ppb_wdtr(ppbus, data);

	do {
		DELAY(10);
		temp = ppb_rdtr(ppbus) & mask;
	} while (temp != data && --timeout);

	return (temp == data);
}

static int
lpt_detect(device_t dev)
{
	device_t ppbus = device_get_parent(dev);
	static u_char test[18] = {
		0x55,			/* alternating zeros.	*/
		0xaa,			/* alternating ones.	*/
		0xfe, 0xfd, 0xfb, 0xf7,
		0xef, 0xdf, 0xbf, 0x7f,	/* walking zero.	*/
		0x01, 0x02, 0x04, 0x08,
		0x10, 0x20, 0x40, 0x80	/* walking one.		*/
	};
	int i, error, success = 1;	/* assume success.	*/

	ppb_lock(ppbus);

	error = lpt_request_ppbus(dev, PPB_DONTWAIT);
	if (error) {
		ppb_unlock(ppbus);
		device_printf(dev, "cannot allocate ppbus (%d)!\n", error);
		return (0);
	}

	for (i = 0; i < 18; i++)
		if (!lpt_port_test(ppbus, test[i], 0xff)) {
			success = 0;
			break;
		}

	ppb_wdtr(ppbus, 0);
	ppb_wctr(ppbus, 0);

	lpt_release_ppbus(dev);
	ppb_unlock(ppbus);

	return (success);
}

static int
lpt_probe(device_t dev)
{
	if (!lpt_detect(dev))
		return (ENXIO);

	device_set_desc(dev, "Printer");

	return (BUS_PROBE_SPECIFIC);
}

static void
lpt_intr(void *arg)
{
	struct lpt_data *sc = arg;
	device_t lpt_dev = sc->sc_dev;
	device_t ppbus = device_get_parent(lpt_dev);
	int i, status = 0;

	for (i = 0; i < 100 &&
	     ((status = ppb_rstr(ppbus)) & LP_READY_MASK) != LP_READY; i++)
		;	/* nothing. */

	if ((status & LP_READY_MASK) == LP_READY) {
		sc->sc_state = (sc->sc_state | LP_BUSY) & ~LP_ERROR;
		sc->sc_ticks = hz / LP_TOUT_INIT;

		if (sc->sc_transfer_count) {
			ppb_wdtr(ppbus, *sc->sc_cp++);
			ppb_wctr(ppbus, sc->sc_control | LPC_STB);
			ppb_wctr(ppbus, sc->sc_control);

			if (--(sc->sc_transfer_count) > 0)
				return;
		}

		sc->sc_state &= ~LP_BUSY;

		if (!(sc->sc_state & LP_INTERRUPTED))
			wakeup(lpt_dev);

		return;
	} else {
		if (((status & (LPS_NERR | LPS_OUT)) != LPS_NERR) &&
		    (sc->sc_state & LP_OPEN))
			sc->sc_state |= LP_ERROR;
	}
}

static int
lpt_attach(device_t dev)
{
	device_t ppbus = device_get_parent(dev);
	struct lpt_data *sc = device_get_softc(dev);
	int error, unit = device_get_unit(dev);

	sc->sc_primed = 0;
	ppb_init_callout(ppbus, &sc->sc_callout, 0);

	ppb_lock(ppbus);
	error = lpt_request_ppbus(dev, PPB_DONTWAIT);
	if (error) {
		ppb_unlock(ppbus);
		device_printf(dev, "cannot allocate ppbus (%d)!\n", error);
		return (0);
	}

	ppb_wctr(ppbus, LPC_NINIT);

	lpt_release_ppbus(dev);
	ppb_unlock(ppbus);

	/* Declare our interrupt handler. */
	sc->sc_irq_rid = 0;
	sc->sc_irq_resource = bus_alloc_resource_any(dev, SYS_RES_IRQ,
	    &sc->sc_irq_rid, RF_ACTIVE | RF_SHAREABLE);

	/* Register our interrupt handler. */
	if (sc->sc_irq_resource) {
		error = bus_setup_intr(dev, sc->sc_irq_resource,
		    INTR_TYPE_TTY | INTR_MPSAFE, NULL, lpt_intr,
		    sc, &sc->sc_irq_cookie);
		if (error) {
			bus_release_resource(dev, SYS_RES_IRQ, sc->sc_irq_rid,
			    sc->sc_irq_resource);
			device_printf(dev,
			    "unable to register interrupt handler\n");
			return (error);
		}

		sc->sc_irq_status = LP_HAS_IRQ | LP_USE_IRQ | LP_ENABLE_IRQ;
		device_printf(dev, "interrupt-driven port\n");
	} else {
		sc->sc_irq_status = 0;
		device_printf(dev, "polled port\n");
	}

	sc->sc_buf = malloc(BUF_SIZE, M_DEVBUF, M_WAITOK);
	sc->sc_buf_stat = malloc(BUF_STAT_SIZE, M_DEVBUF, M_WAITOK);

	sc->sc_dev = dev;

	sc->sc_cdev = make_dev(&lpt_cdevsw, unit, UID_ROOT, GID_WHEEL, 0600,
	    LPT_NAME "%d", unit);
	sc->sc_cdev->si_drv1 = sc;
	sc->sc_cdev->si_drv2 = 0;

	sc->sc_cdev_bypass = make_dev(&lpt_cdevsw, unit, UID_ROOT, GID_WHEEL,
	    0600, LPT_NAME "%d.ctl", unit);
	sc->sc_cdev_bypass->si_drv1 = sc;
	sc->sc_cdev_bypass->si_drv2 = (void *)LP_BYPASS;

	return (0);
}

static int
lpt_detach(device_t dev)
{
	device_t ppbus = device_get_parent(dev);
	struct lpt_data *sc = device_get_softc(dev);

	destroy_dev(sc->sc_cdev_bypass);
	destroy_dev(sc->sc_cdev);

	ppb_lock(ppbus);
	lpt_release_ppbus(dev);
	ppb_unlock(ppbus);

	callout_drain(&sc->sc_callout);

	if (sc->sc_irq_resource) {
		bus_teardown_intr(dev, sc->sc_irq_resource,
		    sc->sc_irq_cookie);
		bus_release_resource(dev, SYS_RES_IRQ, sc->sc_irq_rid,
		    sc->sc_irq_resource);
	}

	free(sc->sc_buf_stat, M_DEVBUF);
	free(sc->sc_buf, M_DEVBUF);

	return (0);
}

static void
lpt_timeout(void *arg)
{
	struct lpt_data *sc = arg;
	device_t lpt_dev = sc->sc_dev;

	if (sc->sc_state & LP_OPEN) {
		sc->sc_ticks++;
		if (sc->sc_ticks > hz / LP_TOUT_MAX)
			sc->sc_ticks = hz / LP_TOUT_MAX;
		callout_reset(&sc->sc_callout, sc->sc_ticks,
		    lpt_timeout, sc);
	} else
		sc->sc_state &= ~LP_TIMEOUT;

	if (sc->sc_state & LP_ERROR)
		sc->sc_state &= ~LP_ERROR;

	if (sc->sc_transfer_count)
		lpt_intr(sc);
	else {
		sc->sc_state &= ~LP_BUSY;
		wakeup(lpt_dev);
	}
}

static int
lpt_open(struct cdev *dev, int oflags, int devtype, struct thread *td)
{
	struct lpt_data *sc = dev->si_drv1;
	device_t lpt_dev = sc->sc_dev;
	device_t ppbus = device_get_parent(lpt_dev);
	int try, error;

	if (!sc)
		return (ENXIO);

	ppb_lock(ppbus);
	if (sc->sc_state) {
		ppb_unlock(ppbus);
		return (EBUSY);
	} else
		sc->sc_state |= LP_INIT;

	sc->sc_flags = (uintptr_t)dev->si_drv2;
	if (sc->sc_flags & LP_BYPASS) {
		sc->sc_state = LP_OPEN;
		ppb_unlock(ppbus);
		return (0);
	}

	error = lpt_request_ppbus(lpt_dev, PPB_WAIT | PPB_INTR);
	if (error) {
		sc->sc_state = 0;
		ppb_unlock(ppbus);
		return (error);
	}

	/* Use our IRQ? */
	if (sc->sc_irq_status & LP_ENABLE_IRQ)
		sc->sc_irq_status |= LP_USE_IRQ;
	else
		sc->sc_irq_status &= ~LP_USE_IRQ;

	/* Reset printer. */
	if ((sc->sc_flags & LP_NO_PRIME) == 0)
		if ((sc->sc_flags & LP_PRIME_OPEN) || sc->sc_primed == 0) {
			ppb_wctr(ppbus, 0);
			sc->sc_primed++;
			DELAY(500);
		}

	ppb_wctr(ppbus, LPC_SEL | LPC_NINIT);

	/* Wait until ready--printer should be running diagnostics. */
	try = 0;
	do {
		/* Give up? */
		if (try++ >= (LPT_INIT_READY * 4)) {
			lpt_release_ppbus(lpt_dev);
			sc->sc_state = 0;
			ppb_unlock(ppbus);
			return (EBUSY);
		}

		/* Wait 1/4 second. Give up if we get a signal. */
		if (ppb_sleep(ppbus, lpt_dev, LPT_PRI | PCATCH, "lpt_open",
		    hz / 4) != EWOULDBLOCK) {
			lpt_release_ppbus(lpt_dev);
			sc->sc_state = 0;
			ppb_unlock(ppbus);
			return (EBUSY);
		}
	} while ((ppb_rstr(ppbus) & LP_READY_MASK) != LP_READY);

	sc->sc_control = LPC_SEL | LPC_NINIT;
	if (sc->sc_flags & LP_AUTO_LF)
		sc->sc_control |= LPC_AUTOL;
	if (sc->sc_irq_status & LP_USE_IRQ)
		sc->sc_control |= LPC_ENA;

	ppb_wctr(ppbus, sc->sc_control);

	sc->sc_state &= ~LP_INIT;
	sc->sc_state |= LP_OPEN;
	sc->sc_transfer_count = 0;

	if (sc->sc_irq_status & LP_USE_IRQ) {
		sc->sc_state |= LP_TIMEOUT;
		sc->sc_ticks = hz / LP_TOUT_INIT;
		callout_reset(&sc->sc_callout, sc->sc_ticks,
		    lpt_timeout, sc);
	}

	lpt_release_ppbus(lpt_dev);
	ppb_unlock(ppbus);

	return (0);
}

static int
lpt_close(struct cdev *dev, int fflag, int devtype, struct thread *td)
{
	struct lpt_data *sc = dev->si_drv1;
	device_t lpt_dev = sc->sc_dev;
	device_t ppbus = device_get_parent(lpt_dev);
	int error;

	ppb_lock(ppbus);

	if (sc->sc_flags & LP_BYPASS)
		goto end_close;

	error = lpt_request_ppbus(lpt_dev, PPB_WAIT | PPB_INTR);
	if (error) {
		ppb_unlock(ppbus);
		return (error);
	}

	if (!(sc->sc_state & LP_INTERRUPTED) &&
	     (sc->sc_irq_status & LP_USE_IRQ))
		while ((ppb_rstr(ppbus) & LP_READY_MASK) != LP_READY ||
		    sc->sc_transfer_count)
			if (ppb_sleep(ppbus, lpt_dev, LPT_PRI | PCATCH,
			    "lpt_close", hz) != EWOULDBLOCK)
				break;

	sc->sc_state &= ~LP_OPEN;
	callout_stop(&sc->sc_callout);
	ppb_wctr(ppbus, LPC_NINIT);

	lpt_release_ppbus(lpt_dev);

end_close:
	sc->sc_state = 0;
	sc->sc_transfer_count = 0;
	ppb_unlock(ppbus);
	return (0);
}

static int
lpt_read(struct cdev *dev, struct uio *uio, int ioflag)
{
	struct lpt_data *sc = dev->si_drv1;
	device_t lpt_dev = sc->sc_dev;
	device_t ppbus = device_get_parent(lpt_dev);
	int num, error = 0;

	if (sc->sc_flags & LP_BYPASS)
		return (EPERM);

	ppb_lock(ppbus);
	error = ppb_1284_negociate(ppbus, PPB_NIBBLE, 0);
	if (error) {
		ppb_unlock(ppbus);
		return (error);
	}

	num = 0;
	while (uio->uio_resid) {
		error = ppb_1284_read(ppbus, PPB_NIBBLE, sc->sc_buf_stat,
		    min(BUF_STAT_SIZE, uio->uio_resid), &num);
		if (error)
			goto end_read;

		if (!num)
			goto end_read;

		ppb_unlock(ppbus);
		error = uiomove(sc->sc_buf_stat, num, uio);
		ppb_lock(ppbus);
		if (error)
			goto end_read;
	}

end_read:
	ppb_1284_terminate(ppbus);
	ppb_unlock(ppbus);
	return (error);
}

static int
lpt_push_bytes(struct lpt_data *sc)
{
	device_t lpt_dev = sc->sc_dev;
	device_t ppbus = device_get_parent(lpt_dev);
	int error, spin, tick;
	char ch;

	while (sc->sc_transfer_count > 0) {
		ch = *sc->sc_cp;
		sc->sc_cp++;
		sc->sc_transfer_count--;

		for (spin = 0; NOT_READY(ppbus) && spin < MAX_SPIN; spin++)
			DELAY(1);

		if (spin >= MAX_SPIN) {
			tick = 0;
			while (NOT_READY(ppbus)) {
				tick = tick + tick + 1;
				if (tick > MAX_SLEEP)
					tick = MAX_SLEEP;

				error = ppb_sleep(ppbus, lpt_dev, LPT_PRI,
				    "lpt_poll", tick);
				if (error != EWOULDBLOCK)
					return (error);
			}
		}

		ppb_wdtr(ppbus, ch);
		ppb_wctr(ppbus, sc->sc_control | LPC_STB);
		ppb_wctr(ppbus, sc->sc_control);
	}

	return (0);
}

static int
lpt_write(struct cdev *dev, struct uio *uio, int ioflag)
{
	struct lpt_data *sc = dev->si_drv1;
	device_t lpt_dev = sc->sc_dev;
	device_t ppbus = device_get_parent(lpt_dev);
	register unsigned num;
	int error;

	if (sc->sc_flags & LP_BYPASS)
		return (EPERM);

	ppb_lock(ppbus);
	error = lpt_request_ppbus(lpt_dev, PPB_WAIT | PPB_INTR);
	if (error) {
		ppb_unlock(ppbus);
		return (error);
	}

	sc->sc_state &= ~LP_INTERRUPTED;
	while ((num = min(BUF_SIZE, uio->uio_resid))) {
		sc->sc_cp = sc->sc_buf;

		ppb_unlock(ppbus);
		error = uiomove(sc->sc_cp, num, uio);
		ppb_lock(ppbus);
		if (error)
			break;

		sc->sc_transfer_count = num;

		if (sc->sc_irq_status & LP_ENABLE_EXT) {
			error = ppb_write(ppbus, sc->sc_cp,
			    sc->sc_transfer_count, 0);
			switch (error) {
			case 0:
				sc->sc_transfer_count = 0;
				break;
			case EINTR:
				sc->sc_state |= LP_INTERRUPTED;
				ppb_unlock(ppbus);
				return (error);
			case EINVAL:
				log(LOG_NOTICE,
				    "%s: extended mode not available\n",
				    device_get_nameunit(lpt_dev));
				break;
			default:
				ppb_unlock(ppbus);
				return (error);
			}
		} else while ((sc->sc_transfer_count > 0) &&
		              (sc->sc_irq_status & LP_USE_IRQ)) {
			if (!(sc->sc_state & LP_BUSY))
				lpt_intr(sc);

			if (sc->sc_state & LP_BUSY) {
				error = ppb_sleep(ppbus, lpt_dev,
				    LPT_PRI | PCATCH, "lpt_write", 0);
				if (error) {
					sc->sc_state |= LP_INTERRUPTED;
					ppb_unlock(ppbus);
					return (error);
				}
			}
		}

		if (!(sc->sc_irq_status & LP_USE_IRQ) &&
		     (sc->sc_transfer_count)) {
			error = lpt_push_bytes(sc);
			if (error) {
				ppb_unlock(ppbus);
				return (error);
			}
		}
	}

	lpt_release_ppbus(lpt_dev);
	ppb_unlock(ppbus);

	return (error);
}

static int
lpt_ioctl(struct cdev *dev, u_long cmd, caddr_t data, int fflag,
    struct thread *td)
{
	struct lpt_data *sc = dev->si_drv1;
	device_t lpt_dev = sc->sc_dev;
	device_t ppbus = device_get_parent(lpt_dev);
	u_short old_irq_status;
	int error = 0;

	switch (cmd) {
	case LPT_IRQ:
		ppb_lock(ppbus);
		if (sc->sc_irq_status & LP_HAS_IRQ) {
			old_irq_status = sc->sc_irq_status;
			switch (*(int *)data) {
			case 0:
				sc->sc_irq_status &= ~LP_ENABLE_IRQ;
				break;
			case 1:
				sc->sc_irq_status &= ~LP_ENABLE_EXT;
				sc->sc_irq_status |= LP_ENABLE_IRQ;
				break;
			case 2:
				sc->sc_irq_status &= ~LP_ENABLE_IRQ;
				sc->sc_irq_status |= LP_ENABLE_EXT;
				break;
			case 3:
				sc->sc_irq_status &= ~LP_ENABLE_EXT;
				break;
			default:
				break;
			}

			if (old_irq_status != sc->sc_irq_status)
				log(LOG_NOTICE,
				    "%s: switched to %s %s mode\n",
				    device_get_nameunit(lpt_dev),
				    (sc->sc_irq_status & LP_ENABLE_IRQ) ?
				    "interrupt-driven" : "polled",
				    (sc->sc_irq_status & LP_ENABLE_EXT) ?
				    "extended" : "standard");
		} else
			error = EOPNOTSUPP;

		ppb_unlock(ppbus);
		break;
	default:
		error = ENODEV;
		break;
	}

	return (error);
}

static device_method_t lpt_methods[] = {
	DEVMETHOD(device_identify,	lpt_identify),
	DEVMETHOD(device_probe,		lpt_probe),
	DEVMETHOD(device_attach,	lpt_attach),
	DEVMETHOD(device_detach,	lpt_detach),
	{ 0, 0 }
};

static driver_t lpt_driver = {
	LPT_NAME,
	lpt_methods,
	sizeof(struct lpt_data)
};

DRIVER_MODULE(lpt, ppbus, lpt_driver, lpt_devclass, 0, 0);
MODULE_DEPEND(lpt, ppbus, 1, 1, 1);
