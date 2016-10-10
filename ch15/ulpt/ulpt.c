/* $Id: ulpt.c,v 1.10 2012/04/12 18:53:20 ghost Exp $ */

/*-
 * Copyright (c) 1998, 2003 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Lennart Augustsson (lennart@augustsson.net) at
 * Carlstedt Research & Technology.
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
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/param.h>
#include <sys/module.h>
#include <sys/kernel.h>
#include <sys/systm.h>

#include <sys/conf.h>
#include <sys/bus.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/syslog.h>
#include <sys/fcntl.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdi_util.h>

#define ULPT_BUF_SIZE		(1 << 15)
#define ULPT_IFQ_MAX_LEN	2

#define UREQ_GET_PORT_STATUS	0x01
#define UREQ_SOFT_RESET		0x02

#define LPS_NERR		0x08
#define LPS_SELECT		0x10
#define LPS_NOPAPER		0x20
#define LPS_INVERT		(LPS_NERR | LPS_SELECT)
#define LPS_MASK		(LPS_NERR | LPS_SELECT | LPS_NOPAPER)

enum {
	ULPT_BULK_DT_WR,
	ULPT_BULK_DT_RD,
	ULPT_INTR_DT_RD,
	ULPT_N_TRANSFER
};

struct ulpt_softc {
	device_t		sc_dev;
	struct usb_device      *sc_usb_device;
	struct mtx		sc_mutex;
	struct usb_callout	sc_watchdog;
	uint8_t			sc_iface_num;
	struct usb_xfer	       *sc_transfer[ULPT_N_TRANSFER];
	struct usb_fifo_sc	sc_fifo;
	struct usb_fifo_sc	sc_fifo_no_reset;
	int			sc_fflags;
	struct usb_fifo	       *sc_fifo_open[2];
	uint8_t			sc_zero_length_packets;
	uint8_t			sc_previous_status;
};

static device_probe_t		ulpt_probe;
static device_attach_t		ulpt_attach;
static device_detach_t		ulpt_detach;

static usb_fifo_open_t		ulpt_open;
static usb_fifo_open_t		unlpt_open;
static usb_fifo_close_t		ulpt_close;
static usb_fifo_ioctl_t		ulpt_ioctl;
static usb_fifo_cmd_t		ulpt_start_read;
static usb_fifo_cmd_t		ulpt_stop_read;
static usb_fifo_cmd_t		ulpt_start_write;
static usb_fifo_cmd_t		ulpt_stop_write;

static void			ulpt_reset(struct ulpt_softc *);
static void			ulpt_watchdog(void *);

static usb_callback_t		ulpt_write_callback;
static usb_callback_t		ulpt_read_callback;
static usb_callback_t		ulpt_status_callback;

static struct usb_fifo_methods ulpt_fifo_methods = {
	.f_open =		&ulpt_open,
	.f_close =		&ulpt_close,
	.f_ioctl =		&ulpt_ioctl,
	.f_start_read =		&ulpt_start_read,
	.f_stop_read =		&ulpt_stop_read,
	.f_start_write =	&ulpt_start_write,
	.f_stop_write =		&ulpt_stop_write,
	.basename[0] =		"ulpt"
};

static struct usb_fifo_methods unlpt_fifo_methods = {
	.f_open =		&unlpt_open,
	.f_close =		&ulpt_close,
	.f_ioctl =		&ulpt_ioctl,
	.f_start_read =		&ulpt_start_read,
	.f_stop_read =		&ulpt_stop_read,
	.f_start_write =	&ulpt_start_write,
	.f_stop_write =		&ulpt_stop_write,
	.basename[0] =		"unlpt"
};

static const struct usb_config ulpt_config[ULPT_N_TRANSFER] = {
	[ULPT_BULK_DT_WR] = {
		.callback =	&ulpt_write_callback,
		.bufsize =	ULPT_BUF_SIZE,
		.flags =	{.pipe_bof = 1, .proxy_buffer = 1},
		.type =		UE_BULK,
		.endpoint =	UE_ADDR_ANY,
		.direction =	UE_DIR_OUT
	},

	[ULPT_BULK_DT_RD] = {
		.callback =	&ulpt_read_callback,
		.bufsize =	ULPT_BUF_SIZE,
		.flags =	{.short_xfer_ok = 1, .pipe_bof = 1,
				    .proxy_buffer = 1},
		.type =		UE_BULK,
		.endpoint =	UE_ADDR_ANY,
		.direction =	UE_DIR_IN
	},

	[ULPT_INTR_DT_RD] = {
		.callback =	&ulpt_status_callback,
		.bufsize =	sizeof(struct usb_device_request) + 1,
		.timeout =	1000,		/* 1 second. */
		.type =		UE_CONTROL,
		.endpoint =	0x00,
		.direction =	UE_DIR_ANY
	}
};

static int
ulpt_open(struct usb_fifo *fifo, int fflags)
{
	struct ulpt_softc *sc = usb_fifo_softc(fifo);

	if (sc->sc_fflags == 0)
		ulpt_reset(sc);

	return (unlpt_open(fifo, fflags));
}

static void
ulpt_reset(struct ulpt_softc *sc)
{
	struct usb_device_request req;
	int error;

	req.bRequest = UREQ_SOFT_RESET;
	USETW(req.wValue, 0);
	USETW(req.wIndex, sc->sc_iface_num);
	USETW(req.wLength, 0);

	mtx_lock(&sc->sc_mutex);

	req.bmRequestType = UT_WRITE_CLASS_OTHER;
	error = usbd_do_request_flags(sc->sc_usb_device, &sc->sc_mutex,
	    &req, NULL, 0, NULL, 2 * USB_MS_HZ);
	if (error) {
		req.bmRequestType = UT_WRITE_CLASS_INTERFACE;
		usbd_do_request_flags(sc->sc_usb_device, &sc->sc_mutex,
		    &req, NULL, 0, NULL, 2 * USB_MS_HZ);
	}

	mtx_unlock(&sc->sc_mutex);
}

static int
unlpt_open(struct usb_fifo *fifo, int fflags)
{
	struct ulpt_softc *sc = usb_fifo_softc(fifo);
	int error;

	if (sc->sc_fflags & fflags)
		return (EBUSY);

	if (fflags & FREAD) {
		mtx_lock(&sc->sc_mutex);
		usbd_xfer_set_stall(sc->sc_transfer[ULPT_BULK_DT_RD]);
		mtx_unlock(&sc->sc_mutex);

		error = usb_fifo_alloc_buffer(fifo,
		    usbd_xfer_max_len(sc->sc_transfer[ULPT_BULK_DT_RD]),
		    ULPT_IFQ_MAX_LEN);
		if (error)
			return (ENOMEM);

		sc->sc_fifo_open[USB_FIFO_RX] = fifo;
	}

	if (fflags & FWRITE) {
		mtx_lock(&sc->sc_mutex);
		usbd_xfer_set_stall(sc->sc_transfer[ULPT_BULK_DT_WR]);
		mtx_unlock(&sc->sc_mutex);

		error = usb_fifo_alloc_buffer(fifo,
		    usbd_xfer_max_len(sc->sc_transfer[ULPT_BULK_DT_WR]),
		    ULPT_IFQ_MAX_LEN);
		if (error)
			return (ENOMEM);

		sc->sc_fifo_open[USB_FIFO_TX] = fifo;
	}

	sc->sc_fflags |= fflags & (FREAD | FWRITE);
	return (0);
}

static void
ulpt_close(struct usb_fifo *fifo, int fflags)
{
	struct ulpt_softc *sc = usb_fifo_softc(fifo);

	sc->sc_fflags &= ~(fflags & (FREAD | FWRITE));

	if (fflags & (FREAD | FWRITE))
		usb_fifo_free_buffer(fifo);
}

static int
ulpt_ioctl(struct usb_fifo *fifo, u_long cmd, void *data, int fflags)
{
	return (ENODEV);
}

static void
ulpt_watchdog(void *arg)
{
	struct ulpt_softc *sc = arg;

	mtx_assert(&sc->sc_mutex, MA_OWNED);

	if (sc->sc_fflags == 0)
		usbd_transfer_start(sc->sc_transfer[ULPT_INTR_DT_RD]);

	usb_callout_reset(&sc->sc_watchdog, hz, &ulpt_watchdog, sc);
}

static void
ulpt_start_read(struct usb_fifo *fifo)
{
	struct ulpt_softc *sc = usb_fifo_softc(fifo);

	usbd_transfer_start(sc->sc_transfer[ULPT_BULK_DT_RD]);
}

static void
ulpt_stop_read(struct usb_fifo *fifo)
{
	struct ulpt_softc *sc = usb_fifo_softc(fifo);

	usbd_transfer_stop(sc->sc_transfer[ULPT_BULK_DT_RD]);
}

static void
ulpt_start_write(struct usb_fifo *fifo)
{
	struct ulpt_softc *sc = usb_fifo_softc(fifo);

	usbd_transfer_start(sc->sc_transfer[ULPT_BULK_DT_WR]);
}

static void
ulpt_stop_write(struct usb_fifo *fifo)
{
	struct ulpt_softc *sc = usb_fifo_softc(fifo);

	usbd_transfer_stop(sc->sc_transfer[ULPT_BULK_DT_WR]);
}

static void
ulpt_write_callback(struct usb_xfer *transfer, usb_error_t error)
{
	struct ulpt_softc *sc = usbd_xfer_softc(transfer);
	struct usb_fifo *fifo = sc->sc_fifo_open[USB_FIFO_TX];
	struct usb_page_cache *pc;
	int actual, max;

	usbd_xfer_status(transfer, &actual, NULL, NULL, NULL);

	if (fifo == NULL)
		return;

	switch (USB_GET_STATE(transfer)) {
	case USB_ST_SETUP:
	case USB_ST_TRANSFERRED:
setup:
		pc = usbd_xfer_get_frame(transfer, 0);
		max = usbd_xfer_max_len(transfer);
		if (usb_fifo_get_data(fifo, pc, 0, max, &actual, 0)) {
			usbd_xfer_set_frame_len(transfer, 0, actual);
			usbd_transfer_submit(transfer);
		}
		break;
	default:
		if (error != USB_ERR_CANCELLED) {
			/* Issue a clear-stall request. */
			usbd_xfer_set_stall(transfer);
			goto setup;
		}
		break;
	}
}

static void
ulpt_read_callback(struct usb_xfer *transfer, usb_error_t error)
{
	struct ulpt_softc *sc = usbd_xfer_softc(transfer);
	struct usb_fifo *fifo = sc->sc_fifo_open[USB_FIFO_RX];
	struct usb_page_cache *pc;
	int actual, max;

	usbd_xfer_status(transfer, &actual, NULL, NULL, NULL);

	if (fifo == NULL)
		return;

	switch (USB_GET_STATE(transfer)) {
	case USB_ST_TRANSFERRED:
		if (actual == 0) {
			if (sc->sc_zero_length_packets == 4)
				/* Throttle transfers. */
				usbd_xfer_set_interval(transfer, 500);
			else
				sc->sc_zero_length_packets++;
		} else {
			/* Disable throttling. */
			usbd_xfer_set_interval(transfer, 0);
			sc->sc_zero_length_packets = 0;
		}

		pc = usbd_xfer_get_frame(transfer, 0);
		usb_fifo_put_data(fifo, pc, 0, actual, 1);
		/* FALLTHROUGH */
	case USB_ST_SETUP:
setup:
		if (usb_fifo_put_bytes_max(fifo) != 0) {
			max = usbd_xfer_max_len(transfer);
			usbd_xfer_set_frame_len(transfer, 0, max);
			usbd_transfer_submit(transfer);
		}
		break;
	default:
		/* Disable throttling. */
		usbd_xfer_set_interval(transfer, 0);
		sc->sc_zero_length_packets = 0;

		if (error != USB_ERR_CANCELLED) {
			/* Issue a clear-stall request. */
			usbd_xfer_set_stall(transfer);
			goto setup;
		}
		break;
	}
}

static void
ulpt_status_callback(struct usb_xfer *transfer, usb_error_t error)
{
	struct ulpt_softc *sc = usbd_xfer_softc(transfer);
	struct usb_device_request req;
	struct usb_page_cache *pc;
	uint8_t current_status, new_status;

	switch (USB_GET_STATE(transfer)) {
	case USB_ST_SETUP:
		req.bmRequestType = UT_READ_CLASS_INTERFACE;
		req.bRequest = UREQ_GET_PORT_STATUS;
		USETW(req.wValue, 0);
		req.wIndex[0] = sc->sc_iface_num;
		req.wIndex[1] = 0;
		USETW(req.wLength, 1);

		pc = usbd_xfer_get_frame(transfer, 0);
		usbd_copy_in(pc, 0, &req, sizeof(req));
		usbd_xfer_set_frame_len(transfer, 0, sizeof(req));
		usbd_xfer_set_frame_len(transfer, 1, 1);
		usbd_xfer_set_frames(transfer, 2);
		usbd_transfer_submit(transfer);

		break;
	case USB_ST_TRANSFERRED:
		pc = usbd_xfer_get_frame(transfer, 1);
		usbd_copy_out(pc, 0, &current_status, 1);

		current_status = (current_status ^ LPS_INVERT) & LPS_MASK;
		new_status = current_status & ~sc->sc_previous_status;
		sc->sc_previous_status = current_status;

		if (new_status & LPS_NERR)
			log(LOG_NOTICE, "%s: output error\n",
			    device_get_nameunit(sc->sc_dev));
		else if (new_status & LPS_SELECT)
			log(LOG_NOTICE, "%s: offline\n",
			    device_get_nameunit(sc->sc_dev));
		else if (new_status & LPS_NOPAPER)
			log(LOG_NOTICE, "%s: out of paper\n",
			    device_get_nameunit(sc->sc_dev));

		break;
	default:
		break;
	}
}

static int
ulpt_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);

	if (uaa->usb_mode != USB_MODE_HOST)
		return (ENXIO);

	if ((uaa->info.bInterfaceClass == UICLASS_PRINTER) &&
	    (uaa->info.bInterfaceSubClass == UISUBCLASS_PRINTER) &&
	    ((uaa->info.bInterfaceProtocol == UIPROTO_PRINTER_UNI) ||
	     (uaa->info.bInterfaceProtocol == UIPROTO_PRINTER_BI) ||
	     (uaa->info.bInterfaceProtocol == UIPROTO_PRINTER_1284)))
		return (BUS_PROBE_SPECIFIC);

	return (ENXIO);
}

static int
ulpt_attach(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct ulpt_softc *sc = device_get_softc(dev);
	struct usb_interface_descriptor *idesc;
	struct usb_config_descriptor *cdesc;
	uint8_t alt_index, iface_index = uaa->info.bIfaceIndex;
	int error, unit = device_get_unit(dev);

	sc->sc_dev = dev;
	sc->sc_usb_device = uaa->device;
	device_set_usb_desc(dev);
	mtx_init(&sc->sc_mutex, "ulpt", NULL, MTX_DEF | MTX_RECURSE);
	usb_callout_init_mtx(&sc->sc_watchdog, &sc->sc_mutex, 0);

	idesc = usbd_get_interface_descriptor(uaa->iface);
	alt_index = -1;
	for (;;) {
		if (idesc == NULL)
			break;

		if ((idesc->bDescriptorType == UDESC_INTERFACE) &&
		    (idesc->bLength >= sizeof(*idesc))) {
			if (idesc->bInterfaceNumber != uaa->info.bIfaceNum)
				break;
			else {
				alt_index++;
				if ((idesc->bInterfaceClass ==
				     UICLASS_PRINTER) &&
				    (idesc->bInterfaceSubClass ==
				     UISUBCLASS_PRINTER) &&
				    (idesc->bInterfaceProtocol ==
				     UIPROTO_PRINTER_BI))
					goto found;
			}
		}

		cdesc = usbd_get_config_descriptor(uaa->device);
		idesc = (void *)usb_desc_foreach(cdesc, (void *)idesc);
	}
	goto detach;

found:
	if (alt_index) {
		error = usbd_set_alt_interface_index(uaa->device,
		    iface_index, alt_index);
		if (error)
			goto detach;
	}

	sc->sc_iface_num = idesc->bInterfaceNumber;

	error = usbd_transfer_setup(uaa->device, &iface_index,
	    sc->sc_transfer, ulpt_config, ULPT_N_TRANSFER, sc,
	    &sc->sc_mutex);
	if (error)
		goto detach;

	device_printf(dev, "using bi-directional mode\n");

	error = usb_fifo_attach(uaa->device, sc, &sc->sc_mutex,
	    &ulpt_fifo_methods, &sc->sc_fifo, unit, -1,
	    iface_index, UID_ROOT, GID_OPERATOR, 0644);
	if (error)
		goto detach;

	error = usb_fifo_attach(uaa->device, sc, &sc->sc_mutex,
	    &unlpt_fifo_methods, &sc->sc_fifo_no_reset, unit, -1,
	    iface_index, UID_ROOT, GID_OPERATOR, 0644);
	if (error)
		goto detach;

	mtx_lock(&sc->sc_mutex);
	ulpt_watchdog(sc);
	mtx_unlock(&sc->sc_mutex);
	return (0);

detach:
	ulpt_detach(dev);
	return (ENOMEM);
}

static int
ulpt_detach(device_t dev)
{
	struct ulpt_softc *sc = device_get_softc(dev);

	usb_fifo_detach(&sc->sc_fifo);
	usb_fifo_detach(&sc->sc_fifo_no_reset);

	mtx_lock(&sc->sc_mutex);
	usb_callout_stop(&sc->sc_watchdog);
	mtx_unlock(&sc->sc_mutex);

	usbd_transfer_unsetup(sc->sc_transfer, ULPT_N_TRANSFER);
	usb_callout_drain(&sc->sc_watchdog);
	mtx_destroy(&sc->sc_mutex);

	return (0);
}

static device_method_t ulpt_methods[] = {
	/* Device interface. */
	DEVMETHOD(device_probe,		ulpt_probe),
	DEVMETHOD(device_attach,	ulpt_attach),
	DEVMETHOD(device_detach,	ulpt_detach),
	{ 0, 0 }
};

static driver_t ulpt_driver = {
	"ulpt",
	ulpt_methods,
	sizeof(struct ulpt_softc)
};

static devclass_t ulpt_devclass;

DRIVER_MODULE(ulpt, uhub, ulpt_driver, ulpt_devclass, 0, 0);
MODULE_DEPEND(ulpt, usb, 1, 1, 1);
MODULE_DEPEND(ulpt, ucom, 1, 1, 1);
