/* $Id: at45d.c,v 1.4 2012/04/12 17:43:34 ghost Exp $ */

/*-
 * Copyright (c) 2006 M. Warner Losh.
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

#include <sys/param.h>
#include <sys/module.h>
#include <sys/kernel.h>
#include <sys/systm.h>

#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/bio.h>
#include <sys/kthread.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <geom/geom_disk.h>

#include <dev/spibus/spi.h>
#include "spibus_if.h"

#define MANUFACTURER_ID			0x9f
#define STATUS_REGISTER_READ		0xd7
#define CONTINUOUS_ARRAY_READ_HF	0x0b
#define PROGRAM_THROUGH_BUFFER		0x82

struct at45d_softc {
	device_t			at45d_dev;
	struct mtx			at45d_mtx;
	struct intr_config_hook		at45d_ich;
	struct disk		       *at45d_disk;
	struct bio_queue_head		at45d_bioq;
	struct proc		       *at45d_proc;
};

static devclass_t at45d_devclass;

static void				at45d_delayed_attach(void *);
static void				at45d_task(void *);
static void				at45d_strategy(struct bio *);

static int
at45d_probe(device_t dev)
{
	device_set_desc(dev, "AT45 flash family");
	return (BUS_PROBE_SPECIFIC);
}

static int
at45d_attach(device_t dev)
{
	struct at45d_softc *sc = device_get_softc(dev);
	int error;

	sc->at45d_dev = dev;
	mtx_init(&sc->at45d_mtx, device_get_nameunit(dev), "at45d", MTX_DEF);

	sc->at45d_ich.ich_func = at45d_delayed_attach;
	sc->at45d_ich.ich_arg = sc;
	error = config_intrhook_establish(&sc->at45d_ich);
	if (error)
		device_printf(dev, "config_intrhook_establish() failed!\n");

	return (0);
}

static int
at45d_detach(device_t dev)
{
	return (EIO);
}

static int
at45d_get_info(device_t dev, uint8_t *r)
{
	struct spi_command cmd;
	uint8_t tx_buf[8], rx_buf[8];
	int error;

	memset(&cmd, 0, sizeof(cmd));
	memset(tx_buf, 0, sizeof(tx_buf));
	memset(rx_buf, 0, sizeof(rx_buf));

	tx_buf[0] = MANUFACTURER_ID;
	cmd.tx_cmd = &tx_buf[0];
	cmd.rx_cmd = &rx_buf[0];
	cmd.tx_cmd_sz = 5;
	cmd.rx_cmd_sz = 5;
	error = SPIBUS_TRANSFER(device_get_parent(dev), dev, &cmd);
	if (error)
		return (error);

	memcpy(r, &rx_buf[1], 4);
	return (0);
}

static uint8_t
at45d_get_status(device_t dev)
{
	struct spi_command cmd;
	uint8_t tx_buf[8], rx_buf[8];

	memset(&cmd, 0, sizeof(cmd));
	memset(tx_buf, 0, sizeof(tx_buf));
	memset(rx_buf, 0, sizeof(rx_buf));

	tx_buf[0] = STATUS_REGISTER_READ;
	cmd.tx_cmd = &tx_buf[0];
	cmd.rx_cmd = &rx_buf[0];
	cmd.tx_cmd_sz = 2;
	cmd.rx_cmd_sz = 2;
	SPIBUS_TRANSFER(device_get_parent(dev), dev, &cmd);

	return (rx_buf[1]);
}

static void
at45d_wait_for_device_ready(device_t dev)
{
	while ((at45d_get_status(dev) & 0x80) == 0)
		continue;
}

static void
at45d_delayed_attach(void *arg)
{
	struct at45d_softc *sc = arg;
	uint8_t buf[4];

	at45d_get_info(sc->at45d_dev, buf);
	at45d_wait_for_device_ready(sc->at45d_dev);

	sc->at45d_disk = disk_alloc();
	sc->at45d_disk->d_name = "at45d";
	sc->at45d_disk->d_unit = device_get_unit(sc->at45d_dev);
	sc->at45d_disk->d_strategy = at45d_strategy;
	sc->at45d_disk->d_sectorsize = 1056;
	sc->at45d_disk->d_mediasize = 8192 * 1056;
	sc->at45d_disk->d_maxsize = DFLTPHYS;
	sc->at45d_disk->d_drv1 = sc;

	bioq_init(&sc->at45d_bioq);
	kproc_create(&at45d_task, sc, &sc->at45d_proc, 0, 0, "at45d");

	disk_create(sc->at45d_disk, DISK_VERSION);
	config_intrhook_disestablish(&sc->at45d_ich);
}

static void
at45d_strategy(struct bio *bp)
{
	struct at45d_softc *sc = bp->bio_disk->d_drv1;

	mtx_lock(&sc->at45d_mtx);
	bioq_disksort(&sc->at45d_bioq, bp);
	wakeup(sc);
	mtx_unlock(&sc->at45d_mtx);
}

static void
at45d_task(void *arg)
{
	struct at45d_softc *sc = arg;
	struct bio *bp;
	struct spi_command cmd;
	uint8_t tx_buf[8], rx_buf[8];
	int ss = sc->at45d_disk->d_sectorsize;
	daddr_t block, end;
	char *vaddr;

	for (;;) {
		mtx_lock(&sc->at45d_mtx);
		do {
			bp = bioq_first(&sc->at45d_bioq);
			if (bp == NULL)
				mtx_sleep(sc, &sc->at45d_mtx, PRIBIO,
				    "at45d", 0);
		} while (bp == NULL);
		bioq_remove(&sc->at45d_bioq, bp);
		mtx_unlock(&sc->at45d_mtx);

		end = bp->bio_pblkno + (bp->bio_bcount / ss);
		for (block = bp->bio_pblkno; block < end; block++) {
			vaddr = bp->bio_data + (block - bp->bio_pblkno) * ss;
			if (bp->bio_cmd == BIO_READ) {
				tx_buf[0] = CONTINUOUS_ARRAY_READ_HF;
				cmd.tx_cmd_sz = 5;
				cmd.rx_cmd_sz = 5;
			} else {
				tx_buf[0] = PROGRAM_THROUGH_BUFFER;
				cmd.tx_cmd_sz = 4;
				cmd.rx_cmd_sz = 4;
			}

			/* FIXME: This works only on certain devices. */
			tx_buf[1] = ((block >> 5) & 0xff);
			tx_buf[2] = ((block << 3) & 0xf8);
			tx_buf[3] = 0;
			tx_buf[4] = 0;
			cmd.tx_cmd = &tx_buf[0];
			cmd.rx_cmd = &rx_buf[0];
			cmd.tx_data = vaddr;
			cmd.rx_data = vaddr;
			cmd.tx_data_sz = ss;
			cmd.rx_data_sz = ss;
			SPIBUS_TRANSFER(device_get_parent(sc->at45d_dev),
			    sc->at45d_dev, &cmd);
		}
		biodone(bp);
	}
}

static device_method_t at45d_methods[] = {
	/* Device interface. */
	DEVMETHOD(device_probe,		at45d_probe),
	DEVMETHOD(device_attach,	at45d_attach),
	DEVMETHOD(device_detach,	at45d_detach),
	{ 0, 0 }
};

static driver_t at45d_driver = {
	"at45d",
	at45d_methods,
	sizeof(struct at45d_softc)
};

DRIVER_MODULE(at45d, spibus, at45d_driver, at45d_devclass, 0, 0);
