/* $Id: race_sx.c,v 1.4 2012/04/12 15:19:15 ghost Exp $ */

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
#include <sys/malloc.h>
#include <sys/ioccom.h>
#include <sys/queue.h>
#include <sys/lock.h>
#include <sys/sx.h>
#include "race_ioctl.h"

MALLOC_DEFINE(M_RACE, "race", "race object");

struct race_softc {
	LIST_ENTRY(race_softc) list;
	int unit;
};

static LIST_HEAD(, race_softc) race_list = LIST_HEAD_INITIALIZER(&race_list);
static struct sx race_sx;

static struct race_softc *	race_new(void);
static struct race_softc *	race_find(int unit);
static void			race_destroy(struct race_softc *sc);
static d_ioctl_t		race_ioctl_sx;
static d_ioctl_t		race_ioctl;

static struct cdevsw race_cdevsw = {
	.d_version =	D_VERSION,
	.d_ioctl =	race_ioctl_sx,
	.d_name =	RACE_NAME
};

static struct cdev *race_dev;

static int
race_ioctl_sx(struct cdev *dev, u_long cmd, caddr_t data, int fflag,
    struct thread *td)
{
	int error;

	sx_xlock(&race_sx);
	error = race_ioctl(dev, cmd, data, fflag, td);
	sx_xunlock(&race_sx);

	return (error);
}

static int
race_ioctl(struct cdev *dev, u_long cmd, caddr_t data, int fflag,
    struct thread *td)
{
	struct race_softc *sc;
	int error = 0;

	switch (cmd) {
	case RACE_IOC_ATTACH:
		sc = race_new();
		*(int *)data = sc->unit;
		break;
	case RACE_IOC_DETACH:
		sc = race_find(*(int *)data);
		if (sc == NULL)
			return (ENOENT);
		race_destroy(sc);
		break;
	case RACE_IOC_QUERY:
		sc = race_find(*(int *)data);
		if (sc == NULL)
			return (ENOENT);
		break;
	case RACE_IOC_LIST:
		uprintf("  UNIT\n");
		LIST_FOREACH(sc, &race_list, list)
			uprintf("  %d\n", sc->unit);
		break;
	default:
		error = ENOTTY;
		break;
	}

	return (error);
}

static struct race_softc *
race_new(void)
{
	struct race_softc *sc;
	int unit, max = -1;

	LIST_FOREACH(sc, &race_list, list) {
		if (sc->unit > max)
			max = sc->unit;
	}
	unit = max + 1;

	sc = (struct race_softc *)malloc(sizeof(struct race_softc), M_RACE,
	    M_WAITOK | M_ZERO);
	sc->unit = unit;
	LIST_INSERT_HEAD(&race_list, sc, list);

	return (sc);
}

static struct race_softc *
race_find(int unit)
{
	struct race_softc *sc;

	LIST_FOREACH(sc, &race_list, list) {
		if (sc->unit == unit)
			break;
	}

	return (sc);
}

static void
race_destroy(struct race_softc *sc)
{
	LIST_REMOVE(sc, list);
	free(sc, M_RACE);
}

static int
race_modevent(module_t mod __unused, int event, void *arg __unused)
{
	int error = 0;
	struct race_softc *sc, *sc_temp;

	switch (event) {
	case MOD_LOAD:
		sx_init(&race_sx, "race config lock");
		race_dev = make_dev(&race_cdevsw, 0, UID_ROOT, GID_WHEEL,
		    0600, RACE_NAME);
		uprintf("Race driver loaded.\n");
		break;
	case MOD_UNLOAD:
		destroy_dev(race_dev);
		sx_xlock(&race_sx);
		if (!LIST_EMPTY(&race_list)) {
			LIST_FOREACH_SAFE(sc, &race_list, list, sc_temp) {
				LIST_REMOVE(sc, list);
				free(sc, M_RACE);
			}
		}
		sx_xunlock(&race_sx);
		sx_destroy(&race_sx);
		uprintf("Race driver unloaded.\n");
		break;
	case MOD_QUIESCE:
		sx_xlock(&race_sx);
		if (!LIST_EMPTY(&race_list))
			error = EBUSY;
		sx_xunlock(&race_sx);
		break;
	default:
		error = EOPNOTSUPP;
		break;
	}

	return (error);
}

DEV_MODULE(race, race_modevent, NULL);
