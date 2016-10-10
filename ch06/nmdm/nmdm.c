/* $Id: nmdm.c,v 1.5 2012/04/12 15:54:04 ghost Exp $ */

/*-
 * Copyright (c) 1982, 1986, 1989, 1993
 *	The Regents of the University of California.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
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

#include <sys/tty.h>
#include <sys/conf.h>
#include <sys/eventhandler.h>
#include <sys/limits.h>
#include <sys/serial.h>
#include <sys/malloc.h>
#include <sys/queue.h>
#include <sys/taskqueue.h>
#include <sys/lock.h>
#include <sys/mutex.h>

MALLOC_DEFINE(M_NMDM, "nullmodem", "nullmodem data structures");

struct nmdm_part {
	struct tty		*np_tty;
	struct nmdm_part	*np_other;
	struct task		np_task;
	struct callout		np_callout;
	int			np_dcd;
	int			np_rate;
	u_long			np_quota;
	int			np_credits;
	u_long			np_accumulator;

#define QS 8			/* Quota shift. */
};

struct nmdm_softc {
	struct nmdm_part	ns_partA;
	struct nmdm_part	ns_partB;
	struct mtx		ns_mtx;
};

static tsw_outwakeup_t		nmdm_outwakeup;
static tsw_inwakeup_t		nmdm_inwakeup;
static tsw_param_t		nmdm_param;
static tsw_modem_t		nmdm_modem;

static struct ttydevsw nmdm_class = {
	.tsw_flags =		TF_NOPREFIX,
	.tsw_outwakeup =	nmdm_outwakeup,
	.tsw_inwakeup =		nmdm_inwakeup,
	.tsw_param =		nmdm_param,
	.tsw_modem =		nmdm_modem
};

static int nmdm_count = 0;

static void
nmdm_timeout(void *arg)
{
	struct nmdm_part *np = arg;

	if (np->np_rate == 0)
		return;

	/*
	 * Do a simple Floyd-Steinberg dither to avoid FP math.
	 * Wipe out unused quota from last tick.
	 */
	np->np_accumulator += np->np_credits;
	np->np_quota = np->np_accumulator >> QS;
	np->np_accumulator &= ((1 << QS) - 1);

	taskqueue_enqueue(taskqueue_swi, &np->np_task);
	callout_reset(&np->np_callout, np->np_rate, nmdm_timeout, np);
}

static void
nmdm_task_tty(void *arg, int pending __unused)
{
	struct tty *tp, *otp;
	struct nmdm_part *np = arg;
	char c;

	tp = np->np_tty;
	tty_lock(tp);

	otp = np->np_other->np_tty;
	KASSERT(otp != NULL, ("nmdm_task_tty: null otp"));
	KASSERT(otp != tp, ("nmdm_task_tty: otp == tp"));

	if (np->np_other->np_dcd) {
		if (!tty_opened(tp)) {
			np->np_other->np_dcd = 0;
			ttydisc_modem(otp, 0);
		}
	} else {
		if (tty_opened(tp)) {
			np->np_other->np_dcd = 1;
			ttydisc_modem(otp, 1);
		}
	}

	while (ttydisc_rint_poll(otp) > 0) {
		if (np->np_rate && !np->np_quota)
			break;
		if (ttydisc_getc(tp, &c, 1) != 1)
			break;
		np->np_quota--;
		ttydisc_rint(otp, c, 0);
	}
	ttydisc_rint_done(otp);

	tty_unlock(tp);
}

static struct nmdm_softc *
nmdm_alloc(unsigned long unit)
{
	struct nmdm_softc *ns;

	atomic_add_int(&nmdm_count, 1);

	ns = malloc(sizeof(*ns), M_NMDM, M_WAITOK | M_ZERO);
	mtx_init(&ns->ns_mtx, "nmdm", NULL, MTX_DEF);

	/* Connect the pairs together. */
	ns->ns_partA.np_other = &ns->ns_partB;
	TASK_INIT(&ns->ns_partA.np_task, 0, nmdm_task_tty, &ns->ns_partA);
	callout_init_mtx(&ns->ns_partA.np_callout, &ns->ns_mtx, 0);

	ns->ns_partB.np_other = &ns->ns_partA;
	TASK_INIT(&ns->ns_partB.np_task, 0, nmdm_task_tty, &ns->ns_partB);
	callout_init_mtx(&ns->ns_partB.np_callout, &ns->ns_mtx, 0);

	/* Create device nodes. */
	ns->ns_partA.np_tty = tty_alloc_mutex(&nmdm_class, &ns->ns_partA,
	    &ns->ns_mtx);
	tty_makedev(ns->ns_partA.np_tty, NULL, "nmdm%luA", unit);

	ns->ns_partB.np_tty = tty_alloc_mutex(&nmdm_class, &ns->ns_partB,
	    &ns->ns_mtx);
	tty_makedev(ns->ns_partB.np_tty, NULL, "nmdm%luB", unit);

	return (ns);
}

static void
nmdm_clone(void *arg, struct ucred *cred, char *name, int len,
    struct cdev **dev)
{
	unsigned long unit;
	char *end;
	struct nmdm_softc *ns;

	if (*dev != NULL)
		return;
	if (strncmp(name, "nmdm", 4) != 0)
		return;

	/* Device name must be "nmdm%lu%c", where %c is "A" or "B". */
	name += 4;
	unit = strtoul(name, &end, 10);
	if (unit == ULONG_MAX || name == end)
		return;
	if ((end[0] != 'A' && end[0] != 'B') || end[1] != '\0')
		return;

	ns = nmdm_alloc(unit);

	if (end[0] == 'A')
		*dev = ns->ns_partA.np_tty->t_dev;
	else
		*dev = ns->ns_partB.np_tty->t_dev;
}

static void
nmdm_outwakeup(struct tty *tp)
{
	struct nmdm_part *np = tty_softc(tp);

	/* We can transmit again, so wake up our side. */
	taskqueue_enqueue(taskqueue_swi, &np->np_task);
}

static void
nmdm_inwakeup(struct tty *tp)
{
	struct nmdm_part *np = tty_softc(tp);

	/* We can receive again, so wake up the other side. */
	taskqueue_enqueue(taskqueue_swi, &np->np_other->np_task);
}

static int
bits_per_char(struct termios *t)
{
	int bits;

	bits = 1;		/* start bit. */
	switch (t->c_cflag & CSIZE) {
	case CS5:
		bits += 5;
		break;
	case CS6:
		bits += 6;
		break;
	case CS7:
		bits += 7;
		break;
	case CS8:
		bits += 8;
		break;
	}
	bits++;			/* stop bit. */
	if (t->c_cflag & PARENB)
		bits++;
	if (t->c_cflag & CSTOPB)
		bits++;

	return (bits);
}

static int
nmdm_param(struct tty *tp, struct termios *t)
{
	struct nmdm_part *np = tty_softc(tp);
	struct tty *otp;
	int bpc, rate, speed, i;

	otp = np->np_other->np_tty;

	if (!((t->c_cflag | otp->t_termios.c_cflag) & CDSR_OFLOW)) {
		np->np_rate = 0;
		np->np_other->np_rate = 0;
		return (0);
	}

	bpc = imax(bits_per_char(t), bits_per_char(&otp->t_termios));

	for (i = 0; i < 2; i++) {
		/* Use the slower of their transmit or our receive rate. */
		speed = imin(otp->t_termios.c_ospeed, t->c_ispeed);
		if (speed == 0) {
			np->np_rate = 0;
			np->np_other->np_rate = 0;
			return (0);
		}

		speed <<= QS;			/* bits per second, scaled. */
		speed /= bpc;			/* char per second, scaled. */
		rate = (hz << QS) / speed;	/* hz per callout. */
		if (rate == 0)
			rate = 1;

		speed *= rate;
		speed /= hz;			/* (char/sec)/tick, scaled. */

		np->np_credits = speed;
		np->np_rate = rate;
		callout_reset(&np->np_callout, rate, nmdm_timeout, np);

		/* Swap pointers for second pass--to update the other end. */
		np = np->np_other;
		t = &otp->t_termios;
		otp = tp;
	}

	return (0);
}

static int
nmdm_modem(struct tty *tp, int sigon, int sigoff)
{
	struct nmdm_part *np = tty_softc(tp);
	int i = 0;

	/* Set modem control lines. */
	if (sigon || sigoff) {
		if (sigon & SER_DTR)
			np->np_other->np_dcd = 1;
		if (sigoff & SER_DTR)
			np->np_other->np_dcd = 0;

		ttydisc_modem(np->np_other->np_tty, np->np_other->np_dcd);

		return (0);
	/* Get state of modem control lines. */
	} else {
		if (np->np_dcd)
			i |= SER_DCD;
		if (np->np_other->np_dcd)
			i |= SER_DTR;

		return (i);
	}
}

static int
nmdm_modevent(module_t mod __unused, int event, void *arg __unused)
{
	static eventhandler_tag tag;

	switch (event) {
	case MOD_LOAD:
		tag = EVENTHANDLER_REGISTER(dev_clone, nmdm_clone, 0, 1000);
		if (tag == NULL)
			return (ENOMEM);
		break;
	case MOD_SHUTDOWN:
		break;
	case MOD_UNLOAD:
		if (nmdm_count != 0)
			return (EBUSY);
		EVENTHANDLER_DEREGISTER(dev_clone, tag);
		break;
	default:
		return (EOPNOTSUPP);
	}

	return (0);
}

DEV_MODULE(nmdm, nmdm_modevent, NULL);
