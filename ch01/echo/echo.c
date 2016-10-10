/* $Id: echo.c,v 1.4 2012/04/12 14:48:43 ghost Exp $ */

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

#define BUFFER_SIZE	256

/* Forward declarations. */
static d_open_t		echo_open;
static d_close_t	echo_close;
static d_read_t		echo_read;
static d_write_t	echo_write;

static struct cdevsw echo_cdevsw = {
	.d_version =	D_VERSION,
	.d_open =	echo_open,
	.d_close =	echo_close,
	.d_read =	echo_read,
	.d_write =	echo_write,
	.d_name =	"echo"
};

typedef struct echo {
	char buffer[BUFFER_SIZE];
	int length;
} echo_t;

static echo_t *echo_message;
static struct cdev *echo_dev;

static int
echo_open(struct cdev *dev, int oflags, int devtype, struct thread *td)
{
	uprintf("Opening echo device.\n");
	return (0);
}

static int
echo_close(struct cdev *dev, int fflag, int devtype, struct thread *td)
{
	uprintf("Closing echo device.\n");
	return (0);
}

static int
echo_write(struct cdev *dev, struct uio *uio, int ioflag)
{
	int error = 0;

	error = copyin(uio->uio_iov->iov_base, echo_message->buffer,
	    MIN(uio->uio_iov->iov_len, BUFFER_SIZE - 1));
	if (error != 0) {
		uprintf("Write failed.\n");
		return (error);
	}

	*(echo_message->buffer +
	    MIN(uio->uio_iov->iov_len, BUFFER_SIZE - 1)) = 0;

	echo_message->length = MIN(uio->uio_iov->iov_len, BUFFER_SIZE - 1);

	return (error);
}

static int
echo_read(struct cdev *dev, struct uio *uio, int ioflag)
{
	int error = 0;
	int amount;

	amount = MIN(uio->uio_resid,
	    (echo_message->length - uio->uio_offset > 0) ?
	     echo_message->length - uio->uio_offset : 0);

	error = uiomove(echo_message->buffer + uio->uio_offset, amount, uio);
	if (error != 0)
		uprintf("Read failed.\n");

	return (error);
}

static int
echo_modevent(module_t mod __unused, int event, void *arg __unused)
{
	int error = 0;

	switch (event) {
	case MOD_LOAD:
		echo_message = malloc(sizeof(echo_t), M_TEMP, M_WAITOK);
		echo_dev = make_dev(&echo_cdevsw, 0, UID_ROOT, GID_WHEEL,
		    0600, "echo");
		uprintf("Echo driver loaded.\n");
		break;
	case MOD_UNLOAD:
		destroy_dev(echo_dev);
		free(echo_message, M_TEMP);
		uprintf("Echo driver unloaded.\n");
		break;
	default:
		error = EOPNOTSUPP;
		break;
	}

	return (error);
}

DEV_MODULE(echo, echo_modevent, NULL);
