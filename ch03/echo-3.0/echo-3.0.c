/* $Id: echo-3.0.c,v 1.10 2012/04/12 14:59:09 ghost Exp $ */

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

MALLOC_DEFINE(M_ECHO, "echo_buffer", "buffer for echo driver");

#define ECHO_CLEAR_BUFFER	_IO('E', 1)
#define ECHO_SET_BUFFER_SIZE	_IOW('E', 2, int)

static d_open_t		echo_open;
static d_close_t	echo_close;
static d_read_t		echo_read;
static d_write_t	echo_write;
static d_ioctl_t	echo_ioctl;

static struct cdevsw echo_cdevsw = {
	.d_version =	D_VERSION,
	.d_open =	echo_open,
	.d_close =	echo_close,
	.d_read =	echo_read,
	.d_write =	echo_write,
	.d_ioctl =	echo_ioctl,
	.d_name =	"echo"
};

typedef struct echo {
	int buffer_size;
	char *buffer;
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
	int amount;

	amount = MIN(uio->uio_resid,
	    (echo_message->buffer_size - 1 - uio->uio_offset > 0) ?
	     echo_message->buffer_size - 1 - uio->uio_offset : 0);
	if (amount == 0)
		return (error);

	error = uiomove(echo_message->buffer, amount, uio);
	if (error != 0) {
		uprintf("Write failed.\n");
		return (error);
	}

	echo_message->buffer[amount] = '\0';
	echo_message->length = amount;

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
echo_set_buffer_size(int size)
{
	int error = 0;

	if (echo_message->buffer_size == size)
		return (error);

	if (size >= 128 && size <= 512) {
		echo_message->buffer = realloc(echo_message->buffer, size,
		    M_ECHO, M_WAITOK);
		echo_message->buffer_size = size;

		if (echo_message->length >= size) {
			echo_message->length = size - 1;
			echo_message->buffer[size - 1] = '\0';
		}
	} else
		error = EINVAL;

	return (error);
}

static int
echo_ioctl(struct cdev *dev, u_long cmd, caddr_t data, int fflag,
    struct thread *td)
{
	int error = 0;

	switch (cmd) {
	case ECHO_CLEAR_BUFFER:
		memset(echo_message->buffer, '\0',
		    echo_message->buffer_size);
		echo_message->length = 0;
		uprintf("Buffer cleared.\n");
		break;
	case ECHO_SET_BUFFER_SIZE:
		error = echo_set_buffer_size(*(int *)data);
		if (error == 0)
			uprintf("Buffer resized.\n");
		break;
	default:
		error = ENOTTY;
		break;
	}

	return (error);
}

static int
echo_modevent(module_t mod __unused, int event, void *arg __unused)
{
	int error = 0;

	switch (event) {
	case MOD_LOAD:
		echo_message = malloc(sizeof(echo_t), M_ECHO, M_WAITOK);
		echo_message->buffer_size = 256;
		echo_message->buffer = malloc(echo_message->buffer_size,
		    M_ECHO, M_WAITOK);
		echo_dev = make_dev(&echo_cdevsw, 0, UID_ROOT, GID_WHEEL,
		    0600, "echo");
		uprintf("Echo driver loaded.\n");
		break;
	case MOD_UNLOAD:
		destroy_dev(echo_dev);
		free(echo_message->buffer, M_ECHO);
		free(echo_message, M_ECHO);
		uprintf("Echo driver unloaded.\n");
		break;
	default:
		error = EOPNOTSUPP;
		break;
	}

	return (error);
}

DEV_MODULE(echo, echo_modevent, NULL);
