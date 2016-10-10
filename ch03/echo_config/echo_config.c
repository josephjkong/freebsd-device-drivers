/* $Id: echo_config.c,v 1.8 2012/04/12 15:02:04 ghost Exp $ */

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

#include <sys/types.h>
#include <sys/ioctl.h>

#include <err.h>
#include <fcntl.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define ECHO_CLEAR_BUFFER	_IO('E', 1)
#define ECHO_SET_BUFFER_SIZE	_IOW('E', 2, int)

static enum {UNSET, CLEAR, SETSIZE} action = UNSET;


/*
 * The usage statement: echo_config -c | -s size
 */

static void
usage()
{
	/*
	 * Arguments for this program are "either-or." That is,
	 * 'echo_config -c' and 'echo_config -s size' are valid; however,
	 * 'echo_config -c -s size' is invalid.
	 */

	fprintf(stderr, "usage: echo_config -c | -s size\n");
	exit(1);
}


/*
 * This program clears or resizes the memory buffer
 * found in /dev/echo.
 */

int
main(int argc, char *argv[])
{
	int ch, fd, i, size;
	char *p;

	/*
	 * Parse the command-line argument list to determine
	 * the correct course of action.
	 *
	 *    -c:      clear the memory buffer
	 *    -s size: resize the memory buffer to size.
	 */

	while ((ch = getopt(argc, argv, "cs:")) != -1)
		switch (ch) {
		case 'c':
			if (action != UNSET)
				usage();
			action = CLEAR;
			break;
		case 's':
			if (action != UNSET)
				usage();
			action = SETSIZE;
			size = (int)strtol(optarg, &p, 10);
			if (*p)
				errx(1, "illegal size -- %s", optarg);
			break;
		default:
			usage();
		}

	/*
	 * Perform the chosen action.
	 */

	if (action == CLEAR) {
		fd = open("/dev/echo", O_RDWR);
		if (fd < 0)
			err(1, "open(/dev/echo)");

		i = ioctl(fd, ECHO_CLEAR_BUFFER, NULL);
		if (i < 0)
			err(1, "ioctl(/dev/echo)");

		close (fd);
	} else if (action == SETSIZE) {
		fd = open("/dev/echo", O_RDWR);
		if (fd < 0)
			err(1, "open(/dev/echo)");

		i = ioctl(fd, ECHO_SET_BUFFER_SIZE, &size);
		if (i < 0)
			err(1, "ioctl(/dev/echo)");

		close (fd);
	} else
		usage();

	return (0);
}
