/* $Id: pointless.c,v 1.4 2012/04/12 15:04:04 ghost Exp $ */

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
#include <sys/sysctl.h>

static long  a = 100;
static int   b = 200;
static char *c = "Are you suggesting coconuts migrate?";

static struct sysctl_ctx_list clist;
static struct sysctl_oid *poid;

static int
sysctl_pointless_procedure(SYSCTL_HANDLER_ARGS)
{
	char *buf = "Not at all. They could be carried.";
	return (sysctl_handle_string(oidp, buf, strlen(buf), req));
}

static int
pointless_modevent(module_t mod __unused, int event, void *arg __unused)
{
	int error = 0;

	switch (event) {
	case MOD_LOAD:
		sysctl_ctx_init(&clist);

		poid = SYSCTL_ADD_NODE(&clist,
		    SYSCTL_STATIC_CHILDREN(/* tree top */), OID_AUTO,
		    "example", CTLFLAG_RW, 0, "new top-level tree");
		if (poid == NULL) {
			uprintf("SYSCTL_ADD_NODE failed.\n");
			return (EINVAL);
		}
		SYSCTL_ADD_LONG(&clist, SYSCTL_CHILDREN(poid), OID_AUTO,
		    "long", CTLFLAG_RW, &a, "new long leaf");
		SYSCTL_ADD_INT(&clist, SYSCTL_CHILDREN(poid), OID_AUTO,
		    "int", CTLFLAG_RW, &b, 0, "new int leaf");

		poid = SYSCTL_ADD_NODE(&clist, SYSCTL_CHILDREN(poid),
		    OID_AUTO, "node", CTLFLAG_RW, 0,
		    "new tree under example");
		if (poid == NULL) {
			uprintf("SYSCTL_ADD_NODE failed.\n");
			return (EINVAL);
		}
		SYSCTL_ADD_PROC(&clist, SYSCTL_CHILDREN(poid), OID_AUTO,
		    "proc", CTLFLAG_RD, 0, 0, sysctl_pointless_procedure,
		    "A", "new proc leaf");

		poid = SYSCTL_ADD_NODE(&clist,
		    SYSCTL_STATIC_CHILDREN(_debug), OID_AUTO, "example",
		    CTLFLAG_RW, 0, "new tree under debug");
		if (poid == NULL) {
			uprintf("SYSCTL_ADD_NODE failed.\n");
			return (EINVAL);
		}
		SYSCTL_ADD_STRING(&clist, SYSCTL_CHILDREN(poid), OID_AUTO,
		    "string", CTLFLAG_RD, c, 0, "new string leaf");

		uprintf("Pointless module loaded.\n");
		break;
	case MOD_UNLOAD:
		if (sysctl_ctx_free(&clist)) {
			uprintf("sysctl_ctx_free failed.\n");
			return (ENOTEMPTY);
		}
		uprintf("Pointless module unloaded.\n");
		break;
	default:
		error = EOPNOTSUPP;
		break;
	}

	return (error);
}

static moduledata_t pointless_mod = {
	"pointless",
	pointless_modevent,
	NULL
};

DECLARE_MODULE(pointless, pointless_mod, SI_SUB_EXEC, SI_ORDER_ANY);
