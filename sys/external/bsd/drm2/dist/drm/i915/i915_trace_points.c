/*	$NetBSD$	*/

// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright © 2009 Intel Corporation
 *
 * Authors:
 *    Chris Wilson <chris@chris-wilson.co.uk>
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include "i915_drv.h"

#ifndef __CHECKER__
#define CREATE_TRACE_POINTS
#include "i915_trace.h"
#endif
