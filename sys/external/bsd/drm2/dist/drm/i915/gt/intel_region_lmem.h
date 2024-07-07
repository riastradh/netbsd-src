/*	$NetBSD: intel_region_lmem.h,v 1.2 2021/12/18 23:45:28 riastradh Exp $	*/

/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2019 Intel Corporation
 */

#ifndef __INTEL_REGION_LMEM_H
#define __INTEL_REGION_LMEM_H

struct intel_gt;

struct intel_memory_region *intel_gt_setup_lmem(struct intel_gt *gt);

#endif /* !__INTEL_REGION_LMEM_H */
