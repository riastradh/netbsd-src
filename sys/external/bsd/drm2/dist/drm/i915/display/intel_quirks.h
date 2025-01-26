/*	$NetBSD: intel_quirks.h,v 1.2 2021/12/18 23:45:30 riastradh Exp $	*/

/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2019 Intel Corporation
 */

#ifndef __INTEL_QUIRKS_H__
#define __INTEL_QUIRKS_H__

#include <linux/types.h>

struct drm_i915_private;

enum intel_quirk_id {
	QUIRK_BACKLIGHT_PRESENT,
	QUIRK_INCREASE_DDI_DISABLED_TIME,
	QUIRK_INCREASE_T12_DELAY,
	QUIRK_INVERT_BRIGHTNESS,
	QUIRK_LVDS_SSC_DISABLE,
	QUIRK_NO_PPS_BACKLIGHT_POWER_HOOK,
#ifdef __NetBSD__
	/* NetBSD hack to note version was called and thus mmap flags valid. */
	QUIRK_NETBSD_VERSION_CALLED,
#endif
};

void intel_init_quirks(struct drm_i915_private *i915);
bool intel_has_quirk(struct drm_i915_private *i915, enum intel_quirk_id quirk);

#endif /* __INTEL_QUIRKS_H__ */
