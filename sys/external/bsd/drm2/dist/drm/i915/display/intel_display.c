/*	$NetBSD: intel_display.c,v 1.12 2021/12/19 12:37:17 riastradh Exp $	*/

/*
 * Copyright © 2006-2007 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Authors:
 *	Eric Anholt <eric@anholt.net>
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: intel_display.c,v 1.12 2021/12/19 12:37:17 riastradh Exp $");

#include "intel_display.h"	/* for pipe_drmhack */

#include <linux/dma-resv.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string_helpers.h>

#include <drm/display/drm_dp_helper.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_atomic_uapi.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_rect.h>

#include "gem/i915_gem_lmem.h"
#include "gem/i915_gem_object.h"

#include "g4x_dp.h"
#include "g4x_hdmi.h"
#include "hsw_ips.h"
#include "i915_drv.h"
#include "i915_reg.h"
#include "i915_utils.h"
#include "i9xx_plane.h"
#include "i9xx_wm.h"
#include "intel_atomic.h"
#include "intel_atomic_plane.h"
#include "intel_audio.h"
#include "intel_bw.h"
#include "intel_cdclk.h"
#include "intel_clock_gating.h"
#include "intel_color.h"
#include "intel_crt.h"
#include "intel_crtc.h"
#include "intel_crtc_state_dump.h"
#include "intel_ddi.h"
#include "intel_de.h"
#include "intel_display_driver.h"
#include "intel_display_power.h"
#include "intel_display_types.h"
#include "intel_dmc.h"
#include "intel_dp.h"
#include "intel_dp_link_training.h"
#include "intel_dp_mst.h"
#include "intel_dpio_phy.h"
#include "intel_dpll.h"
#include "intel_dpll_mgr.h"
#include "intel_dpt.h"
#include "intel_drrs.h"
#include "intel_dsi.h"
#include "intel_dvo.h"
#include "intel_fb.h"
#include "intel_fbc.h"
#include "intel_fbdev.h"
#include "intel_fdi.h"
#include "intel_fifo_underrun.h"
#include "intel_frontbuffer.h"
#include "intel_hdmi.h"
#include "intel_hotplug.h"
#include "intel_lvds.h"
#include "intel_lvds_regs.h"
#include "intel_modeset_setup.h"
#include "intel_modeset_verify.h"
#include "intel_overlay.h"
#include "intel_panel.h"
#include "intel_pch_display.h"
#include "intel_pch_refclk.h"
#include "intel_pcode.h"
#include "intel_pipe_crc.h"
#include "intel_plane_initial.h"
#include "intel_pmdemand.h"
#include "intel_pps.h"
#include "intel_psr.h"
#include "intel_sdvo.h"
#include "intel_snps_phy.h"
#include "intel_tc.h"
#include "intel_tv.h"
#include "intel_vblank.h"
#include "intel_vdsc.h"
#include "intel_vdsc_regs.h"
#include "intel_vga.h"
#include "intel_vrr.h"
#include "intel_wm.h"
#include "skl_scaler.h"
#include "skl_universal_plane.h"
#include "skl_watermark.h"
#include "vlv_dsi.h"
#include "vlv_dsi_pll.h"
#include "vlv_dsi_regs.h"
#include "vlv_sideband.h"

#include <linux/nbsd-namespace.h>

static void intel_set_transcoder_timings(const struct intel_crtc_state *crtc_state);
static void intel_set_pipe_src_size(const struct intel_crtc_state *crtc_state);
static void hsw_set_transconf(const struct intel_crtc_state *crtc_state);
static void bdw_set_pipe_misc(const struct intel_crtc_state *crtc_state);

/* returns HPLL frequency in kHz */
int vlv_get_hpll_vco(struct drm_i915_private *dev_priv)
{
	int hpll_freq, vco_freq[] = { 800, 1600, 2000, 2400 };

	/* Obtain SKU information */
	hpll_freq = vlv_cck_read(dev_priv, CCK_FUSE_REG) &
		CCK_FUSE_HPLL_FREQ_MASK;

	return vco_freq[hpll_freq] * 1000;
}

int vlv_get_cck_clock(struct drm_i915_private *dev_priv,
		      const char *name, u32 reg, int ref_freq)
{
	u32 val;
	int divider;

	val = vlv_cck_read(dev_priv, reg);
	divider = val & CCK_FREQUENCY_VALUES;

	drm_WARN(&dev_priv->drm, (val & CCK_FREQUENCY_STATUS) !=
		 (divider << CCK_FREQUENCY_STATUS_SHIFT),
		 "%s change in progress\n", name);

	return DIV_ROUND_CLOSEST(ref_freq << 1, divider + 1);
}

int vlv_get_cck_clock_hpll(struct drm_i915_private *dev_priv,
			   const char *name, u32 reg)
{
	int hpll;

	vlv_cck_get(dev_priv);

	if (dev_priv->hpll_freq == 0)
		dev_priv->hpll_freq = vlv_get_hpll_vco(dev_priv);

	hpll = vlv_get_cck_clock(dev_priv, name, reg, dev_priv->hpll_freq);

	vlv_cck_put(dev_priv);

	return hpll;
}

void intel_update_czclk(struct drm_i915_private *dev_priv)
{
	if (!(IS_VALLEYVIEW(dev_priv) || IS_CHERRYVIEW(dev_priv)))
		return;

	dev_priv->czclk_freq = vlv_get_cck_clock_hpll(dev_priv, "czclk",
						      CCK_CZ_CLOCK_CONTROL);

	drm_dbg(&dev_priv->drm, "CZ clock rate: %d kHz\n",
		dev_priv->czclk_freq);
}

static bool is_hdr_mode(const struct intel_crtc_state *crtc_state)
{
	return (crtc_state->active_planes &
		~(icl_hdr_plane_mask() | BIT(PLANE_CURSOR))) == 0;
}

/* WA Display #0827: Gen9:all */
static void
skl_wa_827(struct drm_i915_private *dev_priv, enum pipe pipe, bool enable)
{
	if (enable)
		intel_de_rmw(dev_priv, CLKGATE_DIS_PSL(pipe),
			     0, DUPS1_GATING_DIS | DUPS2_GATING_DIS);
	else
		intel_de_rmw(dev_priv, CLKGATE_DIS_PSL(pipe),
			     DUPS1_GATING_DIS | DUPS2_GATING_DIS, 0);
}

/* Wa_2006604312:icl,ehl */
static void
icl_wa_scalerclkgating(struct drm_i915_private *dev_priv, enum pipe pipe,
		       bool enable)
{
	if (enable)
		intel_de_rmw(dev_priv, CLKGATE_DIS_PSL(pipe), 0, DPFR_GATING_DIS);
	else
		intel_de_rmw(dev_priv, CLKGATE_DIS_PSL(pipe), DPFR_GATING_DIS, 0);
}

/* Wa_1604331009:icl,jsl,ehl */
static void
icl_wa_cursorclkgating(struct drm_i915_private *dev_priv, enum pipe pipe,
		       bool enable)
{
	intel_de_rmw(dev_priv, CLKGATE_DIS_PSL(pipe), CURSOR_GATING_DIS,
		     enable ? CURSOR_GATING_DIS : 0);
}

static bool
is_trans_port_sync_slave(const struct intel_crtc_state *crtc_state)
{
	return crtc_state->master_transcoder != INVALID_TRANSCODER;
}

bool
is_trans_port_sync_master(const struct intel_crtc_state *crtc_state)
{
	return crtc_state->sync_mode_slaves_mask != 0;
}

bool
is_trans_port_sync_mode(const struct intel_crtc_state *crtc_state)
{
	return is_trans_port_sync_master(crtc_state) ||
		is_trans_port_sync_slave(crtc_state);
}

static enum pipe bigjoiner_master_pipe(const struct intel_crtc_state *crtc_state)
{
	return ffs(crtc_state->bigjoiner_pipes) - 1;
}

u8 intel_crtc_bigjoiner_slave_pipes(const struct intel_crtc_state *crtc_state)
{
<<<<<<< HEAD
	clock->m = clock->m1 * clock->m2;
	clock->p = clock->p1 * clock->p2;
	if (WARN_ON(clock->n == 0 || clock->p == 0))
		return 0;
	clock->vco = DIV_ROUND_CLOSEST(refclk * clock->m, clock->n);
	clock->dot = DIV_ROUND_CLOSEST(clock->vco, clock->p);

	return clock->dot / 5;
}

int chv_calc_dpll_params(int refclk, struct dpll *clock)
{
	clock->m = clock->m1 * clock->m2;
	clock->p = clock->p1 * clock->p2;
	if (WARN_ON(clock->n == 0 || clock->p == 0))
		return 0;
	clock->vco = DIV_ROUND_CLOSEST_ULL(mul_u32_u32(refclk, clock->m),
					   clock->n << 22);
	clock->dot = DIV_ROUND_CLOSEST(clock->vco, clock->p);

	return clock->dot / 5;
}

#define INTELPllInvalid(s)   do { /* DRM_DEBUG(s); */ return false; } while (0)

/*
 * Returns whether the given set of divisors are valid for a given refclk with
 * the given connectors.
 */
static bool intel_PLL_is_valid(struct drm_i915_private *dev_priv,
			       const struct intel_limit *limit,
			       const struct dpll *clock)
{
	if (clock->n   < limit->n.min   || limit->n.max   < clock->n)
		INTELPllInvalid("n out of range\n");
	if (clock->p1  < limit->p1.min  || limit->p1.max  < clock->p1)
		INTELPllInvalid("p1 out of range\n");
	if (clock->m2  < limit->m2.min  || limit->m2.max  < clock->m2)
		INTELPllInvalid("m2 out of range\n");
	if (clock->m1  < limit->m1.min  || limit->m1.max  < clock->m1)
		INTELPllInvalid("m1 out of range\n");

	if (!IS_PINEVIEW(dev_priv) && !IS_VALLEYVIEW(dev_priv) &&
	    !IS_CHERRYVIEW(dev_priv) && !IS_GEN9_LP(dev_priv))
		if (clock->m1 <= clock->m2)
			INTELPllInvalid("m1 <= m2\n");

	if (!IS_VALLEYVIEW(dev_priv) && !IS_CHERRYVIEW(dev_priv) &&
	    !IS_GEN9_LP(dev_priv)) {
		if (clock->p < limit->p.min || limit->p.max < clock->p)
			INTELPllInvalid("p out of range\n");
		if (clock->m < limit->m.min || limit->m.max < clock->m)
			INTELPllInvalid("m out of range\n");
	}

	if (clock->vco < limit->vco.min || limit->vco.max < clock->vco)
		INTELPllInvalid("vco out of range\n");
	/* XXX: We may need to be checking "Dot clock" depending on the multiplier,
	 * connector, etc., rather than just a single range.
	 */
	if (clock->dot < limit->dot.min || limit->dot.max < clock->dot)
		INTELPllInvalid("dot out of range\n");

	return true;
}

static int
i9xx_select_p2_div(const struct intel_limit *limit,
		   const struct intel_crtc_state *crtc_state,
		   int target)
{
	struct drm_i915_private *dev_priv = to_i915(crtc_state->uapi.crtc->dev);

	if (intel_crtc_has_type(crtc_state, INTEL_OUTPUT_LVDS)) {
		/*
		 * For LVDS just rely on its current settings for dual-channel.
		 * We haven't figured out how to reliably set up different
		 * single/dual channel state, if we even can.
		 */
		if (intel_is_dual_link_lvds(dev_priv))
			return limit->p2.p2_fast;
		else
			return limit->p2.p2_slow;
	} else {
		if (target < limit->p2.dot_limit)
			return limit->p2.p2_slow;
		else
			return limit->p2.p2_fast;
	}
}

/*
 * Returns a set of divisors for the desired target clock with the given
 * refclk, or FALSE.  The returned values represent the clock equation:
 * reflck * (5 * (m1 + 2) + (m2 + 2)) / (n + 2) / p1 / p2.
 *
 * Target and reference clocks are specified in kHz.
 *
 * If match_clock is provided, then best_clock P divider must match the P
 * divider from @match_clock used for LVDS downclocking.
 */
static bool
i9xx_find_best_dpll(const struct intel_limit *limit,
		    struct intel_crtc_state *crtc_state,
		    int target, int refclk, struct dpll *match_clock,
		    struct dpll *best_clock)
{
	struct drm_device *dev = crtc_state->uapi.crtc->dev;
	struct dpll clock;
	int err = target;

	memset(best_clock, 0, sizeof(*best_clock));

	clock.p2 = i9xx_select_p2_div(limit, crtc_state, target);

	for (clock.m1 = limit->m1.min; clock.m1 <= limit->m1.max;
	     clock.m1++) {
		for (clock.m2 = limit->m2.min;
		     clock.m2 <= limit->m2.max; clock.m2++) {
			if (clock.m2 >= clock.m1)
				break;
			for (clock.n = limit->n.min;
			     clock.n <= limit->n.max; clock.n++) {
				for (clock.p1 = limit->p1.min;
					clock.p1 <= limit->p1.max; clock.p1++) {
					int this_err;

					i9xx_calc_dpll_params(refclk, &clock);
					if (!intel_PLL_is_valid(to_i915(dev),
								limit,
								&clock))
						continue;
					if (match_clock &&
					    clock.p != match_clock->p)
						continue;

					this_err = abs(clock.dot - target);
					if (this_err < err) {
						*best_clock = clock;
						err = this_err;
					}
				}
			}
		}
	}

	return (err != target);
}

/*
 * Returns a set of divisors for the desired target clock with the given
 * refclk, or FALSE.  The returned values represent the clock equation:
 * reflck * (5 * (m1 + 2) + (m2 + 2)) / (n + 2) / p1 / p2.
 *
 * Target and reference clocks are specified in kHz.
 *
 * If match_clock is provided, then best_clock P divider must match the P
 * divider from @match_clock used for LVDS downclocking.
 */
static bool
pnv_find_best_dpll(const struct intel_limit *limit,
		   struct intel_crtc_state *crtc_state,
		   int target, int refclk, struct dpll *match_clock,
		   struct dpll *best_clock)
{
	struct drm_device *dev = crtc_state->uapi.crtc->dev;
	struct dpll clock;
	int err = target;

	memset(best_clock, 0, sizeof(*best_clock));

	clock.p2 = i9xx_select_p2_div(limit, crtc_state, target);

	for (clock.m1 = limit->m1.min; clock.m1 <= limit->m1.max;
	     clock.m1++) {
		for (clock.m2 = limit->m2.min;
		     clock.m2 <= limit->m2.max; clock.m2++) {
			for (clock.n = limit->n.min;
			     clock.n <= limit->n.max; clock.n++) {
				for (clock.p1 = limit->p1.min;
					clock.p1 <= limit->p1.max; clock.p1++) {
					int this_err;

					pnv_calc_dpll_params(refclk, &clock);
					if (!intel_PLL_is_valid(to_i915(dev),
								limit,
								&clock))
						continue;
					if (match_clock &&
					    clock.p != match_clock->p)
						continue;

					this_err = abs(clock.dot - target);
					if (this_err < err) {
						*best_clock = clock;
						err = this_err;
					}
				}
			}
		}
	}

	return (err != target);
}

/*
 * Returns a set of divisors for the desired target clock with the given
 * refclk, or FALSE.  The returned values represent the clock equation:
 * reflck * (5 * (m1 + 2) + (m2 + 2)) / (n + 2) / p1 / p2.
 *
 * Target and reference clocks are specified in kHz.
 *
 * If match_clock is provided, then best_clock P divider must match the P
 * divider from @match_clock used for LVDS downclocking.
 */
static bool
g4x_find_best_dpll(const struct intel_limit *limit,
		   struct intel_crtc_state *crtc_state,
		   int target, int refclk, struct dpll *match_clock,
		   struct dpll *best_clock)
{
	struct drm_device *dev = crtc_state->uapi.crtc->dev;
	struct dpll clock;
	int max_n;
	bool found = false;
	/* approximately equals target * 0.00585 */
	int err_most = (target >> 8) + (target >> 9);

	memset(best_clock, 0, sizeof(*best_clock));

	clock.p2 = i9xx_select_p2_div(limit, crtc_state, target);

	max_n = limit->n.max;
	/* based on hardware requirement, prefer smaller n to precision */
	for (clock.n = limit->n.min; clock.n <= max_n; clock.n++) {
		/* based on hardware requirement, prefere larger m1,m2 */
		for (clock.m1 = limit->m1.max;
		     clock.m1 >= limit->m1.min; clock.m1--) {
			for (clock.m2 = limit->m2.max;
			     clock.m2 >= limit->m2.min; clock.m2--) {
				for (clock.p1 = limit->p1.max;
				     clock.p1 >= limit->p1.min; clock.p1--) {
					int this_err;

					i9xx_calc_dpll_params(refclk, &clock);
					if (!intel_PLL_is_valid(to_i915(dev),
								limit,
								&clock))
						continue;

					this_err = abs(clock.dot - target);
					if (this_err < err_most) {
						*best_clock = clock;
						err_most = this_err;
						max_n = clock.n;
						found = true;
					}
				}
			}
		}
	}
	return found;
}

/*
 * Check if the calculated PLL configuration is more optimal compared to the
 * best configuration and error found so far. Return the calculated error.
 */
static bool vlv_PLL_is_optimal(struct drm_device *dev, int target_freq,
			       const struct dpll *calculated_clock,
			       const struct dpll *best_clock,
			       unsigned int best_error_ppm,
			       unsigned int *error_ppm)
{
	/*
	 * For CHV ignore the error and consider only the P value.
	 * Prefer a bigger P value based on HW requirements.
	 */
	if (IS_CHERRYVIEW(to_i915(dev))) {
		*error_ppm = 0;

		return calculated_clock->p > best_clock->p;
	}

	if (WARN_ON_ONCE(!target_freq))
		return false;

	*error_ppm = div_u64(1000000ULL *
				abs(target_freq - calculated_clock->dot),
			     target_freq);
	/*
	 * Prefer a better P value over a better (smaller) error if the error
	 * is small. Ensure this preference for future configurations too by
	 * setting the error to 0.
	 */
	if (*error_ppm < 100 && calculated_clock->p > best_clock->p) {
		*error_ppm = 0;

		return true;
	}

	return *error_ppm + 10 < best_error_ppm;
}

/*
 * Returns a set of divisors for the desired target clock with the given
 * refclk, or FALSE.  The returned values represent the clock equation:
 * reflck * (5 * (m1 + 2) + (m2 + 2)) / (n + 2) / p1 / p2.
 */
static bool
vlv_find_best_dpll(const struct intel_limit *limit,
		   struct intel_crtc_state *crtc_state,
		   int target, int refclk, struct dpll *match_clock,
		   struct dpll *best_clock)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_device *dev = crtc->base.dev;
	struct dpll clock;
	unsigned int bestppm = 1000000;
	/* min update 19.2 MHz */
	int max_n = min(limit->n.max, refclk / 19200);
	bool found = false;

	target *= 5; /* fast clock */

	memset(best_clock, 0, sizeof(*best_clock));

	/* based on hardware requirement, prefer smaller n to precision */
	for (clock.n = limit->n.min; clock.n <= max_n; clock.n++) {
		for (clock.p1 = limit->p1.max; clock.p1 >= limit->p1.min; clock.p1--) {
			for (clock.p2 = limit->p2.p2_fast; clock.p2 >= limit->p2.p2_slow;
			     clock.p2 -= clock.p2 > 10 ? 2 : 1) {
				clock.p = clock.p1 * clock.p2;
				/* based on hardware requirement, prefer bigger m1,m2 values */
				for (clock.m1 = limit->m1.min; clock.m1 <= limit->m1.max; clock.m1++) {
					unsigned int ppm = 0; /*XXXGCC*/

					clock.m2 = DIV_ROUND_CLOSEST(target * clock.p * clock.n,
								     refclk * clock.m1);

					vlv_calc_dpll_params(refclk, &clock);

					if (!intel_PLL_is_valid(to_i915(dev),
								limit,
								&clock))
						continue;

					if (!vlv_PLL_is_optimal(dev, target,
								&clock,
								best_clock,
								bestppm, &ppm))
						continue;

					*best_clock = clock;
					bestppm = ppm;
					found = true;
				}
			}
		}
	}

	return found;
}

/*
 * Returns a set of divisors for the desired target clock with the given
 * refclk, or FALSE.  The returned values represent the clock equation:
 * reflck * (5 * (m1 + 2) + (m2 + 2)) / (n + 2) / p1 / p2.
 */
static bool
chv_find_best_dpll(const struct intel_limit *limit,
		   struct intel_crtc_state *crtc_state,
		   int target, int refclk, struct dpll *match_clock,
		   struct dpll *best_clock)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_device *dev = crtc->base.dev;
	unsigned int best_error_ppm;
	struct dpll clock;
	u64 m2;
	int found = false;

	memset(best_clock, 0, sizeof(*best_clock));
	best_error_ppm = 1000000;

	/*
	 * Based on hardware doc, the n always set to 1, and m1 always
	 * set to 2.  If requires to support 200Mhz refclk, we need to
	 * revisit this because n may not 1 anymore.
	 */
	clock.n = 1, clock.m1 = 2;
	target *= 5;	/* fast clock */

	for (clock.p1 = limit->p1.max; clock.p1 >= limit->p1.min; clock.p1--) {
		for (clock.p2 = limit->p2.p2_fast;
				clock.p2 >= limit->p2.p2_slow;
				clock.p2 -= clock.p2 > 10 ? 2 : 1) {
			unsigned int error_ppm = 0; /*XXXGCC*/

			clock.p = clock.p1 * clock.p2;

			m2 = DIV_ROUND_CLOSEST_ULL(mul_u32_u32(target, clock.p * clock.n) << 22,
						   refclk * clock.m1);

			if (m2 > INT_MAX/clock.m1)
				continue;

			clock.m2 = m2;

			chv_calc_dpll_params(refclk, &clock);

			if (!intel_PLL_is_valid(to_i915(dev), limit, &clock))
				continue;

			if (!vlv_PLL_is_optimal(dev, target, &clock, best_clock,
						best_error_ppm, &error_ppm))
				continue;

			*best_clock = clock;
			best_error_ppm = error_ppm;
			found = true;
		}
	}

	return found;
}

bool bxt_find_best_dpll(struct intel_crtc_state *crtc_state,
			struct dpll *best_clock)
{
	int refclk = 100000;
	const struct intel_limit *limit = &intel_limits_bxt;

	return chv_find_best_dpll(limit, crtc_state,
				  crtc_state->port_clock, refclk,
				  NULL, best_clock);
}

static bool pipe_scanline_is_moving(struct drm_i915_private *dev_priv,
				    enum pipe pipe)
{
	i915_reg_t reg = PIPEDSL(pipe);
	u32 line1, line2;
	u32 line_mask;

	if (IS_GEN(dev_priv, 2))
		line_mask = DSL_LINEMASK_GEN2;
=======
	if (crtc_state->bigjoiner_pipes)
		return crtc_state->bigjoiner_pipes & ~BIT(bigjoiner_master_pipe(crtc_state));
>>>>>>> vendor/linux-drm-v6.6.35
	else
		return 0;
}

bool intel_crtc_is_bigjoiner_slave(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);

	return crtc_state->bigjoiner_pipes &&
		crtc->pipe != bigjoiner_master_pipe(crtc_state);
}

bool intel_crtc_is_bigjoiner_master(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);

	return crtc_state->bigjoiner_pipes &&
		crtc->pipe == bigjoiner_master_pipe(crtc_state);
}

static int intel_bigjoiner_num_pipes(const struct intel_crtc_state *crtc_state)
{
	return hweight8(crtc_state->bigjoiner_pipes);
}

struct intel_crtc *intel_master_crtc(const struct intel_crtc_state *crtc_state)
{
	struct drm_i915_private *i915 = to_i915(crtc_state->uapi.crtc->dev);

	if (intel_crtc_is_bigjoiner_slave(crtc_state))
		return intel_crtc_for_pipe(i915, bigjoiner_master_pipe(crtc_state));
	else
		return to_intel_crtc(crtc_state->uapi.crtc);
}

static void
intel_wait_for_pipe_off(const struct intel_crtc_state *old_crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(old_crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);

	if (DISPLAY_VER(dev_priv) >= 4) {
		enum transcoder cpu_transcoder = old_crtc_state->cpu_transcoder;

		/* Wait for the Pipe State to go off */
		if (intel_de_wait_for_clear(dev_priv, TRANSCONF(cpu_transcoder),
					    TRANSCONF_STATE_ENABLE, 100))
			drm_WARN(&dev_priv->drm, 1, "pipe_off wait timed out\n");
	} else {
		intel_wait_for_pipe_scanline_stopped(crtc);
	}
}

void assert_transcoder(struct drm_i915_private *dev_priv,
		       enum transcoder cpu_transcoder, bool state)
{
	bool cur_state;
	enum intel_display_power_domain power_domain;
	intel_wakeref_t wakeref;

	/* we keep both pipes enabled on 830 */
	if (IS_I830(dev_priv))
		state = true;

	power_domain = POWER_DOMAIN_TRANSCODER(cpu_transcoder);
	wakeref = intel_display_power_get_if_enabled(dev_priv, power_domain);
	if (wakeref) {
		u32 val = intel_de_read(dev_priv, TRANSCONF(cpu_transcoder));
		cur_state = !!(val & TRANSCONF_ENABLE);

		intel_display_power_put(dev_priv, power_domain, wakeref);
	} else {
		cur_state = false;
	}

	I915_STATE_WARN(dev_priv, cur_state != state,
			"transcoder %s assertion failure (expected %s, current %s)\n",
			transcoder_name(cpu_transcoder), str_on_off(state),
			str_on_off(cur_state));
}

static void assert_plane(struct intel_plane *plane, bool state)
{
	struct drm_i915_private *i915 = to_i915(plane->base.dev);
	enum pipe pipe;
	bool cur_state;

	cur_state = plane->get_hw_state(plane, &pipe);

	I915_STATE_WARN(i915, cur_state != state,
			"%s assertion failure (expected %s, current %s)\n",
			plane->base.name, str_on_off(state),
			str_on_off(cur_state));
}

#define assert_plane_enabled(p) assert_plane(p, true)
#define assert_plane_disabled(p) assert_plane(p, false)

static void assert_planes_disabled(struct intel_crtc *crtc)
{
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	struct intel_plane *plane;

	for_each_intel_plane_on_crtc(&dev_priv->drm, crtc, plane)
		assert_plane_disabled(plane);
}

void vlv_wait_port_ready(struct drm_i915_private *dev_priv,
			 struct intel_digital_port *dig_port,
			 unsigned int expected_mask)
{
	u32 port_mask;
	i915_reg_t dpll_reg;

	switch (dig_port->base.port) {
	default:
		MISSING_CASE(dig_port->base.port);
		fallthrough;
	case PORT_B:
		port_mask = DPLL_PORTB_READY_MASK;
		dpll_reg = DPLL(0);
		break;
	case PORT_C:
		port_mask = DPLL_PORTC_READY_MASK;
		dpll_reg = DPLL(0);
		expected_mask <<= 4;
		break;
	case PORT_D:
		port_mask = DPLL_PORTD_READY_MASK;
		dpll_reg = DPIO_PHY_STATUS;
		break;
	}

	if (intel_de_wait_for_register(dev_priv, dpll_reg,
				       port_mask, expected_mask, 1000))
		drm_WARN(&dev_priv->drm, 1,
			 "timed out waiting for [ENCODER:%d:%s] port ready: got 0x%x, expected 0x%x\n",
			 dig_port->base.base.base.id, dig_port->base.base.name,
			 intel_de_read(dev_priv, dpll_reg) & port_mask,
			 expected_mask);
}

void intel_enable_transcoder(const struct intel_crtc_state *new_crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(new_crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum transcoder cpu_transcoder = new_crtc_state->cpu_transcoder;
	enum pipe pipe = crtc->pipe;
	i915_reg_t reg;
	u32 val;

	drm_dbg_kms(&dev_priv->drm, "enabling pipe %c\n", pipe_name(pipe));

	assert_planes_disabled(crtc);

	/*
	 * A pipe without a PLL won't actually be able to drive bits from
	 * a plane.  On ILK+ the pipe PLLs are integrated, so we don't
	 * need the check.
	 */
	if (HAS_GMCH(dev_priv)) {
		if (intel_crtc_has_type(new_crtc_state, INTEL_OUTPUT_DSI))
			assert_dsi_pll_enabled(dev_priv);
		else
			assert_pll_enabled(dev_priv, pipe);
	} else {
		if (new_crtc_state->has_pch_encoder) {
			/* if driving the PCH, we need FDI enabled */
			assert_fdi_rx_pll_enabled(dev_priv,
						  intel_crtc_pch_transcoder(crtc));
			assert_fdi_tx_pll_enabled(dev_priv,
						  (enum pipe) cpu_transcoder);
		}
		/* FIXME: assert CPU port conditions for SNB+ */
	}

	/* Wa_22012358565:adl-p */
	if (DISPLAY_VER(dev_priv) == 13)
		intel_de_rmw(dev_priv, PIPE_ARB_CTL(pipe),
			     0, PIPE_ARB_USE_PROG_SLOTS);

	reg = TRANSCONF(cpu_transcoder);
	val = intel_de_read(dev_priv, reg);
	if (val & TRANSCONF_ENABLE) {
		/* we keep both pipes enabled on 830 */
		drm_WARN_ON(&dev_priv->drm, !IS_I830(dev_priv));
		return;
	}

	intel_de_write(dev_priv, reg, val | TRANSCONF_ENABLE);
	intel_de_posting_read(dev_priv, reg);

	/*
	 * Until the pipe starts PIPEDSL reads will return a stale value,
	 * which causes an apparent vblank timestamp jump when PIPEDSL
	 * resets to its proper value. That also messes up the frame count
	 * when it's derived from the timestamps. So let's wait for the
	 * pipe to start properly before we call drm_crtc_vblank_on()
	 */
	if (intel_crtc_max_vblank_count(new_crtc_state) == 0)
		intel_wait_for_pipe_scanline_moving(crtc);
}

void intel_disable_transcoder(const struct intel_crtc_state *old_crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(old_crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum transcoder cpu_transcoder = old_crtc_state->cpu_transcoder;
	enum pipe pipe = crtc->pipe;
	i915_reg_t reg;
	u32 val;

	drm_dbg_kms(&dev_priv->drm, "disabling pipe %c\n", pipe_name(pipe));

	/*
	 * Make sure planes won't keep trying to pump pixels to us,
	 * or we might hang the display.
	 */
	assert_planes_disabled(crtc);

	reg = TRANSCONF(cpu_transcoder);
	val = intel_de_read(dev_priv, reg);
	if ((val & TRANSCONF_ENABLE) == 0)
		return;

	/*
	 * Double wide has implications for planes
	 * so best keep it disabled when not needed.
	 */
	if (old_crtc_state->double_wide)
		val &= ~TRANSCONF_DOUBLE_WIDE;

	/* Don't disable pipe or pipe PLLs if needed */
	if (!IS_I830(dev_priv))
		val &= ~TRANSCONF_ENABLE;

	if (DISPLAY_VER(dev_priv) >= 14)
		intel_de_rmw(dev_priv, MTL_CHICKEN_TRANS(cpu_transcoder),
			     FECSTALL_DIS_DPTSTREAM_DPTTG, 0);
	else if (DISPLAY_VER(dev_priv) >= 12)
		intel_de_rmw(dev_priv, CHICKEN_TRANS(cpu_transcoder),
			     FECSTALL_DIS_DPTSTREAM_DPTTG, 0);

	intel_de_write(dev_priv, reg, val);
	if ((val & TRANSCONF_ENABLE) == 0)
		intel_wait_for_pipe_off(old_crtc_state);
}

unsigned int intel_rotation_info_size(const struct intel_rotation_info *rot_info)
{
	unsigned int size = 0;
	int i;

	for (i = 0 ; i < ARRAY_SIZE(rot_info->plane); i++)
		size += rot_info->plane[i].dst_stride * rot_info->plane[i].width;

	return size;
}

unsigned int intel_remapped_info_size(const struct intel_remapped_info *rem_info)
{
	unsigned int size = 0;
	int i;

	for (i = 0 ; i < ARRAY_SIZE(rem_info->plane); i++) {
		unsigned int plane_size;

		if (rem_info->plane[i].linear)
			plane_size = rem_info->plane[i].size;
		else
			plane_size = rem_info->plane[i].dst_stride * rem_info->plane[i].height;

		if (plane_size == 0)
			continue;

		if (rem_info->plane_alignment)
			size = ALIGN(size, rem_info->plane_alignment);

		size += plane_size;
	}

	return size;
}

<<<<<<< HEAD
static void
intel_fill_fb_ggtt_view(struct i915_ggtt_view *view,
			const struct drm_framebuffer *fb,
			unsigned int rotation)
{
	view->type = I915_GGTT_VIEW_NORMAL;
	if (drm_rotation_90_or_270(rotation)) {
		view->type = I915_GGTT_VIEW_ROTATED;
		view->rotated = to_intel_framebuffer((struct drm_framebuffer *)__UNCONST(fb))->rot_info;
	}
}

static unsigned int intel_cursor_alignment(const struct drm_i915_private *dev_priv)
{
	if (IS_I830(dev_priv))
		return 16 * 1024;
	else if (IS_I85X(dev_priv))
		return 256;
	else if (IS_I845G(dev_priv) || IS_I865G(dev_priv))
		return 32;
	else
		return 4 * 1024;
}

static unsigned int intel_linear_alignment(const struct drm_i915_private *dev_priv)
{
	if (INTEL_GEN(dev_priv) >= 9)
		return 256 * 1024;
	else if (IS_I965G(dev_priv) || IS_I965GM(dev_priv) ||
		 IS_VALLEYVIEW(dev_priv) || IS_CHERRYVIEW(dev_priv))
		return 128 * 1024;
	else if (INTEL_GEN(dev_priv) >= 4)
		return 4 * 1024;
	else
		return 0;
}

static unsigned int intel_surf_alignment(const struct drm_framebuffer *fb,
					 int color_plane)
{
	struct drm_i915_private *dev_priv = to_i915(fb->dev);

	/* AUX_DIST needs only 4K alignment */
	if ((INTEL_GEN(dev_priv) < 12 && is_aux_plane(fb, color_plane)) ||
	    is_ccs_plane(fb, color_plane))
		return 4096;

	switch (fb->modifier) {
	case DRM_FORMAT_MOD_LINEAR:
		return intel_linear_alignment(dev_priv);
	case I915_FORMAT_MOD_X_TILED:
		if (INTEL_GEN(dev_priv) >= 9)
			return 256 * 1024;
		return 0;
	case I915_FORMAT_MOD_Y_TILED_GEN12_MC_CCS:
		if (is_semiplanar_uv_plane(fb, color_plane))
			return intel_tile_row_size(fb, color_plane);
		/* Fall-through */
	case I915_FORMAT_MOD_Y_TILED_GEN12_RC_CCS:
		return 16 * 1024;
	case I915_FORMAT_MOD_Y_TILED_CCS:
	case I915_FORMAT_MOD_Yf_TILED_CCS:
	case I915_FORMAT_MOD_Y_TILED:
		if (INTEL_GEN(dev_priv) >= 12 &&
		    is_semiplanar_uv_plane(fb, color_plane))
			return intel_tile_row_size(fb, color_plane);
		/* Fall-through */
	case I915_FORMAT_MOD_Yf_TILED:
		return 1 * 1024 * 1024;
	default:
		MISSING_CASE(fb->modifier);
		return 0;
	}
}

static bool intel_plane_uses_fence(const struct intel_plane_state *plane_state)
=======
bool intel_plane_uses_fence(const struct intel_plane_state *plane_state)
>>>>>>> vendor/linux-drm-v6.6.35
{
	struct intel_plane *plane = to_intel_plane(plane_state->uapi.plane);
	struct drm_i915_private *dev_priv = to_i915(plane->base.dev);

<<<<<<< HEAD
	return INTEL_GEN(dev_priv) < 4 ||
		(plane->has_fbc &&
		 plane_state->view.type == I915_GGTT_VIEW_NORMAL);
}

struct i915_vma *
intel_pin_and_fence_fb_obj(struct drm_framebuffer *fb,
			   const struct i915_ggtt_view *view,
			   bool uses_fence,
			   unsigned long *out_flags)
{
	struct drm_device *dev = fb->dev;
	struct drm_i915_private *dev_priv = to_i915(dev);
	struct drm_i915_gem_object *obj = intel_fb_obj(fb);
	intel_wakeref_t wakeref;
	struct i915_vma *vma;
	unsigned int pinctl;
	u32 alignment;

	if (WARN_ON(!i915_gem_object_is_framebuffer(obj)))
		return ERR_PTR(-EINVAL);

	alignment = intel_surf_alignment(fb, 0);
	if (WARN_ON(alignment && !is_power_of_2(alignment)))
		return ERR_PTR(-EINVAL);

	/* Note that the w/a also requires 64 PTE of padding following the
	 * bo. We currently fill all unused PTE with the shadow page and so
	 * we should always have valid PTE following the scanout preventing
	 * the VT-d warning.
	 */
	if (intel_scanout_needs_vtd_wa(dev_priv) && alignment < 256 * 1024)
		alignment = 256 * 1024;

	/*
	 * Global gtt pte registers are special registers which actually forward
	 * writes to a chunk of system memory. Which means that there is no risk
	 * that the register values disappear as soon as we call
	 * intel_runtime_pm_put(), so it is correct to wrap only the
	 * pin/unpin/fence and not more.
	 */
	wakeref = intel_runtime_pm_get(&dev_priv->runtime_pm);

	atomic_inc(&dev_priv->gpu_error.pending_fb_pin);

	/*
	 * Valleyview is definitely limited to scanning out the first
	 * 512MiB. Lets presume this behaviour was inherited from the
	 * g4x display engine and that all earlier gen are similarly
	 * limited. Testing suggests that it is a little more
	 * complicated than this. For example, Cherryview appears quite
	 * happy to scanout from anywhere within its global aperture.
	 */
	pinctl = 0;
	if (HAS_GMCH(dev_priv))
		pinctl |= PIN_MAPPABLE;

	vma = i915_gem_object_pin_to_display_plane(obj,
						   alignment, view, pinctl);
	if (IS_ERR(vma))
		goto err;

	if (uses_fence && i915_vma_is_map_and_fenceable(vma)) {
		int ret;

		/*
		 * Install a fence for tiled scan-out. Pre-i965 always needs a
		 * fence, whereas 965+ only requires a fence if using
		 * framebuffer compression.  For simplicity, we always, when
		 * possible, install a fence as the cost is not that onerous.
		 *
		 * If we fail to fence the tiled scanout, then either the
		 * modeset will reject the change (which is highly unlikely as
		 * the affected systems, all but one, do not have unmappable
		 * space) or we will not be able to enable full powersaving
		 * techniques (also likely not to apply due to various limits
		 * FBC and the like impose on the size of the buffer, which
		 * presumably we violated anyway with this unmappable buffer).
		 * Anyway, it is presumably better to stumble onwards with
		 * something and try to run the system in a "less than optimal"
		 * mode that matches the user configuration.
		 */
		ret = i915_vma_pin_fence(vma);
		if (ret != 0 && INTEL_GEN(dev_priv) < 4) {
			i915_gem_object_unpin_from_display_plane(vma);
			vma = ERR_PTR(ret);
			goto err;
		}

		if (ret == 0 && vma->fence)
			*out_flags |= PLANE_HAS_FENCE;
	}

	i915_vma_get(vma);
err:
	atomic_dec(&dev_priv->gpu_error.pending_fb_pin);
	intel_runtime_pm_put(&dev_priv->runtime_pm, wakeref);
	return vma;
}

void intel_unpin_fb_vma(struct i915_vma *vma, unsigned long flags)
{
	i915_gem_object_lock(vma->obj);
	if (flags & PLANE_HAS_FENCE)
		i915_vma_unpin_fence(vma);
	i915_gem_object_unpin_from_display_plane(vma);
	i915_gem_object_unlock(vma->obj);

	i915_vma_put(vma);
}

static int intel_fb_pitch(const struct drm_framebuffer *fb, int color_plane,
			  unsigned int rotation)
{
	if (drm_rotation_90_or_270(rotation))
		return to_intel_framebuffer((struct drm_framebuffer *)__UNCONST(fb))->rotated[color_plane].pitch;
	else
		return fb->pitches[color_plane];
=======
	return DISPLAY_VER(dev_priv) < 4 ||
		(plane->fbc &&
		 plane_state->view.gtt.type == I915_GTT_VIEW_NORMAL);
>>>>>>> vendor/linux-drm-v6.6.35
}

/*
 * Convert the x/y offsets into a linear offset.
 * Only valid with 0/180 degree rotation, which is fine since linear
 * offset is only used with linear buffers on pre-hsw and tiled buffers
 * with gen2/3, and 90/270 degree rotations isn't supported on any of them.
 */
u32 intel_fb_xy_to_linear(int x, int y,
			  const struct intel_plane_state *state,
			  int color_plane)
{
	const struct drm_framebuffer *fb = state->hw.fb;
	unsigned int cpp = fb->format->cpp[color_plane];
	unsigned int pitch = state->view.color_plane[color_plane].mapping_stride;

	return y * pitch + x * cpp;
}

/*
 * Add the x/y offsets derived from fb->offsets[] to the user
 * specified plane src x/y offsets. The resulting x/y offsets
 * specify the start of scanout from the beginning of the gtt mapping.
 */
void intel_add_fb_offsets(int *x, int *y,
			  const struct intel_plane_state *state,
			  int color_plane)

{
	*x += state->view.color_plane[color_plane].x;
	*y += state->view.color_plane[color_plane].y;
}

u32 intel_plane_fb_max_stride(struct drm_i915_private *dev_priv,
			      u32 pixel_format, u64 modifier)
{
	struct intel_crtc *crtc;
	struct intel_plane *plane;

	if (!HAS_DISPLAY(dev_priv))
		return 0;

	/*
	 * We assume the primary plane for pipe A has
	 * the highest stride limits of them all,
	 * if in case pipe A is disabled, use the first pipe from pipe_mask.
	 */
	crtc = intel_first_crtc(dev_priv);
	if (!crtc)
		return 0;

	plane = to_intel_plane(crtc->base.primary);

	return plane->max_stride(plane, pixel_format, modifier,
				 DRM_MODE_ROTATE_0);
}

<<<<<<< HEAD
static
u32 intel_fb_max_stride(struct drm_i915_private *dev_priv,
			u32 pixel_format, u64 modifier)
{
	/*
	 * Arbitrary limit for gen4+ chosen to match the
	 * render engine max stride.
	 *
	 * The new CCS hash mode makes remapping impossible
	 */
	if (!is_ccs_modifier(modifier)) {
		if (INTEL_GEN(dev_priv) >= 7)
			return 256*1024;
		else if (INTEL_GEN(dev_priv) >= 4)
			return 128*1024;
	}

	return intel_plane_fb_max_stride(dev_priv, pixel_format, modifier);
}

static u32
intel_fb_stride_alignment(const struct drm_framebuffer *fb, int color_plane)
{
	struct drm_i915_private *dev_priv = to_i915(fb->dev);
	u32 tile_width;

	if (is_surface_linear(fb, color_plane)) {
		u32 max_stride = intel_plane_fb_max_stride(dev_priv,
							   fb->format->format,
							   fb->modifier);

		/*
		 * To make remapping with linear generally feasible
		 * we need the stride to be page aligned.
		 */
		if (fb->pitches[color_plane] > max_stride &&
		    !is_ccs_modifier(fb->modifier))
			return intel_tile_size(dev_priv);
		else
			return 64;
	}

	tile_width = intel_tile_width_bytes(fb, color_plane);
	if (is_ccs_modifier(fb->modifier)) {
		/*
		 * Display WA #0531: skl,bxt,kbl,glk
		 *
		 * Render decompression and plane width > 3840
		 * combined with horizontal panning requires the
		 * plane stride to be a multiple of 4. We'll just
		 * require the entire fb to accommodate that to avoid
		 * potential runtime errors at plane configuration time.
		 */
		if (IS_GEN(dev_priv, 9) && color_plane == 0 && fb->width > 3840)
			tile_width *= 4;
		/*
		 * The main surface pitch must be padded to a multiple of four
		 * tile widths.
		 */
		else if (INTEL_GEN(dev_priv) >= 12)
			tile_width *= 4;
	}
	return tile_width;
}

bool intel_plane_can_remap(const struct intel_plane_state *plane_state)
{
	struct intel_plane *plane = to_intel_plane(plane_state->uapi.plane);
	struct drm_i915_private *dev_priv = to_i915(plane->base.dev);
	const struct drm_framebuffer *fb = plane_state->hw.fb;
	int i;

	/* We don't want to deal with remapping with cursors */
	if (plane->id == PLANE_CURSOR)
		return false;

	/*
	 * The display engine limits already match/exceed the
	 * render engine limits, so not much point in remapping.
	 * Would also need to deal with the fence POT alignment
	 * and gen2 2KiB GTT tile size.
	 */
	if (INTEL_GEN(dev_priv) < 4)
		return false;

	/*
	 * The new CCS hash mode isn't compatible with remapping as
	 * the virtual address of the pages affects the compressed data.
	 */
	if (is_ccs_modifier(fb->modifier))
		return false;

	/* Linear needs a page aligned stride for remapping */
	if (fb->modifier == DRM_FORMAT_MOD_LINEAR) {
		unsigned int alignment = intel_tile_size(dev_priv) - 1;

		for (i = 0; i < fb->format->num_planes; i++) {
			if (fb->pitches[i] & alignment)
				return false;
		}
	}

	return true;
}

static bool intel_plane_needs_remap(const struct intel_plane_state *plane_state)
{
	struct intel_plane *plane = to_intel_plane(plane_state->uapi.plane);
	const struct drm_framebuffer *fb = plane_state->hw.fb;
	unsigned int rotation = plane_state->hw.rotation;
	u32 stride, max_stride;

	/*
	 * No remapping for invisible planes since we don't have
	 * an actual source viewport to remap.
	 */
	if (!plane_state->uapi.visible)
		return false;

	if (!intel_plane_can_remap(plane_state))
		return false;

	/*
	 * FIXME: aux plane limits on gen9+ are
	 * unclear in Bspec, for now no checking.
	 */
	stride = intel_fb_pitch(fb, 0, rotation);
	max_stride = plane->max_stride(plane, fb->format->format,
				       fb->modifier, rotation);

	return stride > max_stride;
}

static void
intel_fb_plane_get_subsampling(int *hsub, int *vsub,
			       const struct drm_framebuffer *fb,
			       int color_plane)
{
	int main_plane;

	if (color_plane == 0) {
		*hsub = 1;
		*vsub = 1;

		return;
	}

	/*
	 * TODO: Deduct the subsampling from the char block for all CCS
	 * formats and planes.
	 */
	if (!is_gen12_ccs_plane(fb, color_plane)) {
		*hsub = fb->format->hsub;
		*vsub = fb->format->vsub;

		return;
	}

	main_plane = ccs_to_main_plane(fb, color_plane);
	*hsub = drm_format_info_block_width(fb->format, color_plane) /
		drm_format_info_block_width(fb->format, main_plane);

	/*
	 * The min stride check in the core framebuffer_check() function
	 * assumes that format->hsub applies to every plane except for the
	 * first plane. That's incorrect for the CCS AUX plane of the first
	 * plane, but for the above check to pass we must define the block
	 * width with that subsampling applied to it. Adjust the width here
	 * accordingly, so we can calculate the actual subsampling factor.
	 */
	if (main_plane == 0)
		*hsub *= fb->format->hsub;

	*vsub = 32;
}
static int
intel_fb_check_ccs_xy(struct drm_framebuffer *fb, int ccs_plane, int x, int y)
{
	struct intel_framebuffer *intel_fb = to_intel_framebuffer(fb);
	int main_plane;
	int hsub, vsub;
	int tile_width, tile_height;
	int ccs_x, ccs_y;
	int main_x, main_y;

	if (!is_ccs_plane(fb, ccs_plane))
		return 0;

	intel_tile_dims(fb, ccs_plane, &tile_width, &tile_height);
	intel_fb_plane_get_subsampling(&hsub, &vsub, fb, ccs_plane);

	tile_width *= hsub;
	tile_height *= vsub;

	ccs_x = (x * hsub) % tile_width;
	ccs_y = (y * vsub) % tile_height;

	main_plane = ccs_to_main_plane(fb, ccs_plane);
	main_x = intel_fb->normal[main_plane].x % tile_width;
	main_y = intel_fb->normal[main_plane].y % tile_height;

	/*
	 * CCS doesn't have its own x/y offset register, so the intra CCS tile
	 * x/y offsets must match between CCS and the main surface.
	 */
	if (main_x != ccs_x || main_y != ccs_y) {
		DRM_DEBUG_KMS("Bad CCS x/y (main %d,%d ccs %d,%d) full (main %d,%d ccs %d,%d)\n",
			      main_x, main_y,
			      ccs_x, ccs_y,
			      intel_fb->normal[main_plane].x,
			      intel_fb->normal[main_plane].y,
			      x, y);
		return -EINVAL;
	}

	return 0;
}

static void
intel_fb_plane_dims(int *w, int *h, struct drm_framebuffer *fb, int color_plane)
{
	int main_plane = is_ccs_plane(fb, color_plane) ?
			 ccs_to_main_plane(fb, color_plane) : 0;
	int main_hsub, main_vsub;
	int hsub, vsub;

	intel_fb_plane_get_subsampling(&main_hsub, &main_vsub, fb, main_plane);
	intel_fb_plane_get_subsampling(&hsub, &vsub, fb, color_plane);
	*w = fb->width / main_hsub / hsub;
	*h = fb->height / main_vsub / vsub;
}

/*
 * Setup the rotated view for an FB plane and return the size the GTT mapping
 * requires for this view.
 */
static u32
setup_fb_rotation(int plane, const struct intel_remapped_plane_info *plane_info,
		  u32 gtt_offset_rotated, int x, int y,
		  unsigned int width, unsigned int height,
		  unsigned int tile_size,
		  unsigned int tile_width, unsigned int tile_height,
		  struct drm_framebuffer *fb)
{
	struct intel_framebuffer *intel_fb = to_intel_framebuffer(fb);
	struct intel_rotation_info *rot_info = &intel_fb->rot_info;
	unsigned int pitch_tiles;
	struct drm_rect r;

	/* Y or Yf modifiers required for 90/270 rotation */
	if (fb->modifier != I915_FORMAT_MOD_Y_TILED &&
	    fb->modifier != I915_FORMAT_MOD_Yf_TILED)
		return 0;

	if (WARN_ON(plane >= ARRAY_SIZE(rot_info->plane)))
		return 0;

	rot_info->plane[plane] = *plane_info;

	intel_fb->rotated[plane].pitch = plane_info->height * tile_height;

	/* rotate the x/y offsets to match the GTT view */
	drm_rect_init(&r, x, y, width, height);
	drm_rect_rotate(&r,
			plane_info->width * tile_width,
			plane_info->height * tile_height,
			DRM_MODE_ROTATE_270);
	x = r.x1;
	y = r.y1;

	/* rotate the tile dimensions to match the GTT view */
	pitch_tiles = intel_fb->rotated[plane].pitch / tile_height;
	swap(tile_width, tile_height);

	/*
	 * We only keep the x/y offsets, so push all of the
	 * gtt offset into the x/y offsets.
	 */
	intel_adjust_tile_offset(&x, &y,
				 tile_width, tile_height,
				 tile_size, pitch_tiles,
				 gtt_offset_rotated * tile_size, 0);

	/*
	 * First pixel of the framebuffer from
	 * the start of the rotated gtt mapping.
	 */
	intel_fb->rotated[plane].x = x;
	intel_fb->rotated[plane].y = y;

	return plane_info->width * plane_info->height;
}

static int
intel_fill_fb_info(struct drm_i915_private *dev_priv,
		   struct drm_framebuffer *fb)
{
	struct intel_framebuffer *intel_fb = to_intel_framebuffer(fb);
	struct drm_i915_gem_object *obj = intel_fb_obj(fb);
	u32 gtt_offset_rotated = 0;
	unsigned int max_size = 0;
	int i, num_planes = fb->format->num_planes;
	unsigned int tile_size = intel_tile_size(dev_priv);

	for (i = 0; i < num_planes; i++) {
		unsigned int width, height;
		unsigned int cpp, size;
		u32 offset;
		int x, y;
		int ret;

		cpp = fb->format->cpp[i];
		intel_fb_plane_dims(&width, &height, fb, i);

		ret = intel_fb_offset_to_xy(&x, &y, fb, i);
		if (ret) {
			DRM_DEBUG_KMS("bad fb plane %d offset: 0x%x\n",
				      i, fb->offsets[i]);
			return ret;
		}

		ret = intel_fb_check_ccs_xy(fb, i, x, y);
		if (ret)
			return ret;

		/*
		 * The fence (if used) is aligned to the start of the object
		 * so having the framebuffer wrap around across the edge of the
		 * fenced region doesn't really work. We have no API to configure
		 * the fence start offset within the object (nor could we probably
		 * on gen2/3). So it's just easier if we just require that the
		 * fb layout agrees with the fence layout. We already check that the
		 * fb stride matches the fence stride elsewhere.
		 */
		if (i == 0 && i915_gem_object_is_tiled(obj) &&
		    (x + width) * cpp > fb->pitches[i]) {
			DRM_DEBUG_KMS("bad fb plane %d offset: 0x%x\n",
				      i, fb->offsets[i]);
			return -EINVAL;
		}

		/*
		 * First pixel of the framebuffer from
		 * the start of the normal gtt mapping.
		 */
		intel_fb->normal[i].x = x;
		intel_fb->normal[i].y = y;

		offset = intel_compute_aligned_offset(dev_priv, &x, &y, fb, i,
						      fb->pitches[i],
						      DRM_MODE_ROTATE_0,
						      tile_size);
		offset /= tile_size;

		if (!is_surface_linear(fb, i)) {
			struct intel_remapped_plane_info plane_info;
			unsigned int tile_width, tile_height;

			intel_tile_dims(fb, i, &tile_width, &tile_height);

			plane_info.offset = offset;
			plane_info.stride = DIV_ROUND_UP(fb->pitches[i],
							 tile_width * cpp);
			plane_info.width = DIV_ROUND_UP(x + width, tile_width);
			plane_info.height = DIV_ROUND_UP(y + height,
							 tile_height);

			/* how many tiles does this plane need */
			size = plane_info.stride * plane_info.height;
			/*
			 * If the plane isn't horizontally tile aligned,
			 * we need one more tile.
			 */
			if (x != 0)
				size++;

			gtt_offset_rotated +=
				setup_fb_rotation(i, &plane_info,
						  gtt_offset_rotated,
						  x, y, width, height,
						  tile_size,
						  tile_width, tile_height,
						  fb);
		} else {
			size = DIV_ROUND_UP((y + height) * fb->pitches[i] +
					    x * cpp, tile_size);
		}

		/* how many tiles in total needed in the bo */
		max_size = max(max_size, offset + size);
	}

	if (mul_u32_u32(max_size, tile_size) > obj->base.size) {
		DRM_DEBUG_KMS("fb too big for bo (need %"PRIu64" bytes, have %zu bytes)\n",
			      mul_u32_u32(max_size, tile_size), obj->base.size);
		return -EINVAL;
	}

	return 0;
}

static void
intel_plane_remap_gtt(struct intel_plane_state *plane_state)
{
	struct drm_i915_private *dev_priv =
		to_i915(plane_state->uapi.plane->dev);
	struct drm_framebuffer *fb = plane_state->hw.fb;
	struct intel_framebuffer *intel_fb = to_intel_framebuffer(fb);
	struct intel_rotation_info *info = &plane_state->view.rotated;
	unsigned int rotation = plane_state->hw.rotation;
	int i, num_planes = fb->format->num_planes;
	unsigned int tile_size = intel_tile_size(dev_priv);
	unsigned int src_x, src_y;
	unsigned int src_w, src_h;
	u32 gtt_offset = 0;

	memset(&plane_state->view, 0, sizeof(plane_state->view));
	plane_state->view.type = drm_rotation_90_or_270(rotation) ?
		I915_GGTT_VIEW_ROTATED : I915_GGTT_VIEW_REMAPPED;

	src_x = plane_state->uapi.src.x1 >> 16;
	src_y = plane_state->uapi.src.y1 >> 16;
	src_w = drm_rect_width(&plane_state->uapi.src) >> 16;
	src_h = drm_rect_height(&plane_state->uapi.src) >> 16;

	WARN_ON(is_ccs_modifier(fb->modifier));

	/* Make src coordinates relative to the viewport */
	drm_rect_translate(&plane_state->uapi.src,
			   -(src_x << 16), -(src_y << 16));

	/* Rotate src coordinates to match rotated GTT view */
	if (drm_rotation_90_or_270(rotation))
		drm_rect_rotate(&plane_state->uapi.src,
				src_w << 16, src_h << 16,
				DRM_MODE_ROTATE_270);

	for (i = 0; i < num_planes; i++) {
		unsigned int hsub = i ? fb->format->hsub : 1;
		unsigned int vsub = i ? fb->format->vsub : 1;
		unsigned int cpp = fb->format->cpp[i];
		unsigned int tile_width, tile_height;
		unsigned int width, height;
		unsigned int pitch_tiles;
		unsigned int x, y;
		u32 offset;

		intel_tile_dims(fb, i, &tile_width, &tile_height);

		x = src_x / hsub;
		y = src_y / vsub;
		width = src_w / hsub;
		height = src_h / vsub;

		/*
		 * First pixel of the src viewport from the
		 * start of the normal gtt mapping.
		 */
		x += intel_fb->normal[i].x;
		y += intel_fb->normal[i].y;

		offset = intel_compute_aligned_offset(dev_priv, &x, &y,
						      fb, i, fb->pitches[i],
						      DRM_MODE_ROTATE_0, tile_size);
		offset /= tile_size;

		WARN_ON(i >= ARRAY_SIZE(info->plane));
		info->plane[i].offset = offset;
		info->plane[i].stride = DIV_ROUND_UP(fb->pitches[i],
						     tile_width * cpp);
		info->plane[i].width = DIV_ROUND_UP(x + width, tile_width);
		info->plane[i].height = DIV_ROUND_UP(y + height, tile_height);

		if (drm_rotation_90_or_270(rotation)) {
			struct drm_rect r;

			/* rotate the x/y offsets to match the GTT view */
			drm_rect_init(&r, x, y, width, height);
			drm_rect_rotate(&r,
					info->plane[i].width * tile_width,
					info->plane[i].height * tile_height,
					DRM_MODE_ROTATE_270);
			x = r.x1;
			y = r.y1;

			pitch_tiles = info->plane[i].height;
			plane_state->color_plane[i].stride = pitch_tiles * tile_height;

			/* rotate the tile dimensions to match the GTT view */
			swap(tile_width, tile_height);
		} else {
			pitch_tiles = info->plane[i].width;
			plane_state->color_plane[i].stride = pitch_tiles * tile_width * cpp;
		}

		/*
		 * We only keep the x/y offsets, so push all of the
		 * gtt offset into the x/y offsets.
		 */
		intel_adjust_tile_offset(&x, &y,
					 tile_width, tile_height,
					 tile_size, pitch_tiles,
					 gtt_offset * tile_size, 0);

		gtt_offset += info->plane[i].width * info->plane[i].height;

		plane_state->color_plane[i].offset = 0;
		plane_state->color_plane[i].x = x;
		plane_state->color_plane[i].y = y;
	}
}

static int
intel_plane_compute_gtt(struct intel_plane_state *plane_state)
{
	const struct intel_framebuffer *fb =
		to_intel_framebuffer(plane_state->hw.fb);
	unsigned int rotation = plane_state->hw.rotation;
	int i, num_planes;

	if (!fb)
		return 0;

	num_planes = fb->base.format->num_planes;

	if (intel_plane_needs_remap(plane_state)) {
		intel_plane_remap_gtt(plane_state);

		/*
		 * Sometimes even remapping can't overcome
		 * the stride limitations :( Can happen with
		 * big plane sizes and suitably misaligned
		 * offsets.
		 */
		return intel_plane_check_stride(plane_state);
	}

	intel_fill_fb_ggtt_view(&plane_state->view, &fb->base, rotation);

	for (i = 0; i < num_planes; i++) {
		plane_state->color_plane[i].stride = intel_fb_pitch(&fb->base, i, rotation);
		plane_state->color_plane[i].offset = 0;

		if (drm_rotation_90_or_270(rotation)) {
			plane_state->color_plane[i].x = fb->rotated[i].x;
			plane_state->color_plane[i].y = fb->rotated[i].y;
		} else {
			plane_state->color_plane[i].x = fb->normal[i].x;
			plane_state->color_plane[i].y = fb->normal[i].y;
		}
	}

	/* Rotate src coordinates to match rotated GTT view */
	if (drm_rotation_90_or_270(rotation))
		drm_rect_rotate(&plane_state->uapi.src,
				fb->base.width << 16, fb->base.height << 16,
				DRM_MODE_ROTATE_270);

	return intel_plane_check_stride(plane_state);
}

static int i9xx_format_to_fourcc(int format)
{
	switch (format) {
	case DISPPLANE_8BPP:
		return DRM_FORMAT_C8;
	case DISPPLANE_BGRA555:
		return DRM_FORMAT_ARGB1555;
	case DISPPLANE_BGRX555:
		return DRM_FORMAT_XRGB1555;
	case DISPPLANE_BGRX565:
		return DRM_FORMAT_RGB565;
	default:
	case DISPPLANE_BGRX888:
		return DRM_FORMAT_XRGB8888;
	case DISPPLANE_RGBX888:
		return DRM_FORMAT_XBGR8888;
	case DISPPLANE_BGRA888:
		return DRM_FORMAT_ARGB8888;
	case DISPPLANE_RGBA888:
		return DRM_FORMAT_ABGR8888;
	case DISPPLANE_BGRX101010:
		return DRM_FORMAT_XRGB2101010;
	case DISPPLANE_RGBX101010:
		return DRM_FORMAT_XBGR2101010;
	case DISPPLANE_BGRA101010:
		return DRM_FORMAT_ARGB2101010;
	case DISPPLANE_RGBA101010:
		return DRM_FORMAT_ABGR2101010;
	case DISPPLANE_RGBX161616:
		return DRM_FORMAT_XBGR16161616F;
	}
}

int skl_format_to_fourcc(int format, bool rgb_order, bool alpha)
{
	switch (format) {
	case PLANE_CTL_FORMAT_RGB_565:
		return DRM_FORMAT_RGB565;
	case PLANE_CTL_FORMAT_NV12:
		return DRM_FORMAT_NV12;
	case PLANE_CTL_FORMAT_P010:
		return DRM_FORMAT_P010;
	case PLANE_CTL_FORMAT_P012:
		return DRM_FORMAT_P012;
	case PLANE_CTL_FORMAT_P016:
		return DRM_FORMAT_P016;
	case PLANE_CTL_FORMAT_Y210:
		return DRM_FORMAT_Y210;
	case PLANE_CTL_FORMAT_Y212:
		return DRM_FORMAT_Y212;
	case PLANE_CTL_FORMAT_Y216:
		return DRM_FORMAT_Y216;
	case PLANE_CTL_FORMAT_Y410:
		return DRM_FORMAT_XVYU2101010;
	case PLANE_CTL_FORMAT_Y412:
		return DRM_FORMAT_XVYU12_16161616;
	case PLANE_CTL_FORMAT_Y416:
		return DRM_FORMAT_XVYU16161616;
	default:
	case PLANE_CTL_FORMAT_XRGB_8888:
		if (rgb_order) {
			if (alpha)
				return DRM_FORMAT_ABGR8888;
			else
				return DRM_FORMAT_XBGR8888;
		} else {
			if (alpha)
				return DRM_FORMAT_ARGB8888;
			else
				return DRM_FORMAT_XRGB8888;
		}
	case PLANE_CTL_FORMAT_XRGB_2101010:
		if (rgb_order) {
			if (alpha)
				return DRM_FORMAT_ABGR2101010;
			else
				return DRM_FORMAT_XBGR2101010;
		} else {
			if (alpha)
				return DRM_FORMAT_ARGB2101010;
			else
				return DRM_FORMAT_XRGB2101010;
		}
	case PLANE_CTL_FORMAT_XRGB_16161616F:
		if (rgb_order) {
			if (alpha)
				return DRM_FORMAT_ABGR16161616F;
			else
				return DRM_FORMAT_XBGR16161616F;
		} else {
			if (alpha)
				return DRM_FORMAT_ARGB16161616F;
			else
				return DRM_FORMAT_XRGB16161616F;
		}
	}
}

static bool
intel_alloc_initial_plane_obj(struct intel_crtc *crtc,
			      struct intel_initial_plane_config *plane_config)
{
	struct drm_device *dev = crtc->base.dev;
	struct drm_i915_private *dev_priv = to_i915(dev);
	struct drm_mode_fb_cmd2 mode_cmd = { 0 };
	struct drm_framebuffer *fb = &plane_config->fb->base;
	u32 base_aligned = round_down(plane_config->base, PAGE_SIZE);
	u32 size_aligned = round_up(plane_config->base + plane_config->size,
				    PAGE_SIZE);
	struct drm_i915_gem_object *obj;
	bool ret = false;

	size_aligned -= base_aligned;

	if (plane_config->size == 0)
		return false;

	/* If the FB is too big, just don't use it since fbdev is not very
	 * important and we should probably use that space with FBC or other
	 * features. */
	if (size_aligned * 2 > dev_priv->stolen_usable_size)
		return false;

	switch (fb->modifier) {
	case DRM_FORMAT_MOD_LINEAR:
	case I915_FORMAT_MOD_X_TILED:
	case I915_FORMAT_MOD_Y_TILED:
		break;
	default:
		DRM_DEBUG_DRIVER("Unsupported modifier for initial FB: 0x%"PRIx64"\n",
				 fb->modifier);
		return false;
	}

	obj = i915_gem_object_create_stolen_for_preallocated(dev_priv,
							     base_aligned,
							     base_aligned,
							     size_aligned);
	if (IS_ERR(obj))
		return false;

	switch (plane_config->tiling) {
	case I915_TILING_NONE:
		break;
	case I915_TILING_X:
	case I915_TILING_Y:
		obj->tiling_and_stride = fb->pitches[0] | plane_config->tiling;
		break;
	default:
		MISSING_CASE(plane_config->tiling);
		goto out;
	}

	mode_cmd.pixel_format = fb->format->format;
	mode_cmd.width = fb->width;
	mode_cmd.height = fb->height;
	mode_cmd.pitches[0] = fb->pitches[0];
	mode_cmd.modifier[0] = fb->modifier;
	mode_cmd.flags = DRM_MODE_FB_MODIFIERS;

	if (intel_framebuffer_init(to_intel_framebuffer(fb), obj, &mode_cmd)) {
		DRM_DEBUG_KMS("intel fb init failed\n");
		goto out;
	}


	DRM_DEBUG_KMS("initial plane fb obj %p\n", obj);
	ret = true;
out:
	i915_gem_object_put(obj);
	return ret;
}

static void
intel_set_plane_visible(struct intel_crtc_state *crtc_state,
			struct intel_plane_state *plane_state,
			bool visible)
=======
void intel_set_plane_visible(struct intel_crtc_state *crtc_state,
			     struct intel_plane_state *plane_state,
			     bool visible)
>>>>>>> vendor/linux-drm-v6.6.35
{
	struct intel_plane *plane = to_intel_plane(plane_state->uapi.plane);

	plane_state->uapi.visible = visible;

	if (visible)
		crtc_state->uapi.plane_mask |= drm_plane_mask(&plane->base);
	else
		crtc_state->uapi.plane_mask &= ~drm_plane_mask(&plane->base);
}

void intel_plane_fixup_bitmasks(struct intel_crtc_state *crtc_state)
{
	struct drm_i915_private *dev_priv = to_i915(crtc_state->uapi.crtc->dev);
	struct drm_plane *plane;

	/*
	 * Active_planes aliases if multiple "primary" or cursor planes
	 * have been used on the same (or wrong) pipe. plane_mask uses
	 * unique ids, hence we can use that to reconstruct active_planes.
	 */
	crtc_state->enabled_planes = 0;
	crtc_state->active_planes = 0;

	drm_for_each_plane_mask(plane, &dev_priv->drm,
				crtc_state->uapi.plane_mask) {
		crtc_state->enabled_planes |= BIT(to_intel_plane(plane)->id);
		crtc_state->active_planes |= BIT(to_intel_plane(plane)->id);
	}
}

void intel_plane_disable_noatomic(struct intel_crtc *crtc,
				  struct intel_plane *plane)
{
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	struct intel_crtc_state *crtc_state =
		to_intel_crtc_state(crtc->base.state);
	struct intel_plane_state *plane_state =
		to_intel_plane_state(plane->base.state);

	drm_dbg_kms(&dev_priv->drm,
		    "Disabling [PLANE:%d:%s] on [CRTC:%d:%s]\n",
		    plane->base.base.id, plane->base.name,
		    crtc->base.base.id, crtc->base.name);

	intel_set_plane_visible(crtc_state, plane_state, false);
	intel_plane_fixup_bitmasks(crtc_state);
	crtc_state->data_rate[plane->id] = 0;
	crtc_state->data_rate_y[plane->id] = 0;
	crtc_state->rel_data_rate[plane->id] = 0;
	crtc_state->rel_data_rate_y[plane->id] = 0;
	crtc_state->min_cdclk[plane->id] = 0;

	if ((crtc_state->active_planes & ~BIT(PLANE_CURSOR)) == 0 &&
	    hsw_ips_disable(crtc_state)) {
		crtc_state->ips_enabled = false;
		intel_crtc_wait_for_next_vblank(crtc);
	}

	/*
	 * Vblank time updates from the shadow to live plane control register
	 * are blocked if the memory self-refresh mode is active at that
	 * moment. So to make sure the plane gets truly disabled, disable
	 * first the self-refresh mode. The self-refresh enable bit in turn
	 * will be checked/applied by the HW only at the next frame start
	 * event which is after the vblank start event, so we need to have a
	 * wait-for-vblank between disabling the plane and the pipe.
	 */
	if (HAS_GMCH(dev_priv) &&
	    intel_set_memory_cxsr(dev_priv, false))
		intel_crtc_wait_for_next_vblank(crtc);

	/*
	 * Gen2 reports pipe underruns whenever all planes are disabled.
	 * So disable underrun reporting before all the planes get disabled.
	 */
	if (DISPLAY_VER(dev_priv) == 2 && !crtc_state->active_planes)
		intel_set_cpu_fifo_underrun_reporting(dev_priv, crtc->pipe, false);

	intel_plane_disable_arm(plane, crtc_state);
	intel_crtc_wait_for_next_vblank(crtc);
}

unsigned int
intel_plane_fence_y_offset(const struct intel_plane_state *plane_state)
{
	int x = 0, y = 0;

	intel_plane_adjust_aligned_offset(&x, &y, plane_state, 0,
					  plane_state->view.color_plane[0].offset, 0);

	return y;
}

static void icl_set_pipe_chicken(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
<<<<<<< HEAD
	u32 dspcntr = 0;

	if (crtc_state->gamma_enable)
		dspcntr |= DISPPLANE_GAMMA_ENABLE;

	if (crtc_state->csc_enable)
		dspcntr |= DISPPLANE_PIPE_CSC_ENABLE;

	if (INTEL_GEN(dev_priv) < 5)
		dspcntr |= DISPPLANE_SEL_PIPE(crtc->pipe);

	return dspcntr;
}

static u32 i9xx_plane_ctl(const struct intel_crtc_state *crtc_state,
			  const struct intel_plane_state *plane_state)
{
	struct drm_i915_private *dev_priv =
		to_i915(plane_state->uapi.plane->dev);
	const struct drm_framebuffer *fb = plane_state->hw.fb;
	unsigned int rotation = plane_state->hw.rotation;
	u32 dspcntr;

	dspcntr = DISPLAY_PLANE_ENABLE;

	if (IS_G4X(dev_priv) || IS_GEN(dev_priv, 5) ||
	    IS_GEN(dev_priv, 6) || IS_IVYBRIDGE(dev_priv))
		dspcntr |= DISPPLANE_TRICKLE_FEED_DISABLE;

	switch (fb->format->format) {
	case DRM_FORMAT_C8:
		dspcntr |= DISPPLANE_8BPP;
		break;
	case DRM_FORMAT_XRGB1555:
		dspcntr |= DISPPLANE_BGRX555;
		break;
	case DRM_FORMAT_ARGB1555:
		dspcntr |= DISPPLANE_BGRA555;
		break;
	case DRM_FORMAT_RGB565:
		dspcntr |= DISPPLANE_BGRX565;
		break;
	case DRM_FORMAT_XRGB8888:
		dspcntr |= DISPPLANE_BGRX888;
		break;
	case DRM_FORMAT_XBGR8888:
		dspcntr |= DISPPLANE_RGBX888;
		break;
	case DRM_FORMAT_ARGB8888:
		dspcntr |= DISPPLANE_BGRA888;
		break;
	case DRM_FORMAT_ABGR8888:
		dspcntr |= DISPPLANE_RGBA888;
		break;
	case DRM_FORMAT_XRGB2101010:
		dspcntr |= DISPPLANE_BGRX101010;
		break;
	case DRM_FORMAT_XBGR2101010:
		dspcntr |= DISPPLANE_RGBX101010;
		break;
	case DRM_FORMAT_ARGB2101010:
		dspcntr |= DISPPLANE_BGRA101010;
		break;
	case DRM_FORMAT_ABGR2101010:
		dspcntr |= DISPPLANE_RGBA101010;
		break;
	case DRM_FORMAT_XBGR16161616F:
		dspcntr |= DISPPLANE_RGBX161616;
		break;
	default:
		MISSING_CASE(fb->format->format);
		return 0;
	}

	if (INTEL_GEN(dev_priv) >= 4 &&
	    fb->modifier == I915_FORMAT_MOD_X_TILED)
		dspcntr |= DISPPLANE_TILED;

	if (rotation & DRM_MODE_ROTATE_180)
		dspcntr |= DISPPLANE_ROTATE_180;

	if (rotation & DRM_MODE_REFLECT_X)
		dspcntr |= DISPPLANE_MIRROR;

	return dspcntr;
}

int i9xx_check_plane_surface(struct intel_plane_state *plane_state)
{
	struct drm_i915_private *dev_priv =
		to_i915(plane_state->uapi.plane->dev);
	const struct drm_framebuffer *fb = plane_state->hw.fb;
	int src_x, src_y, src_w;
	u32 offset;
	int ret;

	ret = intel_plane_compute_gtt(plane_state);
	if (ret)
		return ret;

	if (!plane_state->uapi.visible)
		return 0;

	src_w = drm_rect_width(&plane_state->uapi.src) >> 16;
	src_x = plane_state->uapi.src.x1 >> 16;
	src_y = plane_state->uapi.src.y1 >> 16;

	/* Undocumented hardware limit on i965/g4x/vlv/chv */
	if (HAS_GMCH(dev_priv) && fb->format->cpp[0] == 8 && src_w > 2048)
		return -EINVAL;

	intel_add_fb_offsets(&src_x, &src_y, plane_state, 0);

	if (INTEL_GEN(dev_priv) >= 4)
		offset = intel_plane_compute_aligned_offset(&src_x, &src_y,
							    plane_state, 0);
	else
		offset = 0;

	/*
	 * Put the final coordinates back so that the src
	 * coordinate checks will see the right values.
	 */
	drm_rect_translate_to(&plane_state->uapi.src,
			      src_x << 16, src_y << 16);

	/* HSW/BDW do this automagically in hardware */
	if (!IS_HASWELL(dev_priv) && !IS_BROADWELL(dev_priv)) {
		unsigned int rotation = plane_state->hw.rotation;
		int src_w = drm_rect_width(&plane_state->uapi.src) >> 16;
		int src_h = drm_rect_height(&plane_state->uapi.src) >> 16;

		if (rotation & DRM_MODE_ROTATE_180) {
			src_x += src_w - 1;
			src_y += src_h - 1;
		} else if (rotation & DRM_MODE_REFLECT_X) {
			src_x += src_w - 1;
		}
	}

	plane_state->color_plane[0].offset = offset;
	plane_state->color_plane[0].x = src_x;
	plane_state->color_plane[0].y = src_y;

	return 0;
}

static bool i9xx_plane_has_windowing(struct intel_plane *plane)
{
	struct drm_i915_private *dev_priv = to_i915(plane->base.dev);
	enum i9xx_plane_id i9xx_plane = plane->i9xx_plane;

	if (IS_CHERRYVIEW(dev_priv))
		return i9xx_plane == PLANE_B;
	else if (INTEL_GEN(dev_priv) >= 5 || IS_G4X(dev_priv))
		return false;
	else if (IS_GEN(dev_priv, 4))
		return i9xx_plane == PLANE_C;
	else
		return i9xx_plane == PLANE_B ||
			i9xx_plane == PLANE_C;
}

static int
i9xx_plane_check(struct intel_crtc_state *crtc_state,
		 struct intel_plane_state *plane_state)
{
	struct intel_plane *plane = to_intel_plane(plane_state->uapi.plane);
	int ret;

	ret = chv_plane_check_rotation(plane_state);
	if (ret)
		return ret;

	ret = drm_atomic_helper_check_plane_state(&plane_state->uapi,
						  &crtc_state->uapi,
						  DRM_PLANE_HELPER_NO_SCALING,
						  DRM_PLANE_HELPER_NO_SCALING,
						  i9xx_plane_has_windowing(plane),
						  true);
	if (ret)
		return ret;

	ret = i9xx_check_plane_surface(plane_state);
	if (ret)
		return ret;

	if (!plane_state->uapi.visible)
		return 0;

	ret = intel_plane_check_src_coordinates(plane_state);
	if (ret)
		return ret;

	plane_state->ctl = i9xx_plane_ctl(crtc_state, plane_state);

	return 0;
}

static void i9xx_update_plane(struct intel_plane *plane,
			      const struct intel_crtc_state *crtc_state,
			      const struct intel_plane_state *plane_state)
{
	struct drm_i915_private *dev_priv = to_i915(plane->base.dev);
	enum i9xx_plane_id i9xx_plane = plane->i9xx_plane;
	u32 linear_offset;
	int x = plane_state->color_plane[0].x;
	int y = plane_state->color_plane[0].y;
	int crtc_x = plane_state->uapi.dst.x1;
	int crtc_y = plane_state->uapi.dst.y1;
	int crtc_w = drm_rect_width(&plane_state->uapi.dst);
	int crtc_h = drm_rect_height(&plane_state->uapi.dst);
	unsigned long irqflags;
	u32 dspaddr_offset;
	u32 dspcntr;

	dspcntr = plane_state->ctl | i9xx_plane_ctl_crtc(crtc_state);

	linear_offset = intel_fb_xy_to_linear(x, y, plane_state, 0);

	if (INTEL_GEN(dev_priv) >= 4)
		dspaddr_offset = plane_state->color_plane[0].offset;
	else
		dspaddr_offset = linear_offset;

	spin_lock_irqsave(&dev_priv->uncore.lock, irqflags);

	I915_WRITE_FW(DSPSTRIDE(i9xx_plane), plane_state->color_plane[0].stride);

	if (INTEL_GEN(dev_priv) < 4) {
		/*
		 * PLANE_A doesn't actually have a full window
		 * generator but let's assume we still need to
		 * program whatever is there.
		 */
		I915_WRITE_FW(DSPPOS(i9xx_plane), (crtc_y << 16) | crtc_x);
		I915_WRITE_FW(DSPSIZE(i9xx_plane),
			      ((crtc_h - 1) << 16) | (crtc_w - 1));
	} else if (IS_CHERRYVIEW(dev_priv) && i9xx_plane == PLANE_B) {
		I915_WRITE_FW(PRIMPOS(i9xx_plane), (crtc_y << 16) | crtc_x);
		I915_WRITE_FW(PRIMSIZE(i9xx_plane),
			      ((crtc_h - 1) << 16) | (crtc_w - 1));
		I915_WRITE_FW(PRIMCNSTALPHA(i9xx_plane), 0);
	}

	if (IS_HASWELL(dev_priv) || IS_BROADWELL(dev_priv)) {
		I915_WRITE_FW(DSPOFFSET(i9xx_plane), (y << 16) | x);
	} else if (INTEL_GEN(dev_priv) >= 4) {
		I915_WRITE_FW(DSPLINOFF(i9xx_plane), linear_offset);
		I915_WRITE_FW(DSPTILEOFF(i9xx_plane), (y << 16) | x);
	}

	/*
	 * The control register self-arms if the plane was previously
	 * disabled. Try to make the plane enable atomic by writing
	 * the control register just before the surface register.
	 */
	I915_WRITE_FW(DSPCNTR(i9xx_plane), dspcntr);
	if (INTEL_GEN(dev_priv) >= 4)
		I915_WRITE_FW(DSPSURF(i9xx_plane),
			      intel_plane_ggtt_offset(plane_state) +
			      dspaddr_offset);
	else
		I915_WRITE_FW(DSPADDR(i9xx_plane),
			      intel_plane_ggtt_offset(plane_state) +
			      dspaddr_offset);

	spin_unlock_irqrestore(&dev_priv->uncore.lock, irqflags);
}

static void i9xx_disable_plane(struct intel_plane *plane,
			       const struct intel_crtc_state *crtc_state)
{
	struct drm_i915_private *dev_priv = to_i915(plane->base.dev);
	enum i9xx_plane_id i9xx_plane = plane->i9xx_plane;
	unsigned long irqflags;
	u32 dspcntr;

	/*
	 * DSPCNTR pipe gamma enable on g4x+ and pipe csc
	 * enable on ilk+ affect the pipe bottom color as
	 * well, so we must configure them even if the plane
	 * is disabled.
	 *
	 * On pre-g4x there is no way to gamma correct the
	 * pipe bottom color but we'll keep on doing this
	 * anyway so that the crtc state readout works correctly.
	 */
	dspcntr = i9xx_plane_ctl_crtc(crtc_state);

	spin_lock_irqsave(&dev_priv->uncore.lock, irqflags);

	I915_WRITE_FW(DSPCNTR(i9xx_plane), dspcntr);
	if (INTEL_GEN(dev_priv) >= 4)
		I915_WRITE_FW(DSPSURF(i9xx_plane), 0);
	else
		I915_WRITE_FW(DSPADDR(i9xx_plane), 0);

	spin_unlock_irqrestore(&dev_priv->uncore.lock, irqflags);
}

static bool i9xx_plane_get_hw_state(struct intel_plane *plane,
				    enum pipe *pipe)
{
	struct drm_i915_private *dev_priv = to_i915(plane->base.dev);
	enum intel_display_power_domain power_domain;
	enum i9xx_plane_id i9xx_plane = plane->i9xx_plane;
	intel_wakeref_t wakeref;
	bool ret;
	u32 val;

	/*
	 * Not 100% correct for planes that can move between pipes,
	 * but that's only the case for gen2-4 which don't have any
	 * display power wells.
	 */
	power_domain = POWER_DOMAIN_PIPE(plane->pipe);
	wakeref = intel_display_power_get_if_enabled(dev_priv, power_domain);
	if (!wakeref)
		return false;

	val = I915_READ(DSPCNTR(i9xx_plane));

	ret = val & DISPLAY_PLANE_ENABLE;

	if (INTEL_GEN(dev_priv) >= 5)
		*pipe = plane->pipe;
	else
		*pipe = (val & DISPPLANE_SEL_PIPE_MASK) >>
			DISPPLANE_SEL_PIPE_SHIFT;

	intel_display_power_put(dev_priv, power_domain, wakeref);

	return ret;
}

static void skl_detach_scaler(struct intel_crtc *intel_crtc, int id)
{
	struct drm_device *dev = intel_crtc->base.dev;
	struct drm_i915_private *dev_priv = to_i915(dev);

	I915_WRITE(SKL_PS_CTRL(intel_crtc->pipe, id), 0);
	I915_WRITE(SKL_PS_WIN_POS(intel_crtc->pipe, id), 0);
	I915_WRITE(SKL_PS_WIN_SZ(intel_crtc->pipe, id), 0);
}

/*
 * This function detaches (aka. unbinds) unused scalers in hardware
 */
static void skl_detach_scalers(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc_state->uapi.crtc);
	const struct intel_crtc_scaler_state *scaler_state =
		&crtc_state->scaler_state;
	int i;

	/* loop through and disable scalers that aren't in use */
	for (i = 0; i < intel_crtc->num_scalers; i++) {
		if (!scaler_state->scalers[i].in_use)
			skl_detach_scaler(intel_crtc, i);
	}
}

static unsigned int skl_plane_stride_mult(const struct drm_framebuffer *fb,
					  int color_plane, unsigned int rotation)
{
	/*
	 * The stride is either expressed as a multiple of 64 bytes chunks for
	 * linear buffers or in number of tiles for tiled buffers.
	 */
	if (is_surface_linear(fb, color_plane))
		return 64;
	else if (drm_rotation_90_or_270(rotation))
		return intel_tile_height(fb, color_plane);
	else
		return intel_tile_width_bytes(fb, color_plane);
}

u32 skl_plane_stride(const struct intel_plane_state *plane_state,
		     int color_plane)
{
	const struct drm_framebuffer *fb = plane_state->hw.fb;
	unsigned int rotation = plane_state->hw.rotation;
	u32 stride = plane_state->color_plane[color_plane].stride;

	if (color_plane >= fb->format->num_planes)
		return 0;

	return stride / skl_plane_stride_mult(fb, color_plane, rotation);
}

static u32 skl_plane_ctl_format(u32 pixel_format)
{
	switch (pixel_format) {
	case DRM_FORMAT_C8:
		return PLANE_CTL_FORMAT_INDEXED;
	case DRM_FORMAT_RGB565:
		return PLANE_CTL_FORMAT_RGB_565;
	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_ABGR8888:
		return PLANE_CTL_FORMAT_XRGB_8888 | PLANE_CTL_ORDER_RGBX;
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ARGB8888:
		return PLANE_CTL_FORMAT_XRGB_8888;
	case DRM_FORMAT_XBGR2101010:
	case DRM_FORMAT_ABGR2101010:
		return PLANE_CTL_FORMAT_XRGB_2101010 | PLANE_CTL_ORDER_RGBX;
	case DRM_FORMAT_XRGB2101010:
	case DRM_FORMAT_ARGB2101010:
		return PLANE_CTL_FORMAT_XRGB_2101010;
	case DRM_FORMAT_XBGR16161616F:
	case DRM_FORMAT_ABGR16161616F:
		return PLANE_CTL_FORMAT_XRGB_16161616F | PLANE_CTL_ORDER_RGBX;
	case DRM_FORMAT_XRGB16161616F:
	case DRM_FORMAT_ARGB16161616F:
		return PLANE_CTL_FORMAT_XRGB_16161616F;
	case DRM_FORMAT_YUYV:
		return PLANE_CTL_FORMAT_YUV422 | PLANE_CTL_YUV422_YUYV;
	case DRM_FORMAT_YVYU:
		return PLANE_CTL_FORMAT_YUV422 | PLANE_CTL_YUV422_YVYU;
	case DRM_FORMAT_UYVY:
		return PLANE_CTL_FORMAT_YUV422 | PLANE_CTL_YUV422_UYVY;
	case DRM_FORMAT_VYUY:
		return PLANE_CTL_FORMAT_YUV422 | PLANE_CTL_YUV422_VYUY;
	case DRM_FORMAT_NV12:
		return PLANE_CTL_FORMAT_NV12;
	case DRM_FORMAT_P010:
		return PLANE_CTL_FORMAT_P010;
	case DRM_FORMAT_P012:
		return PLANE_CTL_FORMAT_P012;
	case DRM_FORMAT_P016:
		return PLANE_CTL_FORMAT_P016;
	case DRM_FORMAT_Y210:
		return PLANE_CTL_FORMAT_Y210;
	case DRM_FORMAT_Y212:
		return PLANE_CTL_FORMAT_Y212;
	case DRM_FORMAT_Y216:
		return PLANE_CTL_FORMAT_Y216;
	case DRM_FORMAT_XVYU2101010:
		return PLANE_CTL_FORMAT_Y410;
	case DRM_FORMAT_XVYU12_16161616:
		return PLANE_CTL_FORMAT_Y412;
	case DRM_FORMAT_XVYU16161616:
		return PLANE_CTL_FORMAT_Y416;
	default:
		MISSING_CASE(pixel_format);
	}

	return 0;
}

static u32 skl_plane_ctl_alpha(const struct intel_plane_state *plane_state)
{
	if (!plane_state->hw.fb->format->has_alpha)
		return PLANE_CTL_ALPHA_DISABLE;

	switch (plane_state->hw.pixel_blend_mode) {
	case DRM_MODE_BLEND_PIXEL_NONE:
		return PLANE_CTL_ALPHA_DISABLE;
	case DRM_MODE_BLEND_PREMULTI:
		return PLANE_CTL_ALPHA_SW_PREMULTIPLY;
	case DRM_MODE_BLEND_COVERAGE:
		return PLANE_CTL_ALPHA_HW_PREMULTIPLY;
	default:
		MISSING_CASE(plane_state->hw.pixel_blend_mode);
		return PLANE_CTL_ALPHA_DISABLE;
	}
}

static u32 glk_plane_color_ctl_alpha(const struct intel_plane_state *plane_state)
{
	if (!plane_state->hw.fb->format->has_alpha)
		return PLANE_COLOR_ALPHA_DISABLE;

	switch (plane_state->hw.pixel_blend_mode) {
	case DRM_MODE_BLEND_PIXEL_NONE:
		return PLANE_COLOR_ALPHA_DISABLE;
	case DRM_MODE_BLEND_PREMULTI:
		return PLANE_COLOR_ALPHA_SW_PREMULTIPLY;
	case DRM_MODE_BLEND_COVERAGE:
		return PLANE_COLOR_ALPHA_HW_PREMULTIPLY;
	default:
		MISSING_CASE(plane_state->hw.pixel_blend_mode);
		return PLANE_COLOR_ALPHA_DISABLE;
	}
}

static u32 skl_plane_ctl_tiling(u64 fb_modifier)
{
	switch (fb_modifier) {
	case DRM_FORMAT_MOD_LINEAR:
		break;
	case I915_FORMAT_MOD_X_TILED:
		return PLANE_CTL_TILED_X;
	case I915_FORMAT_MOD_Y_TILED:
		return PLANE_CTL_TILED_Y;
	case I915_FORMAT_MOD_Y_TILED_CCS:
		return PLANE_CTL_TILED_Y | PLANE_CTL_RENDER_DECOMPRESSION_ENABLE;
	case I915_FORMAT_MOD_Y_TILED_GEN12_RC_CCS:
		return PLANE_CTL_TILED_Y |
		       PLANE_CTL_RENDER_DECOMPRESSION_ENABLE |
		       PLANE_CTL_CLEAR_COLOR_DISABLE;
	case I915_FORMAT_MOD_Y_TILED_GEN12_MC_CCS:
		return PLANE_CTL_TILED_Y | PLANE_CTL_MEDIA_DECOMPRESSION_ENABLE;
	case I915_FORMAT_MOD_Yf_TILED:
		return PLANE_CTL_TILED_YF;
	case I915_FORMAT_MOD_Yf_TILED_CCS:
		return PLANE_CTL_TILED_YF | PLANE_CTL_RENDER_DECOMPRESSION_ENABLE;
	default:
		MISSING_CASE(fb_modifier);
	}

	return 0;
}

static u32 skl_plane_ctl_rotate(unsigned int rotate)
{
	switch (rotate) {
	case DRM_MODE_ROTATE_0:
		break;
	/*
	 * DRM_MODE_ROTATE_ is counter clockwise to stay compatible with Xrandr
	 * while i915 HW rotation is clockwise, thats why this swapping.
	 */
	case DRM_MODE_ROTATE_90:
		return PLANE_CTL_ROTATE_270;
	case DRM_MODE_ROTATE_180:
		return PLANE_CTL_ROTATE_180;
	case DRM_MODE_ROTATE_270:
		return PLANE_CTL_ROTATE_90;
	default:
		MISSING_CASE(rotate);
	}

	return 0;
}

static u32 cnl_plane_ctl_flip(unsigned int reflect)
{
	switch (reflect) {
	case 0:
		break;
	case DRM_MODE_REFLECT_X:
		return PLANE_CTL_FLIP_HORIZONTAL;
	case DRM_MODE_REFLECT_Y:
	default:
		MISSING_CASE(reflect);
	}

	return 0;
}

u32 skl_plane_ctl_crtc(const struct intel_crtc_state *crtc_state)
{
	struct drm_i915_private *dev_priv = to_i915(crtc_state->uapi.crtc->dev);
	u32 plane_ctl = 0;

	if (INTEL_GEN(dev_priv) >= 10 || IS_GEMINILAKE(dev_priv))
		return plane_ctl;

	if (crtc_state->gamma_enable)
		plane_ctl |= PLANE_CTL_PIPE_GAMMA_ENABLE;

	if (crtc_state->csc_enable)
		plane_ctl |= PLANE_CTL_PIPE_CSC_ENABLE;

	return plane_ctl;
}

u32 skl_plane_ctl(const struct intel_crtc_state *crtc_state,
		  const struct intel_plane_state *plane_state)
{
	struct drm_i915_private *dev_priv =
		to_i915(plane_state->uapi.plane->dev);
	const struct drm_framebuffer *fb = plane_state->hw.fb;
	unsigned int rotation = plane_state->hw.rotation;
	const struct drm_intel_sprite_colorkey *key = &plane_state->ckey;
	u32 plane_ctl;

	plane_ctl = PLANE_CTL_ENABLE;

	if (INTEL_GEN(dev_priv) < 10 && !IS_GEMINILAKE(dev_priv)) {
		plane_ctl |= skl_plane_ctl_alpha(plane_state);
		plane_ctl |= PLANE_CTL_PLANE_GAMMA_DISABLE;

		if (plane_state->hw.color_encoding == DRM_COLOR_YCBCR_BT709)
			plane_ctl |= PLANE_CTL_YUV_TO_RGB_CSC_FORMAT_BT709;

		if (plane_state->hw.color_range == DRM_COLOR_YCBCR_FULL_RANGE)
			plane_ctl |= PLANE_CTL_YUV_RANGE_CORRECTION_DISABLE;
	}

	plane_ctl |= skl_plane_ctl_format(fb->format->format);
	plane_ctl |= skl_plane_ctl_tiling(fb->modifier);
	plane_ctl |= skl_plane_ctl_rotate(rotation & DRM_MODE_ROTATE_MASK);

	if (INTEL_GEN(dev_priv) >= 10)
		plane_ctl |= cnl_plane_ctl_flip(rotation &
						DRM_MODE_REFLECT_MASK);

	if (key->flags & I915_SET_COLORKEY_DESTINATION)
		plane_ctl |= PLANE_CTL_KEY_ENABLE_DESTINATION;
	else if (key->flags & I915_SET_COLORKEY_SOURCE)
		plane_ctl |= PLANE_CTL_KEY_ENABLE_SOURCE;

	return plane_ctl;
}

u32 glk_plane_color_ctl_crtc(const struct intel_crtc_state *crtc_state)
{
	struct drm_i915_private *dev_priv = to_i915(crtc_state->uapi.crtc->dev);
	u32 plane_color_ctl = 0;

	if (INTEL_GEN(dev_priv) >= 11)
		return plane_color_ctl;

	if (crtc_state->gamma_enable)
		plane_color_ctl |= PLANE_COLOR_PIPE_GAMMA_ENABLE;

	if (crtc_state->csc_enable)
		plane_color_ctl |= PLANE_COLOR_PIPE_CSC_ENABLE;

	return plane_color_ctl;
}

u32 glk_plane_color_ctl(const struct intel_crtc_state *crtc_state,
			const struct intel_plane_state *plane_state)
{
	struct drm_i915_private *dev_priv =
		to_i915(plane_state->uapi.plane->dev);
	const struct drm_framebuffer *fb = plane_state->hw.fb;
	struct intel_plane *plane = to_intel_plane(plane_state->uapi.plane);
	u32 plane_color_ctl = 0;

	plane_color_ctl |= PLANE_COLOR_PLANE_GAMMA_DISABLE;
	plane_color_ctl |= glk_plane_color_ctl_alpha(plane_state);

	if (fb->format->is_yuv && !icl_is_hdr_plane(dev_priv, plane->id)) {
		if (plane_state->hw.color_encoding == DRM_COLOR_YCBCR_BT709)
			plane_color_ctl |= PLANE_COLOR_CSC_MODE_YUV709_TO_RGB709;
		else
			plane_color_ctl |= PLANE_COLOR_CSC_MODE_YUV601_TO_RGB709;

		if (plane_state->hw.color_range == DRM_COLOR_YCBCR_FULL_RANGE)
			plane_color_ctl |= PLANE_COLOR_YUV_RANGE_CORRECTION_DISABLE;
	} else if (fb->format->is_yuv) {
		plane_color_ctl |= PLANE_COLOR_INPUT_CSC_ENABLE;
	}

	return plane_color_ctl;
}

static int
__intel_display_resume(struct drm_device *dev,
		       struct drm_atomic_state *state,
		       struct drm_modeset_acquire_ctx *ctx)
{
	struct drm_crtc_state *crtc_state;
	struct drm_crtc *crtc;
	int i, ret;

	intel_modeset_setup_hw_state(dev, ctx);
	intel_vga_redisable(to_i915(dev));

	if (!state)
		return 0;

	/*
	 * We've duplicated the state, pointers to the old state are invalid.
	 *
	 * Don't attempt to use the old state until we commit the duplicated state.
	 */
	for_each_new_crtc_in_state(state, crtc, crtc_state, i) {
		/*
		 * Force recalculation even if we restore
		 * current state. With fast modeset this may not result
		 * in a modeset when the state is compatible.
		 */
		crtc_state->mode_changed = true;
	}

	/* ignore any reset values/BIOS leftovers in the WM registers */
	if (!HAS_GMCH(to_i915(dev)))
		to_intel_atomic_state(state)->skip_intermediate_wm = true;

	ret = drm_atomic_helper_commit_duplicated_state(state, ctx);

	WARN_ON(ret == -EDEADLK);
	return ret;
}

static bool gpu_reset_clobbers_display(struct drm_i915_private *dev_priv)
{
	return (INTEL_INFO(dev_priv)->gpu_reset_clobbers_display &&
		intel_has_gpu_reset(&dev_priv->gt));
}

void intel_prepare_reset(struct drm_i915_private *dev_priv)
{
	struct drm_device *dev = &dev_priv->drm;
	struct drm_modeset_acquire_ctx *ctx = &dev_priv->reset_ctx;
	struct drm_atomic_state *state;
	int ret;

	/* reset doesn't touch the display */
	if (!i915_modparams.force_reset_modeset_test &&
	    !gpu_reset_clobbers_display(dev_priv))
		return;

	/* We have a modeset vs reset deadlock, defensively unbreak it. */
	spin_lock(&dev_priv->atomic_commit_lock);
	set_bit(I915_RESET_MODESET, &dev_priv->gt.reset.flags);
	DRM_SPIN_WAKEUP_ALL(&dev_priv->atomic_commit_wq,
	    &dev_priv->atomic_commit_lock);
	spin_unlock(&dev_priv->atomic_commit_lock);

	if (atomic_read(&dev_priv->gpu_error.pending_fb_pin)) {
		DRM_DEBUG_KMS("Modeset potentially stuck, unbreaking through wedging\n");
		intel_gt_set_wedged(&dev_priv->gt);
	}

	/*
	 * Need mode_config.mutex so that we don't
	 * trample ongoing ->detect() and whatnot.
	 */
	mutex_lock(&dev->mode_config.mutex);
	drm_modeset_acquire_init(ctx, 0);
	while (1) {
		ret = drm_modeset_lock_all_ctx(dev, ctx);
		if (ret != -EDEADLK)
			break;

		drm_modeset_backoff(ctx);
	}
	/*
	 * Disabling the crtcs gracefully seems nicer. Also the
	 * g33 docs say we should at least disable all the planes.
	 */
	state = drm_atomic_helper_duplicate_state(dev, ctx);
	if (IS_ERR(state)) {
		ret = PTR_ERR(state);
		DRM_ERROR("Duplicating state failed with %i\n", ret);
		return;
	}

	ret = drm_atomic_helper_disable_all(dev, ctx);
	if (ret) {
		DRM_ERROR("Suspending crtc's failed with %i\n", ret);
		drm_atomic_state_put(state);
		return;
	}

	dev_priv->modeset_restore_state = state;
	state->acquire_ctx = ctx;
}

void intel_finish_reset(struct drm_i915_private *dev_priv)
{
	struct drm_device *dev = &dev_priv->drm;
	struct drm_modeset_acquire_ctx *ctx = &dev_priv->reset_ctx;
	struct drm_atomic_state *state;
	int ret;

	/* reset doesn't touch the display */
	if (!test_bit(I915_RESET_MODESET, &dev_priv->gt.reset.flags))
		return;

	state = fetch_and_zero(&dev_priv->modeset_restore_state);
	if (!state)
		goto unlock;

	/* reset doesn't touch the display */
	if (!gpu_reset_clobbers_display(dev_priv)) {
		/* for testing only restore the display */
		ret = __intel_display_resume(dev, state, ctx);
		if (ret)
			DRM_ERROR("Restoring old state failed with %i\n", ret);
	} else {
		/*
		 * The display has been reset as well,
		 * so need a full re-initialization.
		 */
		intel_pps_unlock_regs_wa(dev_priv);
		intel_modeset_init_hw(dev_priv);
		intel_init_clock_gating(dev_priv);

		spin_lock_irq(&dev_priv->irq_lock);
		if (dev_priv->display.hpd_irq_setup)
			dev_priv->display.hpd_irq_setup(dev_priv);
		spin_unlock_irq(&dev_priv->irq_lock);

		ret = __intel_display_resume(dev, state, ctx);
		if (ret)
			DRM_ERROR("Restoring old state failed with %i\n", ret);

		intel_hpd_init(dev_priv);
	}

	drm_atomic_state_put(state);
unlock:
	drm_modeset_drop_locks(ctx);
	drm_modeset_acquire_fini(ctx);
	mutex_unlock(&dev->mode_config.mutex);

	clear_bit_unlock(I915_RESET_MODESET, &dev_priv->gt.reset.flags);
}

static void icl_set_pipe_chicken(struct intel_crtc *crtc)
{
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
=======
>>>>>>> vendor/linux-drm-v6.6.35
	enum pipe pipe = crtc->pipe;
	u32 tmp;

	tmp = intel_de_read(dev_priv, PIPE_CHICKEN(pipe));

	/*
	 * Display WA #1153: icl
	 * enable hardware to bypass the alpha math
	 * and rounding for per-pixel values 00 and 0xff
	 */
	tmp |= PER_PIXEL_ALPHA_BYPASS_EN;
	/*
	 * Display WA # 1605353570: icl
	 * Set the pixel rounding bit to 1 for allowing
	 * passthrough of Frame buffer pixels unmodified
	 * across pipe
	 */
	tmp |= PIXEL_ROUNDING_TRUNC_FB_PASSTHRU;

	/*
	 * Underrun recovery must always be disabled on display 13+.
	 * DG2 chicken bit meaning is inverted compared to other platforms.
	 */
	if (IS_DG2(dev_priv))
		tmp &= ~UNDERRUN_RECOVERY_ENABLE_DG2;
	else if (DISPLAY_VER(dev_priv) >= 13)
		tmp |= UNDERRUN_RECOVERY_DISABLE_ADLP;

	/* Wa_14010547955:dg2 */
	if (IS_DG2_DISPLAY_STEP(dev_priv, STEP_B0, STEP_FOREVER))
		tmp |= DG2_RENDER_CCSTAG_4_3_EN;

	intel_de_write(dev_priv, PIPE_CHICKEN(pipe), tmp);
}

bool intel_has_pending_fb_unpin(struct drm_i915_private *dev_priv)
{
	struct drm_crtc *crtc;
	bool cleanup_done;

	drm_for_each_crtc(crtc, &dev_priv->drm) {
		struct drm_crtc_commit *commit;
		spin_lock(&crtc->commit_lock);
		commit = list_first_entry_or_null(&crtc->commit_list,
						  struct drm_crtc_commit, commit_entry);
		cleanup_done = commit ?
			try_wait_for_completion(&commit->cleanup_done) : true;
		spin_unlock(&crtc->commit_lock);

		if (cleanup_done)
			continue;

		intel_crtc_wait_for_next_vblank(to_intel_crtc(crtc));

		return true;
	}

	return false;
}

/*
 * Finds the encoder associated with the given CRTC. This can only be
 * used when we know that the CRTC isn't feeding multiple encoders!
 */
struct intel_encoder *
intel_get_crtc_new_encoder(const struct intel_atomic_state *state,
			   const struct intel_crtc_state *crtc_state)
{
	const struct drm_connector_state *connector_state;
	const struct drm_connector *connector;
	struct intel_encoder *encoder = NULL;
	struct intel_crtc *master_crtc;
	int num_encoders = 0;
	int i;

	master_crtc = intel_master_crtc(crtc_state);

	for_each_new_connector_in_state(&state->base, connector, connector_state, i) {
		if (connector_state->crtc != &master_crtc->base)
			continue;

		encoder = to_intel_encoder(connector_state->best_encoder);
		num_encoders++;
	}

	drm_WARN(state->base.dev, num_encoders != 1,
		 "%d encoders for pipe %c\n",
		 num_encoders, pipe_name(master_crtc->pipe));

	return encoder;
}

static void ilk_pfit_enable(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	const struct drm_rect *dst = &crtc_state->pch_pfit.dst;
	enum pipe pipe = crtc->pipe;
	int width = drm_rect_width(dst);
	int height = drm_rect_height(dst);
	int x = dst->x1;
	int y = dst->y1;

	if (!crtc_state->pch_pfit.enabled)
		return;

	/* Force use of hard-coded filter coefficients
	 * as some pre-programmed values are broken,
	 * e.g. x201.
	 */
	if (IS_IVYBRIDGE(dev_priv) || IS_HASWELL(dev_priv))
		intel_de_write_fw(dev_priv, PF_CTL(pipe), PF_ENABLE |
				  PF_FILTER_MED_3x3 | PF_PIPE_SEL_IVB(pipe));
	else
		intel_de_write_fw(dev_priv, PF_CTL(pipe), PF_ENABLE |
				  PF_FILTER_MED_3x3);
	intel_de_write_fw(dev_priv, PF_WIN_POS(pipe),
			  PF_WIN_XPOS(x) | PF_WIN_YPOS(y));
	intel_de_write_fw(dev_priv, PF_WIN_SZ(pipe),
			  PF_WIN_XSIZE(width) | PF_WIN_YSIZE(height));
}

static void intel_crtc_dpms_overlay_disable(struct intel_crtc *crtc)
{
	if (crtc->overlay)
		(void) intel_overlay_switch_off(crtc->overlay);

	/* Let userspace switch the overlay on again. In most cases userspace
	 * has to recompute where to put it anyway.
	 */
}

static bool needs_nv12_wa(const struct intel_crtc_state *crtc_state)
{
	struct drm_i915_private *dev_priv = to_i915(crtc_state->uapi.crtc->dev);

	if (!crtc_state->nv12_planes)
		return false;

	/* WA Display #0827: Gen9:all */
	if (DISPLAY_VER(dev_priv) == 9)
		return true;

	return false;
}

static bool needs_scalerclk_wa(const struct intel_crtc_state *crtc_state)
{
	struct drm_i915_private *dev_priv = to_i915(crtc_state->uapi.crtc->dev);

	/* Wa_2006604312:icl,ehl */
	if (crtc_state->scaler_state.scaler_users > 0 && DISPLAY_VER(dev_priv) == 11)
		return true;

	return false;
}

static bool needs_cursorclk_wa(const struct intel_crtc_state *crtc_state)
{
	struct drm_i915_private *dev_priv = to_i915(crtc_state->uapi.crtc->dev);

	/* Wa_1604331009:icl,jsl,ehl */
	if (is_hdr_mode(crtc_state) &&
	    crtc_state->active_planes & BIT(PLANE_CURSOR) &&
	    DISPLAY_VER(dev_priv) == 11)
		return true;

	return false;
}

static void intel_async_flip_vtd_wa(struct drm_i915_private *i915,
				    enum pipe pipe, bool enable)
{
	if (DISPLAY_VER(i915) == 9) {
		/*
		 * "Plane N strech max must be programmed to 11b (x1)
		 *  when Async flips are enabled on that plane."
		 */
		intel_de_rmw(i915, CHICKEN_PIPESL_1(pipe),
			     SKL_PLANE1_STRETCH_MAX_MASK,
			     enable ? SKL_PLANE1_STRETCH_MAX_X1 : SKL_PLANE1_STRETCH_MAX_X8);
	} else {
		/* Also needed on HSW/BDW albeit undocumented */
		intel_de_rmw(i915, CHICKEN_PIPESL_1(pipe),
			     HSW_PRI_STRETCH_MAX_MASK,
			     enable ? HSW_PRI_STRETCH_MAX_X1 : HSW_PRI_STRETCH_MAX_X8);
	}
}

static bool needs_async_flip_vtd_wa(const struct intel_crtc_state *crtc_state)
{
	struct drm_i915_private *i915 = to_i915(crtc_state->uapi.crtc->dev);

	return crtc_state->uapi.async_flip && i915_vtd_active(i915) &&
		(DISPLAY_VER(i915) == 9 || IS_BROADWELL(i915) || IS_HASWELL(i915));
}

#define is_enabling(feature, old_crtc_state, new_crtc_state) \
	((!(old_crtc_state)->feature || intel_crtc_needs_modeset(new_crtc_state)) && \
	 (new_crtc_state)->feature)
#define is_disabling(feature, old_crtc_state, new_crtc_state) \
	((old_crtc_state)->feature && \
	 (!(new_crtc_state)->feature || intel_crtc_needs_modeset(new_crtc_state)))

static bool planes_enabling(const struct intel_crtc_state *old_crtc_state,
			    const struct intel_crtc_state *new_crtc_state)
{
	return is_enabling(active_planes, old_crtc_state, new_crtc_state);
}

static bool planes_disabling(const struct intel_crtc_state *old_crtc_state,
			     const struct intel_crtc_state *new_crtc_state)
{
	return is_disabling(active_planes, old_crtc_state, new_crtc_state);
}

static bool vrr_enabling(const struct intel_crtc_state *old_crtc_state,
			 const struct intel_crtc_state *new_crtc_state)
{
	return is_enabling(vrr.enable, old_crtc_state, new_crtc_state);
}

static bool vrr_disabling(const struct intel_crtc_state *old_crtc_state,
			  const struct intel_crtc_state *new_crtc_state)
{
	return is_disabling(vrr.enable, old_crtc_state, new_crtc_state);
}

#undef is_disabling
#undef is_enabling

static void intel_post_plane_update(struct intel_atomic_state *state,
				    struct intel_crtc *crtc)
{
	struct drm_i915_private *dev_priv = to_i915(state->base.dev);
	const struct intel_crtc_state *old_crtc_state =
		intel_atomic_get_old_crtc_state(state, crtc);
	const struct intel_crtc_state *new_crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	enum pipe pipe = crtc->pipe;

	intel_frontbuffer_flip(dev_priv, new_crtc_state->fb_bits);

	if (new_crtc_state->update_wm_post && new_crtc_state->hw.active)
		intel_update_watermarks(dev_priv);

	intel_fbc_post_update(state, crtc);

	if (needs_async_flip_vtd_wa(old_crtc_state) &&
	    !needs_async_flip_vtd_wa(new_crtc_state))
		intel_async_flip_vtd_wa(dev_priv, pipe, false);

	if (needs_nv12_wa(old_crtc_state) &&
	    !needs_nv12_wa(new_crtc_state))
		skl_wa_827(dev_priv, pipe, false);

	if (needs_scalerclk_wa(old_crtc_state) &&
	    !needs_scalerclk_wa(new_crtc_state))
		icl_wa_scalerclkgating(dev_priv, pipe, false);

	if (needs_cursorclk_wa(old_crtc_state) &&
	    !needs_cursorclk_wa(new_crtc_state))
		icl_wa_cursorclkgating(dev_priv, pipe, false);

	if (intel_crtc_needs_color_update(new_crtc_state))
		intel_color_post_update(new_crtc_state);
}

static void intel_crtc_enable_flip_done(struct intel_atomic_state *state,
					struct intel_crtc *crtc)
{
	const struct intel_crtc_state *crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	u8 update_planes = crtc_state->update_planes;
	const struct intel_plane_state __maybe_unused *plane_state;
	struct intel_plane *plane;
	int i;

	for_each_new_intel_plane_in_state(state, plane, plane_state, i) {
		if (plane->pipe == crtc->pipe &&
		    update_planes & BIT(plane->id))
			plane->enable_flip_done(plane);
	}
}

static void intel_crtc_disable_flip_done(struct intel_atomic_state *state,
					 struct intel_crtc *crtc)
{
	const struct intel_crtc_state *crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	u8 update_planes = crtc_state->update_planes;
	const struct intel_plane_state __maybe_unused *plane_state;
	struct intel_plane *plane;
	int i;

	for_each_new_intel_plane_in_state(state, plane, plane_state, i) {
		if (plane->pipe == crtc->pipe &&
		    update_planes & BIT(plane->id))
			plane->disable_flip_done(plane);
	}
}

static void intel_crtc_async_flip_disable_wa(struct intel_atomic_state *state,
					     struct intel_crtc *crtc)
{
	const struct intel_crtc_state *old_crtc_state =
		intel_atomic_get_old_crtc_state(state, crtc);
	const struct intel_crtc_state *new_crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	u8 disable_async_flip_planes = old_crtc_state->async_flip_planes &
				       ~new_crtc_state->async_flip_planes;
	const struct intel_plane_state *old_plane_state;
	struct intel_plane *plane;
	bool need_vbl_wait = false;
	int i;

	for_each_old_intel_plane_in_state(state, plane, old_plane_state, i) {
		if (plane->need_async_flip_disable_wa &&
		    plane->pipe == crtc->pipe &&
		    disable_async_flip_planes & BIT(plane->id)) {
			/*
			 * Apart from the async flip bit we want to
			 * preserve the old state for the plane.
			 */
			plane->async_flip(plane, old_crtc_state,
					  old_plane_state, false);
			need_vbl_wait = true;
		}
	}

	if (need_vbl_wait)
		intel_crtc_wait_for_next_vblank(crtc);
}

static void intel_pre_plane_update(struct intel_atomic_state *state,
				   struct intel_crtc *crtc)
{
	struct drm_i915_private *dev_priv = to_i915(state->base.dev);
	const struct intel_crtc_state *old_crtc_state =
		intel_atomic_get_old_crtc_state(state, crtc);
	const struct intel_crtc_state *new_crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	enum pipe pipe = crtc->pipe;

	if (vrr_disabling(old_crtc_state, new_crtc_state)) {
		intel_vrr_disable(old_crtc_state);
		intel_crtc_update_active_timings(old_crtc_state, false);
	}

	intel_drrs_deactivate(old_crtc_state);

	intel_psr_pre_plane_update(state, crtc);

	if (hsw_ips_pre_update(state, crtc))
		intel_crtc_wait_for_next_vblank(crtc);

	if (intel_fbc_pre_update(state, crtc))
		intel_crtc_wait_for_next_vblank(crtc);

	if (!needs_async_flip_vtd_wa(old_crtc_state) &&
	    needs_async_flip_vtd_wa(new_crtc_state))
		intel_async_flip_vtd_wa(dev_priv, pipe, true);

	/* Display WA 827 */
	if (!needs_nv12_wa(old_crtc_state) &&
	    needs_nv12_wa(new_crtc_state))
		skl_wa_827(dev_priv, pipe, true);

	/* Wa_2006604312:icl,ehl */
	if (!needs_scalerclk_wa(old_crtc_state) &&
	    needs_scalerclk_wa(new_crtc_state))
		icl_wa_scalerclkgating(dev_priv, pipe, true);

	/* Wa_1604331009:icl,jsl,ehl */
	if (!needs_cursorclk_wa(old_crtc_state) &&
	    needs_cursorclk_wa(new_crtc_state))
		icl_wa_cursorclkgating(dev_priv, pipe, true);

	/*
	 * Vblank time updates from the shadow to live plane control register
	 * are blocked if the memory self-refresh mode is active at that
	 * moment. So to make sure the plane gets truly disabled, disable
	 * first the self-refresh mode. The self-refresh enable bit in turn
	 * will be checked/applied by the HW only at the next frame start
	 * event which is after the vblank start event, so we need to have a
	 * wait-for-vblank between disabling the plane and the pipe.
	 */
	if (HAS_GMCH(dev_priv) && old_crtc_state->hw.active &&
	    new_crtc_state->disable_cxsr && intel_set_memory_cxsr(dev_priv, false))
		intel_crtc_wait_for_next_vblank(crtc);

	/*
	 * IVB workaround: must disable low power watermarks for at least
	 * one frame before enabling scaling.  LP watermarks can be re-enabled
	 * when scaling is disabled.
	 *
	 * WaCxSRDisabledForSpriteScaling:ivb
	 */
	if (old_crtc_state->hw.active &&
	    new_crtc_state->disable_lp_wm && ilk_disable_lp_wm(dev_priv))
		intel_crtc_wait_for_next_vblank(crtc);

	/*
	 * If we're doing a modeset we don't need to do any
	 * pre-vblank watermark programming here.
	 */
	if (!intel_crtc_needs_modeset(new_crtc_state)) {
		/*
		 * For platforms that support atomic watermarks, program the
		 * 'intermediate' watermarks immediately.  On pre-gen9 platforms, these
		 * will be the intermediate values that are safe for both pre- and
		 * post- vblank; when vblank happens, the 'active' values will be set
		 * to the final 'target' values and we'll do this again to get the
		 * optimal watermarks.  For gen9+ platforms, the values we program here
		 * will be the final target values which will get automatically latched
		 * at vblank time; no further programming will be necessary.
		 *
		 * If a platform hasn't been transitioned to atomic watermarks yet,
		 * we'll continue to update watermarks the old way, if flags tell
		 * us to.
		 */
		if (!intel_initial_watermarks(state, crtc))
			if (new_crtc_state->update_wm_pre)
				intel_update_watermarks(dev_priv);
	}

	/*
	 * Gen2 reports pipe underruns whenever all planes are disabled.
	 * So disable underrun reporting before all the planes get disabled.
	 *
	 * We do this after .initial_watermarks() so that we have a
	 * chance of catching underruns with the intermediate watermarks
	 * vs. the old plane configuration.
	 */
	if (DISPLAY_VER(dev_priv) == 2 && planes_disabling(old_crtc_state, new_crtc_state))
		intel_set_cpu_fifo_underrun_reporting(dev_priv, pipe, false);

	/*
	 * WA for platforms where async address update enable bit
	 * is double buffered and only latched at start of vblank.
	 */
	if (old_crtc_state->async_flip_planes & ~new_crtc_state->async_flip_planes)
		intel_crtc_async_flip_disable_wa(state, crtc);
}

static void intel_crtc_disable_planes(struct intel_atomic_state *state,
				      struct intel_crtc *crtc)
{
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	const struct intel_crtc_state *new_crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	unsigned int update_mask = new_crtc_state->update_planes;
	const struct intel_plane_state *old_plane_state;
	struct intel_plane *plane;
	unsigned fb_bits = 0;
	int i;

	intel_crtc_dpms_overlay_disable(crtc);

	for_each_old_intel_plane_in_state(state, plane, old_plane_state, i) {
		if (crtc->pipe != plane->pipe ||
		    !(update_mask & BIT(plane->id)))
			continue;

		intel_plane_disable_arm(plane, new_crtc_state);

		if (old_plane_state->uapi.visible)
			fb_bits |= plane->frontbuffer_bit;
	}

	intel_frontbuffer_flip(dev_priv, fb_bits);
}

static void intel_encoders_update_prepare(struct intel_atomic_state *state)
{
	struct drm_i915_private *i915 = to_i915(state->base.dev);
	struct intel_crtc_state *new_crtc_state, *old_crtc_state;
	struct intel_crtc *crtc;
	int i;

	/*
	 * Make sure the DPLL state is up-to-date for fastset TypeC ports after non-blocking commits.
	 * TODO: Update the DPLL state for all cases in the encoder->update_prepare() hook.
	 */
	if (i915->display.dpll.mgr) {
		for_each_oldnew_intel_crtc_in_state(state, crtc, old_crtc_state, new_crtc_state, i) {
			if (intel_crtc_needs_modeset(new_crtc_state))
				continue;

			new_crtc_state->shared_dpll = old_crtc_state->shared_dpll;
			new_crtc_state->dpll_hw_state = old_crtc_state->dpll_hw_state;
		}
	}
}

static void intel_encoders_pre_pll_enable(struct intel_atomic_state *state,
					  struct intel_crtc *crtc)
{
	const struct intel_crtc_state *crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	const struct drm_connector_state *conn_state;
	struct drm_connector *conn;
	int i;

	for_each_new_connector_in_state(&state->base, conn, conn_state, i) {
		struct intel_encoder *encoder =
			to_intel_encoder(conn_state->best_encoder);

		if (conn_state->crtc != &crtc->base)
			continue;

		if (encoder->pre_pll_enable)
			encoder->pre_pll_enable(state, encoder,
						crtc_state, conn_state);
	}
}

static void intel_encoders_pre_enable(struct intel_atomic_state *state,
				      struct intel_crtc *crtc)
{
	const struct intel_crtc_state *crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	const struct drm_connector_state *conn_state;
	struct drm_connector *conn;
	int i;

	for_each_new_connector_in_state(&state->base, conn, conn_state, i) {
		struct intel_encoder *encoder =
			to_intel_encoder(conn_state->best_encoder);

		if (conn_state->crtc != &crtc->base)
			continue;

		if (encoder->pre_enable)
			encoder->pre_enable(state, encoder,
					    crtc_state, conn_state);
	}
}

static void intel_encoders_enable(struct intel_atomic_state *state,
				  struct intel_crtc *crtc)
{
	const struct intel_crtc_state *crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	const struct drm_connector_state *conn_state;
	struct drm_connector *conn;
	int i;

	for_each_new_connector_in_state(&state->base, conn, conn_state, i) {
		struct intel_encoder *encoder =
			to_intel_encoder(conn_state->best_encoder);

		if (conn_state->crtc != &crtc->base)
			continue;

		if (encoder->enable)
			encoder->enable(state, encoder,
					crtc_state, conn_state);
		intel_opregion_notify_encoder(encoder, true);
	}
}

static void intel_encoders_disable(struct intel_atomic_state *state,
				   struct intel_crtc *crtc)
{
	const struct intel_crtc_state *old_crtc_state =
		intel_atomic_get_old_crtc_state(state, crtc);
	const struct drm_connector_state *old_conn_state;
	struct drm_connector *conn;
	int i;

	for_each_old_connector_in_state(&state->base, conn, old_conn_state, i) {
		struct intel_encoder *encoder =
			to_intel_encoder(old_conn_state->best_encoder);

		if (old_conn_state->crtc != &crtc->base)
			continue;

		intel_opregion_notify_encoder(encoder, false);
		if (encoder->disable)
			encoder->disable(state, encoder,
					 old_crtc_state, old_conn_state);
	}
}

static void intel_encoders_post_disable(struct intel_atomic_state *state,
					struct intel_crtc *crtc)
{
	const struct intel_crtc_state *old_crtc_state =
		intel_atomic_get_old_crtc_state(state, crtc);
	const struct drm_connector_state *old_conn_state;
	struct drm_connector *conn;
	int i;

	for_each_old_connector_in_state(&state->base, conn, old_conn_state, i) {
		struct intel_encoder *encoder =
			to_intel_encoder(old_conn_state->best_encoder);

		if (old_conn_state->crtc != &crtc->base)
			continue;

		if (encoder->post_disable)
			encoder->post_disable(state, encoder,
					      old_crtc_state, old_conn_state);
	}
}

static void intel_encoders_post_pll_disable(struct intel_atomic_state *state,
					    struct intel_crtc *crtc)
{
	const struct intel_crtc_state *old_crtc_state =
		intel_atomic_get_old_crtc_state(state, crtc);
	const struct drm_connector_state *old_conn_state;
	struct drm_connector *conn;
	int i;

	for_each_old_connector_in_state(&state->base, conn, old_conn_state, i) {
		struct intel_encoder *encoder =
			to_intel_encoder(old_conn_state->best_encoder);

		if (old_conn_state->crtc != &crtc->base)
			continue;

		if (encoder->post_pll_disable)
			encoder->post_pll_disable(state, encoder,
						  old_crtc_state, old_conn_state);
	}
}

static void intel_encoders_update_pipe(struct intel_atomic_state *state,
				       struct intel_crtc *crtc)
{
	const struct intel_crtc_state *crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	const struct drm_connector_state *conn_state;
	struct drm_connector *conn;
	int i;

	for_each_new_connector_in_state(&state->base, conn, conn_state, i) {
		struct intel_encoder *encoder =
			to_intel_encoder(conn_state->best_encoder);

		if (conn_state->crtc != &crtc->base)
			continue;

		if (encoder->update_pipe)
			encoder->update_pipe(state, encoder,
					     crtc_state, conn_state);
	}
}

static void intel_disable_primary_plane(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct intel_plane *plane = to_intel_plane(crtc->base.primary);

	plane->disable_arm(plane, crtc_state);
}

static void ilk_configure_cpu_transcoder(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	enum transcoder cpu_transcoder = crtc_state->cpu_transcoder;

	if (crtc_state->has_pch_encoder) {
		intel_cpu_transcoder_set_m1_n1(crtc, cpu_transcoder,
					       &crtc_state->fdi_m_n);
	} else if (intel_crtc_has_dp_encoder(crtc_state)) {
		intel_cpu_transcoder_set_m1_n1(crtc, cpu_transcoder,
					       &crtc_state->dp_m_n);
		intel_cpu_transcoder_set_m2_n2(crtc, cpu_transcoder,
					       &crtc_state->dp_m2_n2);
	}

	intel_set_transcoder_timings(crtc_state);

	ilk_set_pipeconf(crtc_state);
}

static void ilk_crtc_enable(struct intel_atomic_state *state,
			    struct intel_crtc *crtc)
{
	const struct intel_crtc_state *new_crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum pipe pipe = crtc->pipe;

	if (drm_WARN_ON(&dev_priv->drm, crtc->active))
		return;

	/*
	 * Sometimes spurious CPU pipe underruns happen during FDI
	 * training, at least with VGA+HDMI cloning. Suppress them.
	 *
	 * On ILK we get an occasional spurious CPU pipe underruns
	 * between eDP port A enable and vdd enable. Also PCH port
	 * enable seems to result in the occasional CPU pipe underrun.
	 *
	 * Spurious PCH underruns also occur during PCH enabling.
	 */
	intel_set_cpu_fifo_underrun_reporting(dev_priv, pipe, false);
	intel_set_pch_fifo_underrun_reporting(dev_priv, pipe, false);

	ilk_configure_cpu_transcoder(new_crtc_state);

	intel_set_pipe_src_size(new_crtc_state);

	crtc->active = true;

	intel_encoders_pre_enable(state, crtc);

	if (new_crtc_state->has_pch_encoder) {
		ilk_pch_pre_enable(state, crtc);
	} else {
		assert_fdi_tx_disabled(dev_priv, pipe);
		assert_fdi_rx_disabled(dev_priv, pipe);
	}

	ilk_pfit_enable(new_crtc_state);

	/*
	 * On ILK+ LUT must be loaded before the pipe is running but with
	 * clocks enabled
	 */
	intel_color_load_luts(new_crtc_state);
	intel_color_commit_noarm(new_crtc_state);
	intel_color_commit_arm(new_crtc_state);
	/* update DSPCNTR to configure gamma for pipe bottom color */
	intel_disable_primary_plane(new_crtc_state);

	intel_initial_watermarks(state, crtc);
	intel_enable_transcoder(new_crtc_state);

	if (new_crtc_state->has_pch_encoder)
		ilk_pch_enable(state, crtc);

	intel_crtc_vblank_on(new_crtc_state);

	intel_encoders_enable(state, crtc);

	if (HAS_PCH_CPT(dev_priv))
		intel_wait_for_pipe_scanline_moving(crtc);

	/*
	 * Must wait for vblank to avoid spurious PCH FIFO underruns.
	 * And a second vblank wait is needed at least on ILK with
	 * some interlaced HDMI modes. Let's do the double wait always
	 * in case there are more corner cases we don't know about.
	 */
	if (new_crtc_state->has_pch_encoder) {
		intel_crtc_wait_for_next_vblank(crtc);
		intel_crtc_wait_for_next_vblank(crtc);
	}
	intel_set_cpu_fifo_underrun_reporting(dev_priv, pipe, true);
	intel_set_pch_fifo_underrun_reporting(dev_priv, pipe, true);
}

static void glk_pipe_scaler_clock_gating_wa(struct drm_i915_private *dev_priv,
					    enum pipe pipe, bool apply)
{
	u32 val = intel_de_read(dev_priv, CLKGATE_DIS_PSL(pipe));
	u32 mask = DPF_GATING_DIS | DPF_RAM_GATING_DIS | DPFR_GATING_DIS;

	if (apply)
		val |= mask;
	else
		val &= ~mask;

	intel_de_write(dev_priv, CLKGATE_DIS_PSL(pipe), val);
}

static void hsw_set_linetime_wm(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);

	intel_de_write(dev_priv, WM_LINETIME(crtc->pipe),
		       HSW_LINETIME(crtc_state->linetime) |
		       HSW_IPS_LINETIME(crtc_state->ips_linetime));
}

static void hsw_set_frame_start_delay(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum transcoder transcoder = crtc_state->cpu_transcoder;
	i915_reg_t reg = DISPLAY_VER(dev_priv) >= 14 ? MTL_CHICKEN_TRANS(transcoder) :
			 CHICKEN_TRANS(transcoder);

	intel_de_rmw(dev_priv, reg,
		     HSW_FRAME_START_DELAY_MASK,
		     HSW_FRAME_START_DELAY(crtc_state->framestart_delay - 1));
}

static void icl_ddi_bigjoiner_pre_enable(struct intel_atomic_state *state,
					 const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *master_crtc = intel_master_crtc(crtc_state);

	/*
	 * Enable sequence steps 1-7 on bigjoiner master
	 */
	if (intel_crtc_is_bigjoiner_slave(crtc_state))
		intel_encoders_pre_pll_enable(state, master_crtc);

	if (crtc_state->shared_dpll)
		intel_enable_shared_dpll(crtc_state);

	if (intel_crtc_is_bigjoiner_slave(crtc_state))
		intel_encoders_pre_enable(state, master_crtc);
}

static void hsw_configure_cpu_transcoder(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum transcoder cpu_transcoder = crtc_state->cpu_transcoder;

	if (crtc_state->has_pch_encoder) {
		intel_cpu_transcoder_set_m1_n1(crtc, cpu_transcoder,
					       &crtc_state->fdi_m_n);
	} else if (intel_crtc_has_dp_encoder(crtc_state)) {
		intel_cpu_transcoder_set_m1_n1(crtc, cpu_transcoder,
					       &crtc_state->dp_m_n);
		intel_cpu_transcoder_set_m2_n2(crtc, cpu_transcoder,
					       &crtc_state->dp_m2_n2);
	}

	intel_set_transcoder_timings(crtc_state);
	if (HAS_VRR(dev_priv))
		intel_vrr_set_transcoder_timings(crtc_state);

	if (cpu_transcoder != TRANSCODER_EDP)
		intel_de_write(dev_priv, TRANS_MULT(cpu_transcoder),
			       crtc_state->pixel_multiplier - 1);

	hsw_set_frame_start_delay(crtc_state);

	hsw_set_transconf(crtc_state);
}

static void hsw_crtc_enable(struct intel_atomic_state *state,
			    struct intel_crtc *crtc)
{
	const struct intel_crtc_state *new_crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum pipe pipe = crtc->pipe, hsw_workaround_pipe;
	enum transcoder cpu_transcoder = new_crtc_state->cpu_transcoder;
	bool psl_clkgate_wa;

	if (drm_WARN_ON(&dev_priv->drm, crtc->active))
		return;

	intel_dmc_enable_pipe(dev_priv, crtc->pipe);

	if (!new_crtc_state->bigjoiner_pipes) {
		intel_encoders_pre_pll_enable(state, crtc);

		if (new_crtc_state->shared_dpll)
			intel_enable_shared_dpll(new_crtc_state);

		intel_encoders_pre_enable(state, crtc);
	} else {
		icl_ddi_bigjoiner_pre_enable(state, new_crtc_state);
	}

	intel_dsc_enable(new_crtc_state);

	if (DISPLAY_VER(dev_priv) >= 13)
		intel_uncompressed_joiner_enable(new_crtc_state);

	intel_set_pipe_src_size(new_crtc_state);
	if (DISPLAY_VER(dev_priv) >= 9 || IS_BROADWELL(dev_priv))
		bdw_set_pipe_misc(new_crtc_state);

	if (!intel_crtc_is_bigjoiner_slave(new_crtc_state) &&
	    !transcoder_is_dsi(cpu_transcoder))
		hsw_configure_cpu_transcoder(new_crtc_state);

	crtc->active = true;

	/* Display WA #1180: WaDisableScalarClockGating: glk */
	psl_clkgate_wa = DISPLAY_VER(dev_priv) == 10 &&
		new_crtc_state->pch_pfit.enabled;
	if (psl_clkgate_wa)
		glk_pipe_scaler_clock_gating_wa(dev_priv, pipe, true);

	if (DISPLAY_VER(dev_priv) >= 9)
		skl_pfit_enable(new_crtc_state);
	else
		ilk_pfit_enable(new_crtc_state);

	/*
	 * On ILK+ LUT must be loaded before the pipe is running but with
	 * clocks enabled
	 */
	intel_color_load_luts(new_crtc_state);
	intel_color_commit_noarm(new_crtc_state);
	intel_color_commit_arm(new_crtc_state);
	/* update DSPCNTR to configure gamma/csc for pipe bottom color */
	if (DISPLAY_VER(dev_priv) < 9)
		intel_disable_primary_plane(new_crtc_state);

	hsw_set_linetime_wm(new_crtc_state);

	if (DISPLAY_VER(dev_priv) >= 11)
		icl_set_pipe_chicken(new_crtc_state);

	intel_initial_watermarks(state, crtc);

	if (intel_crtc_is_bigjoiner_slave(new_crtc_state))
		intel_crtc_vblank_on(new_crtc_state);

	intel_encoders_enable(state, crtc);

	if (psl_clkgate_wa) {
		intel_crtc_wait_for_next_vblank(crtc);
		glk_pipe_scaler_clock_gating_wa(dev_priv, pipe, false);
	}

	/* If we change the relative order between pipe/planes enabling, we need
	 * to change the workaround. */
	hsw_workaround_pipe = new_crtc_state->hsw_workaround_pipe;
	if (IS_HASWELL(dev_priv) && hsw_workaround_pipe != INVALID_PIPE) {
		struct intel_crtc *wa_crtc;

		wa_crtc = intel_crtc_for_pipe(dev_priv, hsw_workaround_pipe);

		intel_crtc_wait_for_next_vblank(wa_crtc);
		intel_crtc_wait_for_next_vblank(wa_crtc);
	}
}

void ilk_pfit_disable(const struct intel_crtc_state *old_crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(old_crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum pipe pipe = crtc->pipe;

	/* To avoid upsetting the power well on haswell only disable the pfit if
	 * it's in use. The hw state code will make sure we get this right. */
	if (!old_crtc_state->pch_pfit.enabled)
		return;

	intel_de_write_fw(dev_priv, PF_CTL(pipe), 0);
	intel_de_write_fw(dev_priv, PF_WIN_POS(pipe), 0);
	intel_de_write_fw(dev_priv, PF_WIN_SZ(pipe), 0);
}

static void ilk_crtc_disable(struct intel_atomic_state *state,
			     struct intel_crtc *crtc)
{
	const struct intel_crtc_state *old_crtc_state =
		intel_atomic_get_old_crtc_state(state, crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum pipe pipe = crtc->pipe;

	/*
	 * Sometimes spurious CPU pipe underruns happen when the
	 * pipe is already disabled, but FDI RX/TX is still enabled.
	 * Happens at least with VGA+HDMI cloning. Suppress them.
	 */
	intel_set_cpu_fifo_underrun_reporting(dev_priv, pipe, false);
	intel_set_pch_fifo_underrun_reporting(dev_priv, pipe, false);

	intel_encoders_disable(state, crtc);

	intel_crtc_vblank_off(old_crtc_state);

	intel_disable_transcoder(old_crtc_state);

	ilk_pfit_disable(old_crtc_state);

	if (old_crtc_state->has_pch_encoder)
		ilk_pch_disable(state, crtc);

	intel_encoders_post_disable(state, crtc);

	if (old_crtc_state->has_pch_encoder)
		ilk_pch_post_disable(state, crtc);

	intel_set_cpu_fifo_underrun_reporting(dev_priv, pipe, true);
	intel_set_pch_fifo_underrun_reporting(dev_priv, pipe, true);

	intel_disable_shared_dpll(old_crtc_state);
}

static void hsw_crtc_disable(struct intel_atomic_state *state,
			     struct intel_crtc *crtc)
{
	const struct intel_crtc_state *old_crtc_state =
		intel_atomic_get_old_crtc_state(state, crtc);
	struct drm_i915_private *i915 = to_i915(crtc->base.dev);

	/*
	 * FIXME collapse everything to one hook.
	 * Need care with mst->ddi interactions.
	 */
	if (!intel_crtc_is_bigjoiner_slave(old_crtc_state)) {
		intel_encoders_disable(state, crtc);
		intel_encoders_post_disable(state, crtc);
	}

	intel_disable_shared_dpll(old_crtc_state);

	if (!intel_crtc_is_bigjoiner_slave(old_crtc_state)) {
		struct intel_crtc *slave_crtc;

		intel_encoders_post_pll_disable(state, crtc);

		intel_dmc_disable_pipe(i915, crtc->pipe);

		for_each_intel_crtc_in_pipe_mask(&i915->drm, slave_crtc,
						 intel_crtc_bigjoiner_slave_pipes(old_crtc_state))
			intel_dmc_disable_pipe(i915, slave_crtc->pipe);
	}
}

static void i9xx_pfit_enable(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);

	if (!crtc_state->gmch_pfit.control)
		return;

	/*
	 * The panel fitter should only be adjusted whilst the pipe is disabled,
	 * according to register description and PRM.
	 */
	drm_WARN_ON(&dev_priv->drm,
		    intel_de_read(dev_priv, PFIT_CONTROL) & PFIT_ENABLE);
	assert_transcoder_disabled(dev_priv, crtc_state->cpu_transcoder);

	intel_de_write(dev_priv, PFIT_PGM_RATIOS,
		       crtc_state->gmch_pfit.pgm_ratios);
	intel_de_write(dev_priv, PFIT_CONTROL, crtc_state->gmch_pfit.control);

	/* Border color in case we don't scale up to the full screen. Black by
	 * default, change to something else for debugging. */
	intel_de_write(dev_priv, BCLRPAT(crtc->pipe), 0);
}

bool intel_phy_is_combo(struct drm_i915_private *dev_priv, enum phy phy)
{
	if (phy == PHY_NONE)
		return false;
	else if (IS_ALDERLAKE_S(dev_priv))
		return phy <= PHY_E;
	else if (IS_DG1(dev_priv) || IS_ROCKETLAKE(dev_priv))
		return phy <= PHY_D;
	else if (IS_JASPERLAKE(dev_priv) || IS_ELKHARTLAKE(dev_priv))
		return phy <= PHY_C;
	else if (IS_ALDERLAKE_P(dev_priv) || IS_DISPLAY_VER(dev_priv, 11, 12))
		return phy <= PHY_B;
	else
		/*
		 * DG2 outputs labelled as "combo PHY" in the bspec use
		 * SNPS PHYs with completely different programming,
		 * hence we always return false here.
		 */
		return false;
}

bool intel_phy_is_tc(struct drm_i915_private *dev_priv, enum phy phy)
{
	if (IS_DG2(dev_priv))
		/* DG2's "TC1" output uses a SNPS PHY */
		return false;
	else if (IS_ALDERLAKE_P(dev_priv) || IS_METEORLAKE(dev_priv))
		return phy >= PHY_F && phy <= PHY_I;
	else if (IS_TIGERLAKE(dev_priv))
		return phy >= PHY_D && phy <= PHY_I;
	else if (IS_ICELAKE(dev_priv))
		return phy >= PHY_C && phy <= PHY_F;
	else
		return false;
}

bool intel_phy_is_snps(struct drm_i915_private *dev_priv, enum phy phy)
{
	if (phy == PHY_NONE)
		return false;
	else if (IS_DG2(dev_priv))
		/*
		 * All four "combo" ports and the TC1 port (PHY E) use
		 * Synopsis PHYs.
		 */
		return phy <= PHY_E;

	return false;
}

enum phy intel_port_to_phy(struct drm_i915_private *i915, enum port port)
{
	if (DISPLAY_VER(i915) >= 13 && port >= PORT_D_XELPD)
		return PHY_D + port - PORT_D_XELPD;
	else if (DISPLAY_VER(i915) >= 13 && port >= PORT_TC1)
		return PHY_F + port - PORT_TC1;
	else if (IS_ALDERLAKE_S(i915) && port >= PORT_TC1)
		return PHY_B + port - PORT_TC1;
	else if ((IS_DG1(i915) || IS_ROCKETLAKE(i915)) && port >= PORT_TC1)
		return PHY_C + port - PORT_TC1;
	else if ((IS_JASPERLAKE(i915) || IS_ELKHARTLAKE(i915)) &&
		 port == PORT_D)
		return PHY_A;

	return PHY_A + port - PORT_A;
}

enum tc_port intel_port_to_tc(struct drm_i915_private *dev_priv, enum port port)
{
	if (!intel_phy_is_tc(dev_priv, intel_port_to_phy(dev_priv, port)))
		return TC_PORT_NONE;

	if (DISPLAY_VER(dev_priv) >= 12)
		return TC_PORT_1 + port - PORT_TC1;
	else
		return TC_PORT_1 + port - PORT_C;
}

enum intel_display_power_domain
intel_aux_power_domain(struct intel_digital_port *dig_port)
{
	struct drm_i915_private *i915 = to_i915(dig_port->base.base.dev);

	if (intel_tc_port_in_tbt_alt_mode(dig_port))
		return intel_display_power_tbt_aux_domain(i915, dig_port->aux_ch);

	return intel_display_power_legacy_aux_domain(i915, dig_port->aux_ch);
}

static void get_crtc_power_domains(struct intel_crtc_state *crtc_state,
				   struct intel_power_domain_mask *mask)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum transcoder cpu_transcoder = crtc_state->cpu_transcoder;
	struct drm_encoder *encoder;
	enum pipe pipe = crtc->pipe;

	bitmap_zero(mask->bits, POWER_DOMAIN_NUM);

	if (!crtc_state->hw.active)
		return;

	set_bit(POWER_DOMAIN_PIPE(pipe), mask->bits);
	set_bit(POWER_DOMAIN_TRANSCODER(cpu_transcoder), mask->bits);
	if (crtc_state->pch_pfit.enabled ||
	    crtc_state->pch_pfit.force_thru)
		set_bit(POWER_DOMAIN_PIPE_PANEL_FITTER(pipe), mask->bits);

	drm_for_each_encoder_mask(encoder, &dev_priv->drm,
				  crtc_state->uapi.encoder_mask) {
		struct intel_encoder *intel_encoder = to_intel_encoder(encoder);

		set_bit(intel_encoder->power_domain, mask->bits);
	}

	if (HAS_DDI(dev_priv) && crtc_state->has_audio)
		set_bit(POWER_DOMAIN_AUDIO_MMIO, mask->bits);

	if (crtc_state->shared_dpll)
		set_bit(POWER_DOMAIN_DISPLAY_CORE, mask->bits);

	if (crtc_state->dsc.compression_enable)
		set_bit(intel_dsc_power_domain(crtc, cpu_transcoder), mask->bits);
}

void intel_modeset_get_crtc_power_domains(struct intel_crtc_state *crtc_state,
					  struct intel_power_domain_mask *old_domains)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum intel_display_power_domain domain;
	struct intel_power_domain_mask domains, new_domains;

	get_crtc_power_domains(crtc_state, &domains);

	bitmap_andnot(new_domains.bits,
		      domains.bits,
		      crtc->enabled_power_domains.mask.bits,
		      POWER_DOMAIN_NUM);
	bitmap_andnot(old_domains->bits,
		      crtc->enabled_power_domains.mask.bits,
		      domains.bits,
		      POWER_DOMAIN_NUM);

	for_each_power_domain(domain, &new_domains)
		intel_display_power_get_in_set(dev_priv,
					       &crtc->enabled_power_domains,
					       domain);
}

void intel_modeset_put_crtc_power_domains(struct intel_crtc *crtc,
					  struct intel_power_domain_mask *domains)
{
	intel_display_power_put_mask_in_set(to_i915(crtc->base.dev),
					    &crtc->enabled_power_domains,
					    domains);
}

static void i9xx_configure_cpu_transcoder(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	enum transcoder cpu_transcoder = crtc_state->cpu_transcoder;

	if (intel_crtc_has_dp_encoder(crtc_state)) {
		intel_cpu_transcoder_set_m1_n1(crtc, cpu_transcoder,
					       &crtc_state->dp_m_n);
		intel_cpu_transcoder_set_m2_n2(crtc, cpu_transcoder,
					       &crtc_state->dp_m2_n2);
	}

	intel_set_transcoder_timings(crtc_state);

	i9xx_set_pipeconf(crtc_state);
}

static void valleyview_crtc_enable(struct intel_atomic_state *state,
				   struct intel_crtc *crtc)
{
	const struct intel_crtc_state *new_crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum pipe pipe = crtc->pipe;

	if (drm_WARN_ON(&dev_priv->drm, crtc->active))
		return;

	i9xx_configure_cpu_transcoder(new_crtc_state);

	intel_set_pipe_src_size(new_crtc_state);

	intel_de_write(dev_priv, VLV_PIPE_MSA_MISC(pipe), 0);

	if (IS_CHERRYVIEW(dev_priv) && pipe == PIPE_B) {
		intel_de_write(dev_priv, CHV_BLEND(pipe), CHV_BLEND_LEGACY);
		intel_de_write(dev_priv, CHV_CANVAS(pipe), 0);
	}

	crtc->active = true;

	intel_set_cpu_fifo_underrun_reporting(dev_priv, pipe, true);

	intel_encoders_pre_pll_enable(state, crtc);

	if (IS_CHERRYVIEW(dev_priv))
		chv_enable_pll(new_crtc_state);
	else
		vlv_enable_pll(new_crtc_state);

	intel_encoders_pre_enable(state, crtc);

	i9xx_pfit_enable(new_crtc_state);

	intel_color_load_luts(new_crtc_state);
	intel_color_commit_noarm(new_crtc_state);
	intel_color_commit_arm(new_crtc_state);
	/* update DSPCNTR to configure gamma for pipe bottom color */
	intel_disable_primary_plane(new_crtc_state);

	intel_initial_watermarks(state, crtc);
	intel_enable_transcoder(new_crtc_state);

	intel_crtc_vblank_on(new_crtc_state);

	intel_encoders_enable(state, crtc);
}

static void i9xx_crtc_enable(struct intel_atomic_state *state,
			     struct intel_crtc *crtc)
{
	const struct intel_crtc_state *new_crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum pipe pipe = crtc->pipe;

	if (drm_WARN_ON(&dev_priv->drm, crtc->active))
		return;

	i9xx_configure_cpu_transcoder(new_crtc_state);

	intel_set_pipe_src_size(new_crtc_state);

	crtc->active = true;

	if (DISPLAY_VER(dev_priv) != 2)
		intel_set_cpu_fifo_underrun_reporting(dev_priv, pipe, true);

	intel_encoders_pre_enable(state, crtc);

	i9xx_enable_pll(new_crtc_state);

	i9xx_pfit_enable(new_crtc_state);

	intel_color_load_luts(new_crtc_state);
	intel_color_commit_noarm(new_crtc_state);
	intel_color_commit_arm(new_crtc_state);
	/* update DSPCNTR to configure gamma for pipe bottom color */
	intel_disable_primary_plane(new_crtc_state);

	if (!intel_initial_watermarks(state, crtc))
		intel_update_watermarks(dev_priv);
	intel_enable_transcoder(new_crtc_state);

	intel_crtc_vblank_on(new_crtc_state);

	intel_encoders_enable(state, crtc);

	/* prevents spurious underruns */
	if (DISPLAY_VER(dev_priv) == 2)
		intel_crtc_wait_for_next_vblank(crtc);
}

static void i9xx_pfit_disable(const struct intel_crtc_state *old_crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(old_crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);

	if (!old_crtc_state->gmch_pfit.control)
		return;

	assert_transcoder_disabled(dev_priv, old_crtc_state->cpu_transcoder);

	drm_dbg_kms(&dev_priv->drm, "disabling pfit, current: 0x%08x\n",
		    intel_de_read(dev_priv, PFIT_CONTROL));
	intel_de_write(dev_priv, PFIT_CONTROL, 0);
}

static void i9xx_crtc_disable(struct intel_atomic_state *state,
			      struct intel_crtc *crtc)
{
	struct intel_crtc_state *old_crtc_state =
		intel_atomic_get_old_crtc_state(state, crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum pipe pipe = crtc->pipe;

	/*
	 * On gen2 planes are double buffered but the pipe isn't, so we must
	 * wait for planes to fully turn off before disabling the pipe.
	 */
	if (DISPLAY_VER(dev_priv) == 2)
		intel_crtc_wait_for_next_vblank(crtc);

	intel_encoders_disable(state, crtc);

	intel_crtc_vblank_off(old_crtc_state);

	intel_disable_transcoder(old_crtc_state);

	i9xx_pfit_disable(old_crtc_state);

	intel_encoders_post_disable(state, crtc);

	if (!intel_crtc_has_type(old_crtc_state, INTEL_OUTPUT_DSI)) {
		if (IS_CHERRYVIEW(dev_priv))
			chv_disable_pll(dev_priv, pipe);
		else if (IS_VALLEYVIEW(dev_priv))
			vlv_disable_pll(dev_priv, pipe);
		else
			i9xx_disable_pll(old_crtc_state);
	}

	intel_encoders_post_pll_disable(state, crtc);

	if (DISPLAY_VER(dev_priv) != 2)
		intel_set_cpu_fifo_underrun_reporting(dev_priv, pipe, false);

	if (!dev_priv->display.funcs.wm->initial_watermarks)
		intel_update_watermarks(dev_priv);

	/* clock the pipe down to 640x480@60 to potentially save power */
	if (IS_I830(dev_priv))
		i830_enable_pipe(dev_priv, pipe);
}

void intel_encoder_destroy(struct drm_encoder *encoder)
{
	struct intel_encoder *intel_encoder = to_intel_encoder(encoder);

	drm_encoder_cleanup(encoder);
	kfree(intel_encoder);
}

static bool intel_crtc_supports_double_wide(const struct intel_crtc *crtc)
{
	const struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);

	/* GDG double wide on either pipe, otherwise pipe A only */
	return DISPLAY_VER(dev_priv) < 4 &&
		(crtc->pipe == PIPE_A || IS_I915G(dev_priv));
}

static u32 ilk_pipe_pixel_rate(const struct intel_crtc_state *crtc_state)
{
	u32 pixel_rate = crtc_state->hw.pipe_mode.crtc_clock;
	struct drm_rect src;

	/*
	 * We only use IF-ID interlacing. If we ever use
	 * PF-ID we'll need to adjust the pixel_rate here.
	 */

	if (!crtc_state->pch_pfit.enabled)
		return pixel_rate;

	drm_rect_init(&src, 0, 0,
		      drm_rect_width(&crtc_state->pipe_src) << 16,
		      drm_rect_height(&crtc_state->pipe_src) << 16);

	return intel_adjusted_rate(&src, &crtc_state->pch_pfit.dst,
				   pixel_rate);
}

static void intel_mode_from_crtc_timings(struct drm_display_mode *mode,
					 const struct drm_display_mode *timings)
{
	mode->hdisplay = timings->crtc_hdisplay;
	mode->htotal = timings->crtc_htotal;
	mode->hsync_start = timings->crtc_hsync_start;
	mode->hsync_end = timings->crtc_hsync_end;

	mode->vdisplay = timings->crtc_vdisplay;
	mode->vtotal = timings->crtc_vtotal;
	mode->vsync_start = timings->crtc_vsync_start;
	mode->vsync_end = timings->crtc_vsync_end;

	mode->flags = timings->flags;
	mode->type = DRM_MODE_TYPE_DRIVER;

	mode->clock = timings->crtc_clock;

	drm_mode_set_name(mode);
}

static void intel_crtc_compute_pixel_rate(struct intel_crtc_state *crtc_state)
{
	struct drm_i915_private *dev_priv = to_i915(crtc_state->uapi.crtc->dev);

	if (HAS_GMCH(dev_priv))
		/* FIXME calculate proper pipe pixel rate for GMCH pfit */
		crtc_state->pixel_rate =
			crtc_state->hw.pipe_mode.crtc_clock;
	else
		crtc_state->pixel_rate =
			ilk_pipe_pixel_rate(crtc_state);
}

static void intel_bigjoiner_adjust_timings(const struct intel_crtc_state *crtc_state,
					   struct drm_display_mode *mode)
{
	int num_pipes = intel_bigjoiner_num_pipes(crtc_state);

	if (num_pipes < 2)
		return;

	mode->crtc_clock /= num_pipes;
	mode->crtc_hdisplay /= num_pipes;
	mode->crtc_hblank_start /= num_pipes;
	mode->crtc_hblank_end /= num_pipes;
	mode->crtc_hsync_start /= num_pipes;
	mode->crtc_hsync_end /= num_pipes;
	mode->crtc_htotal /= num_pipes;
}

static void intel_splitter_adjust_timings(const struct intel_crtc_state *crtc_state,
					  struct drm_display_mode *mode)
{
	int overlap = crtc_state->splitter.pixel_overlap;
	int n = crtc_state->splitter.link_count;

	if (!crtc_state->splitter.enable)
		return;

	/*
	 * eDP MSO uses segment timings from EDID for transcoder
	 * timings, but full mode for everything else.
	 *
	 * h_full = (h_segment - pixel_overlap) * link_count
	 */
	mode->crtc_hdisplay = (mode->crtc_hdisplay - overlap) * n;
	mode->crtc_hblank_start = (mode->crtc_hblank_start - overlap) * n;
	mode->crtc_hblank_end = (mode->crtc_hblank_end - overlap) * n;
	mode->crtc_hsync_start = (mode->crtc_hsync_start - overlap) * n;
	mode->crtc_hsync_end = (mode->crtc_hsync_end - overlap) * n;
	mode->crtc_htotal = (mode->crtc_htotal - overlap) * n;
	mode->crtc_clock *= n;
}

static void intel_crtc_readout_derived_state(struct intel_crtc_state *crtc_state)
{
	struct drm_display_mode *mode = &crtc_state->hw.mode;
	struct drm_display_mode *pipe_mode = &crtc_state->hw.pipe_mode;
	struct drm_display_mode *adjusted_mode = &crtc_state->hw.adjusted_mode;

	/*
	 * Start with the adjusted_mode crtc timings, which
	 * have been filled with the transcoder timings.
	 */
	drm_mode_copy(pipe_mode, adjusted_mode);

	/* Expand MSO per-segment transcoder timings to full */
	intel_splitter_adjust_timings(crtc_state, pipe_mode);

	/*
	 * We want the full numbers in adjusted_mode normal timings,
	 * adjusted_mode crtc timings are left with the raw transcoder
	 * timings.
	 */
	intel_mode_from_crtc_timings(adjusted_mode, pipe_mode);

	/* Populate the "user" mode with full numbers */
	drm_mode_copy(mode, pipe_mode);
	intel_mode_from_crtc_timings(mode, mode);
	mode->hdisplay = drm_rect_width(&crtc_state->pipe_src) *
		(intel_bigjoiner_num_pipes(crtc_state) ?: 1);
	mode->vdisplay = drm_rect_height(&crtc_state->pipe_src);

	/* Derive per-pipe timings in case bigjoiner is used */
	intel_bigjoiner_adjust_timings(crtc_state, pipe_mode);
	intel_mode_from_crtc_timings(pipe_mode, pipe_mode);

	intel_crtc_compute_pixel_rate(crtc_state);
}

void intel_encoder_get_config(struct intel_encoder *encoder,
			      struct intel_crtc_state *crtc_state)
{
	encoder->get_config(encoder, crtc_state);

	intel_crtc_readout_derived_state(crtc_state);
}

static void intel_bigjoiner_compute_pipe_src(struct intel_crtc_state *crtc_state)
{
	int num_pipes = intel_bigjoiner_num_pipes(crtc_state);
	int width, height;

	if (num_pipes < 2)
		return;

	width = drm_rect_width(&crtc_state->pipe_src);
	height = drm_rect_height(&crtc_state->pipe_src);

	drm_rect_init(&crtc_state->pipe_src, 0, 0,
		      width / num_pipes, height);
}

static int intel_crtc_compute_pipe_src(struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *i915 = to_i915(crtc->base.dev);

	intel_bigjoiner_compute_pipe_src(crtc_state);

	/*
	 * Pipe horizontal size must be even in:
	 * - DVO ganged mode
	 * - LVDS dual channel mode
	 * - Double wide pipe
	 */
	if (drm_rect_width(&crtc_state->pipe_src) & 1) {
		if (crtc_state->double_wide) {
			drm_dbg_kms(&i915->drm,
				    "[CRTC:%d:%s] Odd pipe source width not supported with double wide pipe\n",
				    crtc->base.base.id, crtc->base.name);
			return -EINVAL;
		}

		if (intel_crtc_has_type(crtc_state, INTEL_OUTPUT_LVDS) &&
		    intel_is_dual_link_lvds(i915)) {
			drm_dbg_kms(&i915->drm,
				    "[CRTC:%d:%s] Odd pipe source width not supported with dual link LVDS\n",
				    crtc->base.base.id, crtc->base.name);
			return -EINVAL;
		}
	}

	return 0;
}

static int intel_crtc_compute_pipe_mode(struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *i915 = to_i915(crtc->base.dev);
	struct drm_display_mode *adjusted_mode = &crtc_state->hw.adjusted_mode;
	struct drm_display_mode *pipe_mode = &crtc_state->hw.pipe_mode;
	int clock_limit = i915->max_dotclk_freq;

	/*
	 * Start with the adjusted_mode crtc timings, which
	 * have been filled with the transcoder timings.
	 */
	drm_mode_copy(pipe_mode, adjusted_mode);

	/* Expand MSO per-segment transcoder timings to full */
	intel_splitter_adjust_timings(crtc_state, pipe_mode);

	/* Derive per-pipe timings in case bigjoiner is used */
	intel_bigjoiner_adjust_timings(crtc_state, pipe_mode);
	intel_mode_from_crtc_timings(pipe_mode, pipe_mode);

	if (DISPLAY_VER(i915) < 4) {
		clock_limit = i915->display.cdclk.max_cdclk_freq * 9 / 10;

		/*
		 * Enable double wide mode when the dot clock
		 * is > 90% of the (display) core speed.
		 */
		if (intel_crtc_supports_double_wide(crtc) &&
		    pipe_mode->crtc_clock > clock_limit) {
			clock_limit = i915->max_dotclk_freq;
			crtc_state->double_wide = true;
		}
	}

	if (pipe_mode->crtc_clock > clock_limit) {
		drm_dbg_kms(&i915->drm,
			    "[CRTC:%d:%s] requested pixel clock (%d kHz) too high (max: %d kHz, double wide: %s)\n",
			    crtc->base.base.id, crtc->base.name,
			    pipe_mode->crtc_clock, clock_limit,
			    str_yes_no(crtc_state->double_wide));
		return -EINVAL;
	}

	return 0;
}

static int intel_crtc_compute_config(struct intel_atomic_state *state,
				     struct intel_crtc *crtc)
{
	struct intel_crtc_state *crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	int ret;

	ret = intel_dpll_crtc_compute_clock(state, crtc);
	if (ret)
		return ret;

	ret = intel_crtc_compute_pipe_src(crtc_state);
	if (ret)
		return ret;

	ret = intel_crtc_compute_pipe_mode(crtc_state);
	if (ret)
		return ret;

	intel_crtc_compute_pixel_rate(crtc_state);

	if (crtc_state->has_pch_encoder)
		return ilk_fdi_compute_config(crtc, crtc_state);

	return 0;
}

static void
intel_reduce_m_n_ratio(u32 *num, u32 *den)
{
	while (*num > DATA_LINK_M_N_MASK ||
	       *den > DATA_LINK_M_N_MASK) {
		*num >>= 1;
		*den >>= 1;
	}
}

static void compute_m_n(u32 *ret_m, u32 *ret_n,
			u32 m, u32 n, u32 constant_n)
{
	if (constant_n)
		*ret_n = constant_n;
	else
		*ret_n = min_t(unsigned int, roundup_pow_of_two(n), DATA_LINK_N_MAX);

	*ret_m = div_u64(mul_u32_u32(m, *ret_n), n);
	intel_reduce_m_n_ratio(ret_m, ret_n);
}

void
intel_link_compute_m_n(u16 bits_per_pixel, int nlanes,
		       int pixel_clock, int link_clock,
		       struct intel_link_m_n *m_n,
		       bool fec_enable)
{
	u32 data_clock = bits_per_pixel * pixel_clock;

	if (fec_enable)
		data_clock = intel_dp_mode_to_fec_clock(data_clock);

	/*
	 * Windows/BIOS uses fixed M/N values always. Follow suit.
	 *
	 * Also several DP dongles in particular seem to be fussy
	 * about too large link M/N values. Presumably the 20bit
	 * value used by Windows/BIOS is acceptable to everyone.
	 */
	m_n->tu = 64;
	compute_m_n(&m_n->data_m, &m_n->data_n,
		    data_clock, link_clock * nlanes * 8,
		    0x8000000);

	compute_m_n(&m_n->link_m, &m_n->link_n,
		    pixel_clock, link_clock,
		    0x80000);
}

void intel_panel_sanitize_ssc(struct drm_i915_private *dev_priv)
{
	/*
	 * There may be no VBT; and if the BIOS enabled SSC we can
	 * just keep using it to avoid unnecessary flicker.  Whereas if the
	 * BIOS isn't using it, don't assume it will work even if the VBT
	 * indicates as much.
	 */
	if (HAS_PCH_IBX(dev_priv) || HAS_PCH_CPT(dev_priv)) {
		bool bios_lvds_use_ssc = intel_de_read(dev_priv,
						       PCH_DREF_CONTROL) &
			DREF_SSC1_ENABLE;

		if (dev_priv->display.vbt.lvds_use_ssc != bios_lvds_use_ssc) {
			drm_dbg_kms(&dev_priv->drm,
				    "SSC %s by BIOS, overriding VBT which says %s\n",
				    str_enabled_disabled(bios_lvds_use_ssc),
				    str_enabled_disabled(dev_priv->display.vbt.lvds_use_ssc));
			dev_priv->display.vbt.lvds_use_ssc = bios_lvds_use_ssc;
		}
	}
}

void intel_zero_m_n(struct intel_link_m_n *m_n)
{
	/* corresponds to 0 register value */
	memset(m_n, 0, sizeof(*m_n));
	m_n->tu = 1;
}

void intel_set_m_n(struct drm_i915_private *i915,
		   const struct intel_link_m_n *m_n,
		   i915_reg_t data_m_reg, i915_reg_t data_n_reg,
		   i915_reg_t link_m_reg, i915_reg_t link_n_reg)
{
	intel_de_write(i915, data_m_reg, TU_SIZE(m_n->tu) | m_n->data_m);
	intel_de_write(i915, data_n_reg, m_n->data_n);
	intel_de_write(i915, link_m_reg, m_n->link_m);
	/*
	 * On BDW+ writing LINK_N arms the double buffered update
	 * of all the M/N registers, so it must be written last.
	 */
	intel_de_write(i915, link_n_reg, m_n->link_n);
}

bool intel_cpu_transcoder_has_m2_n2(struct drm_i915_private *dev_priv,
				    enum transcoder transcoder)
{
	if (IS_HASWELL(dev_priv))
		return transcoder == TRANSCODER_EDP;

	return IS_DISPLAY_VER(dev_priv, 5, 7) || IS_CHERRYVIEW(dev_priv);
}

<<<<<<< HEAD
static void intel_cpu_transcoder_set_m_n(const struct intel_crtc_state *crtc_state,
					 const struct intel_link_m_n *m_n,
					 const struct intel_link_m_n *m2_n2)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum pipe pipe = crtc->pipe;
	enum transcoder transcoder = crtc_state->cpu_transcoder;

	if (INTEL_GEN(dev_priv) >= 5) {
		I915_WRITE(PIPE_DATA_M1(transcoder), TU_SIZE(m_n->tu) | m_n->gmch_m);
		I915_WRITE(PIPE_DATA_N1(transcoder), m_n->gmch_n);
		I915_WRITE(PIPE_LINK_M1(transcoder), m_n->link_m);
		I915_WRITE(PIPE_LINK_N1(transcoder), m_n->link_n);
		/*
		 *  M2_N2 registers are set only if DRRS is supported
		 * (to make sure the registers are not unnecessarily accessed).
		 */
		if (m2_n2 && crtc_state->has_drrs &&
		    transcoder_has_m2_n2(dev_priv, transcoder)) {
			I915_WRITE(PIPE_DATA_M2(transcoder),
					TU_SIZE(m2_n2->tu) | m2_n2->gmch_m);
			I915_WRITE(PIPE_DATA_N2(transcoder), m2_n2->gmch_n);
			I915_WRITE(PIPE_LINK_M2(transcoder), m2_n2->link_m);
			I915_WRITE(PIPE_LINK_N2(transcoder), m2_n2->link_n);
		}
	} else {
		I915_WRITE(PIPE_DATA_M_G4X(pipe), TU_SIZE(m_n->tu) | m_n->gmch_m);
		I915_WRITE(PIPE_DATA_N_G4X(pipe), m_n->gmch_n);
		I915_WRITE(PIPE_LINK_M_G4X(pipe), m_n->link_m);
		I915_WRITE(PIPE_LINK_N_G4X(pipe), m_n->link_n);
	}
}

void intel_dp_set_m_n(const struct intel_crtc_state *crtc_state, enum link_m_n_set m_n)
{
	const struct intel_link_m_n *dp_m_n, *dp_m2_n2 = NULL;

	if (m_n == M1_N1) {
		dp_m_n = &crtc_state->dp_m_n;
		dp_m2_n2 = &crtc_state->dp_m2_n2;
	} else if (m_n == M2_N2) {

		/*
		 * M2_N2 registers are not supported. Hence m2_n2 divider value
		 * needs to be programmed into M1_N1.
		 */
		dp_m_n = &crtc_state->dp_m2_n2;
	} else {
		DRM_ERROR("Unsupported divider value\n");
		return;
	}

	if (crtc_state->has_pch_encoder)
		intel_pch_transcoder_set_m_n(crtc_state, &crtc_state->dp_m_n);
	else
		intel_cpu_transcoder_set_m_n(crtc_state, dp_m_n, dp_m2_n2);
}

static void vlv_compute_dpll(struct intel_crtc *crtc,
			     struct intel_crtc_state *pipe_config)
{
	pipe_config->dpll_hw_state.dpll = DPLL_INTEGRATED_REF_CLK_VLV |
		DPLL_REF_CLK_ENABLE_VLV | DPLL_VGA_MODE_DIS;
	if (crtc->pipe != PIPE_A)
		pipe_config->dpll_hw_state.dpll |= DPLL_INTEGRATED_CRI_CLK_VLV;

	/* DPLL not used with DSI, but still need the rest set up */
	if (!intel_crtc_has_type(pipe_config, INTEL_OUTPUT_DSI))
		pipe_config->dpll_hw_state.dpll |= DPLL_VCO_ENABLE |
			DPLL_EXT_BUFFER_ENABLE_VLV;

	pipe_config->dpll_hw_state.dpll_md =
		(pipe_config->pixel_multiplier - 1) << DPLL_MD_UDI_MULTIPLIER_SHIFT;
}

static void chv_compute_dpll(struct intel_crtc *crtc,
			     struct intel_crtc_state *pipe_config)
{
	pipe_config->dpll_hw_state.dpll = DPLL_SSC_REF_CLK_CHV |
		DPLL_REF_CLK_ENABLE_VLV | DPLL_VGA_MODE_DIS;
	if (crtc->pipe != PIPE_A)
		pipe_config->dpll_hw_state.dpll |= DPLL_INTEGRATED_CRI_CLK_VLV;

	/* DPLL not used with DSI, but still need the rest set up */
	if (!intel_crtc_has_type(pipe_config, INTEL_OUTPUT_DSI))
		pipe_config->dpll_hw_state.dpll |= DPLL_VCO_ENABLE;

	pipe_config->dpll_hw_state.dpll_md =
		(pipe_config->pixel_multiplier - 1) << DPLL_MD_UDI_MULTIPLIER_SHIFT;
}

static void vlv_prepare_pll(struct intel_crtc *crtc,
			    const struct intel_crtc_state *pipe_config)
{
	struct drm_device *dev = crtc->base.dev;
	struct drm_i915_private *dev_priv = to_i915(dev);
	enum pipe pipe = crtc->pipe;
	u32 mdiv;
	u32 bestn, bestm1, bestm2, bestp1, bestp2;
	u32 coreclk, reg_val;

	/* Enable Refclk */
	I915_WRITE(DPLL(pipe),
		   pipe_config->dpll_hw_state.dpll &
		   ~(DPLL_VCO_ENABLE | DPLL_EXT_BUFFER_ENABLE_VLV));

	/* No need to actually set up the DPLL with DSI */
	if ((pipe_config->dpll_hw_state.dpll & DPLL_VCO_ENABLE) == 0)
		return;

	vlv_dpio_get(dev_priv);

	bestn = pipe_config->dpll.n;
	bestm1 = pipe_config->dpll.m1;
	bestm2 = pipe_config->dpll.m2;
	bestp1 = pipe_config->dpll.p1;
	bestp2 = pipe_config->dpll.p2;

	/* See eDP HDMI DPIO driver vbios notes doc */

	/* PLL B needs special handling */
	if (pipe == PIPE_B)
		vlv_pllb_recal_opamp(dev_priv, pipe);

	/* Set up Tx target for periodic Rcomp update */
	vlv_dpio_write(dev_priv, pipe, VLV_PLL_DW9_BCAST, 0x0100000f);

	/* Disable target IRef on PLL */
	reg_val = vlv_dpio_read(dev_priv, pipe, VLV_PLL_DW8(pipe));
	reg_val &= 0x00ffffff;
	vlv_dpio_write(dev_priv, pipe, VLV_PLL_DW8(pipe), reg_val);

	/* Disable fast lock */
	vlv_dpio_write(dev_priv, pipe, VLV_CMN_DW0, 0x610);

	/* Set idtafcrecal before PLL is enabled */
	mdiv = ((bestm1 << DPIO_M1DIV_SHIFT) | (bestm2 & DPIO_M2DIV_MASK));
	mdiv |= ((bestp1 << DPIO_P1_SHIFT) | (bestp2 << DPIO_P2_SHIFT));
	mdiv |= ((bestn << DPIO_N_SHIFT));
	mdiv |= (1 << DPIO_K_SHIFT);

	/*
	 * Post divider depends on pixel clock rate, DAC vs digital (and LVDS,
	 * but we don't support that).
	 * Note: don't use the DAC post divider as it seems unstable.
	 */
	mdiv |= (DPIO_POST_DIV_HDMIDP << DPIO_POST_DIV_SHIFT);
	vlv_dpio_write(dev_priv, pipe, VLV_PLL_DW3(pipe), mdiv);

	mdiv |= DPIO_ENABLE_CALIBRATION;
	vlv_dpio_write(dev_priv, pipe, VLV_PLL_DW3(pipe), mdiv);

	/* Set HBR and RBR LPF coefficients */
	if (pipe_config->port_clock == 162000 ||
	    intel_crtc_has_type(pipe_config, INTEL_OUTPUT_ANALOG) ||
	    intel_crtc_has_type(pipe_config, INTEL_OUTPUT_HDMI))
		vlv_dpio_write(dev_priv, pipe, VLV_PLL_DW10(pipe),
				 0x009f0003);
	else
		vlv_dpio_write(dev_priv, pipe, VLV_PLL_DW10(pipe),
				 0x00d0000f);

	if (intel_crtc_has_dp_encoder(pipe_config)) {
		/* Use SSC source */
		if (pipe == PIPE_A)
			vlv_dpio_write(dev_priv, pipe, VLV_PLL_DW5(pipe),
					 0x0df40000);
		else
			vlv_dpio_write(dev_priv, pipe, VLV_PLL_DW5(pipe),
					 0x0df70000);
	} else { /* HDMI or VGA */
		/* Use bend source */
		if (pipe == PIPE_A)
			vlv_dpio_write(dev_priv, pipe, VLV_PLL_DW5(pipe),
					 0x0df70000);
		else
			vlv_dpio_write(dev_priv, pipe, VLV_PLL_DW5(pipe),
					 0x0df40000);
	}

	coreclk = vlv_dpio_read(dev_priv, pipe, VLV_PLL_DW7(pipe));
	coreclk = (coreclk & 0x0000ff00) | 0x01c00000;
	if (intel_crtc_has_dp_encoder(pipe_config))
		coreclk |= 0x01000000;
	vlv_dpio_write(dev_priv, pipe, VLV_PLL_DW7(pipe), coreclk);

	vlv_dpio_write(dev_priv, pipe, VLV_PLL_DW11(pipe), 0x87871000);

	vlv_dpio_put(dev_priv);
}

static void chv_prepare_pll(struct intel_crtc *crtc,
			    const struct intel_crtc_state *pipe_config)
{
	struct drm_device *dev = crtc->base.dev;
	struct drm_i915_private *dev_priv = to_i915(dev);
	enum pipe pipe = crtc->pipe;
	enum dpio_channel port = vlv_pipe_to_channel(pipe);
	u32 loopfilter, tribuf_calcntr;
	u32 bestn __unused, bestm1 __unused, bestm2, bestp1, bestp2, bestm2_frac;
	u32 dpio_val;
	int vco;

	/* Enable Refclk and SSC */
	I915_WRITE(DPLL(pipe),
		   pipe_config->dpll_hw_state.dpll & ~DPLL_VCO_ENABLE);

	/* No need to actually set up the DPLL with DSI */
	if ((pipe_config->dpll_hw_state.dpll & DPLL_VCO_ENABLE) == 0)
		return;

	bestn = pipe_config->dpll.n;
	bestm2_frac = pipe_config->dpll.m2 & 0x3fffff;
	bestm1 = pipe_config->dpll.m1;
	bestm2 = pipe_config->dpll.m2 >> 22;
	bestp1 = pipe_config->dpll.p1;
	bestp2 = pipe_config->dpll.p2;
	vco = pipe_config->dpll.vco;
	dpio_val = 0;
	loopfilter = 0;

	vlv_dpio_get(dev_priv);

	/* p1 and p2 divider */
	vlv_dpio_write(dev_priv, pipe, CHV_CMN_DW13(port),
			5 << DPIO_CHV_S1_DIV_SHIFT |
			bestp1 << DPIO_CHV_P1_DIV_SHIFT |
			bestp2 << DPIO_CHV_P2_DIV_SHIFT |
			1 << DPIO_CHV_K_DIV_SHIFT);

	/* Feedback post-divider - m2 */
	vlv_dpio_write(dev_priv, pipe, CHV_PLL_DW0(port), bestm2);

	/* Feedback refclk divider - n and m1 */
	vlv_dpio_write(dev_priv, pipe, CHV_PLL_DW1(port),
			DPIO_CHV_M1_DIV_BY_2 |
			1 << DPIO_CHV_N_DIV_SHIFT);

	/* M2 fraction division */
	vlv_dpio_write(dev_priv, pipe, CHV_PLL_DW2(port), bestm2_frac);

	/* M2 fraction division enable */
	dpio_val = vlv_dpio_read(dev_priv, pipe, CHV_PLL_DW3(port));
	dpio_val &= ~(DPIO_CHV_FEEDFWD_GAIN_MASK | DPIO_CHV_FRAC_DIV_EN);
	dpio_val |= (2 << DPIO_CHV_FEEDFWD_GAIN_SHIFT);
	if (bestm2_frac)
		dpio_val |= DPIO_CHV_FRAC_DIV_EN;
	vlv_dpio_write(dev_priv, pipe, CHV_PLL_DW3(port), dpio_val);

	/* Program digital lock detect threshold */
	dpio_val = vlv_dpio_read(dev_priv, pipe, CHV_PLL_DW9(port));
	dpio_val &= ~(DPIO_CHV_INT_LOCK_THRESHOLD_MASK |
					DPIO_CHV_INT_LOCK_THRESHOLD_SEL_COARSE);
	dpio_val |= (0x5 << DPIO_CHV_INT_LOCK_THRESHOLD_SHIFT);
	if (!bestm2_frac)
		dpio_val |= DPIO_CHV_INT_LOCK_THRESHOLD_SEL_COARSE;
	vlv_dpio_write(dev_priv, pipe, CHV_PLL_DW9(port), dpio_val);

	/* Loop filter */
	if (vco == 5400000) {
		loopfilter |= (0x3 << DPIO_CHV_PROP_COEFF_SHIFT);
		loopfilter |= (0x8 << DPIO_CHV_INT_COEFF_SHIFT);
		loopfilter |= (0x1 << DPIO_CHV_GAIN_CTRL_SHIFT);
		tribuf_calcntr = 0x9;
	} else if (vco <= 6200000) {
		loopfilter |= (0x5 << DPIO_CHV_PROP_COEFF_SHIFT);
		loopfilter |= (0xB << DPIO_CHV_INT_COEFF_SHIFT);
		loopfilter |= (0x3 << DPIO_CHV_GAIN_CTRL_SHIFT);
		tribuf_calcntr = 0x9;
	} else if (vco <= 6480000) {
		loopfilter |= (0x4 << DPIO_CHV_PROP_COEFF_SHIFT);
		loopfilter |= (0x9 << DPIO_CHV_INT_COEFF_SHIFT);
		loopfilter |= (0x3 << DPIO_CHV_GAIN_CTRL_SHIFT);
		tribuf_calcntr = 0x8;
	} else {
		/* Not supported. Apply the same limits as in the max case */
		loopfilter |= (0x4 << DPIO_CHV_PROP_COEFF_SHIFT);
		loopfilter |= (0x9 << DPIO_CHV_INT_COEFF_SHIFT);
		loopfilter |= (0x3 << DPIO_CHV_GAIN_CTRL_SHIFT);
		tribuf_calcntr = 0;
	}
	vlv_dpio_write(dev_priv, pipe, CHV_PLL_DW6(port), loopfilter);

	dpio_val = vlv_dpio_read(dev_priv, pipe, CHV_PLL_DW8(port));
	dpio_val &= ~DPIO_CHV_TDC_TARGET_CNT_MASK;
	dpio_val |= (tribuf_calcntr << DPIO_CHV_TDC_TARGET_CNT_SHIFT);
	vlv_dpio_write(dev_priv, pipe, CHV_PLL_DW8(port), dpio_val);

	/* AFC Recal */
	vlv_dpio_write(dev_priv, pipe, CHV_CMN_DW14(port),
			vlv_dpio_read(dev_priv, pipe, CHV_CMN_DW14(port)) |
			DPIO_AFC_RECAL);

	vlv_dpio_put(dev_priv);
}

/**
 * vlv_force_pll_on - forcibly enable just the PLL
 * @dev_priv: i915 private structure
 * @pipe: pipe PLL to enable
 * @dpll: PLL configuration
 *
 * Enable the PLL for @pipe using the supplied @dpll config. To be used
 * in cases where we need the PLL enabled even when @pipe is not going to
 * be enabled.
 */
int vlv_force_pll_on(struct drm_i915_private *dev_priv, enum pipe pipe,
		     const struct dpll *dpll)
{
	struct intel_crtc *crtc = intel_get_crtc_for_pipe(dev_priv, pipe);
	struct intel_crtc_state *pipe_config;

	pipe_config = intel_crtc_state_alloc(crtc);
	if (!pipe_config)
		return -ENOMEM;

	pipe_config->cpu_transcoder = (enum transcoder)pipe;
	pipe_config->pixel_multiplier = 1;
	pipe_config->dpll = *dpll;

	if (IS_CHERRYVIEW(dev_priv)) {
		chv_compute_dpll(crtc, pipe_config);
		chv_prepare_pll(crtc, pipe_config);
		chv_enable_pll(crtc, pipe_config);
	} else {
		vlv_compute_dpll(crtc, pipe_config);
		vlv_prepare_pll(crtc, pipe_config);
		vlv_enable_pll(crtc, pipe_config);
	}

	kfree(pipe_config);

	return 0;
}

/**
 * vlv_force_pll_off - forcibly disable just the PLL
 * @dev_priv: i915 private structure
 * @pipe: pipe PLL to disable
 *
 * Disable the PLL for @pipe. To be used in cases where we need
 * the PLL enabled even when @pipe is not going to be enabled.
 */
void vlv_force_pll_off(struct drm_i915_private *dev_priv, enum pipe pipe)
{
	if (IS_CHERRYVIEW(dev_priv))
		chv_disable_pll(dev_priv, pipe);
	else
		vlv_disable_pll(dev_priv, pipe);
}

static void i9xx_compute_dpll(struct intel_crtc *crtc,
			      struct intel_crtc_state *crtc_state,
			      struct dpll *reduced_clock)
=======
void intel_cpu_transcoder_set_m1_n1(struct intel_crtc *crtc,
				    enum transcoder transcoder,
				    const struct intel_link_m_n *m_n)
>>>>>>> vendor/linux-drm-v6.6.35
{
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum pipe pipe = crtc->pipe;

	if (DISPLAY_VER(dev_priv) >= 5)
		intel_set_m_n(dev_priv, m_n,
			      PIPE_DATA_M1(transcoder), PIPE_DATA_N1(transcoder),
			      PIPE_LINK_M1(transcoder), PIPE_LINK_N1(transcoder));
	else
		intel_set_m_n(dev_priv, m_n,
			      PIPE_DATA_M_G4X(pipe), PIPE_DATA_N_G4X(pipe),
			      PIPE_LINK_M_G4X(pipe), PIPE_LINK_N_G4X(pipe));
}

void intel_cpu_transcoder_set_m2_n2(struct intel_crtc *crtc,
				    enum transcoder transcoder,
				    const struct intel_link_m_n *m_n)
{
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);

	if (!intel_cpu_transcoder_has_m2_n2(dev_priv, transcoder))
		return;

	intel_set_m_n(dev_priv, m_n,
		      PIPE_DATA_M2(transcoder), PIPE_DATA_N2(transcoder),
		      PIPE_LINK_M2(transcoder), PIPE_LINK_N2(transcoder));
}

static void intel_set_transcoder_timings(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum pipe pipe = crtc->pipe;
	enum transcoder cpu_transcoder = crtc_state->cpu_transcoder;
	const struct drm_display_mode *adjusted_mode = &crtc_state->hw.adjusted_mode;
	u32 crtc_vdisplay, crtc_vtotal, crtc_vblank_start, crtc_vblank_end;
	int vsyncshift = 0;

	/* We need to be careful not to changed the adjusted mode, for otherwise
	 * the hw state checker will get angry at the mismatch. */
	crtc_vdisplay = adjusted_mode->crtc_vdisplay;
	crtc_vtotal = adjusted_mode->crtc_vtotal;
	crtc_vblank_start = adjusted_mode->crtc_vblank_start;
	crtc_vblank_end = adjusted_mode->crtc_vblank_end;

	if (adjusted_mode->flags & DRM_MODE_FLAG_INTERLACE) {
		/* the chip adds 2 halflines automatically */
		crtc_vtotal -= 1;
		crtc_vblank_end -= 1;

		if (intel_crtc_has_type(crtc_state, INTEL_OUTPUT_SDVO))
			vsyncshift = (adjusted_mode->crtc_htotal - 1) / 2;
		else
			vsyncshift = adjusted_mode->crtc_hsync_start -
				adjusted_mode->crtc_htotal / 2;
		if (vsyncshift < 0)
			vsyncshift += adjusted_mode->crtc_htotal;
	}

	/*
	 * VBLANK_START no longer works on ADL+, instead we must use
	 * TRANS_SET_CONTEXT_LATENCY to configure the pipe vblank start.
	 */
	if (DISPLAY_VER(dev_priv) >= 13) {
		intel_de_write(dev_priv, TRANS_SET_CONTEXT_LATENCY(cpu_transcoder),
			       crtc_vblank_start - crtc_vdisplay);

		/*
		 * VBLANK_START not used by hw, just clear it
		 * to make it stand out in register dumps.
		 */
		crtc_vblank_start = 1;
	}

	if (DISPLAY_VER(dev_priv) > 3)
		intel_de_write(dev_priv, TRANS_VSYNCSHIFT(cpu_transcoder),
			       vsyncshift);

	intel_de_write(dev_priv, TRANS_HTOTAL(cpu_transcoder),
		       HACTIVE(adjusted_mode->crtc_hdisplay - 1) |
		       HTOTAL(adjusted_mode->crtc_htotal - 1));
	intel_de_write(dev_priv, TRANS_HBLANK(cpu_transcoder),
		       HBLANK_START(adjusted_mode->crtc_hblank_start - 1) |
		       HBLANK_END(adjusted_mode->crtc_hblank_end - 1));
	intel_de_write(dev_priv, TRANS_HSYNC(cpu_transcoder),
		       HSYNC_START(adjusted_mode->crtc_hsync_start - 1) |
		       HSYNC_END(adjusted_mode->crtc_hsync_end - 1));

	intel_de_write(dev_priv, TRANS_VTOTAL(cpu_transcoder),
		       VACTIVE(crtc_vdisplay - 1) |
		       VTOTAL(crtc_vtotal - 1));
	intel_de_write(dev_priv, TRANS_VBLANK(cpu_transcoder),
		       VBLANK_START(crtc_vblank_start - 1) |
		       VBLANK_END(crtc_vblank_end - 1));
	intel_de_write(dev_priv, TRANS_VSYNC(cpu_transcoder),
		       VSYNC_START(adjusted_mode->crtc_vsync_start - 1) |
		       VSYNC_END(adjusted_mode->crtc_vsync_end - 1));

	/* Workaround: when the EDP input selection is B, the VTOTAL_B must be
	 * programmed with the VTOTAL_EDP value. Same for VTOTAL_C. This is
	 * documented on the DDI_FUNC_CTL register description, EDP Input Select
	 * bits. */
	if (IS_HASWELL(dev_priv) && cpu_transcoder == TRANSCODER_EDP &&
	    (pipe == PIPE_B || pipe == PIPE_C))
		intel_de_write(dev_priv, TRANS_VTOTAL(pipe),
			       VACTIVE(crtc_vdisplay - 1) |
			       VTOTAL(crtc_vtotal - 1));
}

static void intel_set_pipe_src_size(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	int width = drm_rect_width(&crtc_state->pipe_src);
	int height = drm_rect_height(&crtc_state->pipe_src);
	enum pipe pipe = crtc->pipe;

	/* pipesrc controls the size that is scaled from, which should
	 * always be the user's requested size.
	 */
	intel_de_write(dev_priv, PIPESRC(pipe),
		       PIPESRC_WIDTH(width - 1) | PIPESRC_HEIGHT(height - 1));
}

static bool intel_pipe_is_interlaced(const struct intel_crtc_state *crtc_state)
{
	struct drm_i915_private *dev_priv = to_i915(crtc_state->uapi.crtc->dev);
	enum transcoder cpu_transcoder = crtc_state->cpu_transcoder;

	if (DISPLAY_VER(dev_priv) == 2)
		return false;

	if (DISPLAY_VER(dev_priv) >= 9 ||
	    IS_BROADWELL(dev_priv) || IS_HASWELL(dev_priv))
		return intel_de_read(dev_priv, TRANSCONF(cpu_transcoder)) & TRANSCONF_INTERLACE_MASK_HSW;
	else
		return intel_de_read(dev_priv, TRANSCONF(cpu_transcoder)) & TRANSCONF_INTERLACE_MASK;
}

static void intel_get_transcoder_timings(struct intel_crtc *crtc,
					 struct intel_crtc_state *pipe_config)
{
	struct drm_device *dev = crtc->base.dev;
	struct drm_i915_private *dev_priv = to_i915(dev);
	enum transcoder cpu_transcoder = pipe_config->cpu_transcoder;
	struct drm_display_mode *adjusted_mode = &pipe_config->hw.adjusted_mode;
	u32 tmp;

	tmp = intel_de_read(dev_priv, TRANS_HTOTAL(cpu_transcoder));
	adjusted_mode->crtc_hdisplay = REG_FIELD_GET(HACTIVE_MASK, tmp) + 1;
	adjusted_mode->crtc_htotal = REG_FIELD_GET(HTOTAL_MASK, tmp) + 1;

	if (!transcoder_is_dsi(cpu_transcoder)) {
		tmp = intel_de_read(dev_priv, TRANS_HBLANK(cpu_transcoder));
		adjusted_mode->crtc_hblank_start = REG_FIELD_GET(HBLANK_START_MASK, tmp) + 1;
		adjusted_mode->crtc_hblank_end = REG_FIELD_GET(HBLANK_END_MASK, tmp) + 1;
	}

	tmp = intel_de_read(dev_priv, TRANS_HSYNC(cpu_transcoder));
	adjusted_mode->crtc_hsync_start = REG_FIELD_GET(HSYNC_START_MASK, tmp) + 1;
	adjusted_mode->crtc_hsync_end = REG_FIELD_GET(HSYNC_END_MASK, tmp) + 1;

	tmp = intel_de_read(dev_priv, TRANS_VTOTAL(cpu_transcoder));
	adjusted_mode->crtc_vdisplay = REG_FIELD_GET(VACTIVE_MASK, tmp) + 1;
	adjusted_mode->crtc_vtotal = REG_FIELD_GET(VTOTAL_MASK, tmp) + 1;

	/* FIXME TGL+ DSI transcoders have this! */
	if (!transcoder_is_dsi(cpu_transcoder)) {
		tmp = intel_de_read(dev_priv, TRANS_VBLANK(cpu_transcoder));
		adjusted_mode->crtc_vblank_start = REG_FIELD_GET(VBLANK_START_MASK, tmp) + 1;
		adjusted_mode->crtc_vblank_end = REG_FIELD_GET(VBLANK_END_MASK, tmp) + 1;
	}
	tmp = intel_de_read(dev_priv, TRANS_VSYNC(cpu_transcoder));
	adjusted_mode->crtc_vsync_start = REG_FIELD_GET(VSYNC_START_MASK, tmp) + 1;
	adjusted_mode->crtc_vsync_end = REG_FIELD_GET(VSYNC_END_MASK, tmp) + 1;

	if (intel_pipe_is_interlaced(pipe_config)) {
		adjusted_mode->flags |= DRM_MODE_FLAG_INTERLACE;
		adjusted_mode->crtc_vtotal += 1;
		adjusted_mode->crtc_vblank_end += 1;
	}

	if (DISPLAY_VER(dev_priv) >= 13 && !transcoder_is_dsi(cpu_transcoder))
		adjusted_mode->crtc_vblank_start =
			adjusted_mode->crtc_vdisplay +
			intel_de_read(dev_priv, TRANS_SET_CONTEXT_LATENCY(cpu_transcoder));
}

static void intel_bigjoiner_adjust_pipe_src(struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	int num_pipes = intel_bigjoiner_num_pipes(crtc_state);
	enum pipe master_pipe, pipe = crtc->pipe;
	int width;

	if (num_pipes < 2)
		return;

	master_pipe = bigjoiner_master_pipe(crtc_state);
	width = drm_rect_width(&crtc_state->pipe_src);

	drm_rect_translate_to(&crtc_state->pipe_src,
			      (pipe - master_pipe) * width, 0);
}

static void intel_get_pipe_src_size(struct intel_crtc *crtc,
				    struct intel_crtc_state *pipe_config)
{
	struct drm_device *dev = crtc->base.dev;
	struct drm_i915_private *dev_priv = to_i915(dev);
	u32 tmp;

	tmp = intel_de_read(dev_priv, PIPESRC(crtc->pipe));

	drm_rect_init(&pipe_config->pipe_src, 0, 0,
		      REG_FIELD_GET(PIPESRC_WIDTH_MASK, tmp) + 1,
		      REG_FIELD_GET(PIPESRC_HEIGHT_MASK, tmp) + 1);

	intel_bigjoiner_adjust_pipe_src(pipe_config);
}

void i9xx_set_pipeconf(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum transcoder cpu_transcoder = crtc_state->cpu_transcoder;
	u32 val = 0;

	/*
	 * - We keep both pipes enabled on 830
	 * - During modeset the pipe is still disabled and must remain so
	 * - During fastset the pipe is already enabled and must remain so
	 */
	if (IS_I830(dev_priv) || !intel_crtc_needs_modeset(crtc_state))
		val |= TRANSCONF_ENABLE;

	if (crtc_state->double_wide)
		val |= TRANSCONF_DOUBLE_WIDE;

	/* only g4x and later have fancy bpc/dither controls */
	if (IS_G4X(dev_priv) || IS_VALLEYVIEW(dev_priv) ||
	    IS_CHERRYVIEW(dev_priv)) {
		/* Bspec claims that we can't use dithering for 30bpp pipes. */
		if (crtc_state->dither && crtc_state->pipe_bpp != 30)
			val |= TRANSCONF_DITHER_EN |
				TRANSCONF_DITHER_TYPE_SP;

		switch (crtc_state->pipe_bpp) {
		default:
			/* Case prevented by intel_choose_pipe_bpp_dither. */
			MISSING_CASE(crtc_state->pipe_bpp);
			fallthrough;
		case 18:
			val |= TRANSCONF_BPC_6;
			break;
		case 24:
			val |= TRANSCONF_BPC_8;
			break;
		case 30:
			val |= TRANSCONF_BPC_10;
			break;
		}
	}

	if (crtc_state->hw.adjusted_mode.flags & DRM_MODE_FLAG_INTERLACE) {
		if (DISPLAY_VER(dev_priv) < 4 ||
		    intel_crtc_has_type(crtc_state, INTEL_OUTPUT_SDVO))
			val |= TRANSCONF_INTERLACE_W_FIELD_INDICATION;
		else
			val |= TRANSCONF_INTERLACE_W_SYNC_SHIFT;
	} else {
		val |= TRANSCONF_INTERLACE_PROGRESSIVE;
	}

	if ((IS_VALLEYVIEW(dev_priv) || IS_CHERRYVIEW(dev_priv)) &&
	     crtc_state->limited_color_range)
		val |= TRANSCONF_COLOR_RANGE_SELECT;

	val |= TRANSCONF_GAMMA_MODE(crtc_state->gamma_mode);

	if (crtc_state->wgc_enable)
		val |= TRANSCONF_WGC_ENABLE;

	val |= TRANSCONF_FRAME_START_DELAY(crtc_state->framestart_delay - 1);

	intel_de_write(dev_priv, TRANSCONF(cpu_transcoder), val);
	intel_de_posting_read(dev_priv, TRANSCONF(cpu_transcoder));
}

static bool i9xx_has_pfit(struct drm_i915_private *dev_priv)
{
	if (IS_I830(dev_priv))
		return false;

	return DISPLAY_VER(dev_priv) >= 4 ||
		IS_PINEVIEW(dev_priv) || IS_MOBILE(dev_priv);
}

static void i9xx_get_pfit_config(struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum pipe pipe;
	u32 tmp;

	if (!i9xx_has_pfit(dev_priv))
		return;

	tmp = intel_de_read(dev_priv, PFIT_CONTROL);
	if (!(tmp & PFIT_ENABLE))
		return;

	/* Check whether the pfit is attached to our pipe. */
	if (DISPLAY_VER(dev_priv) >= 4)
		pipe = REG_FIELD_GET(PFIT_PIPE_MASK, tmp);
	else
		pipe = PIPE_B;

	if (pipe != crtc->pipe)
		return;

	crtc_state->gmch_pfit.control = tmp;
	crtc_state->gmch_pfit.pgm_ratios =
		intel_de_read(dev_priv, PFIT_PGM_RATIOS);
}

static void vlv_crtc_clock_get(struct intel_crtc *crtc,
			       struct intel_crtc_state *pipe_config)
{
	struct drm_device *dev = crtc->base.dev;
	struct drm_i915_private *dev_priv = to_i915(dev);
	enum pipe pipe = crtc->pipe;
	struct dpll clock;
	u32 mdiv;
	int refclk = 100000;

	/* In case of DSI, DPLL will not be used */
	if ((pipe_config->dpll_hw_state.dpll & DPLL_VCO_ENABLE) == 0)
		return;

	vlv_dpio_get(dev_priv);
	mdiv = vlv_dpio_read(dev_priv, pipe, VLV_PLL_DW3(pipe));
	vlv_dpio_put(dev_priv);

	clock.m1 = (mdiv >> DPIO_M1DIV_SHIFT) & 7;
	clock.m2 = mdiv & DPIO_M2DIV_MASK;
	clock.n = (mdiv >> DPIO_N_SHIFT) & 0xf;
	clock.p1 = (mdiv >> DPIO_P1_SHIFT) & 7;
	clock.p2 = (mdiv >> DPIO_P2_SHIFT) & 0x1f;

	pipe_config->port_clock = vlv_calc_dpll_params(refclk, &clock);
}

<<<<<<< HEAD
static void
i9xx_get_initial_plane_config(struct intel_crtc *crtc,
			      struct intel_initial_plane_config *plane_config)
{
	struct drm_device *dev = crtc->base.dev;
	struct drm_i915_private *dev_priv = to_i915(dev);
	struct intel_plane *plane = to_intel_plane(crtc->base.primary);
	enum i9xx_plane_id i9xx_plane = plane->i9xx_plane;
	enum pipe pipe;
	u32 val, base, offset __unused;
	int fourcc, pixel_format;
	unsigned int aligned_height;
	struct drm_framebuffer *fb;
	struct intel_framebuffer *intel_fb;

	if (!plane->get_hw_state(plane, &pipe))
		return;

	WARN_ON(pipe != crtc->pipe);

	intel_fb = kzalloc(sizeof(*intel_fb), GFP_KERNEL);
	if (!intel_fb) {
		DRM_DEBUG_KMS("failed to alloc fb\n");
		return;
	}

	fb = &intel_fb->base;

	fb->dev = dev;

	val = I915_READ(DSPCNTR(i9xx_plane));

	if (INTEL_GEN(dev_priv) >= 4) {
		if (val & DISPPLANE_TILED) {
			plane_config->tiling = I915_TILING_X;
			fb->modifier = I915_FORMAT_MOD_X_TILED;
		}

		if (val & DISPPLANE_ROTATE_180)
			plane_config->rotation = DRM_MODE_ROTATE_180;
	}

	if (IS_CHERRYVIEW(dev_priv) && pipe == PIPE_B &&
	    val & DISPPLANE_MIRROR)
		plane_config->rotation |= DRM_MODE_REFLECT_X;

	pixel_format = val & DISPPLANE_PIXFORMAT_MASK;
	fourcc = i9xx_format_to_fourcc(pixel_format);
	fb->format = drm_format_info(fourcc);

	if (IS_HASWELL(dev_priv) || IS_BROADWELL(dev_priv)) {
		offset = I915_READ(DSPOFFSET(i9xx_plane));
		base = I915_READ(DSPSURF(i9xx_plane)) & 0xfffff000;
	} else if (INTEL_GEN(dev_priv) >= 4) {
		if (plane_config->tiling)
			offset = I915_READ(DSPTILEOFF(i9xx_plane));
		else
			offset = I915_READ(DSPLINOFF(i9xx_plane));
		base = I915_READ(DSPSURF(i9xx_plane)) & 0xfffff000;
	} else {
		base = I915_READ(DSPADDR(i9xx_plane));
	}
	plane_config->base = base;

	val = I915_READ(PIPESRC(pipe));
	fb->width = ((val >> 16) & 0xfff) + 1;
	fb->height = ((val >> 0) & 0xfff) + 1;

	val = I915_READ(DSPSTRIDE(i9xx_plane));
	fb->pitches[0] = val & 0xffffffc0;

	aligned_height = intel_fb_align_height(fb, 0, fb->height);

	plane_config->size = fb->pitches[0] * aligned_height;

	DRM_DEBUG_KMS("%s/%s with fb: size=%dx%d@%d, offset=%x, pitch %d, size 0x%x\n",
		      crtc->base.name, plane->base.name, fb->width, fb->height,
		      fb->format->cpp[0] * 8, base, fb->pitches[0],
		      plane_config->size);

	plane_config->fb = intel_fb;
}

=======
>>>>>>> vendor/linux-drm-v6.6.35
static void chv_crtc_clock_get(struct intel_crtc *crtc,
			       struct intel_crtc_state *pipe_config)
{
	struct drm_device *dev = crtc->base.dev;
	struct drm_i915_private *dev_priv = to_i915(dev);
	enum pipe pipe = crtc->pipe;
	enum dpio_channel port = vlv_pipe_to_channel(pipe);
	struct dpll clock;
	u32 cmn_dw13, pll_dw0, pll_dw1, pll_dw2, pll_dw3;
	int refclk = 100000;

	/* In case of DSI, DPLL will not be used */
	if ((pipe_config->dpll_hw_state.dpll & DPLL_VCO_ENABLE) == 0)
		return;

	vlv_dpio_get(dev_priv);
	cmn_dw13 = vlv_dpio_read(dev_priv, pipe, CHV_CMN_DW13(port));
	pll_dw0 = vlv_dpio_read(dev_priv, pipe, CHV_PLL_DW0(port));
	pll_dw1 = vlv_dpio_read(dev_priv, pipe, CHV_PLL_DW1(port));
	pll_dw2 = vlv_dpio_read(dev_priv, pipe, CHV_PLL_DW2(port));
	pll_dw3 = vlv_dpio_read(dev_priv, pipe, CHV_PLL_DW3(port));
	vlv_dpio_put(dev_priv);

	clock.m1 = (pll_dw1 & 0x7) == DPIO_CHV_M1_DIV_BY_2 ? 2 : 0;
	clock.m2 = (pll_dw0 & 0xff) << 22;
	if (pll_dw3 & DPIO_CHV_FRAC_DIV_EN)
		clock.m2 |= pll_dw2 & 0x3fffff;
	clock.n = (pll_dw1 >> DPIO_CHV_N_DIV_SHIFT) & 0xf;
	clock.p1 = (cmn_dw13 >> DPIO_CHV_P1_DIV_SHIFT) & 0x7;
	clock.p2 = (cmn_dw13 >> DPIO_CHV_P2_DIV_SHIFT) & 0x1f;

	pipe_config->port_clock = chv_calc_dpll_params(refclk, &clock);
}

static enum intel_output_format
bdw_get_pipe_misc_output_format(struct intel_crtc *crtc)
{
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	u32 tmp;

	tmp = intel_de_read(dev_priv, PIPE_MISC(crtc->pipe));

	if (tmp & PIPE_MISC_YUV420_ENABLE) {
		/* We support 4:2:0 in full blend mode only */
		drm_WARN_ON(&dev_priv->drm,
			    (tmp & PIPE_MISC_YUV420_MODE_FULL_BLEND) == 0);

		return INTEL_OUTPUT_FORMAT_YCBCR420;
	} else if (tmp & PIPE_MISC_OUTPUT_COLORSPACE_YUV) {
		return INTEL_OUTPUT_FORMAT_YCBCR444;
	} else {
		return INTEL_OUTPUT_FORMAT_RGB;
	}
}

static void i9xx_get_pipe_color_config(struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct intel_plane *plane = to_intel_plane(crtc->base.primary);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum i9xx_plane_id i9xx_plane = plane->i9xx_plane;
	u32 tmp;

	tmp = intel_de_read(dev_priv, DSPCNTR(i9xx_plane));

	if (tmp & DISP_PIPE_GAMMA_ENABLE)
		crtc_state->gamma_enable = true;

	if (!HAS_GMCH(dev_priv) &&
	    tmp & DISP_PIPE_CSC_ENABLE)
		crtc_state->csc_enable = true;
}

static bool i9xx_get_pipe_config(struct intel_crtc *crtc,
				 struct intel_crtc_state *pipe_config)
{
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum intel_display_power_domain power_domain;
	intel_wakeref_t wakeref;
	u32 tmp;
	bool ret;

	power_domain = POWER_DOMAIN_PIPE(crtc->pipe);
	wakeref = intel_display_power_get_if_enabled(dev_priv, power_domain);
	if (!wakeref)
		return false;

	pipe_config->output_format = INTEL_OUTPUT_FORMAT_RGB;
	pipe_config->sink_format = pipe_config->output_format;
	pipe_config->cpu_transcoder = (enum transcoder) crtc->pipe;
	pipe_config->shared_dpll = NULL;

	ret = false;

	tmp = intel_de_read(dev_priv, TRANSCONF(pipe_config->cpu_transcoder));
	if (!(tmp & TRANSCONF_ENABLE))
		goto out;

	if (IS_G4X(dev_priv) || IS_VALLEYVIEW(dev_priv) ||
	    IS_CHERRYVIEW(dev_priv)) {
		switch (tmp & TRANSCONF_BPC_MASK) {
		case TRANSCONF_BPC_6:
			pipe_config->pipe_bpp = 18;
			break;
		case TRANSCONF_BPC_8:
			pipe_config->pipe_bpp = 24;
			break;
		case TRANSCONF_BPC_10:
			pipe_config->pipe_bpp = 30;
			break;
		default:
			MISSING_CASE(tmp);
			break;
		}
	}

	if ((IS_VALLEYVIEW(dev_priv) || IS_CHERRYVIEW(dev_priv)) &&
	    (tmp & TRANSCONF_COLOR_RANGE_SELECT))
		pipe_config->limited_color_range = true;

	pipe_config->gamma_mode = REG_FIELD_GET(TRANSCONF_GAMMA_MODE_MASK_I9XX, tmp);

	pipe_config->framestart_delay = REG_FIELD_GET(TRANSCONF_FRAME_START_DELAY_MASK, tmp) + 1;

	if ((IS_VALLEYVIEW(dev_priv) || IS_CHERRYVIEW(dev_priv)) &&
	    (tmp & TRANSCONF_WGC_ENABLE))
		pipe_config->wgc_enable = true;

	if (IS_CHERRYVIEW(dev_priv))
		pipe_config->cgm_mode = intel_de_read(dev_priv,
						      CGM_PIPE_MODE(crtc->pipe));

	i9xx_get_pipe_color_config(pipe_config);
	intel_color_get_config(pipe_config);

	if (DISPLAY_VER(dev_priv) < 4)
		pipe_config->double_wide = tmp & TRANSCONF_DOUBLE_WIDE;

	intel_get_transcoder_timings(crtc, pipe_config);
	intel_get_pipe_src_size(crtc, pipe_config);

	i9xx_get_pfit_config(pipe_config);

	if (DISPLAY_VER(dev_priv) >= 4) {
		/* No way to read it out on pipes B and C */
		if (IS_CHERRYVIEW(dev_priv) && crtc->pipe != PIPE_A)
			tmp = dev_priv->display.state.chv_dpll_md[crtc->pipe];
		else
			tmp = intel_de_read(dev_priv, DPLL_MD(crtc->pipe));
		pipe_config->pixel_multiplier =
			((tmp & DPLL_MD_UDI_MULTIPLIER_MASK)
			 >> DPLL_MD_UDI_MULTIPLIER_SHIFT) + 1;
		pipe_config->dpll_hw_state.dpll_md = tmp;
	} else if (IS_I945G(dev_priv) || IS_I945GM(dev_priv) ||
		   IS_G33(dev_priv) || IS_PINEVIEW(dev_priv)) {
		tmp = intel_de_read(dev_priv, DPLL(crtc->pipe));
		pipe_config->pixel_multiplier =
			((tmp & SDVO_MULTIPLIER_MASK)
			 >> SDVO_MULTIPLIER_SHIFT_HIRES) + 1;
	} else {
		/* Note that on i915G/GM the pixel multiplier is in the sdvo
		 * port and will be fixed up in the encoder->get_config
		 * function. */
		pipe_config->pixel_multiplier = 1;
	}
	pipe_config->dpll_hw_state.dpll = intel_de_read(dev_priv,
							DPLL(crtc->pipe));
	if (!IS_VALLEYVIEW(dev_priv) && !IS_CHERRYVIEW(dev_priv)) {
		pipe_config->dpll_hw_state.fp0 = intel_de_read(dev_priv,
							       FP0(crtc->pipe));
		pipe_config->dpll_hw_state.fp1 = intel_de_read(dev_priv,
							       FP1(crtc->pipe));
	} else {
		/* Mask out read-only status bits. */
		pipe_config->dpll_hw_state.dpll &= ~(DPLL_LOCK_VLV |
						     DPLL_PORTC_READY_MASK |
						     DPLL_PORTB_READY_MASK);
	}

	if (IS_CHERRYVIEW(dev_priv))
		chv_crtc_clock_get(crtc, pipe_config);
	else if (IS_VALLEYVIEW(dev_priv))
		vlv_crtc_clock_get(crtc, pipe_config);
	else
		i9xx_crtc_clock_get(crtc, pipe_config);

	/*
	 * Normally the dotclock is filled in by the encoder .get_config()
	 * but in case the pipe is enabled w/o any ports we need a sane
	 * default.
	 */
	pipe_config->hw.adjusted_mode.crtc_clock =
		pipe_config->port_clock / pipe_config->pixel_multiplier;

	ret = true;

out:
	intel_display_power_put(dev_priv, power_domain, wakeref);

	return ret;
}

void ilk_set_pipeconf(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum transcoder cpu_transcoder = crtc_state->cpu_transcoder;
	u32 val = 0;

	/*
	 * - During modeset the pipe is still disabled and must remain so
	 * - During fastset the pipe is already enabled and must remain so
	 */
	if (!intel_crtc_needs_modeset(crtc_state))
		val |= TRANSCONF_ENABLE;

	switch (crtc_state->pipe_bpp) {
	default:
		/* Case prevented by intel_choose_pipe_bpp_dither. */
		MISSING_CASE(crtc_state->pipe_bpp);
		fallthrough;
	case 18:
		val |= TRANSCONF_BPC_6;
		break;
	case 24:
		val |= TRANSCONF_BPC_8;
		break;
	case 30:
		val |= TRANSCONF_BPC_10;
		break;
	case 36:
		val |= TRANSCONF_BPC_12;
		break;
	}

	if (crtc_state->dither)
		val |= TRANSCONF_DITHER_EN | TRANSCONF_DITHER_TYPE_SP;

	if (crtc_state->hw.adjusted_mode.flags & DRM_MODE_FLAG_INTERLACE)
		val |= TRANSCONF_INTERLACE_IF_ID_ILK;
	else
		val |= TRANSCONF_INTERLACE_PF_PD_ILK;

	/*
	 * This would end up with an odd purple hue over
	 * the entire display. Make sure we don't do it.
	 */
	drm_WARN_ON(&dev_priv->drm, crtc_state->limited_color_range &&
		    crtc_state->output_format != INTEL_OUTPUT_FORMAT_RGB);

	if (crtc_state->limited_color_range &&
	    !intel_crtc_has_type(crtc_state, INTEL_OUTPUT_SDVO))
		val |= TRANSCONF_COLOR_RANGE_SELECT;

	if (crtc_state->output_format != INTEL_OUTPUT_FORMAT_RGB)
		val |= TRANSCONF_OUTPUT_COLORSPACE_YUV709;

	val |= TRANSCONF_GAMMA_MODE(crtc_state->gamma_mode);

	val |= TRANSCONF_FRAME_START_DELAY(crtc_state->framestart_delay - 1);
	val |= TRANSCONF_MSA_TIMING_DELAY(crtc_state->msa_timing_delay);

	intel_de_write(dev_priv, TRANSCONF(cpu_transcoder), val);
	intel_de_posting_read(dev_priv, TRANSCONF(cpu_transcoder));
}

static void hsw_set_transconf(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum transcoder cpu_transcoder = crtc_state->cpu_transcoder;
	u32 val = 0;

	/*
	 * - During modeset the pipe is still disabled and must remain so
	 * - During fastset the pipe is already enabled and must remain so
	 */
	if (!intel_crtc_needs_modeset(crtc_state))
		val |= TRANSCONF_ENABLE;

	if (IS_HASWELL(dev_priv) && crtc_state->dither)
		val |= TRANSCONF_DITHER_EN | TRANSCONF_DITHER_TYPE_SP;

	if (crtc_state->hw.adjusted_mode.flags & DRM_MODE_FLAG_INTERLACE)
		val |= TRANSCONF_INTERLACE_IF_ID_ILK;
	else
		val |= TRANSCONF_INTERLACE_PF_PD_ILK;

	if (IS_HASWELL(dev_priv) &&
	    crtc_state->output_format != INTEL_OUTPUT_FORMAT_RGB)
		val |= TRANSCONF_OUTPUT_COLORSPACE_YUV_HSW;

	intel_de_write(dev_priv, TRANSCONF(cpu_transcoder), val);
	intel_de_posting_read(dev_priv, TRANSCONF(cpu_transcoder));
}

static void bdw_set_pipe_misc(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	u32 val = 0;

	switch (crtc_state->pipe_bpp) {
	case 18:
		val |= PIPE_MISC_BPC_6;
		break;
	case 24:
		val |= PIPE_MISC_BPC_8;
		break;
	case 30:
		val |= PIPE_MISC_BPC_10;
		break;
	case 36:
		/* Port output 12BPC defined for ADLP+ */
		if (DISPLAY_VER(dev_priv) > 12)
			val |= PIPE_MISC_BPC_12_ADLP;
		break;
	default:
		MISSING_CASE(crtc_state->pipe_bpp);
		break;
	}

	if (crtc_state->dither)
		val |= PIPE_MISC_DITHER_ENABLE | PIPE_MISC_DITHER_TYPE_SP;

	if (crtc_state->output_format == INTEL_OUTPUT_FORMAT_YCBCR420 ||
	    crtc_state->output_format == INTEL_OUTPUT_FORMAT_YCBCR444)
		val |= PIPE_MISC_OUTPUT_COLORSPACE_YUV;

	if (crtc_state->output_format == INTEL_OUTPUT_FORMAT_YCBCR420)
		val |= PIPE_MISC_YUV420_ENABLE |
			PIPE_MISC_YUV420_MODE_FULL_BLEND;

	if (DISPLAY_VER(dev_priv) >= 11 && is_hdr_mode(crtc_state))
		val |= PIPE_MISC_HDR_MODE_PRECISION;

	if (DISPLAY_VER(dev_priv) >= 12)
		val |= PIPE_MISC_PIXEL_ROUNDING_TRUNC;

	/* allow PSR with sprite enabled */
	if (IS_BROADWELL(dev_priv))
		val |= PIPE_MISC_PSR_MASK_SPRITE_ENABLE;

	intel_de_write(dev_priv, PIPE_MISC(crtc->pipe), val);
}

int bdw_get_pipe_misc_bpp(struct intel_crtc *crtc)
{
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	u32 tmp;

	tmp = intel_de_read(dev_priv, PIPE_MISC(crtc->pipe));

	switch (tmp & PIPE_MISC_BPC_MASK) {
	case PIPE_MISC_BPC_6:
		return 18;
	case PIPE_MISC_BPC_8:
		return 24;
	case PIPE_MISC_BPC_10:
		return 30;
	/*
	 * PORT OUTPUT 12 BPC defined for ADLP+.
	 *
	 * TODO:
	 * For previous platforms with DSI interface, bits 5:7
	 * are used for storing pipe_bpp irrespective of dithering.
	 * Since the value of 12 BPC is not defined for these bits
	 * on older platforms, need to find a workaround for 12 BPC
	 * MIPI DSI HW readout.
	 */
	case PIPE_MISC_BPC_12_ADLP:
		if (DISPLAY_VER(dev_priv) > 12)
			return 36;
		fallthrough;
	default:
		MISSING_CASE(tmp);
		return 0;
	}
}

int ilk_get_lanes_required(int target_clock, int link_bw, int bpp)
{
	/*
	 * Account for spread spectrum to avoid
	 * oversubscribing the link. Max center spread
	 * is 2.5%; use 5% for safety's sake.
	 */
	u32 bps = target_clock * bpp * 21 / 20;
	return DIV_ROUND_UP(bps, link_bw * 8);
}

void intel_get_m_n(struct drm_i915_private *i915,
		   struct intel_link_m_n *m_n,
		   i915_reg_t data_m_reg, i915_reg_t data_n_reg,
		   i915_reg_t link_m_reg, i915_reg_t link_n_reg)
{
	m_n->link_m = intel_de_read(i915, link_m_reg) & DATA_LINK_M_N_MASK;
	m_n->link_n = intel_de_read(i915, link_n_reg) & DATA_LINK_M_N_MASK;
	m_n->data_m = intel_de_read(i915, data_m_reg) & DATA_LINK_M_N_MASK;
	m_n->data_n = intel_de_read(i915, data_n_reg) & DATA_LINK_M_N_MASK;
	m_n->tu = REG_FIELD_GET(TU_SIZE_MASK, intel_de_read(i915, data_m_reg)) + 1;
}

void intel_cpu_transcoder_get_m1_n1(struct intel_crtc *crtc,
				    enum transcoder transcoder,
				    struct intel_link_m_n *m_n)
{
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum pipe pipe = crtc->pipe;

	if (DISPLAY_VER(dev_priv) >= 5)
		intel_get_m_n(dev_priv, m_n,
			      PIPE_DATA_M1(transcoder), PIPE_DATA_N1(transcoder),
			      PIPE_LINK_M1(transcoder), PIPE_LINK_N1(transcoder));
	else
		intel_get_m_n(dev_priv, m_n,
			      PIPE_DATA_M_G4X(pipe), PIPE_DATA_N_G4X(pipe),
			      PIPE_LINK_M_G4X(pipe), PIPE_LINK_N_G4X(pipe));
}

void intel_cpu_transcoder_get_m2_n2(struct intel_crtc *crtc,
				    enum transcoder transcoder,
				    struct intel_link_m_n *m_n)
{
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);

	if (!intel_cpu_transcoder_has_m2_n2(dev_priv, transcoder))
		return;

	intel_get_m_n(dev_priv, m_n,
		      PIPE_DATA_M2(transcoder), PIPE_DATA_N2(transcoder),
		      PIPE_LINK_M2(transcoder), PIPE_LINK_N2(transcoder));
}

static void ilk_get_pfit_config(struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	u32 ctl, pos, size;
	enum pipe pipe;
<<<<<<< HEAD
	u32 val, base, offset __unused, stride_mult, tiling, alpha;
	int fourcc, pixel_format;
	unsigned int aligned_height;
	struct drm_framebuffer *fb;
	struct intel_framebuffer *intel_fb;
=======
>>>>>>> vendor/linux-drm-v6.6.35

	ctl = intel_de_read(dev_priv, PF_CTL(crtc->pipe));
	if ((ctl & PF_ENABLE) == 0)
		return;

	if (IS_IVYBRIDGE(dev_priv) || IS_HASWELL(dev_priv))
		pipe = REG_FIELD_GET(PF_PIPE_SEL_MASK_IVB, ctl);
	else
		pipe = crtc->pipe;

	crtc_state->pch_pfit.enabled = true;

	pos = intel_de_read(dev_priv, PF_WIN_POS(crtc->pipe));
	size = intel_de_read(dev_priv, PF_WIN_SZ(crtc->pipe));

	drm_rect_init(&crtc_state->pch_pfit.dst,
		      REG_FIELD_GET(PF_WIN_XPOS_MASK, pos),
		      REG_FIELD_GET(PF_WIN_YPOS_MASK, pos),
		      REG_FIELD_GET(PF_WIN_XSIZE_MASK, size),
		      REG_FIELD_GET(PF_WIN_YSIZE_MASK, size));

	/*
	 * We currently do not free assignements of panel fitters on
	 * ivb/hsw (since we don't use the higher upscaling modes which
	 * differentiates them) so just WARN about this case for now.
	 */
	drm_WARN_ON(&dev_priv->drm, pipe != crtc->pipe);
}

static bool ilk_get_pipe_config(struct intel_crtc *crtc,
				struct intel_crtc_state *pipe_config)
{
	struct drm_device *dev = crtc->base.dev;
	struct drm_i915_private *dev_priv = to_i915(dev);
	enum intel_display_power_domain power_domain;
	intel_wakeref_t wakeref;
	u32 tmp;
	bool ret;

	power_domain = POWER_DOMAIN_PIPE(crtc->pipe);
	wakeref = intel_display_power_get_if_enabled(dev_priv, power_domain);
	if (!wakeref)
		return false;

	pipe_config->cpu_transcoder = (enum transcoder) crtc->pipe;
	pipe_config->shared_dpll = NULL;

	ret = false;
	tmp = intel_de_read(dev_priv, TRANSCONF(pipe_config->cpu_transcoder));
	if (!(tmp & TRANSCONF_ENABLE))
		goto out;

	switch (tmp & TRANSCONF_BPC_MASK) {
	case TRANSCONF_BPC_6:
		pipe_config->pipe_bpp = 18;
		break;
	case TRANSCONF_BPC_8:
		pipe_config->pipe_bpp = 24;
		break;
	case TRANSCONF_BPC_10:
		pipe_config->pipe_bpp = 30;
		break;
	case TRANSCONF_BPC_12:
		pipe_config->pipe_bpp = 36;
		break;
	default:
		break;
	}

	if (tmp & TRANSCONF_COLOR_RANGE_SELECT)
		pipe_config->limited_color_range = true;

	switch (tmp & TRANSCONF_OUTPUT_COLORSPACE_MASK) {
	case TRANSCONF_OUTPUT_COLORSPACE_YUV601:
	case TRANSCONF_OUTPUT_COLORSPACE_YUV709:
		pipe_config->output_format = INTEL_OUTPUT_FORMAT_YCBCR444;
		break;
	default:
		pipe_config->output_format = INTEL_OUTPUT_FORMAT_RGB;
		break;
	}

	pipe_config->sink_format = pipe_config->output_format;

	pipe_config->gamma_mode = REG_FIELD_GET(TRANSCONF_GAMMA_MODE_MASK_ILK, tmp);

	pipe_config->framestart_delay = REG_FIELD_GET(TRANSCONF_FRAME_START_DELAY_MASK, tmp) + 1;

	pipe_config->msa_timing_delay = REG_FIELD_GET(TRANSCONF_MSA_TIMING_DELAY_MASK, tmp);

	pipe_config->csc_mode = intel_de_read(dev_priv,
					      PIPE_CSC_MODE(crtc->pipe));

	i9xx_get_pipe_color_config(pipe_config);
	intel_color_get_config(pipe_config);

	pipe_config->pixel_multiplier = 1;

	ilk_pch_get_config(pipe_config);

	intel_get_transcoder_timings(crtc, pipe_config);
	intel_get_pipe_src_size(crtc, pipe_config);

	ilk_get_pfit_config(pipe_config);

	ret = true;

out:
	intel_display_power_put(dev_priv, power_domain, wakeref);

	return ret;
}

static u8 bigjoiner_pipes(struct drm_i915_private *i915)
{
	u8 pipes;

	if (DISPLAY_VER(i915) >= 12)
		pipes = BIT(PIPE_A) | BIT(PIPE_B) | BIT(PIPE_C) | BIT(PIPE_D);
	else if (DISPLAY_VER(i915) >= 11)
		pipes = BIT(PIPE_B) | BIT(PIPE_C);
	else
		pipes = 0;

	return pipes & DISPLAY_RUNTIME_INFO(i915)->pipe_mask;
}

static bool transcoder_ddi_func_is_enabled(struct drm_i915_private *dev_priv,
					   enum transcoder cpu_transcoder)
{
	enum intel_display_power_domain power_domain;
	intel_wakeref_t wakeref;
	u32 tmp = 0;

	power_domain = POWER_DOMAIN_TRANSCODER(cpu_transcoder);

	with_intel_display_power_if_enabled(dev_priv, power_domain, wakeref)
		tmp = intel_de_read(dev_priv, TRANS_DDI_FUNC_CTL(cpu_transcoder));

	return tmp & TRANS_DDI_FUNC_ENABLE;
}

static void enabled_bigjoiner_pipes(struct drm_i915_private *dev_priv,
				    u8 *master_pipes, u8 *slave_pipes)
{
	struct intel_crtc *crtc;

	*master_pipes = 0;
	*slave_pipes = 0;

	for_each_intel_crtc_in_pipe_mask(&dev_priv->drm, crtc,
					 bigjoiner_pipes(dev_priv)) {
		enum intel_display_power_domain power_domain;
		enum pipe pipe = crtc->pipe;
		intel_wakeref_t wakeref;

		power_domain = intel_dsc_power_domain(crtc, (enum transcoder) pipe);
		with_intel_display_power_if_enabled(dev_priv, power_domain, wakeref) {
			u32 tmp = intel_de_read(dev_priv, ICL_PIPE_DSS_CTL1(pipe));

			if (!(tmp & BIG_JOINER_ENABLE))
				continue;

			if (tmp & MASTER_BIG_JOINER_ENABLE)
				*master_pipes |= BIT(pipe);
			else
				*slave_pipes |= BIT(pipe);
		}

		if (DISPLAY_VER(dev_priv) < 13)
			continue;

		power_domain = POWER_DOMAIN_PIPE(pipe);
		with_intel_display_power_if_enabled(dev_priv, power_domain, wakeref) {
			u32 tmp = intel_de_read(dev_priv, ICL_PIPE_DSS_CTL1(pipe));

			if (tmp & UNCOMPRESSED_JOINER_MASTER)
				*master_pipes |= BIT(pipe);
			if (tmp & UNCOMPRESSED_JOINER_SLAVE)
				*slave_pipes |= BIT(pipe);
		}
	}

	/* Bigjoiner pipes should always be consecutive master and slave */
	drm_WARN(&dev_priv->drm, *slave_pipes != *master_pipes << 1,
		 "Bigjoiner misconfigured (master pipes 0x%x, slave pipes 0x%x)\n",
		 *master_pipes, *slave_pipes);
}

static enum pipe get_bigjoiner_master_pipe(enum pipe pipe, u8 master_pipes, u8 slave_pipes)
{
	if ((slave_pipes & BIT(pipe)) == 0)
		return pipe;

	/* ignore everything above our pipe */
	master_pipes &= ~GENMASK(7, pipe);

	/* highest remaining bit should be our master pipe */
	return fls(master_pipes) - 1;
}

static u8 get_bigjoiner_slave_pipes(enum pipe pipe, u8 master_pipes, u8 slave_pipes)
{
	enum pipe master_pipe, next_master_pipe;

	master_pipe = get_bigjoiner_master_pipe(pipe, master_pipes, slave_pipes);

	if ((master_pipes & BIT(master_pipe)) == 0)
		return 0;

	/* ignore our master pipe and everything below it */
	master_pipes &= ~GENMASK(master_pipe, 0);
	/* make sure a high bit is set for the ffs() */
	master_pipes |= BIT(7);
	/* lowest remaining bit should be the next master pipe */
	next_master_pipe = ffs(master_pipes) - 1;

	return slave_pipes & GENMASK(next_master_pipe - 1, master_pipe);
}

static u8 hsw_panel_transcoders(struct drm_i915_private *i915)
{
	u8 panel_transcoder_mask = BIT(TRANSCODER_EDP);

	if (DISPLAY_VER(i915) >= 11)
		panel_transcoder_mask |= BIT(TRANSCODER_DSI_0) | BIT(TRANSCODER_DSI_1);

	return panel_transcoder_mask;
}

static u8 hsw_enabled_transcoders(struct intel_crtc *crtc)
{
	struct drm_device *dev = crtc->base.dev;
	struct drm_i915_private *dev_priv = to_i915(dev);
	u8 panel_transcoder_mask = hsw_panel_transcoders(dev_priv);
	enum transcoder cpu_transcoder;
	u8 master_pipes, slave_pipes;
	u8 enabled_transcoders = 0;

	/*
	 * XXX: Do intel_display_power_get_if_enabled before reading this (for
	 * consistency and less surprising code; it's in always on power).
	 */
	for_each_cpu_transcoder_masked(dev_priv, cpu_transcoder,
				       panel_transcoder_mask) {
		enum intel_display_power_domain power_domain;
		intel_wakeref_t wakeref;
		enum pipe trans_pipe;
		u32 tmp = 0;

		power_domain = POWER_DOMAIN_TRANSCODER(cpu_transcoder);
		with_intel_display_power_if_enabled(dev_priv, power_domain, wakeref)
			tmp = intel_de_read(dev_priv, TRANS_DDI_FUNC_CTL(cpu_transcoder));

		if (!(tmp & TRANS_DDI_FUNC_ENABLE))
			continue;

		switch (tmp & TRANS_DDI_EDP_INPUT_MASK) {
		default:
			drm_WARN(dev, 1,
				 "unknown pipe linked to transcoder %s\n",
				 transcoder_name(cpu_transcoder));
			fallthrough;
		case TRANS_DDI_EDP_INPUT_A_ONOFF:
		case TRANS_DDI_EDP_INPUT_A_ON:
			trans_pipe = PIPE_A;
			break;
		case TRANS_DDI_EDP_INPUT_B_ONOFF:
			trans_pipe = PIPE_B;
			break;
		case TRANS_DDI_EDP_INPUT_C_ONOFF:
			trans_pipe = PIPE_C;
			break;
		case TRANS_DDI_EDP_INPUT_D_ONOFF:
			trans_pipe = PIPE_D;
			break;
		}

		if (trans_pipe == crtc->pipe)
			enabled_transcoders |= BIT(cpu_transcoder);
	}

	/* single pipe or bigjoiner master */
	cpu_transcoder = (enum transcoder) crtc->pipe;
	if (transcoder_ddi_func_is_enabled(dev_priv, cpu_transcoder))
		enabled_transcoders |= BIT(cpu_transcoder);

	/* bigjoiner slave -> consider the master pipe's transcoder as well */
	enabled_bigjoiner_pipes(dev_priv, &master_pipes, &slave_pipes);
	if (slave_pipes & BIT(crtc->pipe)) {
		cpu_transcoder = (enum transcoder)
			get_bigjoiner_master_pipe(crtc->pipe, master_pipes, slave_pipes);
		if (transcoder_ddi_func_is_enabled(dev_priv, cpu_transcoder))
			enabled_transcoders |= BIT(cpu_transcoder);
	}

	return enabled_transcoders;
}

static bool has_edp_transcoders(u8 enabled_transcoders)
{
	return enabled_transcoders & BIT(TRANSCODER_EDP);
}

static bool has_dsi_transcoders(u8 enabled_transcoders)
{
	return enabled_transcoders & (BIT(TRANSCODER_DSI_0) |
				      BIT(TRANSCODER_DSI_1));
}

static bool has_pipe_transcoders(u8 enabled_transcoders)
{
	return enabled_transcoders & ~(BIT(TRANSCODER_EDP) |
				       BIT(TRANSCODER_DSI_0) |
				       BIT(TRANSCODER_DSI_1));
}

static void assert_enabled_transcoders(struct drm_i915_private *i915,
				       u8 enabled_transcoders)
{
	/* Only one type of transcoder please */
	drm_WARN_ON(&i915->drm,
		    has_edp_transcoders(enabled_transcoders) +
		    has_dsi_transcoders(enabled_transcoders) +
		    has_pipe_transcoders(enabled_transcoders) > 1);

	/* Only DSI transcoders can be ganged */
	drm_WARN_ON(&i915->drm,
		    !has_dsi_transcoders(enabled_transcoders) &&
		    !is_power_of_2(enabled_transcoders));
}

static bool hsw_get_transcoder_state(struct intel_crtc *crtc,
				     struct intel_crtc_state *pipe_config,
				     struct intel_display_power_domain_set *power_domain_set)
{
	struct drm_device *dev = crtc->base.dev;
	struct drm_i915_private *dev_priv = to_i915(dev);
	unsigned long enabled_transcoders;
	u32 tmp;

	enabled_transcoders = hsw_enabled_transcoders(crtc);
	if (!enabled_transcoders)
		return false;

	assert_enabled_transcoders(dev_priv, enabled_transcoders);

	/*
	 * With the exception of DSI we should only ever have
	 * a single enabled transcoder. With DSI let's just
	 * pick the first one.
	 */
	pipe_config->cpu_transcoder = ffs(enabled_transcoders) - 1;

	if (!intel_display_power_get_in_set_if_enabled(dev_priv, power_domain_set,
						       POWER_DOMAIN_TRANSCODER(pipe_config->cpu_transcoder)))
		return false;

	if (hsw_panel_transcoders(dev_priv) & BIT(pipe_config->cpu_transcoder)) {
		tmp = intel_de_read(dev_priv, TRANS_DDI_FUNC_CTL(pipe_config->cpu_transcoder));

		if ((tmp & TRANS_DDI_EDP_INPUT_MASK) == TRANS_DDI_EDP_INPUT_A_ONOFF)
			pipe_config->pch_pfit.force_thru = true;
	}

	tmp = intel_de_read(dev_priv, TRANSCONF(pipe_config->cpu_transcoder));

	return tmp & TRANSCONF_ENABLE;
}

static bool bxt_get_dsi_transcoder_state(struct intel_crtc *crtc,
					 struct intel_crtc_state *pipe_config,
					 struct intel_display_power_domain_set *power_domain_set)
{
	struct drm_device *dev = crtc->base.dev;
	struct drm_i915_private *dev_priv = to_i915(dev);
	enum transcoder cpu_transcoder;
	enum port port;
	u32 tmp;

	for_each_port_masked(port, BIT(PORT_A) | BIT(PORT_C)) {
		if (port == PORT_A)
			cpu_transcoder = TRANSCODER_DSI_A;
		else
			cpu_transcoder = TRANSCODER_DSI_C;

		if (!intel_display_power_get_in_set_if_enabled(dev_priv, power_domain_set,
							       POWER_DOMAIN_TRANSCODER(cpu_transcoder)))
			continue;

		/*
		 * The PLL needs to be enabled with a valid divider
		 * configuration, otherwise accessing DSI registers will hang
		 * the machine. See BSpec North Display Engine
		 * registers/MIPI[BXT]. We can break out here early, since we
		 * need the same DSI PLL to be enabled for both DSI ports.
		 */
		if (!bxt_dsi_pll_is_enabled(dev_priv))
			break;

		/* XXX: this works for video mode only */
		tmp = intel_de_read(dev_priv, BXT_MIPI_PORT_CTRL(port));
		if (!(tmp & DPI_ENABLE))
			continue;

		tmp = intel_de_read(dev_priv, MIPI_CTRL(port));
		if ((tmp & BXT_PIPE_SELECT_MASK) != BXT_PIPE_SELECT(crtc->pipe))
			continue;

		pipe_config->cpu_transcoder = cpu_transcoder;
		break;
	}

	return transcoder_is_dsi(pipe_config->cpu_transcoder);
}

static void intel_bigjoiner_get_config(struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *i915 = to_i915(crtc->base.dev);
	u8 master_pipes, slave_pipes;
	enum pipe pipe = crtc->pipe;

	enabled_bigjoiner_pipes(i915, &master_pipes, &slave_pipes);

	if (((master_pipes | slave_pipes) & BIT(pipe)) == 0)
		return;

	crtc_state->bigjoiner_pipes =
		BIT(get_bigjoiner_master_pipe(pipe, master_pipes, slave_pipes)) |
		get_bigjoiner_slave_pipes(pipe, master_pipes, slave_pipes);
}

static bool hsw_get_pipe_config(struct intel_crtc *crtc,
				struct intel_crtc_state *pipe_config)
{
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	bool active;
	u32 tmp;

	if (!intel_display_power_get_in_set_if_enabled(dev_priv, &crtc->hw_readout_power_domains,
						       POWER_DOMAIN_PIPE(crtc->pipe)))
		return false;

	pipe_config->shared_dpll = NULL;

	active = hsw_get_transcoder_state(crtc, pipe_config, &crtc->hw_readout_power_domains);

	if ((IS_GEMINILAKE(dev_priv) || IS_BROXTON(dev_priv)) &&
	    bxt_get_dsi_transcoder_state(crtc, pipe_config, &crtc->hw_readout_power_domains)) {
		drm_WARN_ON(&dev_priv->drm, active);
		active = true;
	}

	if (!active)
		goto out;

	intel_dsc_get_config(pipe_config);
	intel_bigjoiner_get_config(pipe_config);

	if (!transcoder_is_dsi(pipe_config->cpu_transcoder) ||
	    DISPLAY_VER(dev_priv) >= 11)
		intel_get_transcoder_timings(crtc, pipe_config);

	if (HAS_VRR(dev_priv) && !transcoder_is_dsi(pipe_config->cpu_transcoder))
		intel_vrr_get_config(pipe_config);

	intel_get_pipe_src_size(crtc, pipe_config);

	if (IS_HASWELL(dev_priv)) {
		u32 tmp = intel_de_read(dev_priv,
					TRANSCONF(pipe_config->cpu_transcoder));

		if (tmp & TRANSCONF_OUTPUT_COLORSPACE_YUV_HSW)
			pipe_config->output_format = INTEL_OUTPUT_FORMAT_YCBCR444;
		else
			pipe_config->output_format = INTEL_OUTPUT_FORMAT_RGB;
	} else {
		pipe_config->output_format =
			bdw_get_pipe_misc_output_format(crtc);
	}

	pipe_config->sink_format = pipe_config->output_format;

	pipe_config->gamma_mode = intel_de_read(dev_priv,
						GAMMA_MODE(crtc->pipe));

	pipe_config->csc_mode = intel_de_read(dev_priv,
					      PIPE_CSC_MODE(crtc->pipe));

	if (DISPLAY_VER(dev_priv) >= 9) {
		tmp = intel_de_read(dev_priv, SKL_BOTTOM_COLOR(crtc->pipe));

		if (tmp & SKL_BOTTOM_COLOR_GAMMA_ENABLE)
			pipe_config->gamma_enable = true;

		if (tmp & SKL_BOTTOM_COLOR_CSC_ENABLE)
			pipe_config->csc_enable = true;
	} else {
		i9xx_get_pipe_color_config(pipe_config);
	}

	intel_color_get_config(pipe_config);

	tmp = intel_de_read(dev_priv, WM_LINETIME(crtc->pipe));
	pipe_config->linetime = REG_FIELD_GET(HSW_LINETIME_MASK, tmp);
	if (IS_BROADWELL(dev_priv) || IS_HASWELL(dev_priv))
		pipe_config->ips_linetime =
			REG_FIELD_GET(HSW_IPS_LINETIME_MASK, tmp);

	if (intel_display_power_get_in_set_if_enabled(dev_priv, &crtc->hw_readout_power_domains,
						      POWER_DOMAIN_PIPE_PANEL_FITTER(crtc->pipe))) {
		if (DISPLAY_VER(dev_priv) >= 9)
			skl_scaler_get_config(pipe_config);
		else
			ilk_get_pfit_config(pipe_config);
	}

	hsw_ips_get_config(pipe_config);

	if (pipe_config->cpu_transcoder != TRANSCODER_EDP &&
	    !transcoder_is_dsi(pipe_config->cpu_transcoder)) {
		pipe_config->pixel_multiplier =
			intel_de_read(dev_priv,
				      TRANS_MULT(pipe_config->cpu_transcoder)) + 1;
	} else {
		pipe_config->pixel_multiplier = 1;
	}

	if (!transcoder_is_dsi(pipe_config->cpu_transcoder)) {
		tmp = intel_de_read(dev_priv, DISPLAY_VER(dev_priv) >= 14 ?
				    MTL_CHICKEN_TRANS(pipe_config->cpu_transcoder) :
				    CHICKEN_TRANS(pipe_config->cpu_transcoder));

		pipe_config->framestart_delay = REG_FIELD_GET(HSW_FRAME_START_DELAY_MASK, tmp) + 1;
	} else {
		/* no idea if this is correct */
		pipe_config->framestart_delay = 1;
	}

out:
	intel_display_power_put_all_in_set(dev_priv, &crtc->hw_readout_power_domains);

	return active;
}

bool intel_crtc_get_pipe_config(struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *i915 = to_i915(crtc->base.dev);

	if (!i915->display.funcs.display->get_pipe_config(crtc, crtc_state))
		return false;

	crtc_state->hw.active = true;

	intel_crtc_readout_derived_state(crtc_state);

	return true;
}

static int i9xx_pll_refclk(struct drm_device *dev,
			   const struct intel_crtc_state *pipe_config)
{
	struct drm_i915_private *dev_priv = to_i915(dev);
	u32 dpll = pipe_config->dpll_hw_state.dpll;

	if ((dpll & PLL_REF_INPUT_MASK) == PLLB_REF_INPUT_SPREADSPECTRUMIN)
		return dev_priv->display.vbt.lvds_ssc_freq;
	else if (HAS_PCH_SPLIT(dev_priv))
		return 120000;
	else if (DISPLAY_VER(dev_priv) != 2)
		return 96000;
	else
		return 48000;
}

/* Returns the clock of the currently programmed mode of the given pipe. */
void i9xx_crtc_clock_get(struct intel_crtc *crtc,
			 struct intel_crtc_state *pipe_config)
{
	struct drm_device *dev = crtc->base.dev;
	struct drm_i915_private *dev_priv = to_i915(dev);
	u32 dpll = pipe_config->dpll_hw_state.dpll;
	u32 fp;
	struct dpll clock;
	int port_clock;
	int refclk = i9xx_pll_refclk(dev, pipe_config);

	if ((dpll & DISPLAY_RATE_SELECT_FPA1) == 0)
		fp = pipe_config->dpll_hw_state.fp0;
	else
		fp = pipe_config->dpll_hw_state.fp1;

	clock.m1 = (fp & FP_M1_DIV_MASK) >> FP_M1_DIV_SHIFT;
	if (IS_PINEVIEW(dev_priv)) {
		clock.n = ffs((fp & FP_N_PINEVIEW_DIV_MASK) >> FP_N_DIV_SHIFT) - 1;
		clock.m2 = (fp & FP_M2_PINEVIEW_DIV_MASK) >> FP_M2_DIV_SHIFT;
	} else {
		clock.n = (fp & FP_N_DIV_MASK) >> FP_N_DIV_SHIFT;
		clock.m2 = (fp & FP_M2_DIV_MASK) >> FP_M2_DIV_SHIFT;
	}

	if (DISPLAY_VER(dev_priv) != 2) {
		if (IS_PINEVIEW(dev_priv))
			clock.p1 = ffs((dpll & DPLL_FPA01_P1_POST_DIV_MASK_PINEVIEW) >>
				DPLL_FPA01_P1_POST_DIV_SHIFT_PINEVIEW);
		else
			clock.p1 = ffs((dpll & DPLL_FPA01_P1_POST_DIV_MASK) >>
			       DPLL_FPA01_P1_POST_DIV_SHIFT);

		switch (dpll & DPLL_MODE_MASK) {
		case DPLLB_MODE_DAC_SERIAL:
			clock.p2 = dpll & DPLL_DAC_SERIAL_P2_CLOCK_DIV_5 ?
				5 : 10;
			break;
		case DPLLB_MODE_LVDS:
			clock.p2 = dpll & DPLLB_LVDS_P2_CLOCK_DIV_7 ?
				7 : 14;
			break;
		default:
			drm_dbg_kms(&dev_priv->drm,
				    "Unknown DPLL mode %08x in programmed "
				    "mode\n", (int)(dpll & DPLL_MODE_MASK));
			return;
		}

		if (IS_PINEVIEW(dev_priv))
			port_clock = pnv_calc_dpll_params(refclk, &clock);
		else
			port_clock = i9xx_calc_dpll_params(refclk, &clock);
	} else {
		enum pipe lvds_pipe;

		if (IS_I85X(dev_priv) &&
		    intel_lvds_port_enabled(dev_priv, LVDS, &lvds_pipe) &&
		    lvds_pipe == crtc->pipe) {
			u32 lvds = intel_de_read(dev_priv, LVDS);

			clock.p1 = ffs((dpll & DPLL_FPA01_P1_POST_DIV_MASK_I830_LVDS) >>
				       DPLL_FPA01_P1_POST_DIV_SHIFT);

			if (lvds & LVDS_CLKB_POWER_UP)
				clock.p2 = 7;
			else
				clock.p2 = 14;
		} else {
			if (dpll & PLL_P1_DIVIDE_BY_TWO)
				clock.p1 = 2;
			else {
				clock.p1 = ((dpll & DPLL_FPA01_P1_POST_DIV_MASK_I830) >>
					    DPLL_FPA01_P1_POST_DIV_SHIFT) + 2;
			}
			if (dpll & PLL_P2_DIVIDE_BY_4)
				clock.p2 = 4;
			else
				clock.p2 = 2;
		}

		port_clock = i9xx_calc_dpll_params(refclk, &clock);
	}

	/*
	 * This value includes pixel_multiplier. We will use
	 * port_clock to compute adjusted_mode.crtc_clock in the
	 * encoder's get_config() function.
	 */
	pipe_config->port_clock = port_clock;
}

int intel_dotclock_calculate(int link_freq,
			     const struct intel_link_m_n *m_n)
{
	/*
	 * The calculation for the data clock is:
	 * pixel_clock = ((m/n)*(link_clock * nr_lanes))/bpp
	 * But we want to avoid losing precison if possible, so:
	 * pixel_clock = ((m * link_clock * nr_lanes)/(n*bpp))
	 *
	 * and the link clock is simpler:
	 * link_clock = (m * link_clock) / n
	 */

	if (!m_n->link_n)
		return 0;

	return DIV_ROUND_UP_ULL(mul_u32_u32(m_n->link_m, link_freq),
				m_n->link_n);
}

int intel_crtc_dotclock(const struct intel_crtc_state *pipe_config)
{
	int dotclock;

	if (intel_crtc_has_dp_encoder(pipe_config))
		dotclock = intel_dotclock_calculate(pipe_config->port_clock,
						    &pipe_config->dp_m_n);
	else if (pipe_config->has_hdmi_sink && pipe_config->pipe_bpp > 24)
		dotclock = DIV_ROUND_CLOSEST(pipe_config->port_clock * 24,
					     pipe_config->pipe_bpp);
	else
		dotclock = pipe_config->port_clock;

	if (pipe_config->output_format == INTEL_OUTPUT_FORMAT_YCBCR420 &&
	    !intel_crtc_has_dp_encoder(pipe_config))
		dotclock *= 2;

	if (pipe_config->pixel_multiplier)
		dotclock /= pipe_config->pixel_multiplier;

	return dotclock;
}

/* Returns the currently programmed mode of the given encoder. */
struct drm_display_mode *
intel_encoder_current_mode(struct intel_encoder *encoder)
{
	struct drm_i915_private *dev_priv = to_i915(encoder->base.dev);
	struct intel_crtc_state *crtc_state;
	struct drm_display_mode *mode;
	struct intel_crtc *crtc;
	enum pipe pipe;

	if (!encoder->get_hw_state(encoder, &pipe))
		return NULL;

	crtc = intel_crtc_for_pipe(dev_priv, pipe);

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	crtc_state = intel_crtc_state_alloc(crtc);
	if (!crtc_state) {
		kfree(mode);
		return NULL;
	}

	if (!intel_crtc_get_pipe_config(crtc_state)) {
		kfree(crtc_state);
		kfree(mode);
		return NULL;
	}

	intel_encoder_get_config(encoder, crtc_state);

	intel_mode_from_crtc_timings(mode, &crtc_state->hw.adjusted_mode);

	kfree(crtc_state);

	return mode;
}

static bool encoders_cloneable(const struct intel_encoder *a,
			       const struct intel_encoder *b)
{
	/* masks could be asymmetric, so check both ways */
	return a == b || (a->cloneable & BIT(b->type) &&
			  b->cloneable & BIT(a->type));
}

static bool check_single_encoder_cloning(struct intel_atomic_state *state,
					 struct intel_crtc *crtc,
					 struct intel_encoder *encoder)
{
	struct intel_encoder *source_encoder;
	struct drm_connector *connector;
	struct drm_connector_state *connector_state;
	int i;

	for_each_new_connector_in_state(&state->base, connector, connector_state, i) {
		if (connector_state->crtc != &crtc->base)
			continue;

		source_encoder =
			to_intel_encoder(connector_state->best_encoder);
		if (!encoders_cloneable(encoder, source_encoder))
			return false;
	}

	return true;
}

static int icl_add_linked_planes(struct intel_atomic_state *state)
{
	struct intel_plane *plane, *linked;
	struct intel_plane_state *plane_state, *linked_plane_state;
	int i;

	for_each_new_intel_plane_in_state(state, plane, plane_state, i) {
		linked = plane_state->planar_linked_plane;

		if (!linked)
			continue;

		linked_plane_state = intel_atomic_get_plane_state(state, linked);
		if (IS_ERR(linked_plane_state))
			return PTR_ERR(linked_plane_state);

		drm_WARN_ON(state->base.dev,
			    linked_plane_state->planar_linked_plane != plane);
		drm_WARN_ON(state->base.dev,
			    linked_plane_state->planar_slave == plane_state->planar_slave);
	}

	return 0;
}

static int icl_check_nv12_planes(struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	struct intel_atomic_state *state = to_intel_atomic_state(crtc_state->uapi.state);
	struct intel_plane *plane, *linked;
	struct intel_plane_state *plane_state;
	int i;

	if (DISPLAY_VER(dev_priv) < 11)
		return 0;

	/*
	 * Destroy all old plane links and make the slave plane invisible
	 * in the crtc_state->active_planes mask.
	 */
	for_each_new_intel_plane_in_state(state, plane, plane_state, i) {
		if (plane->pipe != crtc->pipe || !plane_state->planar_linked_plane)
			continue;

		plane_state->planar_linked_plane = NULL;
		if (plane_state->planar_slave && !plane_state->uapi.visible) {
			crtc_state->enabled_planes &= ~BIT(plane->id);
			crtc_state->active_planes &= ~BIT(plane->id);
			crtc_state->update_planes |= BIT(plane->id);
			crtc_state->data_rate[plane->id] = 0;
			crtc_state->rel_data_rate[plane->id] = 0;
		}

		plane_state->planar_slave = false;
	}

	if (!crtc_state->nv12_planes)
		return 0;

	for_each_new_intel_plane_in_state(state, plane, plane_state, i) {
		struct intel_plane_state *linked_state = NULL;

		if (plane->pipe != crtc->pipe ||
		    !(crtc_state->nv12_planes & BIT(plane->id)))
			continue;

		for_each_intel_plane_on_crtc(&dev_priv->drm, crtc, linked) {
			if (!icl_is_nv12_y_plane(dev_priv, linked->id))
				continue;

			if (crtc_state->active_planes & BIT(linked->id))
				continue;

			linked_state = intel_atomic_get_plane_state(state, linked);
			if (IS_ERR(linked_state))
				return PTR_ERR(linked_state);

			break;
		}

		if (!linked_state) {
			drm_dbg_kms(&dev_priv->drm,
				    "Need %d free Y planes for planar YUV\n",
				    hweight8(crtc_state->nv12_planes));

			return -EINVAL;
		}

		plane_state->planar_linked_plane = linked;

		linked_state->planar_slave = true;
		linked_state->planar_linked_plane = plane;
		crtc_state->enabled_planes |= BIT(linked->id);
		crtc_state->active_planes |= BIT(linked->id);
		crtc_state->update_planes |= BIT(linked->id);
		crtc_state->data_rate[linked->id] =
			crtc_state->data_rate_y[plane->id];
		crtc_state->rel_data_rate[linked->id] =
			crtc_state->rel_data_rate_y[plane->id];
		drm_dbg_kms(&dev_priv->drm, "Using %s as Y plane for %s\n",
			    linked->base.name, plane->base.name);

		/* Copy parameters to slave plane */
		linked_state->ctl = plane_state->ctl | PLANE_CTL_YUV420_Y_PLANE;
		linked_state->color_ctl = plane_state->color_ctl;
		linked_state->view = plane_state->view;
		linked_state->decrypt = plane_state->decrypt;

		intel_plane_copy_hw_state(linked_state, plane_state);
		linked_state->uapi.src = plane_state->uapi.src;
		linked_state->uapi.dst = plane_state->uapi.dst;

		if (icl_is_hdr_plane(dev_priv, plane->id)) {
			if (linked->id == PLANE_SPRITE5)
				plane_state->cus_ctl |= PLANE_CUS_Y_PLANE_7_ICL;
			else if (linked->id == PLANE_SPRITE4)
				plane_state->cus_ctl |= PLANE_CUS_Y_PLANE_6_ICL;
			else if (linked->id == PLANE_SPRITE3)
				plane_state->cus_ctl |= PLANE_CUS_Y_PLANE_5_RKL;
			else if (linked->id == PLANE_SPRITE2)
				plane_state->cus_ctl |= PLANE_CUS_Y_PLANE_4_RKL;
			else
				MISSING_CASE(linked->id);
		}
	}

	return 0;
}

static bool c8_planes_changed(const struct intel_crtc_state *new_crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(new_crtc_state->uapi.crtc);
	struct intel_atomic_state *state =
		to_intel_atomic_state(new_crtc_state->uapi.state);
	const struct intel_crtc_state *old_crtc_state =
		intel_atomic_get_old_crtc_state(state, crtc);

	return !old_crtc_state->c8_planes != !new_crtc_state->c8_planes;
}

static u16 hsw_linetime_wm(const struct intel_crtc_state *crtc_state)
{
	const struct drm_display_mode *pipe_mode =
		&crtc_state->hw.pipe_mode;
	int linetime_wm;

	if (!crtc_state->hw.enable)
		return 0;

	linetime_wm = DIV_ROUND_CLOSEST(pipe_mode->crtc_htotal * 1000 * 8,
					pipe_mode->crtc_clock);

	return min(linetime_wm, 0x1ff);
}

static u16 hsw_ips_linetime_wm(const struct intel_crtc_state *crtc_state,
			       const struct intel_cdclk_state *cdclk_state)
{
	const struct drm_display_mode *pipe_mode =
		&crtc_state->hw.pipe_mode;
	int linetime_wm;

	if (!crtc_state->hw.enable)
		return 0;

	linetime_wm = DIV_ROUND_CLOSEST(pipe_mode->crtc_htotal * 1000 * 8,
					cdclk_state->logical.cdclk);

	return min(linetime_wm, 0x1ff);
}

static u16 skl_linetime_wm(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	const struct drm_display_mode *pipe_mode =
		&crtc_state->hw.pipe_mode;
	int linetime_wm;

	if (!crtc_state->hw.enable)
		return 0;

	linetime_wm = DIV_ROUND_UP(pipe_mode->crtc_htotal * 1000 * 8,
				   crtc_state->pixel_rate);

	/* Display WA #1135: BXT:ALL GLK:ALL */
	if ((IS_GEMINILAKE(dev_priv) || IS_BROXTON(dev_priv)) &&
	    skl_watermark_ipc_enabled(dev_priv))
		linetime_wm /= 2;

	return min(linetime_wm, 0x1ff);
}

static int hsw_compute_linetime_wm(struct intel_atomic_state *state,
				   struct intel_crtc *crtc)
{
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	struct intel_crtc_state *crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	const struct intel_cdclk_state *cdclk_state;

	if (DISPLAY_VER(dev_priv) >= 9)
		crtc_state->linetime = skl_linetime_wm(crtc_state);
	else
		crtc_state->linetime = hsw_linetime_wm(crtc_state);

	if (!hsw_crtc_supports_ips(crtc))
		return 0;

	cdclk_state = intel_atomic_get_cdclk_state(state);
	if (IS_ERR(cdclk_state))
		return PTR_ERR(cdclk_state);

	crtc_state->ips_linetime = hsw_ips_linetime_wm(crtc_state,
						       cdclk_state);

	return 0;
}

static int intel_crtc_atomic_check(struct intel_atomic_state *state,
				   struct intel_crtc *crtc)
{
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	struct intel_crtc_state *crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	int ret;

	if (DISPLAY_VER(dev_priv) < 5 && !IS_G4X(dev_priv) &&
	    intel_crtc_needs_modeset(crtc_state) &&
	    !crtc_state->hw.active)
		crtc_state->update_wm_post = true;

	if (intel_crtc_needs_modeset(crtc_state)) {
		ret = intel_dpll_crtc_get_shared_dpll(state, crtc);
		if (ret)
			return ret;
	}

	/*
	 * May need to update pipe gamma enable bits
	 * when C8 planes are getting enabled/disabled.
	 */
	if (c8_planes_changed(crtc_state))
		crtc_state->uapi.color_mgmt_changed = true;

	if (intel_crtc_needs_color_update(crtc_state)) {
		ret = intel_color_check(crtc_state);
		if (ret)
			return ret;
	}

	ret = intel_compute_pipe_wm(state, crtc);
	if (ret) {
		drm_dbg_kms(&dev_priv->drm,
			    "Target pipe watermarks are invalid\n");
		return ret;
	}

	/*
	 * Calculate 'intermediate' watermarks that satisfy both the
	 * old state and the new state.  We can program these
	 * immediately.
	 */
	ret = intel_compute_intermediate_wm(state, crtc);
	if (ret) {
		drm_dbg_kms(&dev_priv->drm,
			    "No valid intermediate pipe watermarks are possible\n");
		return ret;
	}

	if (DISPLAY_VER(dev_priv) >= 9) {
		if (intel_crtc_needs_modeset(crtc_state) ||
		    intel_crtc_needs_fastset(crtc_state)) {
			ret = skl_update_scaler_crtc(crtc_state);
			if (ret)
				return ret;
		}

		ret = intel_atomic_setup_scalers(dev_priv, crtc, crtc_state);
		if (ret)
			return ret;
	}

	if (HAS_IPS(dev_priv)) {
		ret = hsw_ips_compute_config(state, crtc);
		if (ret)
			return ret;
	}

	if (DISPLAY_VER(dev_priv) >= 9 ||
	    IS_BROADWELL(dev_priv) || IS_HASWELL(dev_priv)) {
		ret = hsw_compute_linetime_wm(state, crtc);
		if (ret)
			return ret;

	}

	ret = intel_psr2_sel_fetch_update(state, crtc);
	if (ret)
		return ret;

	return 0;
}

static int
compute_sink_pipe_bpp(const struct drm_connector_state *conn_state,
		      struct intel_crtc_state *crtc_state)
{
	struct drm_connector *connector = conn_state->connector;
	struct drm_i915_private *i915 = to_i915(crtc_state->uapi.crtc->dev);
	const struct drm_display_info *info = &connector->display_info;
	int bpp;

	switch (conn_state->max_bpc) {
	case 6 ... 7:
		bpp = 6 * 3;
		break;
	case 8 ... 9:
		bpp = 8 * 3;
		break;
	case 10 ... 11:
		bpp = 10 * 3;
		break;
	case 12 ... 16:
		bpp = 12 * 3;
		break;
	default:
		MISSING_CASE(conn_state->max_bpc);
		return -EINVAL;
	}

	if (bpp < crtc_state->pipe_bpp) {
		drm_dbg_kms(&i915->drm,
			    "[CONNECTOR:%d:%s] Limiting display bpp to %d "
			    "(EDID bpp %d, max requested bpp %d, max platform bpp %d)\n",
			    connector->base.id, connector->name,
			    bpp, 3 * info->bpc,
			    3 * conn_state->max_requested_bpc,
			    crtc_state->pipe_bpp);

		crtc_state->pipe_bpp = bpp;
	}

	return 0;
}

static int
compute_baseline_pipe_bpp(struct intel_atomic_state *state,
			  struct intel_crtc *crtc)
{
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	struct intel_crtc_state *crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	struct drm_connector *connector;
	struct drm_connector_state *connector_state;
	int bpp, i;

	if ((IS_G4X(dev_priv) || IS_VALLEYVIEW(dev_priv) ||
	    IS_CHERRYVIEW(dev_priv)))
		bpp = 10*3;
	else if (DISPLAY_VER(dev_priv) >= 5)
		bpp = 12*3;
	else
		bpp = 8*3;

	crtc_state->pipe_bpp = bpp;

	/* Clamp display bpp to connector max bpp */
	for_each_new_connector_in_state(&state->base, connector, connector_state, i) {
		int ret;

		if (connector_state->crtc != &crtc->base)
			continue;

		ret = compute_sink_pipe_bpp(connector_state, crtc_state);
		if (ret)
			return ret;
	}

	return 0;
}

static bool check_digital_port_conflicts(struct intel_atomic_state *state)
{
	struct drm_device *dev = state->base.dev;
	struct drm_connector *connector;
	struct drm_connector_list_iter conn_iter;
	unsigned int used_ports = 0;
	unsigned int used_mst_ports = 0;
	bool ret = true;

	/*
	 * We're going to peek into connector->state,
	 * hence connection_mutex must be held.
	 */
	drm_modeset_lock_assert_held(&dev->mode_config.connection_mutex);

	/*
	 * Walk the connector list instead of the encoder
	 * list to detect the problem on ddi platforms
	 * where there's just one encoder per digital port.
	 */
	drm_connector_list_iter_begin(dev, &conn_iter);
	drm_for_each_connector_iter(connector, &conn_iter) {
		struct drm_connector_state *connector_state;
		struct intel_encoder *encoder;

		connector_state =
			drm_atomic_get_new_connector_state(&state->base,
							   connector);
		if (!connector_state)
			connector_state = connector->state;

		if (!connector_state->best_encoder)
			continue;

		encoder = to_intel_encoder(connector_state->best_encoder);

		drm_WARN_ON(dev, !connector_state->crtc);

		switch (encoder->type) {
		case INTEL_OUTPUT_DDI:
			if (drm_WARN_ON(dev, !HAS_DDI(to_i915(dev))))
				break;
			fallthrough;
		case INTEL_OUTPUT_DP:
		case INTEL_OUTPUT_HDMI:
		case INTEL_OUTPUT_EDP:
			/* the same port mustn't appear more than once */
			if (used_ports & BIT(encoder->port))
				ret = false;

			used_ports |= BIT(encoder->port);
			break;
		case INTEL_OUTPUT_DP_MST:
			used_mst_ports |=
				1 << encoder->port;
			break;
		default:
			break;
		}
	}
	drm_connector_list_iter_end(&conn_iter);

	/* can't mix MST and SST/HDMI on the same port */
	if (used_ports & used_mst_ports)
		return false;

	return ret;
}

static void
intel_crtc_copy_uapi_to_hw_state_nomodeset(struct intel_atomic_state *state,
					   struct intel_crtc *crtc)
{
	struct intel_crtc_state *crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);

	WARN_ON(intel_crtc_is_bigjoiner_slave(crtc_state));

	drm_property_replace_blob(&crtc_state->hw.degamma_lut,
				  crtc_state->uapi.degamma_lut);
	drm_property_replace_blob(&crtc_state->hw.gamma_lut,
				  crtc_state->uapi.gamma_lut);
	drm_property_replace_blob(&crtc_state->hw.ctm,
				  crtc_state->uapi.ctm);
}

static void
intel_crtc_copy_uapi_to_hw_state_modeset(struct intel_atomic_state *state,
					 struct intel_crtc *crtc)
{
	struct intel_crtc_state *crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);

	WARN_ON(intel_crtc_is_bigjoiner_slave(crtc_state));

	crtc_state->hw.enable = crtc_state->uapi.enable;
	crtc_state->hw.active = crtc_state->uapi.active;
	drm_mode_copy(&crtc_state->hw.mode,
		      &crtc_state->uapi.mode);
	drm_mode_copy(&crtc_state->hw.adjusted_mode,
		      &crtc_state->uapi.adjusted_mode);
	crtc_state->hw.scaling_filter = crtc_state->uapi.scaling_filter;

	intel_crtc_copy_uapi_to_hw_state_nomodeset(state, crtc);
}

static void
copy_bigjoiner_crtc_state_nomodeset(struct intel_atomic_state *state,
				    struct intel_crtc *slave_crtc)
{
	struct intel_crtc_state *slave_crtc_state =
		intel_atomic_get_new_crtc_state(state, slave_crtc);
	struct intel_crtc *master_crtc = intel_master_crtc(slave_crtc_state);
	const struct intel_crtc_state *master_crtc_state =
		intel_atomic_get_new_crtc_state(state, master_crtc);

	drm_property_replace_blob(&slave_crtc_state->hw.degamma_lut,
				  master_crtc_state->hw.degamma_lut);
	drm_property_replace_blob(&slave_crtc_state->hw.gamma_lut,
				  master_crtc_state->hw.gamma_lut);
	drm_property_replace_blob(&slave_crtc_state->hw.ctm,
				  master_crtc_state->hw.ctm);

	slave_crtc_state->uapi.color_mgmt_changed = master_crtc_state->uapi.color_mgmt_changed;
}

static int
copy_bigjoiner_crtc_state_modeset(struct intel_atomic_state *state,
				  struct intel_crtc *slave_crtc)
{
	struct intel_crtc_state *slave_crtc_state =
		intel_atomic_get_new_crtc_state(state, slave_crtc);
	struct intel_crtc *master_crtc = intel_master_crtc(slave_crtc_state);
	const struct intel_crtc_state *master_crtc_state =
		intel_atomic_get_new_crtc_state(state, master_crtc);
	struct intel_crtc_state *saved_state;

	WARN_ON(master_crtc_state->bigjoiner_pipes !=
		slave_crtc_state->bigjoiner_pipes);

	saved_state = kmemdup(master_crtc_state, sizeof(*saved_state), GFP_KERNEL);
	if (!saved_state)
		return -ENOMEM;

	/* preserve some things from the slave's original crtc state */
	saved_state->uapi = slave_crtc_state->uapi;
	saved_state->scaler_state = slave_crtc_state->scaler_state;
	saved_state->shared_dpll = slave_crtc_state->shared_dpll;
	saved_state->crc_enabled = slave_crtc_state->crc_enabled;

	intel_crtc_free_hw_state(slave_crtc_state);
	memcpy(slave_crtc_state, saved_state, sizeof(*slave_crtc_state));
	kfree(saved_state);

	/* Re-init hw state */
	memset(&slave_crtc_state->hw, 0, sizeof(slave_crtc_state->hw));
	slave_crtc_state->hw.enable = master_crtc_state->hw.enable;
	slave_crtc_state->hw.active = master_crtc_state->hw.active;
	drm_mode_copy(&slave_crtc_state->hw.mode,
		      &master_crtc_state->hw.mode);
	drm_mode_copy(&slave_crtc_state->hw.pipe_mode,
		      &master_crtc_state->hw.pipe_mode);
	drm_mode_copy(&slave_crtc_state->hw.adjusted_mode,
		      &master_crtc_state->hw.adjusted_mode);
	slave_crtc_state->hw.scaling_filter = master_crtc_state->hw.scaling_filter;

	copy_bigjoiner_crtc_state_nomodeset(state, slave_crtc);

	slave_crtc_state->uapi.mode_changed = master_crtc_state->uapi.mode_changed;
	slave_crtc_state->uapi.connectors_changed = master_crtc_state->uapi.connectors_changed;
	slave_crtc_state->uapi.active_changed = master_crtc_state->uapi.active_changed;

	WARN_ON(master_crtc_state->bigjoiner_pipes !=
		slave_crtc_state->bigjoiner_pipes);

	return 0;
}

static int
intel_crtc_prepare_cleared_state(struct intel_atomic_state *state,
				 struct intel_crtc *crtc)
{
	struct intel_crtc_state *crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	struct intel_crtc_state *saved_state;

	saved_state = intel_crtc_state_alloc(crtc);
	if (!saved_state)
		return -ENOMEM;

	/* free the old crtc_state->hw members */
	intel_crtc_free_hw_state(crtc_state);

	/* FIXME: before the switch to atomic started, a new pipe_config was
	 * kzalloc'd. Code that depends on any field being zero should be
	 * fixed, so that the crtc_state can be safely duplicated. For now,
	 * only fields that are know to not cause problems are preserved. */

	saved_state->uapi = crtc_state->uapi;
	saved_state->inherited = crtc_state->inherited;
	saved_state->scaler_state = crtc_state->scaler_state;
	saved_state->shared_dpll = crtc_state->shared_dpll;
	saved_state->dpll_hw_state = crtc_state->dpll_hw_state;
	memcpy(saved_state->icl_port_dplls, crtc_state->icl_port_dplls,
	       sizeof(saved_state->icl_port_dplls));
	saved_state->crc_enabled = crtc_state->crc_enabled;
	if (IS_G4X(dev_priv) ||
	    IS_VALLEYVIEW(dev_priv) || IS_CHERRYVIEW(dev_priv))
		saved_state->wm = crtc_state->wm;

	memcpy(crtc_state, saved_state, sizeof(*crtc_state));
	kfree(saved_state);

	intel_crtc_copy_uapi_to_hw_state_modeset(state, crtc);

	return 0;
}

static int
intel_modeset_pipe_config(struct intel_atomic_state *state,
			  struct intel_crtc *crtc)
{
	struct drm_i915_private *i915 = to_i915(crtc->base.dev);
	struct intel_crtc_state *crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	struct drm_connector *connector;
	struct drm_connector_state *connector_state;
	int pipe_src_w, pipe_src_h;
	int base_bpp, ret, i;
	bool retry = true;

	crtc_state->cpu_transcoder = (enum transcoder) crtc->pipe;

	crtc_state->framestart_delay = 1;

	/*
	 * Sanitize sync polarity flags based on requested ones. If neither
	 * positive or negative polarity is requested, treat this as meaning
	 * negative polarity.
	 */
	if (!(crtc_state->hw.adjusted_mode.flags &
	      (DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NHSYNC)))
		crtc_state->hw.adjusted_mode.flags |= DRM_MODE_FLAG_NHSYNC;

	if (!(crtc_state->hw.adjusted_mode.flags &
	      (DRM_MODE_FLAG_PVSYNC | DRM_MODE_FLAG_NVSYNC)))
		crtc_state->hw.adjusted_mode.flags |= DRM_MODE_FLAG_NVSYNC;

	ret = compute_baseline_pipe_bpp(state, crtc);
	if (ret)
		return ret;

	base_bpp = crtc_state->pipe_bpp;

	/*
	 * Determine the real pipe dimensions. Note that stereo modes can
	 * increase the actual pipe size due to the frame doubling and
	 * insertion of additional space for blanks between the frame. This
	 * is stored in the crtc timings. We use the requested mode to do this
	 * computation to clearly distinguish it from the adjusted mode, which
	 * can be changed by the connectors in the below retry loop.
	 */
	drm_mode_get_hv_timing(&crtc_state->hw.mode,
			       &pipe_src_w, &pipe_src_h);
	drm_rect_init(&crtc_state->pipe_src, 0, 0,
		      pipe_src_w, pipe_src_h);

	for_each_new_connector_in_state(&state->base, connector, connector_state, i) {
		struct intel_encoder *encoder =
			to_intel_encoder(connector_state->best_encoder);

		if (connector_state->crtc != &crtc->base)
			continue;

		if (!check_single_encoder_cloning(state, crtc, encoder)) {
			drm_dbg_kms(&i915->drm,
				    "[ENCODER:%d:%s] rejecting invalid cloning configuration\n",
				    encoder->base.base.id, encoder->base.name);
			return -EINVAL;
		}

		/*
		 * Determine output_types before calling the .compute_config()
		 * hooks so that the hooks can use this information safely.
		 */
		if (encoder->compute_output_type)
			crtc_state->output_types |=
				BIT(encoder->compute_output_type(encoder, crtc_state,
								 connector_state));
		else
			crtc_state->output_types |= BIT(encoder->type);
	}

encoder_retry:
	/* Ensure the port clock defaults are reset when retrying. */
	crtc_state->port_clock = 0;
	crtc_state->pixel_multiplier = 1;

	/* Fill in default crtc timings, allow encoders to overwrite them. */
	drm_mode_set_crtcinfo(&crtc_state->hw.adjusted_mode,
			      CRTC_STEREO_DOUBLE);

	/* Pass our mode to the connectors and the CRTC to give them a chance to
	 * adjust it according to limitations or connector properties, and also
	 * a chance to reject the mode entirely.
	 */
	for_each_new_connector_in_state(&state->base, connector, connector_state, i) {
		struct intel_encoder *encoder =
			to_intel_encoder(connector_state->best_encoder);

		if (connector_state->crtc != &crtc->base)
			continue;

		ret = encoder->compute_config(encoder, crtc_state,
					      connector_state);
		if (ret == -EDEADLK)
			return ret;
		if (ret < 0) {
			drm_dbg_kms(&i915->drm, "[ENCODER:%d:%s] config failure: %d\n",
				    encoder->base.base.id, encoder->base.name, ret);
			return ret;
		}
	}

	/* Set default port clock if not overwritten by the encoder. Needs to be
	 * done afterwards in case the encoder adjusts the mode. */
	if (!crtc_state->port_clock)
		crtc_state->port_clock = crtc_state->hw.adjusted_mode.crtc_clock
			* crtc_state->pixel_multiplier;

	ret = intel_crtc_compute_config(state, crtc);
	if (ret == -EDEADLK)
		return ret;
	if (ret == -EAGAIN) {
		if (drm_WARN(&i915->drm, !retry,
			     "[CRTC:%d:%s] loop in pipe configuration computation\n",
			     crtc->base.base.id, crtc->base.name))
			return -EINVAL;

		drm_dbg_kms(&i915->drm, "[CRTC:%d:%s] bw constrained, retrying\n",
			    crtc->base.base.id, crtc->base.name);
		retry = false;
		goto encoder_retry;
	}
	if (ret < 0) {
		drm_dbg_kms(&i915->drm, "[CRTC:%d:%s] config failure: %d\n",
			    crtc->base.base.id, crtc->base.name, ret);
		return ret;
	}

	/* Dithering seems to not pass-through bits correctly when it should, so
	 * only enable it on 6bpc panels and when its not a compliance
	 * test requesting 6bpc video pattern.
	 */
	crtc_state->dither = (crtc_state->pipe_bpp == 6*3) &&
		!crtc_state->dither_force_disable;
	drm_dbg_kms(&i915->drm,
		    "[CRTC:%d:%s] hw max bpp: %i, pipe bpp: %i, dithering: %i\n",
		    crtc->base.base.id, crtc->base.name,
		    base_bpp, crtc_state->pipe_bpp, crtc_state->dither);

	return 0;
}

static int
intel_modeset_pipe_config_late(struct intel_atomic_state *state,
			       struct intel_crtc *crtc)
{
	struct intel_crtc_state *crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	struct drm_connector_state *conn_state;
	struct drm_connector *connector;
	int i;

	intel_bigjoiner_adjust_pipe_src(crtc_state);

	for_each_new_connector_in_state(&state->base, connector,
					conn_state, i) {
		struct intel_encoder *encoder =
			to_intel_encoder(conn_state->best_encoder);
		int ret;

		if (conn_state->crtc != &crtc->base ||
		    !encoder->compute_config_late)
			continue;

		ret = encoder->compute_config_late(encoder, crtc_state,
						   conn_state);
		if (ret)
			return ret;
	}

	return 0;
}

bool intel_fuzzy_clock_check(int clock1, int clock2)
{
	int diff;

	if (clock1 == clock2)
		return true;

	if (!clock1 || !clock2)
		return false;

	diff = abs(clock1 - clock2);

	if (((((diff + clock1 + clock2) * 100)) / (clock1 + clock2)) < 105)
		return true;

	return false;
}

static bool
intel_compare_link_m_n(const struct intel_link_m_n *m_n,
		       const struct intel_link_m_n *m2_n2)
{
	return m_n->tu == m2_n2->tu &&
		m_n->data_m == m2_n2->data_m &&
		m_n->data_n == m2_n2->data_n &&
		m_n->link_m == m2_n2->link_m &&
		m_n->link_n == m2_n2->link_n;
}

static bool
intel_compare_infoframe(const union hdmi_infoframe *a,
			const union hdmi_infoframe *b)
{
	return memcmp(a, b, sizeof(*a)) == 0;
}

static bool
intel_compare_dp_vsc_sdp(const struct drm_dp_vsc_sdp *a,
			 const struct drm_dp_vsc_sdp *b)
{
	return memcmp(a, b, sizeof(*a)) == 0;
}

static bool
intel_compare_buffer(const u8 *a, const u8 *b, size_t len)
{
	return memcmp(a, b, len) == 0;
}

static void
pipe_config_infoframe_mismatch(struct drm_i915_private *dev_priv,
			       bool fastset, const char *name,
			       const union hdmi_infoframe *a,
			       const union hdmi_infoframe *b)
{
	if (fastset) {
		if (!drm_debug_enabled(DRM_UT_KMS))
			return;

		drm_dbg_kms(&dev_priv->drm,
			    "fastset requirement not met in %s infoframe\n", name);
		drm_dbg_kms(&dev_priv->drm, "expected:\n");
		hdmi_infoframe_log(KERN_DEBUG, dev_priv->drm.dev, a);
		drm_dbg_kms(&dev_priv->drm, "found:\n");
		hdmi_infoframe_log(KERN_DEBUG, dev_priv->drm.dev, b);
	} else {
		drm_err(&dev_priv->drm, "mismatch in %s infoframe\n", name);
		drm_err(&dev_priv->drm, "expected:\n");
		hdmi_infoframe_log(KERN_ERR, dev_priv->drm.dev, a);
		drm_err(&dev_priv->drm, "found:\n");
		hdmi_infoframe_log(KERN_ERR, dev_priv->drm.dev, b);
	}
}

static void
pipe_config_dp_vsc_sdp_mismatch(struct drm_i915_private *dev_priv,
				bool fastset, const char *name,
				const struct drm_dp_vsc_sdp *a,
				const struct drm_dp_vsc_sdp *b)
{
	if (fastset) {
		if (!drm_debug_enabled(DRM_UT_KMS))
			return;

		drm_dbg_kms(&dev_priv->drm,
			    "fastset requirement not met in %s dp sdp\n", name);
		drm_dbg_kms(&dev_priv->drm, "expected:\n");
		drm_dp_vsc_sdp_log(KERN_DEBUG, dev_priv->drm.dev, a);
		drm_dbg_kms(&dev_priv->drm, "found:\n");
		drm_dp_vsc_sdp_log(KERN_DEBUG, dev_priv->drm.dev, b);
	} else {
		drm_err(&dev_priv->drm, "mismatch in %s dp sdp\n", name);
		drm_err(&dev_priv->drm, "expected:\n");
		drm_dp_vsc_sdp_log(KERN_ERR, dev_priv->drm.dev, a);
		drm_err(&dev_priv->drm, "found:\n");
		drm_dp_vsc_sdp_log(KERN_ERR, dev_priv->drm.dev, b);
	}
}

/* Returns the length up to and including the last differing byte */
static size_t
memcmp_diff_len(const u8 *a, const u8 *b, size_t len)
{
	int i;

	for (i = len - 1; i >= 0; i--) {
		if (a[i] != b[i])
			return i + 1;
	}

	return 0;
}

static void
pipe_config_buffer_mismatch(struct drm_i915_private *dev_priv,
			    bool fastset, const char *name,
			    const u8 *a, const u8 *b, size_t len)
{
	if (fastset) {
		if (!drm_debug_enabled(DRM_UT_KMS))
			return;

		/* only dump up to the last difference */
		len = memcmp_diff_len(a, b, len);

		drm_dbg_kms(&dev_priv->drm,
			    "fastset requirement not met in %s buffer\n", name);
		print_hex_dump(KERN_DEBUG, "expected: ", DUMP_PREFIX_NONE,
			       16, 0, a, len, false);
		print_hex_dump(KERN_DEBUG, "found: ", DUMP_PREFIX_NONE,
			       16, 0, b, len, false);
	} else {
		/* only dump up to the last difference */
		len = memcmp_diff_len(a, b, len);

		drm_err(&dev_priv->drm, "mismatch in %s buffer\n", name);
		print_hex_dump(KERN_ERR, "expected: ", DUMP_PREFIX_NONE,
			       16, 0, a, len, false);
		print_hex_dump(KERN_ERR, "found: ", DUMP_PREFIX_NONE,
			       16, 0, b, len, false);
	}
}

static void __printf(4, 5)
pipe_config_mismatch(bool fastset, const struct intel_crtc *crtc,
		     const char *name, const char *format, ...)
{
	struct drm_i915_private *i915 = to_i915(crtc->base.dev);
	struct va_format vaf;
	va_list args;

	va_start(args, format);
	vaf.fmt = format;
	vaf.va = &args;

	if (fastset)
		drm_dbg_kms(&i915->drm,
			    "[CRTC:%d:%s] fastset requirement not met in %s %pV\n",
			    crtc->base.base.id, crtc->base.name, name, &vaf);
	else
		drm_err(&i915->drm, "[CRTC:%d:%s] mismatch in %s %pV\n",
			crtc->base.base.id, crtc->base.name, name, &vaf);

	va_end(args);
}

static bool fastboot_enabled(struct drm_i915_private *dev_priv)
{
	if (dev_priv->params.fastboot != -1)
		return dev_priv->params.fastboot;

	/* Enable fastboot by default on Skylake and newer */
	if (DISPLAY_VER(dev_priv) >= 9)
		return true;

	/* Enable fastboot by default on VLV and CHV */
	if (IS_VALLEYVIEW(dev_priv) || IS_CHERRYVIEW(dev_priv))
		return true;

	/* Disabled by default on all others */
	return false;
}

bool
intel_pipe_config_compare(const struct intel_crtc_state *current_config,
			  const struct intel_crtc_state *pipe_config,
			  bool fastset)
{
	struct drm_i915_private *dev_priv = to_i915(current_config->uapi.crtc->dev);
	struct intel_crtc *crtc = to_intel_crtc(pipe_config->uapi.crtc);
	bool ret = true;
	bool fixup_inherited = fastset &&
		current_config->inherited && !pipe_config->inherited;

	if (fixup_inherited && !fastboot_enabled(dev_priv)) {
		drm_dbg_kms(&dev_priv->drm,
			    "initial modeset and fastboot not set\n");
		ret = false;
	}

#define PIPE_CONF_CHECK_X(name) do { \
	if (current_config->name != pipe_config->name) { \
		pipe_config_mismatch(fastset, crtc, __stringify(name), \
				     "(expected 0x%08x, found 0x%08x)", \
				     current_config->name, \
				     pipe_config->name); \
		ret = false; \
	} \
} while (0)

#define PIPE_CONF_CHECK_X_WITH_MASK(name, mask) do { \
	if ((current_config->name & (mask)) != (pipe_config->name & (mask))) { \
		pipe_config_mismatch(fastset, crtc, __stringify(name), \
				     "(expected 0x%08x, found 0x%08x)", \
				     current_config->name & (mask), \
				     pipe_config->name & (mask)); \
		ret = false; \
	} \
} while (0)

#define PIPE_CONF_CHECK_I(name) do { \
	if (current_config->name != pipe_config->name) { \
		pipe_config_mismatch(fastset, crtc, __stringify(name), \
				     "(expected %i, found %i)", \
				     current_config->name, \
				     pipe_config->name); \
		ret = false; \
	} \
} while (0)

#define PIPE_CONF_CHECK_BOOL(name) do { \
	if (current_config->name != pipe_config->name) { \
		pipe_config_mismatch(fastset, crtc,  __stringify(name), \
				     "(expected %s, found %s)", \
				     str_yes_no(current_config->name), \
				     str_yes_no(pipe_config->name)); \
		ret = false; \
	} \
} while (0)

/*
 * Checks state where we only read out the enabling, but not the entire
 * state itself (like full infoframes or ELD for audio). These states
 * require a full modeset on bootup to fix up.
 */
#define PIPE_CONF_CHECK_BOOL_INCOMPLETE(name) do { \
	if (!fixup_inherited || (!current_config->name && !pipe_config->name)) { \
		PIPE_CONF_CHECK_BOOL(name); \
	} else { \
		pipe_config_mismatch(fastset, crtc, __stringify(name), \
				     "unable to verify whether state matches exactly, forcing modeset (expected %s, found %s)", \
				     str_yes_no(current_config->name), \
				     str_yes_no(pipe_config->name)); \
		ret = false; \
	} \
} while (0)

#define PIPE_CONF_CHECK_P(name) do { \
	if (current_config->name != pipe_config->name) { \
		pipe_config_mismatch(fastset, crtc, __stringify(name), \
				     "(expected %p, found %p)", \
				     current_config->name, \
				     pipe_config->name); \
		ret = false; \
	} \
} while (0)

#define PIPE_CONF_CHECK_M_N(name) do { \
	if (!intel_compare_link_m_n(&current_config->name, \
				    &pipe_config->name)) { \
		pipe_config_mismatch(fastset, crtc, __stringify(name), \
				     "(expected tu %i data %i/%i link %i/%i, " \
				     "found tu %i, data %i/%i link %i/%i)", \
				     current_config->name.tu, \
				     current_config->name.data_m, \
				     current_config->name.data_n, \
				     current_config->name.link_m, \
				     current_config->name.link_n, \
				     pipe_config->name.tu, \
				     pipe_config->name.data_m, \
				     pipe_config->name.data_n, \
				     pipe_config->name.link_m, \
				     pipe_config->name.link_n); \
		ret = false; \
	} \
} while (0)

#define PIPE_CONF_CHECK_TIMINGS(name) do { \
	PIPE_CONF_CHECK_I(name.crtc_hdisplay); \
	PIPE_CONF_CHECK_I(name.crtc_htotal); \
	PIPE_CONF_CHECK_I(name.crtc_hblank_start); \
	PIPE_CONF_CHECK_I(name.crtc_hblank_end); \
	PIPE_CONF_CHECK_I(name.crtc_hsync_start); \
	PIPE_CONF_CHECK_I(name.crtc_hsync_end); \
	PIPE_CONF_CHECK_I(name.crtc_vdisplay); \
	PIPE_CONF_CHECK_I(name.crtc_vtotal); \
	PIPE_CONF_CHECK_I(name.crtc_vblank_start); \
	PIPE_CONF_CHECK_I(name.crtc_vblank_end); \
	PIPE_CONF_CHECK_I(name.crtc_vsync_start); \
	PIPE_CONF_CHECK_I(name.crtc_vsync_end); \
} while (0)

#define PIPE_CONF_CHECK_RECT(name) do { \
	PIPE_CONF_CHECK_I(name.x1); \
	PIPE_CONF_CHECK_I(name.x2); \
	PIPE_CONF_CHECK_I(name.y1); \
	PIPE_CONF_CHECK_I(name.y2); \
} while (0)

#define PIPE_CONF_CHECK_FLAGS(name, mask) do { \
	if ((current_config->name ^ pipe_config->name) & (mask)) { \
		pipe_config_mismatch(fastset, crtc, __stringify(name), \
				     "(%x) (expected %i, found %i)", \
				     (mask), \
				     current_config->name & (mask), \
				     pipe_config->name & (mask)); \
		ret = false; \
	} \
} while (0)

#define PIPE_CONF_CHECK_INFOFRAME(name) do { \
	if (!intel_compare_infoframe(&current_config->infoframes.name, \
				     &pipe_config->infoframes.name)) { \
		pipe_config_infoframe_mismatch(dev_priv, fastset, __stringify(name), \
					       &current_config->infoframes.name, \
					       &pipe_config->infoframes.name); \
		ret = false; \
	} \
} while (0)

#define PIPE_CONF_CHECK_DP_VSC_SDP(name) do { \
	if (!current_config->has_psr && !pipe_config->has_psr && \
	    !intel_compare_dp_vsc_sdp(&current_config->infoframes.name, \
				      &pipe_config->infoframes.name)) { \
		pipe_config_dp_vsc_sdp_mismatch(dev_priv, fastset, __stringify(name), \
						&current_config->infoframes.name, \
						&pipe_config->infoframes.name); \
		ret = false; \
	} \
} while (0)

#define PIPE_CONF_CHECK_BUFFER(name, len) do { \
	BUILD_BUG_ON(sizeof(current_config->name) != (len)); \
	BUILD_BUG_ON(sizeof(pipe_config->name) != (len)); \
	if (!intel_compare_buffer(current_config->name, pipe_config->name, (len))) { \
		pipe_config_buffer_mismatch(dev_priv, fastset, __stringify(name), \
					    current_config->name, \
					    pipe_config->name, \
					    (len)); \
		ret = false; \
	} \
} while (0)

#define PIPE_CONF_CHECK_COLOR_LUT(lut, is_pre_csc_lut) do { \
	if (current_config->gamma_mode == pipe_config->gamma_mode && \
	    !intel_color_lut_equal(current_config, \
				   current_config->lut, pipe_config->lut, \
				   is_pre_csc_lut)) {	\
		pipe_config_mismatch(fastset, crtc, __stringify(lut), \
				     "hw_state doesn't match sw_state"); \
		ret = false; \
	} \
} while (0)

#define PIPE_CONF_CHECK_CSC(name) do { \
	PIPE_CONF_CHECK_X(name.preoff[0]); \
	PIPE_CONF_CHECK_X(name.preoff[1]); \
	PIPE_CONF_CHECK_X(name.preoff[2]); \
	PIPE_CONF_CHECK_X(name.coeff[0]); \
	PIPE_CONF_CHECK_X(name.coeff[1]); \
	PIPE_CONF_CHECK_X(name.coeff[2]); \
	PIPE_CONF_CHECK_X(name.coeff[3]); \
	PIPE_CONF_CHECK_X(name.coeff[4]); \
	PIPE_CONF_CHECK_X(name.coeff[5]); \
	PIPE_CONF_CHECK_X(name.coeff[6]); \
	PIPE_CONF_CHECK_X(name.coeff[7]); \
	PIPE_CONF_CHECK_X(name.coeff[8]); \
	PIPE_CONF_CHECK_X(name.postoff[0]); \
	PIPE_CONF_CHECK_X(name.postoff[1]); \
	PIPE_CONF_CHECK_X(name.postoff[2]); \
} while (0)

#define PIPE_CONF_QUIRK(quirk) \
	((current_config->quirks | pipe_config->quirks) & (quirk))

	PIPE_CONF_CHECK_I(hw.enable);
	PIPE_CONF_CHECK_I(hw.active);

	PIPE_CONF_CHECK_I(cpu_transcoder);
	PIPE_CONF_CHECK_I(mst_master_transcoder);

	PIPE_CONF_CHECK_BOOL(has_pch_encoder);
	PIPE_CONF_CHECK_I(fdi_lanes);
	PIPE_CONF_CHECK_M_N(fdi_m_n);

	PIPE_CONF_CHECK_I(lane_count);
	PIPE_CONF_CHECK_X(lane_lat_optim_mask);

	if (HAS_DOUBLE_BUFFERED_M_N(dev_priv)) {
		if (!fastset || !pipe_config->update_m_n)
			PIPE_CONF_CHECK_M_N(dp_m_n);
	} else {
		PIPE_CONF_CHECK_M_N(dp_m_n);
		PIPE_CONF_CHECK_M_N(dp_m2_n2);
	}

	PIPE_CONF_CHECK_X(output_types);

	PIPE_CONF_CHECK_I(framestart_delay);
	PIPE_CONF_CHECK_I(msa_timing_delay);

	PIPE_CONF_CHECK_TIMINGS(hw.pipe_mode);
	PIPE_CONF_CHECK_TIMINGS(hw.adjusted_mode);

	PIPE_CONF_CHECK_I(pixel_multiplier);

	PIPE_CONF_CHECK_FLAGS(hw.adjusted_mode.flags,
			      DRM_MODE_FLAG_INTERLACE);

	if (!PIPE_CONF_QUIRK(PIPE_CONFIG_QUIRK_MODE_SYNC_FLAGS)) {
		PIPE_CONF_CHECK_FLAGS(hw.adjusted_mode.flags,
				      DRM_MODE_FLAG_PHSYNC);
		PIPE_CONF_CHECK_FLAGS(hw.adjusted_mode.flags,
				      DRM_MODE_FLAG_NHSYNC);
		PIPE_CONF_CHECK_FLAGS(hw.adjusted_mode.flags,
				      DRM_MODE_FLAG_PVSYNC);
		PIPE_CONF_CHECK_FLAGS(hw.adjusted_mode.flags,
				      DRM_MODE_FLAG_NVSYNC);
	}

	PIPE_CONF_CHECK_I(output_format);
	PIPE_CONF_CHECK_BOOL(has_hdmi_sink);
	if ((DISPLAY_VER(dev_priv) < 8 && !IS_HASWELL(dev_priv)) ||
	    IS_VALLEYVIEW(dev_priv) || IS_CHERRYVIEW(dev_priv))
		PIPE_CONF_CHECK_BOOL(limited_color_range);

	PIPE_CONF_CHECK_BOOL(hdmi_scrambling);
	PIPE_CONF_CHECK_BOOL(hdmi_high_tmds_clock_ratio);
	PIPE_CONF_CHECK_BOOL(has_infoframe);
	PIPE_CONF_CHECK_BOOL(enhanced_framing);
	PIPE_CONF_CHECK_BOOL(fec_enable);

	PIPE_CONF_CHECK_BOOL_INCOMPLETE(has_audio);
	PIPE_CONF_CHECK_BUFFER(eld, MAX_ELD_BYTES);

	PIPE_CONF_CHECK_X(gmch_pfit.control);
	/* pfit ratios are autocomputed by the hw on gen4+ */
	if (DISPLAY_VER(dev_priv) < 4)
		PIPE_CONF_CHECK_X(gmch_pfit.pgm_ratios);
	PIPE_CONF_CHECK_X(gmch_pfit.lvds_border_bits);

	/*
	 * Changing the EDP transcoder input mux
	 * (A_ONOFF vs. A_ON) requires a full modeset.
	 */
	PIPE_CONF_CHECK_BOOL(pch_pfit.force_thru);

	if (!fastset) {
		PIPE_CONF_CHECK_RECT(pipe_src);

		PIPE_CONF_CHECK_BOOL(pch_pfit.enabled);
		PIPE_CONF_CHECK_RECT(pch_pfit.dst);

		PIPE_CONF_CHECK_I(scaler_state.scaler_id);
		PIPE_CONF_CHECK_I(pixel_rate);

		PIPE_CONF_CHECK_X(gamma_mode);
		if (IS_CHERRYVIEW(dev_priv))
			PIPE_CONF_CHECK_X(cgm_mode);
		else
			PIPE_CONF_CHECK_X(csc_mode);
		PIPE_CONF_CHECK_BOOL(gamma_enable);
		PIPE_CONF_CHECK_BOOL(csc_enable);
		PIPE_CONF_CHECK_BOOL(wgc_enable);

		PIPE_CONF_CHECK_I(linetime);
		PIPE_CONF_CHECK_I(ips_linetime);

		PIPE_CONF_CHECK_COLOR_LUT(pre_csc_lut, true);
		PIPE_CONF_CHECK_COLOR_LUT(post_csc_lut, false);

		PIPE_CONF_CHECK_CSC(csc);
		PIPE_CONF_CHECK_CSC(output_csc);

		if (current_config->active_planes) {
			PIPE_CONF_CHECK_BOOL(has_psr);
			PIPE_CONF_CHECK_BOOL(has_psr2);
			PIPE_CONF_CHECK_BOOL(enable_psr2_sel_fetch);
			PIPE_CONF_CHECK_I(dc3co_exitline);
		}
	}

	PIPE_CONF_CHECK_BOOL(double_wide);

	if (dev_priv->display.dpll.mgr) {
		PIPE_CONF_CHECK_P(shared_dpll);

		PIPE_CONF_CHECK_X(dpll_hw_state.dpll);
		PIPE_CONF_CHECK_X(dpll_hw_state.dpll_md);
		PIPE_CONF_CHECK_X(dpll_hw_state.fp0);
		PIPE_CONF_CHECK_X(dpll_hw_state.fp1);
		PIPE_CONF_CHECK_X(dpll_hw_state.wrpll);
		PIPE_CONF_CHECK_X(dpll_hw_state.spll);
		PIPE_CONF_CHECK_X(dpll_hw_state.ctrl1);
		PIPE_CONF_CHECK_X(dpll_hw_state.cfgcr1);
		PIPE_CONF_CHECK_X(dpll_hw_state.cfgcr2);
		PIPE_CONF_CHECK_X(dpll_hw_state.cfgcr0);
		PIPE_CONF_CHECK_X(dpll_hw_state.div0);
		PIPE_CONF_CHECK_X(dpll_hw_state.ebb0);
		PIPE_CONF_CHECK_X(dpll_hw_state.ebb4);
		PIPE_CONF_CHECK_X(dpll_hw_state.pll0);
		PIPE_CONF_CHECK_X(dpll_hw_state.pll1);
		PIPE_CONF_CHECK_X(dpll_hw_state.pll2);
		PIPE_CONF_CHECK_X(dpll_hw_state.pll3);
		PIPE_CONF_CHECK_X(dpll_hw_state.pll6);
		PIPE_CONF_CHECK_X(dpll_hw_state.pll8);
		PIPE_CONF_CHECK_X(dpll_hw_state.pll9);
		PIPE_CONF_CHECK_X(dpll_hw_state.pll10);
		PIPE_CONF_CHECK_X(dpll_hw_state.pcsdw12);
		PIPE_CONF_CHECK_X(dpll_hw_state.mg_refclkin_ctl);
		PIPE_CONF_CHECK_X(dpll_hw_state.mg_clktop2_coreclkctl1);
		PIPE_CONF_CHECK_X(dpll_hw_state.mg_clktop2_hsclkctl);
		PIPE_CONF_CHECK_X(dpll_hw_state.mg_pll_div0);
		PIPE_CONF_CHECK_X(dpll_hw_state.mg_pll_div1);
		PIPE_CONF_CHECK_X(dpll_hw_state.mg_pll_lf);
		PIPE_CONF_CHECK_X(dpll_hw_state.mg_pll_frac_lock);
		PIPE_CONF_CHECK_X(dpll_hw_state.mg_pll_ssc);
		PIPE_CONF_CHECK_X(dpll_hw_state.mg_pll_bias);
		PIPE_CONF_CHECK_X(dpll_hw_state.mg_pll_tdc_coldst_bias);
	}

	PIPE_CONF_CHECK_X(dsi_pll.ctrl);
	PIPE_CONF_CHECK_X(dsi_pll.div);

	if (IS_G4X(dev_priv) || DISPLAY_VER(dev_priv) >= 5)
		PIPE_CONF_CHECK_I(pipe_bpp);

	if (!fastset || !pipe_config->update_m_n) {
		PIPE_CONF_CHECK_I(hw.pipe_mode.crtc_clock);
		PIPE_CONF_CHECK_I(hw.adjusted_mode.crtc_clock);
	}
	PIPE_CONF_CHECK_I(port_clock);

	PIPE_CONF_CHECK_I(min_voltage_level);

	if (current_config->has_psr || pipe_config->has_psr)
		PIPE_CONF_CHECK_X_WITH_MASK(infoframes.enable,
					    ~intel_hdmi_infoframe_enable(DP_SDP_VSC));
	else
		PIPE_CONF_CHECK_X(infoframes.enable);

	PIPE_CONF_CHECK_X(infoframes.gcp);
	PIPE_CONF_CHECK_INFOFRAME(avi);
	PIPE_CONF_CHECK_INFOFRAME(spd);
	PIPE_CONF_CHECK_INFOFRAME(hdmi);
	PIPE_CONF_CHECK_INFOFRAME(drm);
	PIPE_CONF_CHECK_DP_VSC_SDP(vsc);

	PIPE_CONF_CHECK_X(sync_mode_slaves_mask);
	PIPE_CONF_CHECK_I(master_transcoder);
	PIPE_CONF_CHECK_X(bigjoiner_pipes);

	PIPE_CONF_CHECK_I(dsc.compression_enable);
	PIPE_CONF_CHECK_I(dsc.dsc_split);
	PIPE_CONF_CHECK_I(dsc.compressed_bpp);

	PIPE_CONF_CHECK_BOOL(splitter.enable);
	PIPE_CONF_CHECK_I(splitter.link_count);
	PIPE_CONF_CHECK_I(splitter.pixel_overlap);

	if (!fastset)
		PIPE_CONF_CHECK_BOOL(vrr.enable);
	PIPE_CONF_CHECK_I(vrr.vmin);
	PIPE_CONF_CHECK_I(vrr.vmax);
	PIPE_CONF_CHECK_I(vrr.flipline);
	PIPE_CONF_CHECK_I(vrr.pipeline_full);
	PIPE_CONF_CHECK_I(vrr.guardband);

#undef PIPE_CONF_CHECK_X
#undef PIPE_CONF_CHECK_I
#undef PIPE_CONF_CHECK_BOOL
#undef PIPE_CONF_CHECK_BOOL_INCOMPLETE
#undef PIPE_CONF_CHECK_P
#undef PIPE_CONF_CHECK_FLAGS
#undef PIPE_CONF_CHECK_COLOR_LUT
#undef PIPE_CONF_CHECK_TIMINGS
#undef PIPE_CONF_CHECK_RECT
#undef PIPE_CONF_QUIRK

	return ret;
}

static void
intel_verify_planes(struct intel_atomic_state *state)
{
	struct intel_plane *plane;
	const struct intel_plane_state *plane_state;
	int i;

	for_each_new_intel_plane_in_state(state, plane,
					  plane_state, i)
		assert_plane(plane, plane_state->planar_slave ||
			     plane_state->uapi.visible);
}

int intel_modeset_all_pipes(struct intel_atomic_state *state,
			    const char *reason)
{
	struct drm_i915_private *dev_priv = to_i915(state->base.dev);
	struct intel_crtc *crtc;

	/*
	 * Add all pipes to the state, and force
	 * a modeset on all the active ones.
	 */
	for_each_intel_crtc(&dev_priv->drm, crtc) {
		struct intel_crtc_state *crtc_state;
		int ret;

		crtc_state = intel_atomic_get_crtc_state(&state->base, crtc);
		if (IS_ERR(crtc_state))
			return PTR_ERR(crtc_state);

		if (!crtc_state->hw.active ||
		    intel_crtc_needs_modeset(crtc_state))
			continue;

		drm_dbg_kms(&dev_priv->drm, "[CRTC:%d:%s] Full modeset due to %s\n",
			    crtc->base.base.id, crtc->base.name, reason);

		crtc_state->uapi.mode_changed = true;
		crtc_state->update_pipe = false;
		crtc_state->update_m_n = false;

		ret = drm_atomic_add_affected_connectors(&state->base,
							 &crtc->base);
		if (ret)
			return ret;

		ret = intel_dp_mst_add_topology_state_for_crtc(state, crtc);
		if (ret)
			return ret;

		ret = intel_atomic_add_affected_planes(state, crtc);
		if (ret)
			return ret;

		crtc_state->update_planes |= crtc_state->active_planes;
		crtc_state->async_flip_planes = 0;
		crtc_state->do_async_flip = false;
	}

	return 0;
}

/*
 * This implements the workaround described in the "notes" section of the mode
 * set sequence documentation. When going from no pipes or single pipe to
 * multiple pipes, and planes are enabled after the pipe, we need to wait at
 * least 2 vblanks on the first pipe before enabling planes on the second pipe.
 */
static int hsw_mode_set_planes_workaround(struct intel_atomic_state *state)
{
	struct intel_crtc_state *crtc_state;
	struct intel_crtc *crtc;
	struct intel_crtc_state *first_crtc_state = NULL;
	struct intel_crtc_state *other_crtc_state = NULL;
	enum pipe first_pipe = INVALID_PIPE, enabled_pipe = INVALID_PIPE;
	int i;

	/* look at all crtc's that are going to be enabled in during modeset */
	for_each_new_intel_crtc_in_state(state, crtc, crtc_state, i) {
		if (!crtc_state->hw.active ||
		    !intel_crtc_needs_modeset(crtc_state))
			continue;

		if (first_crtc_state) {
			other_crtc_state = crtc_state;
			break;
		} else {
			first_crtc_state = crtc_state;
			first_pipe = crtc->pipe;
		}
	}

	/* No workaround needed? */
	if (!first_crtc_state)
		return 0;

	/* w/a possibly needed, check how many crtc's are already enabled. */
	for_each_intel_crtc(state->base.dev, crtc) {
		crtc_state = intel_atomic_get_crtc_state(&state->base, crtc);
		if (IS_ERR(crtc_state))
			return PTR_ERR(crtc_state);

		crtc_state->hsw_workaround_pipe = INVALID_PIPE;

		if (!crtc_state->hw.active ||
		    intel_crtc_needs_modeset(crtc_state))
			continue;

		/* 2 or more enabled crtcs means no need for w/a */
		if (enabled_pipe != INVALID_PIPE)
			return 0;

		enabled_pipe = crtc->pipe;
	}

	if (enabled_pipe != INVALID_PIPE)
		first_crtc_state->hsw_workaround_pipe = enabled_pipe;
	else if (other_crtc_state)
		other_crtc_state->hsw_workaround_pipe = first_pipe;

	return 0;
}

u8 intel_calc_active_pipes(struct intel_atomic_state *state,
			   u8 active_pipes)
{
	const struct intel_crtc_state *crtc_state;
	struct intel_crtc *crtc;
	int i;

	for_each_new_intel_crtc_in_state(state, crtc, crtc_state, i) {
		if (crtc_state->hw.active)
			active_pipes |= BIT(crtc->pipe);
		else
			active_pipes &= ~BIT(crtc->pipe);
	}

	return active_pipes;
}

static int intel_modeset_checks(struct intel_atomic_state *state)
{
	struct drm_i915_private *dev_priv = to_i915(state->base.dev);

	state->modeset = true;

	if (IS_HASWELL(dev_priv))
		return hsw_mode_set_planes_workaround(state);

	return 0;
}

static void intel_crtc_check_fastset(const struct intel_crtc_state *old_crtc_state,
				     struct intel_crtc_state *new_crtc_state)
{
	struct drm_i915_private *i915 = to_i915(old_crtc_state->uapi.crtc->dev);

	if (!intel_pipe_config_compare(old_crtc_state, new_crtc_state, true))
		drm_dbg_kms(&i915->drm, "fastset requirement not met, forcing full modeset\n");
	else
		new_crtc_state->uapi.mode_changed = false;

	if (intel_crtc_needs_modeset(new_crtc_state))
		new_crtc_state->update_m_n = false;

	if (!intel_crtc_needs_modeset(new_crtc_state))
		new_crtc_state->update_pipe = true;
}

static int intel_crtc_add_planes_to_state(struct intel_atomic_state *state,
					  struct intel_crtc *crtc,
					  u8 plane_ids_mask)
{
	struct drm_i915_private *dev_priv = to_i915(state->base.dev);
	struct intel_plane *plane;

	for_each_intel_plane_on_crtc(&dev_priv->drm, crtc, plane) {
		struct intel_plane_state *plane_state;

		if ((plane_ids_mask & BIT(plane->id)) == 0)
			continue;

		plane_state = intel_atomic_get_plane_state(state, plane);
		if (IS_ERR(plane_state))
			return PTR_ERR(plane_state);
	}

	return 0;
}

int intel_atomic_add_affected_planes(struct intel_atomic_state *state,
				     struct intel_crtc *crtc)
{
	const struct intel_crtc_state *old_crtc_state =
		intel_atomic_get_old_crtc_state(state, crtc);
	const struct intel_crtc_state *new_crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);

	return intel_crtc_add_planes_to_state(state, crtc,
					      old_crtc_state->enabled_planes |
					      new_crtc_state->enabled_planes);
}

static bool active_planes_affects_min_cdclk(struct drm_i915_private *dev_priv)
{
	/* See {hsw,vlv,ivb}_plane_ratio() */
	return IS_BROADWELL(dev_priv) || IS_HASWELL(dev_priv) ||
		IS_CHERRYVIEW(dev_priv) || IS_VALLEYVIEW(dev_priv) ||
		IS_IVYBRIDGE(dev_priv);
}

static int intel_crtc_add_bigjoiner_planes(struct intel_atomic_state *state,
					   struct intel_crtc *crtc,
					   struct intel_crtc *other)
{
	const struct intel_plane_state __maybe_unused *plane_state;
	struct intel_plane *plane;
	u8 plane_ids = 0;
	int i;

	for_each_new_intel_plane_in_state(state, plane, plane_state, i) {
		if (plane->pipe == crtc->pipe)
			plane_ids |= BIT(plane->id);
	}

	return intel_crtc_add_planes_to_state(state, other, plane_ids);
}

static int intel_bigjoiner_add_affected_planes(struct intel_atomic_state *state)
{
	struct drm_i915_private *i915 = to_i915(state->base.dev);
	const struct intel_crtc_state *crtc_state;
	struct intel_crtc *crtc;
	int i;

	for_each_new_intel_crtc_in_state(state, crtc, crtc_state, i) {
		struct intel_crtc *other;

		for_each_intel_crtc_in_pipe_mask(&i915->drm, other,
						 crtc_state->bigjoiner_pipes) {
			int ret;

			if (crtc == other)
				continue;

			ret = intel_crtc_add_bigjoiner_planes(state, crtc, other);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static int intel_atomic_check_planes(struct intel_atomic_state *state)
{
	struct drm_i915_private *dev_priv = to_i915(state->base.dev);
	struct intel_crtc_state *old_crtc_state, *new_crtc_state;
<<<<<<< HEAD
	struct intel_plane_state *plane_state __unused;
=======
	struct intel_plane_state __maybe_unused *plane_state;
>>>>>>> vendor/linux-drm-v6.6.35
	struct intel_plane *plane;
	struct intel_crtc *crtc;
	int i, ret;

	ret = icl_add_linked_planes(state);
	if (ret)
		return ret;

	ret = intel_bigjoiner_add_affected_planes(state);
	if (ret)
		return ret;

	for_each_new_intel_plane_in_state(state, plane, plane_state, i) {
		ret = intel_plane_atomic_check(state, plane);
		if (ret) {
			drm_dbg_atomic(&dev_priv->drm,
				       "[PLANE:%d:%s] atomic driver check failed\n",
				       plane->base.base.id, plane->base.name);
			return ret;
		}
	}

	for_each_oldnew_intel_crtc_in_state(state, crtc, old_crtc_state,
					    new_crtc_state, i) {
		u8 old_active_planes, new_active_planes;

		ret = icl_check_nv12_planes(new_crtc_state);
		if (ret)
			return ret;

		/*
		 * On some platforms the number of active planes affects
		 * the planes' minimum cdclk calculation. Add such planes
		 * to the state before we compute the minimum cdclk.
		 */
		if (!active_planes_affects_min_cdclk(dev_priv))
			continue;

		old_active_planes = old_crtc_state->active_planes & ~BIT(PLANE_CURSOR);
		new_active_planes = new_crtc_state->active_planes & ~BIT(PLANE_CURSOR);

		if (hweight8(old_active_planes) == hweight8(new_active_planes))
			continue;

		ret = intel_crtc_add_planes_to_state(state, crtc, new_active_planes);
		if (ret)
			return ret;
	}

	return 0;
}

static int intel_atomic_check_crtcs(struct intel_atomic_state *state)
{
<<<<<<< HEAD
	struct intel_crtc_state *crtc_state __unused;
=======
	struct intel_crtc_state __maybe_unused *crtc_state;
>>>>>>> vendor/linux-drm-v6.6.35
	struct intel_crtc *crtc;
	int i;

	for_each_new_intel_crtc_in_state(state, crtc, crtc_state, i) {
		struct drm_i915_private *i915 = to_i915(crtc->base.dev);
		int ret;

		ret = intel_crtc_atomic_check(state, crtc);
		if (ret) {
			drm_dbg_atomic(&i915->drm,
				       "[CRTC:%d:%s] atomic driver check failed\n",
				       crtc->base.base.id, crtc->base.name);
			return ret;
		}
	}

	return 0;
}

static bool intel_cpu_transcoders_need_modeset(struct intel_atomic_state *state,
					       u8 transcoders)
{
	const struct intel_crtc_state *new_crtc_state;
	struct intel_crtc *crtc;
	int i;

	for_each_new_intel_crtc_in_state(state, crtc, new_crtc_state, i) {
		if (new_crtc_state->hw.enable &&
		    transcoders & BIT(new_crtc_state->cpu_transcoder) &&
		    intel_crtc_needs_modeset(new_crtc_state))
			return true;
	}

	return false;
}

static bool intel_pipes_need_modeset(struct intel_atomic_state *state,
				     u8 pipes)
{
	const struct intel_crtc_state *new_crtc_state;
	struct intel_crtc *crtc;
	int i;

	for_each_new_intel_crtc_in_state(state, crtc, new_crtc_state, i) {
		if (new_crtc_state->hw.enable &&
		    pipes & BIT(crtc->pipe) &&
		    intel_crtc_needs_modeset(new_crtc_state))
			return true;
	}

	return false;
}

static int intel_atomic_check_bigjoiner(struct intel_atomic_state *state,
					struct intel_crtc *master_crtc)
{
<<<<<<< HEAD
	struct drm_i915_private *dev_priv = to_i915(state->base.dev);
	struct drm_connector *connector;
	struct drm_connector_state *old_conn_state __unused, *new_conn_state __unused;
	int i, ret;
=======
	struct drm_i915_private *i915 = to_i915(state->base.dev);
	struct intel_crtc_state *master_crtc_state =
		intel_atomic_get_new_crtc_state(state, master_crtc);
	struct intel_crtc *slave_crtc;
>>>>>>> vendor/linux-drm-v6.6.35

	if (!master_crtc_state->bigjoiner_pipes)
		return 0;

	/* sanity check */
	if (drm_WARN_ON(&i915->drm,
			master_crtc->pipe != bigjoiner_master_pipe(master_crtc_state)))
		return -EINVAL;

	if (master_crtc_state->bigjoiner_pipes & ~bigjoiner_pipes(i915)) {
		drm_dbg_kms(&i915->drm,
			    "[CRTC:%d:%s] Cannot act as big joiner master "
			    "(need 0x%x as pipes, only 0x%x possible)\n",
			    master_crtc->base.base.id, master_crtc->base.name,
			    master_crtc_state->bigjoiner_pipes, bigjoiner_pipes(i915));
		return -EINVAL;
	}

	for_each_intel_crtc_in_pipe_mask(&i915->drm, slave_crtc,
					 intel_crtc_bigjoiner_slave_pipes(master_crtc_state)) {
		struct intel_crtc_state *slave_crtc_state;
		int ret;

		slave_crtc_state = intel_atomic_get_crtc_state(&state->base, slave_crtc);
		if (IS_ERR(slave_crtc_state))
			return PTR_ERR(slave_crtc_state);

		/* master being enabled, slave was already configured? */
		if (slave_crtc_state->uapi.enable) {
			drm_dbg_kms(&i915->drm,
				    "[CRTC:%d:%s] Slave is enabled as normal CRTC, but "
				    "[CRTC:%d:%s] claiming this CRTC for bigjoiner.\n",
				    slave_crtc->base.base.id, slave_crtc->base.name,
				    master_crtc->base.base.id, master_crtc->base.name);
			return -EINVAL;
		}

		/*
		 * The state copy logic assumes the master crtc gets processed
		 * before the slave crtc during the main compute_config loop.
		 * This works because the crtcs are created in pipe order,
		 * and the hardware requires master pipe < slave pipe as well.
		 * Should that change we need to rethink the logic.
		 */
		if (WARN_ON(drm_crtc_index(&master_crtc->base) >
			    drm_crtc_index(&slave_crtc->base)))
			return -EINVAL;

		drm_dbg_kms(&i915->drm,
			    "[CRTC:%d:%s] Used as slave for big joiner master [CRTC:%d:%s]\n",
			    slave_crtc->base.base.id, slave_crtc->base.name,
			    master_crtc->base.base.id, master_crtc->base.name);

		slave_crtc_state->bigjoiner_pipes =
			master_crtc_state->bigjoiner_pipes;

		ret = copy_bigjoiner_crtc_state_modeset(state, slave_crtc);
		if (ret)
			return ret;
	}

	return 0;
}

static void kill_bigjoiner_slave(struct intel_atomic_state *state,
				 struct intel_crtc *master_crtc)
{
	struct drm_i915_private *i915 = to_i915(state->base.dev);
	struct intel_crtc_state *master_crtc_state =
		intel_atomic_get_new_crtc_state(state, master_crtc);
	struct intel_crtc *slave_crtc;

	for_each_intel_crtc_in_pipe_mask(&i915->drm, slave_crtc,
					 intel_crtc_bigjoiner_slave_pipes(master_crtc_state)) {
		struct intel_crtc_state *slave_crtc_state =
			intel_atomic_get_new_crtc_state(state, slave_crtc);

		slave_crtc_state->bigjoiner_pipes = 0;

		intel_crtc_copy_uapi_to_hw_state_modeset(state, slave_crtc);
	}

	master_crtc_state->bigjoiner_pipes = 0;
}

/**
 * DOC: asynchronous flip implementation
 *
 * Asynchronous page flip is the implementation for the DRM_MODE_PAGE_FLIP_ASYNC
 * flag. Currently async flip is only supported via the drmModePageFlip IOCTL.
 * Correspondingly, support is currently added for primary plane only.
 *
 * Async flip can only change the plane surface address, so anything else
 * changing is rejected from the intel_async_flip_check_hw() function.
 * Once this check is cleared, flip done interrupt is enabled using
 * the intel_crtc_enable_flip_done() function.
 *
 * As soon as the surface address register is written, flip done interrupt is
 * generated and the requested events are sent to the usersapce in the interrupt
 * handler itself. The timestamp and sequence sent during the flip done event
 * correspond to the last vblank and have no relation to the actual time when
 * the flip done event was sent.
 */
static int intel_async_flip_check_uapi(struct intel_atomic_state *state,
				       struct intel_crtc *crtc)
{
	struct drm_i915_private *i915 = to_i915(state->base.dev);
	const struct intel_crtc_state *new_crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	const struct intel_plane_state *old_plane_state;
	struct intel_plane_state *new_plane_state;
	struct intel_plane *plane;
	int i;

	if (!new_crtc_state->uapi.async_flip)
		return 0;

	if (!new_crtc_state->uapi.active) {
		drm_dbg_kms(&i915->drm,
			    "[CRTC:%d:%s] not active\n",
			    crtc->base.base.id, crtc->base.name);
		return -EINVAL;
	}

	if (intel_crtc_needs_modeset(new_crtc_state)) {
		drm_dbg_kms(&i915->drm,
			    "[CRTC:%d:%s] modeset required\n",
			    crtc->base.base.id, crtc->base.name);
		return -EINVAL;
	}

	for_each_oldnew_intel_plane_in_state(state, plane, old_plane_state,
					     new_plane_state, i) {
		if (plane->pipe != crtc->pipe)
			continue;

		/*
		 * TODO: Async flip is only supported through the page flip IOCTL
		 * as of now. So support currently added for primary plane only.
		 * Support for other planes on platforms on which supports
		 * this(vlv/chv and icl+) should be added when async flip is
		 * enabled in the atomic IOCTL path.
		 */
		if (!plane->async_flip) {
			drm_dbg_kms(&i915->drm,
				    "[PLANE:%d:%s] async flip not supported\n",
				    plane->base.base.id, plane->base.name);
			return -EINVAL;
		}

		if (!old_plane_state->uapi.fb || !new_plane_state->uapi.fb) {
			drm_dbg_kms(&i915->drm,
				    "[PLANE:%d:%s] no old or new framebuffer\n",
				    plane->base.base.id, plane->base.name);
			return -EINVAL;
		}
	}

	return 0;
}

static int intel_async_flip_check_hw(struct intel_atomic_state *state, struct intel_crtc *crtc)
{
	struct drm_i915_private *i915 = to_i915(state->base.dev);
	const struct intel_crtc_state *old_crtc_state, *new_crtc_state;
	const struct intel_plane_state *new_plane_state, *old_plane_state;
	struct intel_plane *plane;
	int i;

	old_crtc_state = intel_atomic_get_old_crtc_state(state, crtc);
	new_crtc_state = intel_atomic_get_new_crtc_state(state, crtc);

	if (!new_crtc_state->uapi.async_flip)
		return 0;

	if (!new_crtc_state->hw.active) {
		drm_dbg_kms(&i915->drm,
			    "[CRTC:%d:%s] not active\n",
			    crtc->base.base.id, crtc->base.name);
		return -EINVAL;
	}

	if (intel_crtc_needs_modeset(new_crtc_state)) {
		drm_dbg_kms(&i915->drm,
			    "[CRTC:%d:%s] modeset required\n",
			    crtc->base.base.id, crtc->base.name);
		return -EINVAL;
	}

	if (old_crtc_state->active_planes != new_crtc_state->active_planes) {
		drm_dbg_kms(&i915->drm,
			    "[CRTC:%d:%s] Active planes cannot be in async flip\n",
			    crtc->base.base.id, crtc->base.name);
		return -EINVAL;
	}

	/*
	 * FIXME: Bigjoiner+async flip is busted currently.
	 * Remove this check once the issues are fixed.
	 */
	if (new_crtc_state->bigjoiner_pipes) {
		drm_dbg_kms(&i915->drm,
			    "[CRTC:%d:%s] async flip disallowed with bigjoiner\n",
			    crtc->base.base.id, crtc->base.name);
		return -EINVAL;
	}

	for_each_oldnew_intel_plane_in_state(state, plane, old_plane_state,
					     new_plane_state, i) {
		if (plane->pipe != crtc->pipe)
			continue;

		/*
		 * Only async flip capable planes should be in the state
		 * if we're really about to ask the hardware to perform
		 * an async flip. We should never get this far otherwise.
		 */
		if (drm_WARN_ON(&i915->drm,
				new_crtc_state->do_async_flip && !plane->async_flip))
			return -EINVAL;

		/*
		 * Only check async flip capable planes other planes
		 * may be involved in the initial commit due to
		 * the wm0/ddb optimization.
		 *
		 * TODO maybe should track which planes actually
		 * were requested to do the async flip...
		 */
		if (!plane->async_flip)
			continue;

		/*
		 * FIXME: This check is kept generic for all platforms.
		 * Need to verify this for all gen9 platforms to enable
		 * this selectively if required.
		 */
		switch (new_plane_state->hw.fb->modifier) {
		case DRM_FORMAT_MOD_LINEAR:
			/*
			 * FIXME: Async on Linear buffer is supported on ICL as
			 * but with additional alignment and fbc restrictions
			 * need to be taken care of. These aren't applicable for
			 * gen12+.
			 */
			if (DISPLAY_VER(i915) < 12) {
				drm_dbg_kms(&i915->drm,
					    "[PLANE:%d:%s] Modifier 0x%llx does not support async flip on display ver %d\n",
					    plane->base.base.id, plane->base.name,
					    new_plane_state->hw.fb->modifier, DISPLAY_VER(i915));
				return -EINVAL;
			}
			break;

		case I915_FORMAT_MOD_X_TILED:
		case I915_FORMAT_MOD_Y_TILED:
		case I915_FORMAT_MOD_Yf_TILED:
		case I915_FORMAT_MOD_4_TILED:
			break;
		default:
			drm_dbg_kms(&i915->drm,
				    "[PLANE:%d:%s] Modifier 0x%llx does not support async flip\n",
				    plane->base.base.id, plane->base.name,
				    new_plane_state->hw.fb->modifier);
			return -EINVAL;
		}

		if (new_plane_state->hw.fb->format->num_planes > 1) {
			drm_dbg_kms(&i915->drm,
				    "[PLANE:%d:%s] Planar formats do not support async flips\n",
				    plane->base.base.id, plane->base.name);
			return -EINVAL;
		}

		if (old_plane_state->view.color_plane[0].mapping_stride !=
		    new_plane_state->view.color_plane[0].mapping_stride) {
			drm_dbg_kms(&i915->drm,
				    "[PLANE:%d:%s] Stride cannot be changed in async flip\n",
				    plane->base.base.id, plane->base.name);
			return -EINVAL;
		}

		if (old_plane_state->hw.fb->modifier !=
		    new_plane_state->hw.fb->modifier) {
			drm_dbg_kms(&i915->drm,
				    "[PLANE:%d:%s] Modifier cannot be changed in async flip\n",
				    plane->base.base.id, plane->base.name);
			return -EINVAL;
		}

		if (old_plane_state->hw.fb->format !=
		    new_plane_state->hw.fb->format) {
			drm_dbg_kms(&i915->drm,
				    "[PLANE:%d:%s] Pixel format cannot be changed in async flip\n",
				    plane->base.base.id, plane->base.name);
			return -EINVAL;
		}

		if (old_plane_state->hw.rotation !=
		    new_plane_state->hw.rotation) {
			drm_dbg_kms(&i915->drm,
				    "[PLANE:%d:%s] Rotation cannot be changed in async flip\n",
				    plane->base.base.id, plane->base.name);
			return -EINVAL;
		}

		if (!drm_rect_equals(&old_plane_state->uapi.src, &new_plane_state->uapi.src) ||
		    !drm_rect_equals(&old_plane_state->uapi.dst, &new_plane_state->uapi.dst)) {
			drm_dbg_kms(&i915->drm,
				    "[PLANE:%d:%s] Size/co-ordinates cannot be changed in async flip\n",
				    plane->base.base.id, plane->base.name);
			return -EINVAL;
		}

		if (old_plane_state->hw.alpha != new_plane_state->hw.alpha) {
			drm_dbg_kms(&i915->drm,
				    "[PLANES:%d:%s] Alpha value cannot be changed in async flip\n",
				    plane->base.base.id, plane->base.name);
			return -EINVAL;
		}

		if (old_plane_state->hw.pixel_blend_mode !=
		    new_plane_state->hw.pixel_blend_mode) {
			drm_dbg_kms(&i915->drm,
				    "[PLANE:%d:%s] Pixel blend mode cannot be changed in async flip\n",
				    plane->base.base.id, plane->base.name);
			return -EINVAL;
		}

		if (old_plane_state->hw.color_encoding != new_plane_state->hw.color_encoding) {
			drm_dbg_kms(&i915->drm,
				    "[PLANE:%d:%s] Color encoding cannot be changed in async flip\n",
				    plane->base.base.id, plane->base.name);
			return -EINVAL;
		}

		if (old_plane_state->hw.color_range != new_plane_state->hw.color_range) {
			drm_dbg_kms(&i915->drm,
				    "[PLANE:%d:%s] Color range cannot be changed in async flip\n",
				    plane->base.base.id, plane->base.name);
			return -EINVAL;
		}

		/* plane decryption is allow to change only in synchronous flips */
		if (old_plane_state->decrypt != new_plane_state->decrypt) {
			drm_dbg_kms(&i915->drm,
				    "[PLANE:%d:%s] Decryption cannot be changed in async flip\n",
				    plane->base.base.id, plane->base.name);
			return -EINVAL;
		}
	}

	return 0;
}

static int intel_bigjoiner_add_affected_crtcs(struct intel_atomic_state *state)
{
	struct drm_i915_private *i915 = to_i915(state->base.dev);
	struct intel_crtc_state *crtc_state;
	struct intel_crtc *crtc;
	u8 affected_pipes = 0;
	u8 modeset_pipes = 0;
	int i;

	for_each_new_intel_crtc_in_state(state, crtc, crtc_state, i) {
		affected_pipes |= crtc_state->bigjoiner_pipes;
		if (intel_crtc_needs_modeset(crtc_state))
			modeset_pipes |= crtc_state->bigjoiner_pipes;
	}

	for_each_intel_crtc_in_pipe_mask(&i915->drm, crtc, affected_pipes) {
		crtc_state = intel_atomic_get_crtc_state(&state->base, crtc);
		if (IS_ERR(crtc_state))
			return PTR_ERR(crtc_state);
	}

	for_each_intel_crtc_in_pipe_mask(&i915->drm, crtc, modeset_pipes) {
		int ret;

		crtc_state = intel_atomic_get_new_crtc_state(state, crtc);

		crtc_state->uapi.mode_changed = true;

		ret = drm_atomic_add_affected_connectors(&state->base, &crtc->base);
		if (ret)
			return ret;

		ret = intel_atomic_add_affected_planes(state, crtc);
		if (ret)
			return ret;
	}

	for_each_new_intel_crtc_in_state(state, crtc, crtc_state, i) {
		/* Kill old bigjoiner link, we may re-establish afterwards */
		if (intel_crtc_needs_modeset(crtc_state) &&
		    intel_crtc_is_bigjoiner_master(crtc_state))
			kill_bigjoiner_slave(state, crtc);
	}

	return 0;
}

/**
 * intel_atomic_check - validate state object
 * @dev: drm device
 * @_state: state to validate
 */
int intel_atomic_check(struct drm_device *dev,
		       struct drm_atomic_state *_state)
{
	struct drm_i915_private *dev_priv = to_i915(dev);
	struct intel_atomic_state *state = to_intel_atomic_state(_state);
	struct intel_crtc_state *old_crtc_state, *new_crtc_state;
	struct intel_crtc *crtc;
	int ret, i;
	bool any_ms = false;

	for_each_oldnew_intel_crtc_in_state(state, crtc, old_crtc_state,
					    new_crtc_state, i) {
		/*
		 * crtc's state no longer considered to be inherited
		 * after the first userspace/client initiated commit.
		 */
		if (!state->internal)
			new_crtc_state->inherited = false;

		if (new_crtc_state->inherited != old_crtc_state->inherited)
			new_crtc_state->uapi.mode_changed = true;

		if (new_crtc_state->uapi.scaling_filter !=
		    old_crtc_state->uapi.scaling_filter)
			new_crtc_state->uapi.mode_changed = true;
	}

	intel_vrr_check_modeset(state);

	ret = drm_atomic_helper_check_modeset(dev, &state->base);
	if (ret)
		goto fail;

	for_each_new_intel_crtc_in_state(state, crtc, new_crtc_state, i) {
		ret = intel_async_flip_check_uapi(state, crtc);
		if (ret)
			return ret;
	}

	ret = intel_bigjoiner_add_affected_crtcs(state);
	if (ret)
		goto fail;

	for_each_oldnew_intel_crtc_in_state(state, crtc, old_crtc_state,
					    new_crtc_state, i) {
		if (!intel_crtc_needs_modeset(new_crtc_state)) {
			if (intel_crtc_is_bigjoiner_slave(new_crtc_state))
				copy_bigjoiner_crtc_state_nomodeset(state, crtc);
			else
				intel_crtc_copy_uapi_to_hw_state_nomodeset(state, crtc);
			continue;
		}

		if (intel_crtc_is_bigjoiner_slave(new_crtc_state)) {
			drm_WARN_ON(&dev_priv->drm, new_crtc_state->uapi.enable);
			continue;
		}

		ret = intel_crtc_prepare_cleared_state(state, crtc);
		if (ret)
			goto fail;

		if (!new_crtc_state->hw.enable)
			continue;

		ret = intel_modeset_pipe_config(state, crtc);
		if (ret)
			goto fail;

		ret = intel_atomic_check_bigjoiner(state, crtc);
		if (ret)
			goto fail;
	}

	for_each_oldnew_intel_crtc_in_state(state, crtc, old_crtc_state,
					    new_crtc_state, i) {
		if (!intel_crtc_needs_modeset(new_crtc_state))
			continue;

		if (new_crtc_state->hw.enable) {
			ret = intel_modeset_pipe_config_late(state, crtc);
			if (ret)
				goto fail;
		}

		intel_crtc_check_fastset(old_crtc_state, new_crtc_state);
	}

	/**
	 * Check if fastset is allowed by external dependencies like other
	 * pipes and transcoders.
	 *
	 * Right now it only forces a fullmodeset when the MST master
	 * transcoder did not changed but the pipe of the master transcoder
	 * needs a fullmodeset so all slaves also needs to do a fullmodeset or
	 * in case of port synced crtcs, if one of the synced crtcs
	 * needs a full modeset, all other synced crtcs should be
	 * forced a full modeset.
	 */
	for_each_new_intel_crtc_in_state(state, crtc, new_crtc_state, i) {
		if (!new_crtc_state->hw.enable || intel_crtc_needs_modeset(new_crtc_state))
			continue;

		if (intel_dp_mst_is_slave_trans(new_crtc_state)) {
			enum transcoder master = new_crtc_state->mst_master_transcoder;

			if (intel_cpu_transcoders_need_modeset(state, BIT(master))) {
				new_crtc_state->uapi.mode_changed = true;
				new_crtc_state->update_pipe = false;
				new_crtc_state->update_m_n = false;
			}
		}

		if (is_trans_port_sync_mode(new_crtc_state)) {
			u8 trans = new_crtc_state->sync_mode_slaves_mask;

			if (new_crtc_state->master_transcoder != INVALID_TRANSCODER)
				trans |= BIT(new_crtc_state->master_transcoder);

			if (intel_cpu_transcoders_need_modeset(state, trans)) {
				new_crtc_state->uapi.mode_changed = true;
				new_crtc_state->update_pipe = false;
				new_crtc_state->update_m_n = false;
			}
		}

		if (new_crtc_state->bigjoiner_pipes) {
			if (intel_pipes_need_modeset(state, new_crtc_state->bigjoiner_pipes)) {
				new_crtc_state->uapi.mode_changed = true;
				new_crtc_state->update_pipe = false;
				new_crtc_state->update_m_n = false;
			}
		}
	}

	for_each_oldnew_intel_crtc_in_state(state, crtc, old_crtc_state,
					    new_crtc_state, i) {
		if (!intel_crtc_needs_modeset(new_crtc_state))
			continue;

		any_ms = true;

		intel_release_shared_dplls(state, crtc);
	}

	if (any_ms && !check_digital_port_conflicts(state)) {
		drm_dbg_kms(&dev_priv->drm,
			    "rejecting conflicting digital port configuration\n");
		ret = -EINVAL;
		goto fail;
	}

	ret = drm_dp_mst_atomic_check(&state->base);
	if (ret)
		goto fail;

	ret = intel_atomic_check_planes(state);
	if (ret)
		goto fail;

	ret = intel_compute_global_watermarks(state);
	if (ret)
		goto fail;

	ret = intel_bw_atomic_check(state);
	if (ret)
		goto fail;

	ret = intel_cdclk_atomic_check(state, &any_ms);
	if (ret)
		goto fail;

	if (intel_any_crtc_needs_modeset(state))
		any_ms = true;

	if (any_ms) {
		ret = intel_modeset_checks(state);
		if (ret)
			goto fail;

		ret = intel_modeset_calc_cdclk(state);
		if (ret)
			return ret;
	}

	ret = intel_pmdemand_atomic_check(state);
	if (ret)
		goto fail;

	ret = intel_atomic_check_crtcs(state);
	if (ret)
		goto fail;

	ret = intel_fbc_atomic_check(state);
	if (ret)
		goto fail;

	for_each_oldnew_intel_crtc_in_state(state, crtc, old_crtc_state,
					    new_crtc_state, i) {
		intel_color_assert_luts(new_crtc_state);

		ret = intel_async_flip_check_hw(state, crtc);
		if (ret)
			goto fail;

		/* Either full modeset or fastset (or neither), never both */
		drm_WARN_ON(&dev_priv->drm,
			    intel_crtc_needs_modeset(new_crtc_state) &&
			    intel_crtc_needs_fastset(new_crtc_state));

		if (!intel_crtc_needs_modeset(new_crtc_state) &&
		    !intel_crtc_needs_fastset(new_crtc_state))
			continue;

		intel_crtc_state_dump(new_crtc_state, state,
				      intel_crtc_needs_modeset(new_crtc_state) ?
				      "modeset" : "fastset");
	}

	return 0;

 fail:
	if (ret == -EDEADLK)
		return ret;

	/*
	 * FIXME would probably be nice to know which crtc specifically
	 * caused the failure, in cases where we can pinpoint it.
	 */
	for_each_oldnew_intel_crtc_in_state(state, crtc, old_crtc_state,
					    new_crtc_state, i)
		intel_crtc_state_dump(new_crtc_state, state, "failed");

	return ret;
}

static int intel_atomic_prepare_commit(struct intel_atomic_state *state)
{
	struct intel_crtc_state *crtc_state;
	struct intel_crtc *crtc;
	int i, ret;

	ret = drm_atomic_helper_prepare_planes(state->base.dev, &state->base);
	if (ret < 0)
		return ret;

	for_each_new_intel_crtc_in_state(state, crtc, crtc_state, i) {
		if (intel_crtc_needs_color_update(crtc_state))
			intel_color_prepare_commit(crtc_state);
	}

	return 0;
}

void intel_crtc_arm_fifo_underrun(struct intel_crtc *crtc,
				  struct intel_crtc_state *crtc_state)
{
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);

	if (DISPLAY_VER(dev_priv) != 2 || crtc_state->active_planes)
		intel_set_cpu_fifo_underrun_reporting(dev_priv, crtc->pipe, true);

	if (crtc_state->has_pch_encoder) {
		enum pipe pch_transcoder =
			intel_crtc_pch_transcoder(crtc);

		intel_set_pch_fifo_underrun_reporting(dev_priv, pch_transcoder, true);
	}
}

static void intel_pipe_fastset(const struct intel_crtc_state *old_crtc_state,
			       const struct intel_crtc_state *new_crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(new_crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);

	/*
	 * Update pipe size and adjust fitter if needed: the reason for this is
	 * that in compute_mode_changes we check the native mode (not the pfit
	 * mode) to see if we can flip rather than do a full mode set. In the
	 * fastboot case, we'll flip, but if we don't update the pipesrc and
	 * pfit state, we'll end up with a big fb scanned out into the wrong
	 * sized surface.
	 */
	intel_set_pipe_src_size(new_crtc_state);

	/* on skylake this is done by detaching scalers */
	if (DISPLAY_VER(dev_priv) >= 9) {
		if (new_crtc_state->pch_pfit.enabled)
			skl_pfit_enable(new_crtc_state);
	} else if (HAS_PCH_SPLIT(dev_priv)) {
		if (new_crtc_state->pch_pfit.enabled)
			ilk_pfit_enable(new_crtc_state);
		else if (old_crtc_state->pch_pfit.enabled)
			ilk_pfit_disable(old_crtc_state);
	}

	/*
	 * The register is supposedly single buffered so perhaps
	 * not 100% correct to do this here. But SKL+ calculate
	 * this based on the adjust pixel rate so pfit changes do
	 * affect it and so it must be updated for fastsets.
	 * HSW/BDW only really need this here for fastboot, after
	 * that the value should not change without a full modeset.
	 */
	if (DISPLAY_VER(dev_priv) >= 9 ||
	    IS_BROADWELL(dev_priv) || IS_HASWELL(dev_priv))
		hsw_set_linetime_wm(new_crtc_state);

	if (new_crtc_state->update_m_n)
		intel_cpu_transcoder_set_m1_n1(crtc, new_crtc_state->cpu_transcoder,
					       &new_crtc_state->dp_m_n);
}

static void commit_pipe_pre_planes(struct intel_atomic_state *state,
				   struct intel_crtc *crtc)
{
	struct drm_i915_private *dev_priv = to_i915(state->base.dev);
	const struct intel_crtc_state *old_crtc_state =
		intel_atomic_get_old_crtc_state(state, crtc);
	const struct intel_crtc_state *new_crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	bool modeset = intel_crtc_needs_modeset(new_crtc_state);

	/*
	 * During modesets pipe configuration was programmed as the
	 * CRTC was enabled.
	 */
	if (!modeset) {
		if (intel_crtc_needs_color_update(new_crtc_state))
			intel_color_commit_arm(new_crtc_state);

		if (DISPLAY_VER(dev_priv) >= 9 || IS_BROADWELL(dev_priv))
			bdw_set_pipe_misc(new_crtc_state);

		if (intel_crtc_needs_fastset(new_crtc_state))
			intel_pipe_fastset(old_crtc_state, new_crtc_state);
	}

	intel_psr2_program_trans_man_trk_ctl(new_crtc_state);

	intel_atomic_update_watermarks(state, crtc);
}

static void commit_pipe_post_planes(struct intel_atomic_state *state,
				    struct intel_crtc *crtc)
{
	struct drm_i915_private *dev_priv = to_i915(state->base.dev);
	const struct intel_crtc_state *old_crtc_state =
		intel_atomic_get_old_crtc_state(state, crtc);
	const struct intel_crtc_state *new_crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);

	/*
	 * Disable the scaler(s) after the plane(s) so that we don't
	 * get a catastrophic underrun even if the two operations
	 * end up happening in two different frames.
	 */
	if (DISPLAY_VER(dev_priv) >= 9 &&
	    !intel_crtc_needs_modeset(new_crtc_state))
		skl_detach_scalers(new_crtc_state);

	if (vrr_enabling(old_crtc_state, new_crtc_state))
		intel_vrr_enable(new_crtc_state);
}

static void intel_enable_crtc(struct intel_atomic_state *state,
			      struct intel_crtc *crtc)
{
	struct drm_i915_private *dev_priv = to_i915(state->base.dev);
	const struct intel_crtc_state *new_crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);

	if (!intel_crtc_needs_modeset(new_crtc_state))
		return;

	/* VRR will be enable later, if required */
	intel_crtc_update_active_timings(new_crtc_state, false);

	dev_priv->display.funcs.display->crtc_enable(state, crtc);

	if (intel_crtc_is_bigjoiner_slave(new_crtc_state))
		return;

	/* vblanks work again, re-enable pipe CRC. */
	intel_crtc_enable_pipe_crc(crtc);
}

static void intel_update_crtc(struct intel_atomic_state *state,
			      struct intel_crtc *crtc)
{
	struct drm_i915_private *i915 = to_i915(state->base.dev);
	const struct intel_crtc_state *old_crtc_state =
		intel_atomic_get_old_crtc_state(state, crtc);
	struct intel_crtc_state *new_crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	bool modeset = intel_crtc_needs_modeset(new_crtc_state);

	if (old_crtc_state->inherited ||
	    intel_crtc_needs_modeset(new_crtc_state)) {
		if (HAS_DPT(i915))
			intel_dpt_configure(crtc);
	}

	if (!modeset) {
		if (new_crtc_state->preload_luts &&
		    intel_crtc_needs_color_update(new_crtc_state))
			intel_color_load_luts(new_crtc_state);

		intel_pre_plane_update(state, crtc);

		if (intel_crtc_needs_fastset(new_crtc_state))
			intel_encoders_update_pipe(state, crtc);

		if (DISPLAY_VER(i915) >= 11 &&
		    intel_crtc_needs_fastset(new_crtc_state))
			icl_set_pipe_chicken(new_crtc_state);
	}

	intel_fbc_update(state, crtc);

	drm_WARN_ON(&i915->drm, !intel_display_power_is_enabled(i915, POWER_DOMAIN_DC_OFF));

	if (!modeset &&
	    intel_crtc_needs_color_update(new_crtc_state))
		intel_color_commit_noarm(new_crtc_state);

	intel_crtc_planes_update_noarm(state, crtc);

	/* Perform vblank evasion around commit operation */
	intel_pipe_update_start(state, crtc);

	commit_pipe_pre_planes(state, crtc);

	intel_crtc_planes_update_arm(state, crtc);

	commit_pipe_post_planes(state, crtc);

	intel_pipe_update_end(state, crtc);

	/*
	 * VRR/Seamless M/N update may need to update frame timings.
	 *
	 * FIXME Should be synchronized with the start of vblank somehow...
	 */
	if (vrr_enabling(old_crtc_state, new_crtc_state) || new_crtc_state->update_m_n)
		intel_crtc_update_active_timings(new_crtc_state,
						 new_crtc_state->vrr.enable);

	/*
	 * We usually enable FIFO underrun interrupts as part of the
	 * CRTC enable sequence during modesets.  But when we inherit a
	 * valid pipe configuration from the BIOS we need to take care
	 * of enabling them on the CRTC's first fastset.
	 */
	if (intel_crtc_needs_fastset(new_crtc_state) && !modeset &&
	    old_crtc_state->inherited)
		intel_crtc_arm_fifo_underrun(crtc, new_crtc_state);
}

static void intel_old_crtc_state_disables(struct intel_atomic_state *state,
					  struct intel_crtc_state *old_crtc_state,
					  struct intel_crtc_state *new_crtc_state,
					  struct intel_crtc *crtc)
{
	struct drm_i915_private *dev_priv = to_i915(state->base.dev);

	/*
	 * We need to disable pipe CRC before disabling the pipe,
	 * or we race against vblank off.
	 */
	intel_crtc_disable_pipe_crc(crtc);

	dev_priv->display.funcs.display->crtc_disable(state, crtc);
	crtc->active = false;
	intel_fbc_disable(crtc);

	if (!new_crtc_state->hw.active)
		intel_initial_watermarks(state, crtc);
}

static void intel_commit_modeset_disables(struct intel_atomic_state *state)
{
	struct intel_crtc_state *new_crtc_state, *old_crtc_state;
	struct intel_crtc *crtc;
	u32 handled = 0;
	int i;

	for_each_oldnew_intel_crtc_in_state(state, crtc, old_crtc_state,
					    new_crtc_state, i) {
		if (!intel_crtc_needs_modeset(new_crtc_state))
			continue;

		intel_pre_plane_update(state, crtc);

		if (!old_crtc_state->hw.active)
			continue;

		intel_crtc_disable_planes(state, crtc);
	}

	/* Only disable port sync and MST slaves */
	for_each_oldnew_intel_crtc_in_state(state, crtc, old_crtc_state,
					    new_crtc_state, i) {
		if (!intel_crtc_needs_modeset(new_crtc_state))
			continue;

		if (!old_crtc_state->hw.active)
			continue;

		/* In case of Transcoder port Sync master slave CRTCs can be
		 * assigned in any order and we need to make sure that
		 * slave CRTCs are disabled first and then master CRTC since
		 * Slave vblanks are masked till Master Vblanks.
		 */
		if (!is_trans_port_sync_slave(old_crtc_state) &&
		    !intel_dp_mst_is_slave_trans(old_crtc_state) &&
		    !intel_crtc_is_bigjoiner_slave(old_crtc_state))
			continue;

		intel_old_crtc_state_disables(state, old_crtc_state,
					      new_crtc_state, crtc);
		handled |= BIT(crtc->pipe);
	}

	/* Disable everything else left on */
	for_each_oldnew_intel_crtc_in_state(state, crtc, old_crtc_state,
					    new_crtc_state, i) {
		if (!intel_crtc_needs_modeset(new_crtc_state) ||
		    (handled & BIT(crtc->pipe)))
			continue;

		if (!old_crtc_state->hw.active)
			continue;

		intel_old_crtc_state_disables(state, old_crtc_state,
					      new_crtc_state, crtc);
	}
}

static void intel_commit_modeset_enables(struct intel_atomic_state *state)
{
	struct intel_crtc_state *new_crtc_state;
	struct intel_crtc *crtc;
	int i;

	for_each_new_intel_crtc_in_state(state, crtc, new_crtc_state, i) {
		if (!new_crtc_state->hw.active)
			continue;

		intel_enable_crtc(state, crtc);
		intel_update_crtc(state, crtc);
	}
}

static void skl_commit_modeset_enables(struct intel_atomic_state *state)
{
	struct drm_i915_private *dev_priv = to_i915(state->base.dev);
	struct intel_crtc *crtc;
	struct intel_crtc_state *old_crtc_state, *new_crtc_state;
	struct skl_ddb_entry entries[I915_MAX_PIPES] = {};
	u8 update_pipes = 0, modeset_pipes = 0;
	int i;

	for_each_oldnew_intel_crtc_in_state(state, crtc, old_crtc_state, new_crtc_state, i) {
		enum pipe pipe = crtc->pipe;

		if (!new_crtc_state->hw.active)
			continue;

		/* ignore allocations for crtc's that have been turned off. */
		if (!intel_crtc_needs_modeset(new_crtc_state)) {
			entries[pipe] = old_crtc_state->wm.skl.ddb;
			update_pipes |= BIT(pipe);
		} else {
			modeset_pipes |= BIT(pipe);
		}
	}

	/*
	 * Whenever the number of active pipes changes, we need to make sure we
	 * update the pipes in the right order so that their ddb allocations
	 * never overlap with each other between CRTC updates. Otherwise we'll
	 * cause pipe underruns and other bad stuff.
	 *
	 * So first lets enable all pipes that do not need a fullmodeset as
	 * those don't have any external dependency.
	 */
	while (update_pipes) {
		for_each_oldnew_intel_crtc_in_state(state, crtc, old_crtc_state,
						    new_crtc_state, i) {
			enum pipe pipe = crtc->pipe;

			if ((update_pipes & BIT(pipe)) == 0)
				continue;

			if (skl_ddb_allocation_overlaps(&new_crtc_state->wm.skl.ddb,
							entries, I915_MAX_PIPES, pipe))
				continue;

			entries[pipe] = new_crtc_state->wm.skl.ddb;
			update_pipes &= ~BIT(pipe);

			intel_update_crtc(state, crtc);

			/*
			 * If this is an already active pipe, it's DDB changed,
			 * and this isn't the last pipe that needs updating
			 * then we need to wait for a vblank to pass for the
			 * new ddb allocation to take effect.
			 */
			if (!skl_ddb_entry_equal(&new_crtc_state->wm.skl.ddb,
						 &old_crtc_state->wm.skl.ddb) &&
			    (update_pipes | modeset_pipes))
				intel_crtc_wait_for_next_vblank(crtc);
		}
	}

	update_pipes = modeset_pipes;

	/*
	 * Enable all pipes that needs a modeset and do not depends on other
	 * pipes
	 */
	for_each_new_intel_crtc_in_state(state, crtc, new_crtc_state, i) {
		enum pipe pipe = crtc->pipe;

		if ((modeset_pipes & BIT(pipe)) == 0)
			continue;

		if (intel_dp_mst_is_slave_trans(new_crtc_state) ||
		    is_trans_port_sync_master(new_crtc_state) ||
		    intel_crtc_is_bigjoiner_master(new_crtc_state))
			continue;

		modeset_pipes &= ~BIT(pipe);

		intel_enable_crtc(state, crtc);
	}

	/*
	 * Then we enable all remaining pipes that depend on other
	 * pipes: MST slaves and port sync masters, big joiner master
	 */
	for_each_new_intel_crtc_in_state(state, crtc, new_crtc_state, i) {
		enum pipe pipe = crtc->pipe;

		if ((modeset_pipes & BIT(pipe)) == 0)
			continue;

		modeset_pipes &= ~BIT(pipe);

		intel_enable_crtc(state, crtc);
	}

	/*
	 * Finally we do the plane updates/etc. for all pipes that got enabled.
	 */
	for_each_new_intel_crtc_in_state(state, crtc, new_crtc_state, i) {
		enum pipe pipe = crtc->pipe;

		if ((update_pipes & BIT(pipe)) == 0)
			continue;

		drm_WARN_ON(&dev_priv->drm, skl_ddb_allocation_overlaps(&new_crtc_state->wm.skl.ddb,
									entries, I915_MAX_PIPES, pipe));

		entries[pipe] = new_crtc_state->wm.skl.ddb;
		update_pipes &= ~BIT(pipe);

		intel_update_crtc(state, crtc);
	}

	drm_WARN_ON(&dev_priv->drm, modeset_pipes);
	drm_WARN_ON(&dev_priv->drm, update_pipes);
}

static void intel_atomic_helper_free_state(struct drm_i915_private *dev_priv)
{
	struct intel_atomic_state *state, *next;
	struct llist_node *freed;

	freed = llist_del_all(&dev_priv->display.atomic_helper.free_list);
	llist_for_each_entry_safe(state, next, freed, freed)
		drm_atomic_state_put(&state->base);
}

void intel_atomic_helper_free_state_worker(struct work_struct *work)
{
	struct drm_i915_private *dev_priv =
		container_of(work, typeof(*dev_priv), display.atomic_helper.free_work);

	intel_atomic_helper_free_state(dev_priv);
}

static int
intel_atomic_commit_fence_wake(struct i915_sw_fence_waiter *waiter,
    unsigned mode, int flags, void *not_a_cookie)
{
	struct intel_atomic_state *intel_state = waiter->private;
	struct drm_i915_private *dev_priv = to_i915(intel_state->base.dev);

<<<<<<< HEAD
	spin_lock(&dev_priv->atomic_commit_lock);
	DRM_SPIN_WAKEUP_ALL(&dev_priv->atomic_commit_wq,
	    &dev_priv->atomic_commit_lock);
	spin_unlock(&dev_priv->atomic_commit_lock);
=======
	init_wait_entry(&wait_fence, 0);
	init_wait_entry(&wait_reset, 0);
	for (;;) {
		prepare_to_wait(&intel_state->commit_ready.wait,
				&wait_fence, TASK_UNINTERRUPTIBLE);
		prepare_to_wait(bit_waitqueue(&to_gt(dev_priv)->reset.flags,
					      I915_RESET_MODESET),
				&wait_reset, TASK_UNINTERRUPTIBLE);
>>>>>>> vendor/linux-drm-v6.6.35

	return 0;
}

<<<<<<< HEAD
static void intel_atomic_commit_fence_wait(struct intel_atomic_state *intel_state)
{
	struct drm_i915_private *dev_priv = to_i915(intel_state->base.dev);
	struct i915_sw_fence_waiter waiter;
	int ret;

	waiter.flags = 0;
	waiter.func = intel_atomic_commit_fence_wake;
	waiter.private = intel_state;

	spin_lock(&intel_state->commit_ready.wait.lock);
	list_add_tail(&waiter.entry, &intel_state->commit_ready.wait.head);
	spin_unlock(&intel_state->commit_ready.wait.lock);

	spin_lock(&dev_priv->atomic_commit_lock);
	DRM_SPIN_WAIT_NOINTR_UNTIL(ret, &dev_priv->atomic_commit_wq,
	    &dev_priv->atomic_commit_lock,
	    (i915_sw_fence_done(&intel_state->commit_ready) ||
		test_bit(I915_RESET_MODESET, &dev_priv->gt.reset.flags)));
	spin_unlock(&dev_priv->atomic_commit_lock);

	spin_lock(&intel_state->commit_ready.wait.lock);
	list_del(&waiter.entry);
	spin_unlock(&intel_state->commit_ready.wait.lock);
=======
		if (i915_sw_fence_done(&intel_state->commit_ready) ||
		    test_bit(I915_RESET_MODESET, &to_gt(dev_priv)->reset.flags))
			break;

		schedule();
	}
	finish_wait(&intel_state->commit_ready.wait, &wait_fence);
	finish_wait(bit_waitqueue(&to_gt(dev_priv)->reset.flags,
				  I915_RESET_MODESET),
		    &wait_reset);
>>>>>>> vendor/linux-drm-v6.6.35
}

static void intel_atomic_cleanup_work(struct work_struct *work)
{
	struct intel_atomic_state *state =
		container_of(work, struct intel_atomic_state, base.commit_work);
	struct drm_i915_private *i915 = to_i915(state->base.dev);
	struct intel_crtc_state *old_crtc_state;
	struct intel_crtc *crtc;
	int i;

	for_each_old_intel_crtc_in_state(state, crtc, old_crtc_state, i)
		intel_color_cleanup_commit(old_crtc_state);

	drm_atomic_helper_cleanup_planes(&i915->drm, &state->base);
	drm_atomic_helper_commit_cleanup_done(&state->base);
	drm_atomic_state_put(&state->base);

	intel_atomic_helper_free_state(i915);
}

static void intel_atomic_prepare_plane_clear_colors(struct intel_atomic_state *state)
{
	struct drm_i915_private *i915 = to_i915(state->base.dev);
	struct intel_plane *plane;
	struct intel_plane_state *plane_state;
	int i;

	for_each_new_intel_plane_in_state(state, plane, plane_state, i) {
		struct drm_framebuffer *fb = plane_state->hw.fb;
		int cc_plane;
		int ret;

		if (!fb)
			continue;

		cc_plane = intel_fb_rc_ccs_cc_plane(fb);
		if (cc_plane < 0)
			continue;

		/*
		 * The layout of the fast clear color value expected by HW
		 * (the DRM ABI requiring this value to be located in fb at
		 * offset 0 of cc plane, plane #2 previous generations or
		 * plane #1 for flat ccs):
		 * - 4 x 4 bytes per-channel value
		 *   (in surface type specific float/int format provided by the fb user)
		 * - 8 bytes native color value used by the display
		 *   (converted/written by GPU during a fast clear operation using the
		 *    above per-channel values)
		 *
		 * The commit's FB prepare hook already ensured that FB obj is pinned and the
		 * caller made sure that the object is synced wrt. the related color clear value
		 * GPU write on it.
		 */
		ret = i915_gem_object_read_from_page(intel_fb_obj(fb),
						     fb->offsets[cc_plane] + 16,
						     &plane_state->ccval,
						     sizeof(plane_state->ccval));
		/* The above could only fail if the FB obj has an unexpected backing store type. */
		drm_WARN_ON(&i915->drm, ret);
	}
}

static void intel_atomic_commit_tail(struct intel_atomic_state *state)
{
	struct drm_device *dev = state->base.dev;
	struct drm_i915_private *dev_priv = to_i915(dev);
	struct intel_crtc_state *new_crtc_state, *old_crtc_state;
	struct intel_crtc *crtc;
	struct intel_power_domain_mask put_domains[I915_MAX_PIPES] = {};
	intel_wakeref_t wakeref = 0;
	int i;

	intel_atomic_commit_fence_wait(state);

	drm_atomic_helper_wait_for_dependencies(&state->base);
	drm_dp_mst_atomic_wait_for_dependencies(&state->base);

	/*
	 * During full modesets we write a lot of registers, wait
	 * for PLLs, etc. Doing that while DC states are enabled
	 * is not a good idea.
	 *
	 * During fastsets and other updates we also need to
	 * disable DC states due to the following scenario:
	 * 1. DC5 exit and PSR exit happen
	 * 2. Some or all _noarm() registers are written
	 * 3. Due to some long delay PSR is re-entered
	 * 4. DC5 entry -> DMC saves the already written new
	 *    _noarm() registers and the old not yet written
	 *    _arm() registers
	 * 5. DC5 exit -> DMC restores a mixture of old and
	 *    new register values and arms the update
	 * 6. PSR exit -> hardware latches a mixture of old and
	 *    new register values -> corrupted frame, or worse
	 * 7. New _arm() registers are finally written
	 * 8. Hardware finally latches a complete set of new
	 *    register values, and subsequent frames will be OK again
	 *
	 * Also note that due to the pipe CSC hardware issues on
	 * SKL/GLK DC states must remain off until the pipe CSC
	 * state readout has happened. Otherwise we risk corrupting
	 * the CSC latched register values with the readout (see
	 * skl_read_csc() and skl_color_commit_noarm()).
	 */
	wakeref = intel_display_power_get(dev_priv, POWER_DOMAIN_DC_OFF);

	intel_atomic_prepare_plane_clear_colors(state);

	for_each_oldnew_intel_crtc_in_state(state, crtc, old_crtc_state,
					    new_crtc_state, i) {
		if (intel_crtc_needs_modeset(new_crtc_state) ||
		    intel_crtc_needs_fastset(new_crtc_state))
			intel_modeset_get_crtc_power_domains(new_crtc_state, &put_domains[crtc->pipe]);
	}

	intel_commit_modeset_disables(state);

	/* FIXME: Eventually get rid of our crtc->config pointer */
	for_each_new_intel_crtc_in_state(state, crtc, new_crtc_state, i)
		crtc->config = new_crtc_state;

	/*
	 * In XE_LPD+ Pmdemand combines many parameters such as voltage index,
	 * plls, cdclk frequency, QGV point selection parameter etc. Voltage
	 * index, cdclk/ddiclk frequencies are supposed to be configured before
	 * the cdclk config is set.
	 */
	intel_pmdemand_pre_plane_update(state);

	if (state->modeset) {
		drm_atomic_helper_update_legacy_modeset_state(dev, &state->base);

		intel_set_cdclk_pre_plane_update(state);

		intel_modeset_verify_disabled(dev_priv, state);
	}

	intel_sagv_pre_plane_update(state);

	/* Complete the events for pipes that have now been disabled */
	for_each_new_intel_crtc_in_state(state, crtc, new_crtc_state, i) {
		bool modeset = intel_crtc_needs_modeset(new_crtc_state);

		/* Complete events for now disable pipes here. */
		if (modeset && !new_crtc_state->hw.active && new_crtc_state->uapi.event) {
			spin_lock_irq(&dev->event_lock);
			drm_crtc_send_vblank_event(&crtc->base,
						   new_crtc_state->uapi.event);
			spin_unlock_irq(&dev->event_lock);

			new_crtc_state->uapi.event = NULL;
		}
	}

	intel_encoders_update_prepare(state);

	intel_dbuf_pre_plane_update(state);
	intel_mbus_dbox_update(state);

	for_each_new_intel_crtc_in_state(state, crtc, new_crtc_state, i) {
		if (new_crtc_state->do_async_flip)
			intel_crtc_enable_flip_done(state, crtc);
	}

	/* Now enable the clocks, plane, pipe, and connectors that we set up. */
	dev_priv->display.funcs.display->commit_modeset_enables(state);

	if (state->modeset)
		intel_set_cdclk_post_plane_update(state);

	intel_wait_for_vblank_workers(state);

	/* FIXME: We should call drm_atomic_helper_commit_hw_done() here
	 * already, but still need the state for the delayed optimization. To
	 * fix this:
	 * - wrap the optimization/post_plane_update stuff into a per-crtc work.
	 * - schedule that vblank worker _before_ calling hw_done
	 * - at the start of commit_tail, cancel it _synchrously
	 * - switch over to the vblank wait helper in the core after that since
	 *   we don't need out special handling any more.
	 */
	drm_atomic_helper_wait_for_flip_done(dev, &state->base);

	for_each_new_intel_crtc_in_state(state, crtc, new_crtc_state, i) {
		if (new_crtc_state->do_async_flip)
			intel_crtc_disable_flip_done(state, crtc);
	}

	/*
	 * Now that the vblank has passed, we can go ahead and program the
	 * optimal watermarks on platforms that need two-step watermark
	 * programming.
	 *
	 * TODO: Move this (and other cleanup) to an async worker eventually.
	 */
	for_each_oldnew_intel_crtc_in_state(state, crtc, old_crtc_state,
					    new_crtc_state, i) {
		/*
		 * Gen2 reports pipe underruns whenever all planes are disabled.
		 * So re-enable underrun reporting after some planes get enabled.
		 *
		 * We do this before .optimize_watermarks() so that we have a
		 * chance of catching underruns with the intermediate watermarks
		 * vs. the new plane configuration.
		 */
		if (DISPLAY_VER(dev_priv) == 2 && planes_enabling(old_crtc_state, new_crtc_state))
			intel_set_cpu_fifo_underrun_reporting(dev_priv, crtc->pipe, true);

		intel_optimize_watermarks(state, crtc);
	}

	intel_dbuf_post_plane_update(state);
	intel_psr_post_plane_update(state);

	for_each_oldnew_intel_crtc_in_state(state, crtc, old_crtc_state, new_crtc_state, i) {
		intel_post_plane_update(state, crtc);

		intel_modeset_put_crtc_power_domains(crtc, &put_domains[crtc->pipe]);

		intel_modeset_verify_crtc(crtc, state, old_crtc_state, new_crtc_state);

		/* Must be done after gamma readout due to HSW split gamma vs. IPS w/a */
		hsw_ips_post_update(state, crtc);

		/*
		 * Activate DRRS after state readout to avoid
		 * dp_m_n vs. dp_m2_n2 confusion on BDW+.
		 */
		intel_drrs_activate(new_crtc_state);

		/*
		 * DSB cleanup is done in cleanup_work aligning with framebuffer
		 * cleanup. So copy and reset the dsb structure to sync with
		 * commit_done and later do dsb cleanup in cleanup_work.
		 *
		 * FIXME get rid of this funny new->old swapping
		 */
		old_crtc_state->dsb = fetch_and_zero(&new_crtc_state->dsb);
	}

	/* Underruns don't always raise interrupts, so check manually */
	intel_check_cpu_fifo_underruns(dev_priv);
	intel_check_pch_fifo_underruns(dev_priv);

	if (state->modeset)
		intel_verify_planes(state);

	intel_sagv_post_plane_update(state);
	intel_pmdemand_post_plane_update(state);

	drm_atomic_helper_commit_hw_done(&state->base);

	if (state->modeset) {
		/* As one of the primary mmio accessors, KMS has a high
		 * likelihood of triggering bugs in unclaimed access. After we
		 * finish modesetting, see if an error has been flagged, and if
		 * so enable debugging for the next modeset - and hope we catch
		 * the culprit.
		 */
		intel_uncore_arm_unclaimed_mmio_detection(&dev_priv->uncore);
	}
	/*
	 * Delay re-enabling DC states by 17 ms to avoid the off->on->off
	 * toggling overhead at and above 60 FPS.
	 */
	intel_display_power_put_async_delay(dev_priv, POWER_DOMAIN_DC_OFF, wakeref, 17);
	intel_runtime_pm_put(&dev_priv->runtime_pm, state->wakeref);

	/*
	 * Defer the cleanup of the old state to a separate worker to not
	 * impede the current task (userspace for blocking modesets) that
	 * are executed inline. For out-of-line asynchronous modesets/flips,
	 * deferring to a new worker seems overkill, but we would place a
	 * schedule point (cond_resched()) here anyway to keep latencies
	 * down.
	 */
	INIT_WORK(&state->base.commit_work, intel_atomic_cleanup_work);
	queue_work(system_highpri_wq, &state->base.commit_work);
}

static void intel_atomic_commit_work(struct work_struct *work)
{
	struct intel_atomic_state *state =
		container_of(work, struct intel_atomic_state, base.commit_work);

	intel_atomic_commit_tail(state);
}

<<<<<<< HEAD
int __i915_sw_fence_call
=======
static int
>>>>>>> vendor/linux-drm-v6.6.35
intel_atomic_commit_ready(struct i915_sw_fence *fence,
			  enum i915_sw_fence_notify notify)
{
	struct intel_atomic_state *state =
		container_of(fence, struct intel_atomic_state, commit_ready);

	switch (notify) {
	case FENCE_COMPLETE:
		/* we do blocking waits in the worker, nothing to do here */
		break;
	case FENCE_FREE:
		{
			struct drm_i915_private *i915 = to_i915(state->base.dev);
			struct intel_atomic_helper *helper =
				&i915->display.atomic_helper;

			if (llist_add(&state->freed, &helper->free_list))
				queue_work(i915->unordered_wq, &helper->free_work);
			break;
		}
	}

	return NOTIFY_DONE;
}

static void intel_atomic_track_fbs(struct intel_atomic_state *state)
{
	struct intel_plane_state *old_plane_state, *new_plane_state;
	struct intel_plane *plane;
	int i;

	for_each_oldnew_intel_plane_in_state(state, plane, old_plane_state,
					     new_plane_state, i)
		intel_frontbuffer_track(to_intel_frontbuffer(old_plane_state->hw.fb),
					to_intel_frontbuffer(new_plane_state->hw.fb),
					plane->frontbuffer_bit);
}

int intel_atomic_commit(struct drm_device *dev, struct drm_atomic_state *_state,
			bool nonblock)
{
	struct intel_atomic_state *state = to_intel_atomic_state(_state);
	struct drm_i915_private *dev_priv = to_i915(dev);
	int ret = 0;

	state->wakeref = intel_runtime_pm_get(&dev_priv->runtime_pm);

	drm_atomic_state_get(&state->base);
	i915_sw_fence_reinit(&state->commit_ready);

	/*
	 * The intel_legacy_cursor_update() fast path takes care
	 * of avoiding the vblank waits for simple cursor
	 * movement and flips. For cursor on/off and size changes,
	 * we want to perform the vblank waits so that watermark
	 * updates happen during the correct frames. Gen9+ have
	 * double buffered watermarks and so shouldn't need this.
	 *
	 * Unset state->legacy_cursor_update before the call to
	 * drm_atomic_helper_setup_commit() because otherwise
	 * drm_atomic_helper_wait_for_flip_done() is a noop and
	 * we get FIFO underruns because we didn't wait
	 * for vblank.
	 *
	 * FIXME doing watermarks and fb cleanup from a vblank worker
	 * (assuming we had any) would solve these problems.
	 */
	if (DISPLAY_VER(dev_priv) < 9 && state->base.legacy_cursor_update) {
		struct intel_crtc_state *new_crtc_state;
		struct intel_crtc *crtc;
		int i;

		for_each_new_intel_crtc_in_state(state, crtc, new_crtc_state, i)
			if (new_crtc_state->wm.need_postvbl_update ||
			    new_crtc_state->update_wm_post)
				state->base.legacy_cursor_update = false;
	}

	ret = intel_atomic_prepare_commit(state);
	if (ret) {
		drm_dbg_atomic(&dev_priv->drm,
			       "Preparing state failed with %i\n", ret);
		i915_sw_fence_commit(&state->commit_ready);
		intel_runtime_pm_put(&dev_priv->runtime_pm, state->wakeref);
		return ret;
	}

	ret = drm_atomic_helper_setup_commit(&state->base, nonblock);
	if (!ret)
		ret = drm_atomic_helper_swap_state(&state->base, true);
	if (!ret)
		intel_atomic_swap_global_state(state);

	if (ret) {
		struct intel_crtc_state *new_crtc_state;
		struct intel_crtc *crtc;
		int i;

		i915_sw_fence_commit(&state->commit_ready);

		for_each_new_intel_crtc_in_state(state, crtc, new_crtc_state, i)
			intel_color_cleanup_commit(new_crtc_state);

		drm_atomic_helper_unprepare_planes(dev, &state->base);
		intel_runtime_pm_put(&dev_priv->runtime_pm, state->wakeref);
		return ret;
	}
	intel_shared_dpll_swap_state(state);
	intel_atomic_track_fbs(state);

	drm_atomic_state_get(&state->base);
	INIT_WORK(&state->base.commit_work, intel_atomic_commit_work);

	i915_sw_fence_commit(&state->commit_ready);
	if (nonblock && state->modeset) {
		queue_work(dev_priv->display.wq.modeset, &state->base.commit_work);
	} else if (nonblock) {
		queue_work(dev_priv->display.wq.flip, &state->base.commit_work);
	} else {
		if (state->modeset)
			flush_workqueue(dev_priv->display.wq.modeset);
		intel_atomic_commit_tail(state);
	}

	return 0;
}

<<<<<<< HEAD
#ifdef __NetBSD__

/* XXX */

#else

struct wait_rps_boost {
	struct wait_queue_entry wait;

	struct drm_crtc *crtc;
	struct i915_request *request;
};

static int do_rps_boost(struct wait_queue_entry *_wait,
			unsigned mode, int sync, void *key)
{
	struct wait_rps_boost *wait = container_of(_wait, typeof(*wait), wait);
	struct i915_request *rq = wait->request;

	/*
	 * If we missed the vblank, but the request is already running it
	 * is reasonable to assume that it will complete before the next
	 * vblank without our intervention, so leave RPS alone.
	 */
	if (!i915_request_started(rq))
		intel_rps_boost(rq);
	i915_request_put(rq);

	drm_crtc_vblank_put(wait->crtc);

	list_del(&wait->wait.entry);
	kfree(wait);
	return 1;
}

#endif

static void add_rps_boost_after_vblank(struct drm_crtc *crtc,
				       struct dma_fence *fence)
{
#ifndef __NetBSD__		/* XXX i915 rps boost */
	struct wait_rps_boost *wait;

	if (!dma_fence_is_i915(fence))
		return;

	if (INTEL_GEN(to_i915(crtc->dev)) < 6)
		return;

	if (drm_crtc_vblank_get(crtc))
		return;

	wait = kmalloc(sizeof(*wait), GFP_KERNEL);
	if (!wait) {
		drm_crtc_vblank_put(crtc);
		return;
	}

	wait->request = to_request(dma_fence_get(fence));
	wait->crtc = crtc;

	wait->wait.func = do_rps_boost;
	wait->wait.flags = 0;

	add_wait_queue(drm_crtc_vblank_waitqueue(crtc), &wait->wait);
#endif
}

static int intel_plane_pin_fb(struct intel_plane_state *plane_state)
{
	struct intel_plane *plane = to_intel_plane(plane_state->uapi.plane);
	struct drm_i915_private *dev_priv = to_i915(plane->base.dev);
	struct drm_framebuffer *fb = plane_state->hw.fb;
	struct i915_vma *vma;

	if (plane->id == PLANE_CURSOR &&
	    INTEL_INFO(dev_priv)->display.cursor_needs_physical) {
		struct drm_i915_gem_object *obj = intel_fb_obj(fb);
		const int align = intel_cursor_alignment(dev_priv);
		int err;

		err = i915_gem_object_attach_phys(obj, align);
		if (err)
			return err;
	}

	vma = intel_pin_and_fence_fb_obj(fb,
					 &plane_state->view,
					 intel_plane_uses_fence(plane_state),
					 &plane_state->flags);
	if (IS_ERR(vma))
		return PTR_ERR(vma);

	plane_state->vma = vma;

	return 0;
}

static void intel_plane_unpin_fb(struct intel_plane_state *old_plane_state)
{
	struct i915_vma *vma;

	vma = fetch_and_zero(&old_plane_state->vma);
	if (vma)
		intel_unpin_fb_vma(vma, old_plane_state->flags);
}

static void fb_obj_bump_render_priority(struct drm_i915_gem_object *obj)
{
	struct i915_sched_attr attr = {
		.priority = I915_USER_PRIORITY(I915_PRIORITY_DISPLAY),
	};

	i915_gem_object_wait_priority(obj, 0, &attr);
}

/**
 * intel_prepare_plane_fb - Prepare fb for usage on plane
 * @plane: drm plane to prepare for
 * @_new_plane_state: the plane state being prepared
 *
 * Prepares a framebuffer for usage on a display plane.  Generally this
 * involves pinning the underlying object and updating the frontbuffer tracking
 * bits.  Some older platforms need special physical address handling for
 * cursor planes.
 *
 * Returns 0 on success, negative error code on failure.
 */
int
intel_prepare_plane_fb(struct drm_plane *plane,
		       struct drm_plane_state *_new_plane_state)
{
	struct intel_plane_state *new_plane_state =
		to_intel_plane_state(_new_plane_state);
	struct intel_atomic_state *intel_state =
		to_intel_atomic_state(new_plane_state->uapi.state);
	struct drm_i915_private *dev_priv = to_i915(plane->dev);
	struct drm_framebuffer *fb = new_plane_state->hw.fb;
	struct drm_i915_gem_object *obj = intel_fb_obj(fb);
	struct drm_i915_gem_object *old_obj = intel_fb_obj(plane->state->fb);
	int ret;

	if (old_obj) {
		struct intel_crtc_state *crtc_state =
			intel_atomic_get_new_crtc_state(intel_state,
							to_intel_crtc(plane->state->crtc));

		/* Big Hammer, we also need to ensure that any pending
		 * MI_WAIT_FOR_EVENT inside a user batch buffer on the
		 * current scanout is retired before unpinning the old
		 * framebuffer. Note that we rely on userspace rendering
		 * into the buffer attached to the pipe they are waiting
		 * on. If not, userspace generates a GPU hang with IPEHR
		 * point to the MI_WAIT_FOR_EVENT.
		 *
		 * This should only fail upon a hung GPU, in which case we
		 * can safely continue.
		 */
		if (needs_modeset(crtc_state)) {
			ret = i915_sw_fence_await_reservation(&intel_state->commit_ready,
							      old_obj->base.resv, NULL,
							      false, 0,
							      GFP_KERNEL);
			if (ret < 0)
				return ret;
		}
	}

	if (new_plane_state->uapi.fence) { /* explicit fencing */
		ret = i915_sw_fence_await_dma_fence(&intel_state->commit_ready,
						    new_plane_state->uapi.fence,
						    I915_FENCE_TIMEOUT,
						    GFP_KERNEL);
		if (ret < 0)
			return ret;
	}

	if (!obj)
		return 0;

	ret = i915_gem_object_pin_pages(obj);
	if (ret)
		return ret;

	ret = intel_plane_pin_fb(new_plane_state);

	i915_gem_object_unpin_pages(obj);
	if (ret)
		return ret;

	fb_obj_bump_render_priority(obj);
	i915_gem_object_flush_frontbuffer(obj, ORIGIN_DIRTYFB);

	if (!new_plane_state->uapi.fence) { /* implicit fencing */
		struct dma_fence *fence;

		ret = i915_sw_fence_await_reservation(&intel_state->commit_ready,
						      obj->base.resv, NULL,
						      false, I915_FENCE_TIMEOUT,
						      GFP_KERNEL);
		if (ret < 0)
			return ret;

		fence = dma_resv_get_excl_rcu(obj->base.resv);
		if (fence) {
			add_rps_boost_after_vblank(new_plane_state->hw.crtc,
						   fence);
			dma_fence_put(fence);
		}
	} else {
		add_rps_boost_after_vblank(new_plane_state->hw.crtc,
					   new_plane_state->uapi.fence);
	}

	/*
	 * We declare pageflips to be interactive and so merit a small bias
	 * towards upclocking to deliver the frame on time. By only changing
	 * the RPS thresholds to sample more regularly and aim for higher
	 * clocks we can hopefully deliver low power workloads (like kodi)
	 * that are not quite steady state without resorting to forcing
	 * maximum clocks following a vblank miss (see do_rps_boost()).
	 */
	if (!intel_state->rps_interactive) {
		intel_rps_mark_interactive(&dev_priv->gt.rps, true);
		intel_state->rps_interactive = true;
	}

	return 0;
}

/**
 * intel_cleanup_plane_fb - Cleans up an fb after plane use
 * @plane: drm plane to clean up for
 * @_old_plane_state: the state from the previous modeset
 *
 * Cleans up a framebuffer that has just been removed from a plane.
 */
void
intel_cleanup_plane_fb(struct drm_plane *plane,
		       struct drm_plane_state *_old_plane_state)
{
	struct intel_plane_state *old_plane_state =
		to_intel_plane_state(_old_plane_state);
	struct intel_atomic_state *intel_state =
		to_intel_atomic_state(old_plane_state->uapi.state);
	struct drm_i915_private *dev_priv = to_i915(plane->dev);

	if (intel_state->rps_interactive) {
		intel_rps_mark_interactive(&dev_priv->gt.rps, false);
		intel_state->rps_interactive = false;
	}

	/* Should only be called after a successful intel_prepare_plane_fb()! */
	intel_plane_unpin_fb(old_plane_state);
}

=======
>>>>>>> vendor/linux-drm-v6.6.35
/**
 * intel_plane_destroy - destroy a plane
 * @plane: plane to destroy
 *
 * Common destruction function for all types of planes (primary, cursor,
 * sprite).
 */
void intel_plane_destroy(struct drm_plane *plane)
{
	drm_plane_cleanup(plane);
	kfree(to_intel_plane(plane));
}

int intel_get_pipe_from_crtc_id_ioctl(struct drm_device *dev, void *data,
				      struct drm_file *file)
{
	struct drm_i915_get_pipe_from_crtc_id *pipe_from_crtc_id = data;
	struct drm_crtc *drmmode_crtc;
	struct intel_crtc *crtc;

	drmmode_crtc = drm_crtc_find(dev, file, pipe_from_crtc_id->crtc_id);
	if (!drmmode_crtc)
		return -ENOENT;

	crtc = to_intel_crtc(drmmode_crtc);
	pipe_from_crtc_id->pipe = crtc->pipe;

	return 0;
}

static u32 intel_encoder_possible_clones(struct intel_encoder *encoder)
{
	struct drm_device *dev = encoder->base.dev;
	struct intel_encoder *source_encoder;
	u32 possible_clones = 0;

	for_each_intel_encoder(dev, source_encoder) {
		if (encoders_cloneable(encoder, source_encoder))
			possible_clones |= drm_encoder_mask(&source_encoder->base);
	}

	return possible_clones;
}

static u32 intel_encoder_possible_crtcs(struct intel_encoder *encoder)
{
	struct drm_device *dev = encoder->base.dev;
	struct intel_crtc *crtc;
	u32 possible_crtcs = 0;

	for_each_intel_crtc_in_pipe_mask(dev, crtc, encoder->pipe_mask)
		possible_crtcs |= drm_crtc_mask(&crtc->base);

	return possible_crtcs;
}

static bool ilk_has_edp_a(struct drm_i915_private *dev_priv)
{
	if (!IS_MOBILE(dev_priv))
		return false;

	if ((intel_de_read(dev_priv, DP_A) & DP_DETECTED) == 0)
		return false;

	if (IS_IRONLAKE(dev_priv) && (intel_de_read(dev_priv, FUSE_STRAP) & ILK_eDP_A_DISABLE))
		return false;

	return true;
}

static bool intel_ddi_crt_present(struct drm_i915_private *dev_priv)
{
	if (DISPLAY_VER(dev_priv) >= 9)
		return false;

	if (IS_HASWELL_ULT(dev_priv) || IS_BROADWELL_ULT(dev_priv))
		return false;

	if (HAS_PCH_LPT_H(dev_priv) &&
	    intel_de_read(dev_priv, SFUSE_STRAP) & SFUSE_STRAP_CRT_DISABLED)
		return false;

	/* DDI E can't be used if DDI A requires 4 lanes */
	if (intel_de_read(dev_priv, DDI_BUF_CTL(PORT_A)) & DDI_A_4_LANES)
		return false;

	if (!dev_priv->display.vbt.int_crt_support)
		return false;

	return true;
}

bool assert_port_valid(struct drm_i915_private *i915, enum port port)
{
	return !drm_WARN(&i915->drm, !(DISPLAY_RUNTIME_INFO(i915)->port_mask & BIT(port)),
			 "Platform does not support port %c\n", port_name(port));
}

void intel_setup_outputs(struct drm_i915_private *dev_priv)
{
	struct intel_encoder *encoder;
	bool dpd_is_edp = false;

	intel_pps_unlock_regs_wa(dev_priv);

	if (!HAS_DISPLAY(dev_priv))
		return;

	if (HAS_DDI(dev_priv)) {
		if (intel_ddi_crt_present(dev_priv))
			intel_crt_init(dev_priv);

		intel_bios_for_each_encoder(dev_priv, intel_ddi_init);

		if (IS_GEMINILAKE(dev_priv) || IS_BROXTON(dev_priv))
			vlv_dsi_init(dev_priv);
	} else if (HAS_PCH_SPLIT(dev_priv)) {
		int found;

		/*
		 * intel_edp_init_connector() depends on this completing first,
		 * to prevent the registration of both eDP and LVDS and the
		 * incorrect sharing of the PPS.
		 */
		intel_lvds_init(dev_priv);
		intel_crt_init(dev_priv);

		dpd_is_edp = intel_dp_is_port_edp(dev_priv, PORT_D);

		if (ilk_has_edp_a(dev_priv))
			g4x_dp_init(dev_priv, DP_A, PORT_A);

		if (intel_de_read(dev_priv, PCH_HDMIB) & SDVO_DETECTED) {
			/* PCH SDVOB multiplex with HDMIB */
			found = intel_sdvo_init(dev_priv, PCH_SDVOB, PORT_B);
			if (!found)
				g4x_hdmi_init(dev_priv, PCH_HDMIB, PORT_B);
			if (!found && (intel_de_read(dev_priv, PCH_DP_B) & DP_DETECTED))
				g4x_dp_init(dev_priv, PCH_DP_B, PORT_B);
		}

		if (intel_de_read(dev_priv, PCH_HDMIC) & SDVO_DETECTED)
			g4x_hdmi_init(dev_priv, PCH_HDMIC, PORT_C);

		if (!dpd_is_edp && intel_de_read(dev_priv, PCH_HDMID) & SDVO_DETECTED)
			g4x_hdmi_init(dev_priv, PCH_HDMID, PORT_D);

		if (intel_de_read(dev_priv, PCH_DP_C) & DP_DETECTED)
			g4x_dp_init(dev_priv, PCH_DP_C, PORT_C);

		if (intel_de_read(dev_priv, PCH_DP_D) & DP_DETECTED)
			g4x_dp_init(dev_priv, PCH_DP_D, PORT_D);
	} else if (IS_VALLEYVIEW(dev_priv) || IS_CHERRYVIEW(dev_priv)) {
		bool has_edp, has_port;

		if (IS_VALLEYVIEW(dev_priv) && dev_priv->display.vbt.int_crt_support)
			intel_crt_init(dev_priv);

		/*
		 * The DP_DETECTED bit is the latched state of the DDC
		 * SDA pin at boot. However since eDP doesn't require DDC
		 * (no way to plug in a DP->HDMI dongle) the DDC pins for
		 * eDP ports may have been muxed to an alternate function.
		 * Thus we can't rely on the DP_DETECTED bit alone to detect
		 * eDP ports. Consult the VBT as well as DP_DETECTED to
		 * detect eDP ports.
		 *
		 * Sadly the straps seem to be missing sometimes even for HDMI
		 * ports (eg. on Voyo V3 - CHT x7-Z8700), so check both strap
		 * and VBT for the presence of the port. Additionally we can't
		 * trust the port type the VBT declares as we've seen at least
		 * HDMI ports that the VBT claim are DP or eDP.
		 */
		has_edp = intel_dp_is_port_edp(dev_priv, PORT_B);
		has_port = intel_bios_is_port_present(dev_priv, PORT_B);
		if (intel_de_read(dev_priv, VLV_DP_B) & DP_DETECTED || has_port)
			has_edp &= g4x_dp_init(dev_priv, VLV_DP_B, PORT_B);
		if ((intel_de_read(dev_priv, VLV_HDMIB) & SDVO_DETECTED || has_port) && !has_edp)
			g4x_hdmi_init(dev_priv, VLV_HDMIB, PORT_B);

		has_edp = intel_dp_is_port_edp(dev_priv, PORT_C);
		has_port = intel_bios_is_port_present(dev_priv, PORT_C);
		if (intel_de_read(dev_priv, VLV_DP_C) & DP_DETECTED || has_port)
			has_edp &= g4x_dp_init(dev_priv, VLV_DP_C, PORT_C);
		if ((intel_de_read(dev_priv, VLV_HDMIC) & SDVO_DETECTED || has_port) && !has_edp)
			g4x_hdmi_init(dev_priv, VLV_HDMIC, PORT_C);

		if (IS_CHERRYVIEW(dev_priv)) {
			/*
			 * eDP not supported on port D,
			 * so no need to worry about it
			 */
			has_port = intel_bios_is_port_present(dev_priv, PORT_D);
			if (intel_de_read(dev_priv, CHV_DP_D) & DP_DETECTED || has_port)
				g4x_dp_init(dev_priv, CHV_DP_D, PORT_D);
			if (intel_de_read(dev_priv, CHV_HDMID) & SDVO_DETECTED || has_port)
				g4x_hdmi_init(dev_priv, CHV_HDMID, PORT_D);
		}

		vlv_dsi_init(dev_priv);
	} else if (IS_PINEVIEW(dev_priv)) {
		intel_lvds_init(dev_priv);
		intel_crt_init(dev_priv);
	} else if (IS_DISPLAY_VER(dev_priv, 3, 4)) {
		bool found = false;

		if (IS_MOBILE(dev_priv))
			intel_lvds_init(dev_priv);

		intel_crt_init(dev_priv);

		if (intel_de_read(dev_priv, GEN3_SDVOB) & SDVO_DETECTED) {
			drm_dbg_kms(&dev_priv->drm, "probing SDVOB\n");
			found = intel_sdvo_init(dev_priv, GEN3_SDVOB, PORT_B);
			if (!found && IS_G4X(dev_priv)) {
				drm_dbg_kms(&dev_priv->drm,
					    "probing HDMI on SDVOB\n");
				g4x_hdmi_init(dev_priv, GEN4_HDMIB, PORT_B);
			}

			if (!found && IS_G4X(dev_priv))
				g4x_dp_init(dev_priv, DP_B, PORT_B);
		}

		/* Before G4X SDVOC doesn't have its own detect register */

		if (intel_de_read(dev_priv, GEN3_SDVOB) & SDVO_DETECTED) {
			drm_dbg_kms(&dev_priv->drm, "probing SDVOC\n");
			found = intel_sdvo_init(dev_priv, GEN3_SDVOC, PORT_C);
		}

		if (!found && (intel_de_read(dev_priv, GEN3_SDVOC) & SDVO_DETECTED)) {

			if (IS_G4X(dev_priv)) {
				drm_dbg_kms(&dev_priv->drm,
					    "probing HDMI on SDVOC\n");
				g4x_hdmi_init(dev_priv, GEN4_HDMIC, PORT_C);
			}
			if (IS_G4X(dev_priv))
				g4x_dp_init(dev_priv, DP_C, PORT_C);
		}

		if (IS_G4X(dev_priv) && (intel_de_read(dev_priv, DP_D) & DP_DETECTED))
			g4x_dp_init(dev_priv, DP_D, PORT_D);

		if (SUPPORTS_TV(dev_priv))
			intel_tv_init(dev_priv);
	} else if (DISPLAY_VER(dev_priv) == 2) {
		if (IS_I85X(dev_priv))
			intel_lvds_init(dev_priv);

		intel_crt_init(dev_priv);
		intel_dvo_init(dev_priv);
	}

	for_each_intel_encoder(&dev_priv->drm, encoder) {
		encoder->base.possible_crtcs =
			intel_encoder_possible_crtcs(encoder);
		encoder->base.possible_clones =
			intel_encoder_possible_clones(encoder);
	}

	intel_init_pch_refclk(dev_priv);

	drm_helper_move_panel_connectors_to_head(&dev_priv->drm);
}

static int max_dotclock(struct drm_i915_private *i915)
{
	int max_dotclock = i915->max_dotclk_freq;

	/* icl+ might use bigjoiner */
	if (DISPLAY_VER(i915) >= 11)
		max_dotclock *= 2;

	return max_dotclock;
}

<<<<<<< HEAD
static int intel_user_framebuffer_create_handle(struct drm_framebuffer *fb,
						struct drm_file *file,
						unsigned int *handle)
{
	struct drm_i915_gem_object *obj = intel_fb_obj(fb);

	if (obj->userptr.mm) {
		DRM_DEBUG("attempting to use a userptr for a framebuffer, denied\n");
		return -EINVAL;
	}

	return drm_gem_handle_create(file, &obj->base, handle);
}

static int intel_user_framebuffer_dirty(struct drm_framebuffer *fb,
					struct drm_file *file,
					unsigned flags, unsigned color,
					struct drm_clip_rect *clips,
					unsigned num_clips)
{
	struct drm_i915_gem_object *obj = intel_fb_obj(fb);

	i915_gem_object_flush_if_display(obj);
	intel_frontbuffer_flush(to_intel_frontbuffer(fb), ORIGIN_DIRTYFB);

	return 0;
}

static const struct drm_framebuffer_funcs intel_fb_funcs = {
	.destroy = intel_user_framebuffer_destroy,
	.create_handle = intel_user_framebuffer_create_handle,
	.dirty = intel_user_framebuffer_dirty,
};

static int intel_framebuffer_init(struct intel_framebuffer *intel_fb,
				  struct drm_i915_gem_object *obj,
				  struct drm_mode_fb_cmd2 *mode_cmd)
{
	struct drm_i915_private *dev_priv = to_i915(obj->base.dev);
	struct drm_framebuffer *fb = &intel_fb->base;
	u32 max_stride;
	unsigned int tiling, stride;
	int ret = -EINVAL;
	int i;

	intel_fb->frontbuffer = intel_frontbuffer_get(obj);
	if (!intel_fb->frontbuffer)
		return -ENOMEM;

	i915_gem_object_lock(obj);
	tiling = i915_gem_object_get_tiling(obj);
	stride = i915_gem_object_get_stride(obj);
	i915_gem_object_unlock(obj);

	if (mode_cmd->flags & DRM_MODE_FB_MODIFIERS) {
		/*
		 * If there's a fence, enforce that
		 * the fb modifier and tiling mode match.
		 */
		if (tiling != I915_TILING_NONE &&
		    tiling != intel_fb_modifier_to_tiling(mode_cmd->modifier[0])) {
			DRM_DEBUG_KMS("tiling_mode doesn't match fb modifier\n");
			goto err;
		}
	} else {
		if (tiling == I915_TILING_X) {
			mode_cmd->modifier[0] = I915_FORMAT_MOD_X_TILED;
		} else if (tiling == I915_TILING_Y) {
			DRM_DEBUG_KMS("No Y tiling for legacy addfb\n");
			goto err;
		}
	}

	if (!drm_any_plane_has_format(&dev_priv->drm,
				      mode_cmd->pixel_format,
				      mode_cmd->modifier[0])) {
		struct drm_format_name_buf format_name;

		DRM_DEBUG_KMS("unsupported pixel format %s / modifier 0x%"PRIx64"\n",
			      drm_get_format_name(mode_cmd->pixel_format,
						  &format_name),
			      mode_cmd->modifier[0]);
		goto err;
	}

	/*
	 * gen2/3 display engine uses the fence if present,
	 * so the tiling mode must match the fb modifier exactly.
	 */
	if (INTEL_GEN(dev_priv) < 4 &&
	    tiling != intel_fb_modifier_to_tiling(mode_cmd->modifier[0])) {
		DRM_DEBUG_KMS("tiling_mode must match fb modifier exactly on gen2/3\n");
		goto err;
	}

	max_stride = intel_fb_max_stride(dev_priv, mode_cmd->pixel_format,
					 mode_cmd->modifier[0]);
	if (mode_cmd->pitches[0] > max_stride) {
		DRM_DEBUG_KMS("%s pitch (%u) must be at most %d\n",
			      mode_cmd->modifier[0] != DRM_FORMAT_MOD_LINEAR ?
			      "tiled" : "linear",
			      mode_cmd->pitches[0], max_stride);
		goto err;
	}

	/*
	 * If there's a fence, enforce that
	 * the fb pitch and fence stride match.
	 */
	if (tiling != I915_TILING_NONE && mode_cmd->pitches[0] != stride) {
		DRM_DEBUG_KMS("pitch (%d) must match tiling stride (%d)\n",
			      mode_cmd->pitches[0], stride);
		goto err;
	}

	/* FIXME need to adjust LINOFF/TILEOFF accordingly. */
	if (mode_cmd->offsets[0] != 0) {
		DRM_DEBUG_KMS("plane 0 offset (0x%08x) must be 0\n",
			      mode_cmd->offsets[0]);
		goto err;
	}

	drm_helper_mode_fill_fb_struct(&dev_priv->drm, fb, mode_cmd);

	for (i = 0; i < fb->format->num_planes; i++) {
		u32 stride_alignment;

		if (mode_cmd->handles[i] != mode_cmd->handles[0]) {
			DRM_DEBUG_KMS("bad plane %d handle\n", i);
			goto err;
		}

		stride_alignment = intel_fb_stride_alignment(fb, i);
		if (fb->pitches[i] & (stride_alignment - 1)) {
			DRM_DEBUG_KMS("plane %d pitch (%d) must be at least %u byte aligned\n",
				      i, fb->pitches[i], stride_alignment);
			goto err;
		}

		if (is_gen12_ccs_plane(fb, i)) {
			int ccs_aux_stride = gen12_ccs_aux_stride(fb, i);

			if (fb->pitches[i] != ccs_aux_stride) {
				DRM_DEBUG_KMS("ccs aux plane %d pitch (%d) must be %d\n",
					      i,
					      fb->pitches[i], ccs_aux_stride);
				goto err;
			}
		}

		fb->obj[i] = &obj->base;
	}

	ret = intel_fill_fb_info(dev_priv, fb);
	if (ret)
		goto err;

	ret = drm_framebuffer_init(&dev_priv->drm, fb, &intel_fb_funcs);
	if (ret) {
		DRM_ERROR("framebuffer init failed %d\n", ret);
		goto err;
	}

	return 0;

err:
	intel_frontbuffer_put(intel_fb->frontbuffer);
	return ret;
}

static struct drm_framebuffer *
intel_user_framebuffer_create(struct drm_device *dev,
			      struct drm_file *filp,
			      const struct drm_mode_fb_cmd2 *user_mode_cmd)
{
	struct drm_framebuffer *fb;
	struct drm_i915_gem_object *obj;
	struct drm_mode_fb_cmd2 mode_cmd = *user_mode_cmd;

	obj = i915_gem_object_lookup(filp, mode_cmd.handles[0]);
	if (!obj)
		return ERR_PTR(-ENOENT);

	fb = intel_framebuffer_create(obj, &mode_cmd);
	i915_gem_object_put(obj);

	return fb;
}

static void intel_atomic_state_free(struct drm_atomic_state *state)
{
	struct intel_atomic_state *intel_state = to_intel_atomic_state(state);

	drm_atomic_state_default_release(state);

	i915_sw_fence_fini(&intel_state->commit_ready);

	kfree(state);
}

static enum drm_mode_status
intel_mode_valid(struct drm_device *dev,
		 const struct drm_display_mode *mode)
=======
enum drm_mode_status intel_mode_valid(struct drm_device *dev,
				      const struct drm_display_mode *mode)
>>>>>>> vendor/linux-drm-v6.6.35
{
	struct drm_i915_private *dev_priv = to_i915(dev);
	int hdisplay_max, htotal_max;
	int vdisplay_max, vtotal_max;

	/*
	 * Can't reject DBLSCAN here because Xorg ddxen can add piles
	 * of DBLSCAN modes to the output's mode list when they detect
	 * the scaling mode property on the connector. And they don't
	 * ask the kernel to validate those modes in any way until
	 * modeset time at which point the client gets a protocol error.
	 * So in order to not upset those clients we silently ignore the
	 * DBLSCAN flag on such connectors. For other connectors we will
	 * reject modes with the DBLSCAN flag in encoder->compute_config().
	 * And we always reject DBLSCAN modes in connector->mode_valid()
	 * as we never want such modes on the connector's mode list.
	 */

	if (mode->vscan > 1)
		return MODE_NO_VSCAN;

	if (mode->flags & DRM_MODE_FLAG_HSKEW)
		return MODE_H_ILLEGAL;

	if (mode->flags & (DRM_MODE_FLAG_CSYNC |
			   DRM_MODE_FLAG_NCSYNC |
			   DRM_MODE_FLAG_PCSYNC))
		return MODE_HSYNC;

	if (mode->flags & (DRM_MODE_FLAG_BCAST |
			   DRM_MODE_FLAG_PIXMUX |
			   DRM_MODE_FLAG_CLKDIV2))
		return MODE_BAD;

	/*
	 * Reject clearly excessive dotclocks early to
	 * avoid having to worry about huge integers later.
	 */
	if (mode->clock > max_dotclock(dev_priv))
		return MODE_CLOCK_HIGH;

	/* Transcoder timing limits */
	if (DISPLAY_VER(dev_priv) >= 11) {
		hdisplay_max = 16384;
		vdisplay_max = 8192;
		htotal_max = 16384;
		vtotal_max = 8192;
	} else if (DISPLAY_VER(dev_priv) >= 9 ||
		   IS_BROADWELL(dev_priv) || IS_HASWELL(dev_priv)) {
		hdisplay_max = 8192; /* FDI max 4096 handled elsewhere */
		vdisplay_max = 4096;
		htotal_max = 8192;
		vtotal_max = 8192;
	} else if (DISPLAY_VER(dev_priv) >= 3) {
		hdisplay_max = 4096;
		vdisplay_max = 4096;
		htotal_max = 8192;
		vtotal_max = 8192;
	} else {
		hdisplay_max = 2048;
		vdisplay_max = 2048;
		htotal_max = 4096;
		vtotal_max = 4096;
	}

	if (mode->hdisplay > hdisplay_max ||
	    mode->hsync_start > htotal_max ||
	    mode->hsync_end > htotal_max ||
	    mode->htotal > htotal_max)
		return MODE_H_ILLEGAL;

	if (mode->vdisplay > vdisplay_max ||
	    mode->vsync_start > vtotal_max ||
	    mode->vsync_end > vtotal_max ||
	    mode->vtotal > vtotal_max)
		return MODE_V_ILLEGAL;

	return MODE_OK;
}

enum drm_mode_status intel_cpu_transcoder_mode_valid(struct drm_i915_private *dev_priv,
						     const struct drm_display_mode *mode)
{
	/*
	 * Additional transcoder timing limits,
	 * excluding BXT/GLK DSI transcoders.
	 */
	if (DISPLAY_VER(dev_priv) >= 5) {
		if (mode->hdisplay < 64 ||
		    mode->htotal - mode->hdisplay < 32)
			return MODE_H_ILLEGAL;

		if (mode->vtotal - mode->vdisplay < 5)
			return MODE_V_ILLEGAL;
	} else {
		if (mode->htotal - mode->hdisplay < 32)
			return MODE_H_ILLEGAL;

		if (mode->vtotal - mode->vdisplay < 3)
			return MODE_V_ILLEGAL;
	}

	/*
	 * Cantiga+ cannot handle modes with a hsync front porch of 0.
	 * WaPruneModeWithIncorrectHsyncOffset:ctg,elk,ilk,snb,ivb,vlv,hsw.
	 */
	if ((DISPLAY_VER(dev_priv) > 4 || IS_G4X(dev_priv)) &&
	    mode->hsync_start == mode->hdisplay)
		return MODE_H_ILLEGAL;

	return MODE_OK;
}

enum drm_mode_status
intel_mode_valid_max_plane_size(struct drm_i915_private *dev_priv,
				const struct drm_display_mode *mode,
				bool bigjoiner)
{
	int plane_width_max, plane_height_max;

	/*
	 * intel_mode_valid() should be
	 * sufficient on older platforms.
	 */
	if (DISPLAY_VER(dev_priv) < 9)
		return MODE_OK;

	/*
	 * Most people will probably want a fullscreen
	 * plane so let's not advertize modes that are
	 * too big for that.
	 */
	if (DISPLAY_VER(dev_priv) >= 11) {
		plane_width_max = 5120 << bigjoiner;
		plane_height_max = 4320;
	} else {
		plane_width_max = 5120;
		plane_height_max = 4096;
	}

	if (mode->hdisplay > plane_width_max)
		return MODE_H_ILLEGAL;

	if (mode->vdisplay > plane_height_max)
		return MODE_V_ILLEGAL;

	return MODE_OK;
}

static const struct intel_display_funcs skl_display_funcs = {
	.get_pipe_config = hsw_get_pipe_config,
	.crtc_enable = hsw_crtc_enable,
	.crtc_disable = hsw_crtc_disable,
	.commit_modeset_enables = skl_commit_modeset_enables,
	.get_initial_plane_config = skl_get_initial_plane_config,
};

static const struct intel_display_funcs ddi_display_funcs = {
	.get_pipe_config = hsw_get_pipe_config,
	.crtc_enable = hsw_crtc_enable,
	.crtc_disable = hsw_crtc_disable,
	.commit_modeset_enables = intel_commit_modeset_enables,
	.get_initial_plane_config = i9xx_get_initial_plane_config,
};

static const struct intel_display_funcs pch_split_display_funcs = {
	.get_pipe_config = ilk_get_pipe_config,
	.crtc_enable = ilk_crtc_enable,
	.crtc_disable = ilk_crtc_disable,
	.commit_modeset_enables = intel_commit_modeset_enables,
	.get_initial_plane_config = i9xx_get_initial_plane_config,
};

static const struct intel_display_funcs vlv_display_funcs = {
	.get_pipe_config = i9xx_get_pipe_config,
	.crtc_enable = valleyview_crtc_enable,
	.crtc_disable = i9xx_crtc_disable,
	.commit_modeset_enables = intel_commit_modeset_enables,
	.get_initial_plane_config = i9xx_get_initial_plane_config,
};

static const struct intel_display_funcs i9xx_display_funcs = {
	.get_pipe_config = i9xx_get_pipe_config,
	.crtc_enable = i9xx_crtc_enable,
	.crtc_disable = i9xx_crtc_disable,
	.commit_modeset_enables = intel_commit_modeset_enables,
	.get_initial_plane_config = i9xx_get_initial_plane_config,
};

/**
 * intel_init_display_hooks - initialize the display modesetting hooks
 * @dev_priv: device private
 */
void intel_init_display_hooks(struct drm_i915_private *dev_priv)
{
	if (DISPLAY_VER(dev_priv) >= 9) {
		dev_priv->display.funcs.display = &skl_display_funcs;
	} else if (HAS_DDI(dev_priv)) {
		dev_priv->display.funcs.display = &ddi_display_funcs;
	} else if (HAS_PCH_SPLIT(dev_priv)) {
		dev_priv->display.funcs.display = &pch_split_display_funcs;
	} else if (IS_CHERRYVIEW(dev_priv) ||
		   IS_VALLEYVIEW(dev_priv)) {
		dev_priv->display.funcs.display = &vlv_display_funcs;
	} else {
		dev_priv->display.funcs.display = &i9xx_display_funcs;
	}
}

<<<<<<< HEAD
void intel_modeset_init_hw(struct drm_i915_private *i915)
{
	intel_update_cdclk(i915);
	intel_dump_cdclk_state(&i915->cdclk.hw, "Current CDCLK");
	i915->cdclk.logical = i915->cdclk.actual = i915->cdclk.hw;
}

static int sanitize_watermarks_add_affected(struct drm_atomic_state *state)
{
	struct drm_plane *plane;
	struct drm_crtc *crtc;

	drm_for_each_crtc(crtc, state->dev) {
		struct drm_crtc_state *crtc_state;

		crtc_state = drm_atomic_get_crtc_state(state, crtc);
		if (IS_ERR(crtc_state))
			return PTR_ERR(crtc_state);
	}

	drm_for_each_plane(plane, state->dev) {
		struct drm_plane_state *plane_state;

		plane_state = drm_atomic_get_plane_state(state, plane);
		if (IS_ERR(plane_state))
			return PTR_ERR(plane_state);
	}

	return 0;
}

/*
 * Calculate what we think the watermarks should be for the state we've read
 * out of the hardware and then immediately program those watermarks so that
 * we ensure the hardware settings match our internal state.
 *
 * We can calculate what we think WM's should be by creating a duplicate of the
 * current state (which was constructed during hardware readout) and running it
 * through the atomic check code to calculate new watermark values in the
 * state object.
 */
static void sanitize_watermarks(struct drm_i915_private *dev_priv)
{
	struct drm_atomic_state *state;
	struct intel_atomic_state *intel_state;
	struct intel_crtc *crtc;
	struct intel_crtc_state *crtc_state;
	struct drm_modeset_acquire_ctx ctx;
	int ret;
	int i;

	/* Only supported on platforms that use atomic watermark design */
	if (!dev_priv->display.optimize_watermarks)
		return;

	state = drm_atomic_state_alloc(&dev_priv->drm);
	if (WARN_ON(!state))
		return;

	intel_state = to_intel_atomic_state(state);

	drm_modeset_acquire_init(&ctx, 0);

retry:
	state->acquire_ctx = &ctx;

	/*
	 * Hardware readout is the only time we don't want to calculate
	 * intermediate watermarks (since we don't trust the current
	 * watermarks).
	 */
	if (!HAS_GMCH(dev_priv))
		intel_state->skip_intermediate_wm = true;

	ret = sanitize_watermarks_add_affected(state);
	if (ret)
		goto fail;

	ret = intel_atomic_check(&dev_priv->drm, state);
	if (ret)
		goto fail;

	/* Write calculated watermark values back */
	for_each_new_intel_crtc_in_state(intel_state, crtc, crtc_state, i) {
		crtc_state->wm.need_postvbl_update = true;
		dev_priv->display.optimize_watermarks(intel_state, crtc);

		to_intel_crtc_state(crtc->base.state)->wm = crtc_state->wm;
	}

fail:
	if (ret == -EDEADLK) {
		drm_atomic_state_clear(state);
		drm_modeset_backoff(&ctx);
		goto retry;
	}

	/*
	 * If we fail here, it means that the hardware appears to be
	 * programmed in a way that shouldn't be possible, given our
	 * understanding of watermark requirements.  This might mean a
	 * mistake in the hardware readout code or a mistake in the
	 * watermark calculations for a given platform.  Raise a WARN
	 * so that this is noticeable.
	 *
	 * If this actually happens, we'll have to just leave the
	 * BIOS-programmed watermarks untouched and hope for the best.
	 */
	WARN(ret, "Could not determine valid watermarks for inherited state\n");

	drm_atomic_state_put(state);

	drm_modeset_drop_locks(&ctx);
	drm_modeset_acquire_fini(&ctx);
}

static void intel_update_fdi_pll_freq(struct drm_i915_private *dev_priv)
{
	if (IS_GEN(dev_priv, 5)) {
		u32 fdi_pll_clk =
			I915_READ(FDI_PLL_BIOS_0) & FDI_PLL_FB_CLOCK_MASK;

		dev_priv->fdi_pll_freq = (fdi_pll_clk + 2) * 10000;
	} else if (IS_GEN(dev_priv, 6) || IS_IVYBRIDGE(dev_priv)) {
		dev_priv->fdi_pll_freq = 270000;
	} else {
		return;
	}

	DRM_DEBUG_DRIVER("FDI PLL freq=%d\n", dev_priv->fdi_pll_freq);
}

static int intel_initial_commit(struct drm_device *dev)
=======
int intel_initial_commit(struct drm_device *dev)
>>>>>>> vendor/linux-drm-v6.6.35
{
	struct drm_atomic_state *state = NULL;
	struct drm_modeset_acquire_ctx ctx;
	struct intel_crtc *crtc;
	int ret = 0;

	state = drm_atomic_state_alloc(dev);
	if (!state)
		return -ENOMEM;

	drm_modeset_acquire_init(&ctx, 0);

	state->acquire_ctx = &ctx;
	to_intel_atomic_state(state)->internal = true;

retry:
	for_each_intel_crtc(dev, crtc) {
		struct intel_crtc_state *crtc_state =
			intel_atomic_get_crtc_state(state, crtc);

		if (IS_ERR(crtc_state)) {
			ret = PTR_ERR(crtc_state);
			goto out;
		}

		if (crtc_state->hw.active) {
			struct intel_encoder *encoder;

			ret = drm_atomic_add_affected_planes(state, &crtc->base);
			if (ret)
				goto out;

			/*
			 * FIXME hack to force a LUT update to avoid the
			 * plane update forcing the pipe gamma on without
			 * having a proper LUT loaded. Remove once we
			 * have readout for pipe gamma enable.
			 */
			crtc_state->uapi.color_mgmt_changed = true;

			for_each_intel_encoder_mask(dev, encoder,
						    crtc_state->uapi.encoder_mask) {
				if (encoder->initial_fastset_check &&
				    !encoder->initial_fastset_check(encoder, crtc_state)) {
					ret = drm_atomic_add_affected_connectors(state,
										 &crtc->base);
					if (ret)
						goto out;
				}
			}
		}
	}

	ret = drm_atomic_commit(state);

out:
	if (ret == -EDEADLK) {
		drm_atomic_state_clear(state);
		drm_modeset_backoff(&ctx);
		goto retry;
	}

	drm_atomic_state_put(state);

	drm_modeset_drop_locks(&ctx);
	drm_modeset_acquire_fini(&ctx);

	return ret;
}

<<<<<<< HEAD
static void intel_mode_config_init(struct drm_i915_private *i915)
{
	struct drm_mode_config *mode_config = &i915->drm.mode_config;

	drm_mode_config_init(&i915->drm);

	mode_config->min_width = 0;
	mode_config->min_height = 0;

	mode_config->preferred_depth = 24;
	mode_config->prefer_shadow = 1;

	mode_config->allow_fb_modifiers = true;

	mode_config->funcs = &intel_mode_funcs;

	/*
	 * Maximum framebuffer dimensions, chosen to match
	 * the maximum render engine surface size on gen4+.
	 */
	if (INTEL_GEN(i915) >= 7) {
		mode_config->max_width = 16384;
		mode_config->max_height = 16384;
	} else if (INTEL_GEN(i915) >= 4) {
		mode_config->max_width = 8192;
		mode_config->max_height = 8192;
	} else if (IS_GEN(i915, 3)) {
		mode_config->max_width = 4096;
		mode_config->max_height = 4096;
	} else {
		mode_config->max_width = 2048;
		mode_config->max_height = 2048;
	}

	if (IS_I845G(i915) || IS_I865G(i915)) {
		mode_config->cursor_width = IS_I845G(i915) ? 64 : 512;
		mode_config->cursor_height = 1023;
	} else if (IS_GEN(i915, 2)) {
		mode_config->cursor_width = 64;
		mode_config->cursor_height = 64;
	} else {
		mode_config->cursor_width = 256;
		mode_config->cursor_height = 256;
	}
}

int intel_modeset_init(struct drm_i915_private *i915)
{
	struct drm_device *dev = &i915->drm;
	enum pipe pipe;
	struct intel_crtc *crtc;
	int ret;

	mutex_init(&i915->drrs.mutex);

	i915->modeset_wq = alloc_ordered_workqueue("i915_modeset", 0);
	i915->flip_wq = alloc_workqueue("i915_flip", WQ_HIGHPRI |
					WQ_UNBOUND, WQ_UNBOUND_MAX_ACTIVE);

	spin_lock_init(&i915->atomic_commit_lock);
	DRM_INIT_WAITQUEUE(&i915->atomic_commit_wq, "i915cmit");

	intel_mode_config_init(i915);

	ret = intel_bw_init(i915);
	if (ret)
		return ret;

	init_llist_head(&i915->atomic_helper.free_list);
	INIT_WORK(&i915->atomic_helper.free_work,
		  intel_atomic_helper_free_state_worker);

	intel_init_quirks(i915);

	intel_fbc_init(i915);

	intel_init_pm(i915);

	intel_panel_sanitize_ssc(i915);

	intel_gmbus_setup(i915);

	DRM_DEBUG_KMS("%d display pipe%s available.\n",
		      INTEL_NUM_PIPES(i915),
		      INTEL_NUM_PIPES(i915) > 1 ? "s" : "");

	if (HAS_DISPLAY(i915) && INTEL_DISPLAY_ENABLED(i915)) {
		for_each_pipe(i915, pipe) {
			ret = intel_crtc_init(i915, pipe);
			if (ret) {
				drm_mode_config_cleanup(dev);
				return ret;
			}
		}
	}

	intel_shared_dpll_init(dev);
	intel_update_fdi_pll_freq(i915);

	intel_update_czclk(i915);
	intel_modeset_init_hw(i915);

	intel_hdcp_component_init(i915);

	if (i915->max_cdclk_freq == 0)
		intel_update_max_cdclk(i915);

	/* Just disable it once at startup */
#ifndef __NetBSD__		/* XXX We wait until intelfb is ready.  */
	intel_vga_disable(i915);
#endif
	intel_setup_outputs(i915);

	drm_modeset_lock_all(dev);
	intel_modeset_setup_hw_state(dev, dev->mode_config.acquire_ctx);
	drm_modeset_unlock_all(dev);

	for_each_intel_crtc(dev, crtc) {
		struct intel_initial_plane_config plane_config = {};

		if (!crtc->active)
			continue;

		/*
		 * Note that reserving the BIOS fb up front prevents us
		 * from stuffing other stolen allocations like the ring
		 * on top.  This prevents some ugliness at boot time, and
		 * can even allow for smooth boot transitions if the BIOS
		 * fb is large enough for the active pipe configuration.
		 */
		i915->display.get_initial_plane_config(crtc, &plane_config);

		/*
		 * If the fb is shared between multiple heads, we'll
		 * just get the first one.
		 */
		intel_find_initial_plane_obj(crtc, &plane_config);
	}

	/*
	 * Make sure hardware watermarks really match the state we read out.
	 * Note that we need to do this after reconstructing the BIOS fb's
	 * since the watermark calculation done here will use pstate->fb.
	 */
	if (!HAS_GMCH(i915))
		sanitize_watermarks(i915);

	/*
	 * Force all active planes to recompute their states. So that on
	 * mode_setcrtc after probe, all the intel_plane_state variables
	 * are already calculated and there is no assert_plane warnings
	 * during bootup.
	 */
	ret = intel_initial_commit(dev);
	if (ret)
		DRM_DEBUG_KMS("Initial commit in probe failed.\n");

	return 0;
}

=======
>>>>>>> vendor/linux-drm-v6.6.35
void i830_enable_pipe(struct drm_i915_private *dev_priv, enum pipe pipe)
{
	struct intel_crtc *crtc = intel_crtc_for_pipe(dev_priv, pipe);
	enum transcoder cpu_transcoder = (enum transcoder)pipe;
	/* 640x480@60Hz, ~25175 kHz */
	struct dpll clock = {
		.m1 = 18,
		.m2 = 7,
		.p1 = 13,
		.p2 = 4,
		.n = 2,
	};
	u32 dpll, fp;
	int i;

	drm_WARN_ON(&dev_priv->drm,
		    i9xx_calc_dpll_params(48000, &clock) != 25154);

	drm_dbg_kms(&dev_priv->drm,
		    "enabling pipe %c due to force quirk (vco=%d dot=%d)\n",
		    pipe_name(pipe), clock.vco, clock.dot);

	fp = i9xx_dpll_compute_fp(&clock);
	dpll = DPLL_DVO_2X_MODE |
		DPLL_VGA_MODE_DIS |
		((clock.p1 - 2) << DPLL_FPA01_P1_POST_DIV_SHIFT) |
		PLL_P2_DIVIDE_BY_4 |
		PLL_REF_INPUT_DREFCLK |
		DPLL_VCO_ENABLE;

	intel_de_write(dev_priv, TRANS_HTOTAL(cpu_transcoder),
		       HACTIVE(640 - 1) | HTOTAL(800 - 1));
	intel_de_write(dev_priv, TRANS_HBLANK(cpu_transcoder),
		       HBLANK_START(640 - 1) | HBLANK_END(800 - 1));
	intel_de_write(dev_priv, TRANS_HSYNC(cpu_transcoder),
		       HSYNC_START(656 - 1) | HSYNC_END(752 - 1));
	intel_de_write(dev_priv, TRANS_VTOTAL(cpu_transcoder),
		       VACTIVE(480 - 1) | VTOTAL(525 - 1));
	intel_de_write(dev_priv, TRANS_VBLANK(cpu_transcoder),
		       VBLANK_START(480 - 1) | VBLANK_END(525 - 1));
	intel_de_write(dev_priv, TRANS_VSYNC(cpu_transcoder),
		       VSYNC_START(490 - 1) | VSYNC_END(492 - 1));
	intel_de_write(dev_priv, PIPESRC(pipe),
		       PIPESRC_WIDTH(640 - 1) | PIPESRC_HEIGHT(480 - 1));

	intel_de_write(dev_priv, FP0(pipe), fp);
	intel_de_write(dev_priv, FP1(pipe), fp);

	/*
	 * Apparently we need to have VGA mode enabled prior to changing
	 * the P1/P2 dividers. Otherwise the DPLL will keep using the old
	 * dividers, even though the register value does change.
	 */
	intel_de_write(dev_priv, DPLL(pipe), dpll & ~DPLL_VGA_MODE_DIS);
	intel_de_write(dev_priv, DPLL(pipe), dpll);

	/* Wait for the clocks to stabilize. */
	intel_de_posting_read(dev_priv, DPLL(pipe));
	udelay(150);

	/* The pixel multiplier can only be updated once the
	 * DPLL is enabled and the clocks are stable.
	 *
	 * So write it again.
	 */
	intel_de_write(dev_priv, DPLL(pipe), dpll);

	/* We do this three times for luck */
	for (i = 0; i < 3 ; i++) {
		intel_de_write(dev_priv, DPLL(pipe), dpll);
		intel_de_posting_read(dev_priv, DPLL(pipe));
		udelay(150); /* wait for warmup */
	}

	intel_de_write(dev_priv, TRANSCONF(pipe), TRANSCONF_ENABLE);
	intel_de_posting_read(dev_priv, TRANSCONF(pipe));

	intel_wait_for_pipe_scanline_moving(crtc);
}

void i830_disable_pipe(struct drm_i915_private *dev_priv, enum pipe pipe)
{
	struct intel_crtc *crtc = intel_crtc_for_pipe(dev_priv, pipe);

	drm_dbg_kms(&dev_priv->drm, "disabling pipe %c due to force quirk\n",
		    pipe_name(pipe));

	drm_WARN_ON(&dev_priv->drm,
		    intel_de_read(dev_priv, DSPCNTR(PLANE_A)) & DISP_ENABLE);
	drm_WARN_ON(&dev_priv->drm,
		    intel_de_read(dev_priv, DSPCNTR(PLANE_B)) & DISP_ENABLE);
	drm_WARN_ON(&dev_priv->drm,
		    intel_de_read(dev_priv, DSPCNTR(PLANE_C)) & DISP_ENABLE);
	drm_WARN_ON(&dev_priv->drm,
		    intel_de_read(dev_priv, CURCNTR(PIPE_A)) & MCURSOR_MODE_MASK);
	drm_WARN_ON(&dev_priv->drm,
		    intel_de_read(dev_priv, CURCNTR(PIPE_B)) & MCURSOR_MODE_MASK);

	intel_de_write(dev_priv, TRANSCONF(pipe), 0);
	intel_de_posting_read(dev_priv, TRANSCONF(pipe));

	intel_wait_for_pipe_scanline_stopped(crtc);

	intel_de_write(dev_priv, DPLL(pipe), DPLL_VGA_MODE_DIS);
	intel_de_posting_read(dev_priv, DPLL(pipe));
}

void intel_hpd_poll_fini(struct drm_i915_private *i915)
{
	struct intel_connector *connector;
	struct drm_connector_list_iter conn_iter;

	/* Kill all the work that may have been queued by hpd. */
	drm_connector_list_iter_begin(&i915->drm, &conn_iter);
	for_each_intel_connector_iter(connector, &conn_iter) {
		if (connector->modeset_retry_work.func)
			cancel_work_sync(&connector->modeset_retry_work);
		if (connector->hdcp.shim) {
			cancel_delayed_work_sync(&connector->hdcp.check_work);
			cancel_work_sync(&connector->hdcp.prop_work);
		}
	}
	drm_connector_list_iter_end(&conn_iter);
}

bool intel_scanout_needs_vtd_wa(struct drm_i915_private *i915)
{
<<<<<<< HEAD
	flush_workqueue(i915->flip_wq);
	flush_workqueue(i915->modeset_wq);

	flush_work(&i915->atomic_helper.free_work);
	WARN_ON(!llist_empty(&i915->atomic_helper.free_list));

	/*
	 * Interrupts and polling as the first thing to avoid creating havoc.
	 * Too much stuff here (turning of connectors, ...) would
	 * experience fancy races otherwise.
	 */
	intel_irq_uninstall(i915);

	/*
	 * Due to the hpd irq storm handling the hotplug work can re-arm the
	 * poll handlers. Hence disable polling after hpd handling is shut down.
	 */
	intel_hpd_poll_fini(i915);

	/*
	 * MST topology needs to be suspended so we don't have any calls to
	 * fbdev after it's finalized. MST will be destroyed later as part of
	 * drm_mode_config_cleanup()
	 */
	intel_dp_mst_suspend(i915);

	/* poll work can call into fbdev, hence clean that up afterwards */
	intel_fbdev_fini(i915);

	intel_unregister_dsm_handler();

	intel_fbc_global_disable(i915);

	/* flush any delayed tasks or pending work */
	flush_scheduled_work();

	intel_hdcp_component_fini(i915);

	drm_mode_config_cleanup(&i915->drm);

	intel_overlay_cleanup(i915);

	intel_shared_dpll_cleanup(&i915->drm);

	intel_gmbus_teardown(i915);

	intel_fbc_cleanup(i915);

	intel_bw_cleanup(i915);

	DRM_DESTROY_WAITQUEUE(&i915->atomic_commit_wq);
	spin_lock_destroy(&i915->atomic_commit_lock);

	destroy_workqueue(i915->flip_wq);
	destroy_workqueue(i915->modeset_wq);

	mutex_destroy(&i915->drrs.mutex);

	intel_fbc_cleanup_cfb(i915);
=======
	return DISPLAY_VER(i915) >= 6 && i915_vtd_active(i915);
>>>>>>> vendor/linux-drm-v6.6.35
}
