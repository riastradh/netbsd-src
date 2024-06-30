/*	$NetBSD: intel_display_power.c,v 1.5 2021/12/19 12:32:15 riastradh Exp $	*/

/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2019 Intel Corporation
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: intel_display_power.c,v 1.5 2021/12/19 12:32:15 riastradh Exp $");

#include <linux/string_helpers.h>

#include "i915_drv.h"
#include "i915_irq.h"
#include "i915_reg.h"
#include "intel_backlight_regs.h"
#include "intel_cdclk.h"
#include "intel_clock_gating.h"
#include "intel_combo_phy.h"
#include "intel_de.h"
#include "intel_display_power.h"
#include "intel_display_power_map.h"
#include "intel_display_power_well.h"
#include "intel_display_types.h"
#include "intel_dmc.h"
#include "intel_mchbar_regs.h"
#include "intel_pch_refclk.h"
#include "intel_pcode.h"
#include "intel_pmdemand.h"
#include "intel_pps_regs.h"
#include "intel_snps_phy.h"
#include "skl_watermark.h"
#include "skl_watermark_regs.h"
#include "vlv_sideband.h"

<<<<<<< HEAD
#include <linux/nbsd-namespace.h>

bool intel_display_power_well_is_enabled(struct drm_i915_private *dev_priv,
					 enum i915_power_well_id power_well_id);
=======
#define for_each_power_domain_well(__dev_priv, __power_well, __domain)	\
	for_each_power_well(__dev_priv, __power_well)				\
		for_each_if(test_bit((__domain), (__power_well)->domains.bits))

#define for_each_power_domain_well_reverse(__dev_priv, __power_well, __domain) \
	for_each_power_well_reverse(__dev_priv, __power_well)		        \
		for_each_if(test_bit((__domain), (__power_well)->domains.bits))
>>>>>>> vendor/linux-drm-v6.6.35

const char *
intel_display_power_domain_str(enum intel_display_power_domain domain)
{
	switch (domain) {
	case POWER_DOMAIN_DISPLAY_CORE:
		return "DISPLAY_CORE";
	case POWER_DOMAIN_PIPE_A:
		return "PIPE_A";
	case POWER_DOMAIN_PIPE_B:
		return "PIPE_B";
	case POWER_DOMAIN_PIPE_C:
		return "PIPE_C";
	case POWER_DOMAIN_PIPE_D:
		return "PIPE_D";
	case POWER_DOMAIN_PIPE_PANEL_FITTER_A:
		return "PIPE_PANEL_FITTER_A";
	case POWER_DOMAIN_PIPE_PANEL_FITTER_B:
		return "PIPE_PANEL_FITTER_B";
	case POWER_DOMAIN_PIPE_PANEL_FITTER_C:
		return "PIPE_PANEL_FITTER_C";
	case POWER_DOMAIN_PIPE_PANEL_FITTER_D:
		return "PIPE_PANEL_FITTER_D";
	case POWER_DOMAIN_TRANSCODER_A:
		return "TRANSCODER_A";
	case POWER_DOMAIN_TRANSCODER_B:
		return "TRANSCODER_B";
	case POWER_DOMAIN_TRANSCODER_C:
		return "TRANSCODER_C";
	case POWER_DOMAIN_TRANSCODER_D:
		return "TRANSCODER_D";
	case POWER_DOMAIN_TRANSCODER_EDP:
		return "TRANSCODER_EDP";
	case POWER_DOMAIN_TRANSCODER_DSI_A:
		return "TRANSCODER_DSI_A";
	case POWER_DOMAIN_TRANSCODER_DSI_C:
		return "TRANSCODER_DSI_C";
	case POWER_DOMAIN_TRANSCODER_VDSC_PW2:
		return "TRANSCODER_VDSC_PW2";
	case POWER_DOMAIN_PORT_DDI_LANES_A:
		return "PORT_DDI_LANES_A";
	case POWER_DOMAIN_PORT_DDI_LANES_B:
		return "PORT_DDI_LANES_B";
	case POWER_DOMAIN_PORT_DDI_LANES_C:
		return "PORT_DDI_LANES_C";
	case POWER_DOMAIN_PORT_DDI_LANES_D:
		return "PORT_DDI_LANES_D";
	case POWER_DOMAIN_PORT_DDI_LANES_E:
		return "PORT_DDI_LANES_E";
	case POWER_DOMAIN_PORT_DDI_LANES_F:
		return "PORT_DDI_LANES_F";
	case POWER_DOMAIN_PORT_DDI_LANES_TC1:
		return "PORT_DDI_LANES_TC1";
	case POWER_DOMAIN_PORT_DDI_LANES_TC2:
		return "PORT_DDI_LANES_TC2";
	case POWER_DOMAIN_PORT_DDI_LANES_TC3:
		return "PORT_DDI_LANES_TC3";
	case POWER_DOMAIN_PORT_DDI_LANES_TC4:
		return "PORT_DDI_LANES_TC4";
	case POWER_DOMAIN_PORT_DDI_LANES_TC5:
		return "PORT_DDI_LANES_TC5";
	case POWER_DOMAIN_PORT_DDI_LANES_TC6:
		return "PORT_DDI_LANES_TC6";
	case POWER_DOMAIN_PORT_DDI_IO_A:
		return "PORT_DDI_IO_A";
	case POWER_DOMAIN_PORT_DDI_IO_B:
		return "PORT_DDI_IO_B";
	case POWER_DOMAIN_PORT_DDI_IO_C:
		return "PORT_DDI_IO_C";
	case POWER_DOMAIN_PORT_DDI_IO_D:
		return "PORT_DDI_IO_D";
	case POWER_DOMAIN_PORT_DDI_IO_E:
		return "PORT_DDI_IO_E";
	case POWER_DOMAIN_PORT_DDI_IO_F:
		return "PORT_DDI_IO_F";
	case POWER_DOMAIN_PORT_DDI_IO_TC1:
		return "PORT_DDI_IO_TC1";
	case POWER_DOMAIN_PORT_DDI_IO_TC2:
		return "PORT_DDI_IO_TC2";
	case POWER_DOMAIN_PORT_DDI_IO_TC3:
		return "PORT_DDI_IO_TC3";
	case POWER_DOMAIN_PORT_DDI_IO_TC4:
		return "PORT_DDI_IO_TC4";
	case POWER_DOMAIN_PORT_DDI_IO_TC5:
		return "PORT_DDI_IO_TC5";
	case POWER_DOMAIN_PORT_DDI_IO_TC6:
		return "PORT_DDI_IO_TC6";
	case POWER_DOMAIN_PORT_DSI:
		return "PORT_DSI";
	case POWER_DOMAIN_PORT_CRT:
		return "PORT_CRT";
	case POWER_DOMAIN_PORT_OTHER:
		return "PORT_OTHER";
	case POWER_DOMAIN_VGA:
		return "VGA";
	case POWER_DOMAIN_AUDIO_MMIO:
		return "AUDIO_MMIO";
	case POWER_DOMAIN_AUDIO_PLAYBACK:
		return "AUDIO_PLAYBACK";
	case POWER_DOMAIN_AUX_IO_A:
		return "AUX_IO_A";
	case POWER_DOMAIN_AUX_IO_B:
		return "AUX_IO_B";
	case POWER_DOMAIN_AUX_IO_C:
		return "AUX_IO_C";
	case POWER_DOMAIN_AUX_IO_D:
		return "AUX_IO_D";
	case POWER_DOMAIN_AUX_IO_E:
		return "AUX_IO_E";
	case POWER_DOMAIN_AUX_IO_F:
		return "AUX_IO_F";
	case POWER_DOMAIN_AUX_A:
		return "AUX_A";
	case POWER_DOMAIN_AUX_B:
		return "AUX_B";
	case POWER_DOMAIN_AUX_C:
		return "AUX_C";
	case POWER_DOMAIN_AUX_D:
		return "AUX_D";
	case POWER_DOMAIN_AUX_E:
		return "AUX_E";
	case POWER_DOMAIN_AUX_F:
		return "AUX_F";
	case POWER_DOMAIN_AUX_USBC1:
		return "AUX_USBC1";
	case POWER_DOMAIN_AUX_USBC2:
		return "AUX_USBC2";
	case POWER_DOMAIN_AUX_USBC3:
		return "AUX_USBC3";
	case POWER_DOMAIN_AUX_USBC4:
		return "AUX_USBC4";
	case POWER_DOMAIN_AUX_USBC5:
		return "AUX_USBC5";
	case POWER_DOMAIN_AUX_USBC6:
		return "AUX_USBC6";
	case POWER_DOMAIN_AUX_TBT1:
		return "AUX_TBT1";
	case POWER_DOMAIN_AUX_TBT2:
		return "AUX_TBT2";
	case POWER_DOMAIN_AUX_TBT3:
		return "AUX_TBT3";
	case POWER_DOMAIN_AUX_TBT4:
		return "AUX_TBT4";
	case POWER_DOMAIN_AUX_TBT5:
		return "AUX_TBT5";
	case POWER_DOMAIN_AUX_TBT6:
		return "AUX_TBT6";
	case POWER_DOMAIN_GMBUS:
		return "GMBUS";
	case POWER_DOMAIN_INIT:
		return "INIT";
	case POWER_DOMAIN_MODESET:
		return "MODESET";
	case POWER_DOMAIN_GT_IRQ:
		return "GT_IRQ";
	case POWER_DOMAIN_DC_OFF:
		return "DC_OFF";
	case POWER_DOMAIN_TC_COLD_OFF:
		return "TC_COLD_OFF";
	default:
		MISSING_CASE(domain);
		return "?";
	}
}

/**
 * __intel_display_power_is_enabled - unlocked check for a power domain
 * @dev_priv: i915 device instance
 * @domain: power domain to check
 *
 * This is the unlocked version of intel_display_power_is_enabled() and should
 * only be used from error capture and recovery code where deadlocks are
 * possible.
 *
 * Returns:
 * True when the power domain is enabled, false otherwise.
 */
bool __intel_display_power_is_enabled(struct drm_i915_private *dev_priv,
				      enum intel_display_power_domain domain)
{
	struct i915_power_well *power_well;
	bool is_enabled;

	if (dev_priv->runtime_pm.suspended)
		return false;

	is_enabled = true;

	for_each_power_domain_well_reverse(dev_priv, power_well, domain) {
		if (intel_power_well_is_always_on(power_well))
			continue;

		if (!intel_power_well_is_enabled_cached(power_well)) {
			is_enabled = false;
			break;
		}
	}

	return is_enabled;
}

/**
 * intel_display_power_is_enabled - check for a power domain
 * @dev_priv: i915 device instance
 * @domain: power domain to check
 *
 * This function can be used to check the hw power domain state. It is mostly
 * used in hardware state readout functions. Everywhere else code should rely
 * upon explicit power domain reference counting to ensure that the hardware
 * block is powered up before accessing it.
 *
 * Callers must hold the relevant modesetting locks to ensure that concurrent
 * threads can't disable the power well while the caller tries to read a few
 * registers.
 *
 * Returns:
 * True when the power domain is enabled, false otherwise.
 */
bool intel_display_power_is_enabled(struct drm_i915_private *dev_priv,
				    enum intel_display_power_domain domain)
{
	struct i915_power_domains *power_domains;
	bool ret;

	power_domains = &dev_priv->display.power.domains;

	mutex_lock(&power_domains->lock);
	ret = __intel_display_power_is_enabled(dev_priv, domain);
	mutex_unlock(&power_domains->lock);

	return ret;
}

<<<<<<< HEAD
/*
 * Starting with Haswell, we have a "Power Down Well" that can be turned off
 * when not needed anymore. We have 4 registers that can request the power well
 * to be enabled, and it will only be disabled if none of the registers is
 * requesting it to be enabled.
 */
static void hsw_power_well_post_enable(struct drm_i915_private *dev_priv,
				       u8 irq_pipe_mask, bool has_vga)
{
#ifndef __NetBSD__ /* XXX We wait until intelfb is ready.  */
	if (has_vga)
		intel_vga_reset_io_mem(dev_priv);
#endif

	if (irq_pipe_mask)
		gen8_irq_power_well_post_enable(dev_priv, irq_pipe_mask);
}

static void hsw_power_well_pre_disable(struct drm_i915_private *dev_priv,
				       u8 irq_pipe_mask)
{
	if (irq_pipe_mask)
		gen8_irq_power_well_pre_disable(dev_priv, irq_pipe_mask);
}

static void hsw_wait_for_power_well_enable(struct drm_i915_private *dev_priv,
					   struct i915_power_well *power_well)
{
	const struct i915_power_well_regs *regs = power_well->desc->hsw.regs;
	int pw_idx = power_well->desc->hsw.idx;

	/* Timeout for PW1:10 us, AUX:not specified, other PWs:20 us. */
	if (intel_de_wait_for_set(dev_priv, regs->driver,
				  HSW_PWR_WELL_CTL_STATE(pw_idx), 1)) {
		DRM_DEBUG_KMS("%s power well enable timeout\n",
			      power_well->desc->name);

		/* An AUX timeout is expected if the TBT DP tunnel is down. */
		WARN_ON(!power_well->desc->hsw.is_tc_tbt);
	}
}

static u32 hsw_power_well_requesters(struct drm_i915_private *dev_priv,
				     const struct i915_power_well_regs *regs,
				     int pw_idx)
{
	u32 req_mask = HSW_PWR_WELL_CTL_REQ(pw_idx);
	u32 ret;

	ret = I915_READ(regs->bios) & req_mask ? 1 : 0;
	ret |= I915_READ(regs->driver) & req_mask ? 2 : 0;
	if (regs->kvmr.reg)
		ret |= I915_READ(regs->kvmr) & req_mask ? 4 : 0;
	ret |= I915_READ(regs->debug) & req_mask ? 8 : 0;

	return ret;
}

static void hsw_wait_for_power_well_disable(struct drm_i915_private *dev_priv,
					    struct i915_power_well *power_well)
{
	const struct i915_power_well_regs *regs = power_well->desc->hsw.regs;
	int pw_idx = power_well->desc->hsw.idx;
	bool disabled;
	u32 reqs;

	/*
	 * Bspec doesn't require waiting for PWs to get disabled, but still do
	 * this for paranoia. The known cases where a PW will be forced on:
	 * - a KVMR request on any power well via the KVMR request register
	 * - a DMC request on PW1 and MISC_IO power wells via the BIOS and
	 *   DEBUG request registers
	 * Skip the wait in case any of the request bits are set and print a
	 * diagnostic message.
	 */
	wait_for((disabled = !(I915_READ(regs->driver) &
			       HSW_PWR_WELL_CTL_STATE(pw_idx))) ||
		 (reqs = hsw_power_well_requesters(dev_priv, regs, pw_idx)), 1);
	if (disabled)
		return;

	DRM_DEBUG_KMS("%s forced on (bios:%d driver:%d kvmr:%d debug:%d)\n",
		      power_well->desc->name,
		      !!(reqs & 1), !!(reqs & 2), !!(reqs & 4), !!(reqs & 8));
}

static void gen9_wait_for_power_well_fuses(struct drm_i915_private *dev_priv,
					   enum skl_power_gate pg)
{
	/* Timeout 5us for PG#0, for other PGs 1us */
	WARN_ON(intel_de_wait_for_set(dev_priv, SKL_FUSE_STATUS,
				      SKL_FUSE_PG_DIST_STATUS(pg), 1));
}

static void hsw_power_well_enable(struct drm_i915_private *dev_priv,
				  struct i915_power_well *power_well)
{
	const struct i915_power_well_regs *regs = power_well->desc->hsw.regs;
	int pw_idx = power_well->desc->hsw.idx;
	bool wait_fuses = power_well->desc->hsw.has_fuses;
	enum skl_power_gate uninitialized_var(pg);
	u32 val;

	if (wait_fuses) {
		pg = INTEL_GEN(dev_priv) >= 11 ? ICL_PW_CTL_IDX_TO_PG(pw_idx) :
						 SKL_PW_CTL_IDX_TO_PG(pw_idx);
		/*
		 * For PW1 we have to wait both for the PW0/PG0 fuse state
		 * before enabling the power well and PW1/PG1's own fuse
		 * state after the enabling. For all other power wells with
		 * fuses we only have to wait for that PW/PG's fuse state
		 * after the enabling.
		 */
		if (pg == SKL_PG1)
			gen9_wait_for_power_well_fuses(dev_priv, SKL_PG0);
	}

	val = I915_READ(regs->driver);
	I915_WRITE(regs->driver, val | HSW_PWR_WELL_CTL_REQ(pw_idx));
	hsw_wait_for_power_well_enable(dev_priv, power_well);

	/* Display WA #1178: cnl */
	if (IS_CANNONLAKE(dev_priv) &&
	    pw_idx >= GLK_PW_CTL_IDX_AUX_B &&
	    pw_idx <= CNL_PW_CTL_IDX_AUX_F) {
		val = I915_READ(CNL_AUX_ANAOVRD1(pw_idx));
		val |= CNL_AUX_ANAOVRD1_ENABLE | CNL_AUX_ANAOVRD1_LDO_BYPASS;
		I915_WRITE(CNL_AUX_ANAOVRD1(pw_idx), val);
	}

	if (wait_fuses)
		gen9_wait_for_power_well_fuses(dev_priv, pg);

	hsw_power_well_post_enable(dev_priv,
				   power_well->desc->hsw.irq_pipe_mask,
				   power_well->desc->hsw.has_vga);
}

static void hsw_power_well_disable(struct drm_i915_private *dev_priv,
				   struct i915_power_well *power_well)
{
	const struct i915_power_well_regs *regs = power_well->desc->hsw.regs;
	int pw_idx = power_well->desc->hsw.idx;
	u32 val;

	hsw_power_well_pre_disable(dev_priv,
				   power_well->desc->hsw.irq_pipe_mask);

	val = I915_READ(regs->driver);
	I915_WRITE(regs->driver, val & ~HSW_PWR_WELL_CTL_REQ(pw_idx));
	hsw_wait_for_power_well_disable(dev_priv, power_well);
}

#define ICL_AUX_PW_TO_PHY(pw_idx)	((pw_idx) - ICL_PW_CTL_IDX_AUX_A)

static void
icl_combo_phy_aux_power_well_enable(struct drm_i915_private *dev_priv,
				    struct i915_power_well *power_well)
{
	const struct i915_power_well_regs *regs = power_well->desc->hsw.regs;
	int pw_idx = power_well->desc->hsw.idx;
	enum phy phy = ICL_AUX_PW_TO_PHY(pw_idx);
	u32 val;

	WARN_ON(!IS_ICELAKE(dev_priv));

	val = I915_READ(regs->driver);
	I915_WRITE(regs->driver, val | HSW_PWR_WELL_CTL_REQ(pw_idx));

	if (INTEL_GEN(dev_priv) < 12) {
		val = I915_READ(ICL_PORT_CL_DW12(phy));
		I915_WRITE(ICL_PORT_CL_DW12(phy), val | ICL_LANE_ENABLE_AUX);
	}

	hsw_wait_for_power_well_enable(dev_priv, power_well);

	/* Display WA #1178: icl */
	if (pw_idx >= ICL_PW_CTL_IDX_AUX_A && pw_idx <= ICL_PW_CTL_IDX_AUX_B &&
	    !intel_bios_is_port_edp(dev_priv, (enum port)phy)) {
		val = I915_READ(ICL_AUX_ANAOVRD1(pw_idx));
		val |= ICL_AUX_ANAOVRD1_ENABLE | ICL_AUX_ANAOVRD1_LDO_BYPASS;
		I915_WRITE(ICL_AUX_ANAOVRD1(pw_idx), val);
	}
}

static void
icl_combo_phy_aux_power_well_disable(struct drm_i915_private *dev_priv,
				     struct i915_power_well *power_well)
{
	const struct i915_power_well_regs *regs = power_well->desc->hsw.regs;
	int pw_idx = power_well->desc->hsw.idx;
	enum phy phy = ICL_AUX_PW_TO_PHY(pw_idx);
	u32 val;

	WARN_ON(!IS_ICELAKE(dev_priv));

	val = I915_READ(ICL_PORT_CL_DW12(phy));
	I915_WRITE(ICL_PORT_CL_DW12(phy), val & ~ICL_LANE_ENABLE_AUX);

	val = I915_READ(regs->driver);
	I915_WRITE(regs->driver, val & ~HSW_PWR_WELL_CTL_REQ(pw_idx));

	hsw_wait_for_power_well_disable(dev_priv, power_well);
}

#define ICL_AUX_PW_TO_CH(pw_idx)	\
	((pw_idx) - ICL_PW_CTL_IDX_AUX_A + AUX_CH_A)

#define ICL_TBT_AUX_PW_TO_CH(pw_idx)	\
	((pw_idx) - ICL_PW_CTL_IDX_AUX_TBT1 + AUX_CH_C)

static enum aux_ch icl_tc_phy_aux_ch(struct drm_i915_private *dev_priv,
				     struct i915_power_well *power_well)
{
	int pw_idx = power_well->desc->hsw.idx;

	return power_well->desc->hsw.is_tc_tbt ? ICL_TBT_AUX_PW_TO_CH(pw_idx) :
						 ICL_AUX_PW_TO_CH(pw_idx);
}

#if IS_ENABLED(CONFIG_DRM_I915_DEBUG_RUNTIME_PM)

static u64 async_put_domains_mask(struct i915_power_domains *power_domains);

static int power_well_async_ref_count(struct drm_i915_private *dev_priv,
				      struct i915_power_well *power_well)
{
	int refs = hweight64(power_well->desc->domains &
			     async_put_domains_mask(&dev_priv->power_domains));

	WARN_ON(refs > power_well->count);

	return refs;
}

static void icl_tc_port_assert_ref_held(struct drm_i915_private *dev_priv,
					struct i915_power_well *power_well)
{
	enum aux_ch aux_ch = icl_tc_phy_aux_ch(dev_priv, power_well);
	struct intel_digital_port *dig_port = NULL;
	struct intel_encoder *encoder;

	/* Bypass the check if all references are released asynchronously */
	if (power_well_async_ref_count(dev_priv, power_well) ==
	    power_well->count)
		return;

	aux_ch = icl_tc_phy_aux_ch(dev_priv, power_well);

	for_each_intel_encoder(&dev_priv->drm, encoder) {
		enum phy phy = intel_port_to_phy(dev_priv, encoder->port);

		if (!intel_phy_is_tc(dev_priv, phy))
			continue;

		/* We'll check the MST primary port */
		if (encoder->type == INTEL_OUTPUT_DP_MST)
			continue;

		dig_port = enc_to_dig_port(encoder);
		if (WARN_ON(!dig_port))
			continue;

		if (dig_port->aux_ch != aux_ch) {
			dig_port = NULL;
			continue;
		}

		break;
	}

	if (WARN_ON(!dig_port))
		return;

	WARN_ON(!intel_tc_port_ref_held(dig_port));
}

#else

static void icl_tc_port_assert_ref_held(struct drm_i915_private *dev_priv,
					struct i915_power_well *power_well)
{
}

#endif

#define TGL_AUX_PW_TO_TC_PORT(pw_idx)	((pw_idx) - TGL_PW_CTL_IDX_AUX_TC1)

static void
icl_tc_phy_aux_power_well_enable(struct drm_i915_private *dev_priv,
				 struct i915_power_well *power_well)
{
	enum aux_ch aux_ch = icl_tc_phy_aux_ch(dev_priv, power_well);
	u32 val;

	icl_tc_port_assert_ref_held(dev_priv, power_well);

	val = I915_READ(DP_AUX_CH_CTL(aux_ch));
	val &= ~DP_AUX_CH_CTL_TBT_IO;
	if (power_well->desc->hsw.is_tc_tbt)
		val |= DP_AUX_CH_CTL_TBT_IO;
	I915_WRITE(DP_AUX_CH_CTL(aux_ch), val);

	hsw_power_well_enable(dev_priv, power_well);

	if (INTEL_GEN(dev_priv) >= 12 && !power_well->desc->hsw.is_tc_tbt) {
		enum tc_port tc_port;

		tc_port = TGL_AUX_PW_TO_TC_PORT(power_well->desc->hsw.idx);
		I915_WRITE(HIP_INDEX_REG(tc_port), HIP_INDEX_VAL(tc_port, 0x2));

		if (intel_de_wait_for_set(dev_priv, DKL_CMN_UC_DW_27(tc_port),
					  DKL_CMN_UC_DW27_UC_HEALTH, 1))
			DRM_WARN("Timeout waiting TC uC health\n");
	}
}

static void
icl_tc_phy_aux_power_well_disable(struct drm_i915_private *dev_priv,
				  struct i915_power_well *power_well)
{
	icl_tc_port_assert_ref_held(dev_priv, power_well);

	hsw_power_well_disable(dev_priv, power_well);
}

/*
 * We should only use the power well if we explicitly asked the hardware to
 * enable it, so check if it's enabled and also check if we've requested it to
 * be enabled.
 */
static bool hsw_power_well_enabled(struct drm_i915_private *dev_priv,
				   struct i915_power_well *power_well)
{
	const struct i915_power_well_regs *regs = power_well->desc->hsw.regs;
	enum i915_power_well_id id = power_well->desc->id;
	int pw_idx = power_well->desc->hsw.idx;
	u32 mask = HSW_PWR_WELL_CTL_REQ(pw_idx) |
		   HSW_PWR_WELL_CTL_STATE(pw_idx);
	u32 val;

	val = I915_READ(regs->driver);

	/*
	 * On GEN9 big core due to a DMC bug the driver's request bits for PW1
	 * and the MISC_IO PW will be not restored, so check instead for the
	 * BIOS's own request bits, which are forced-on for these power wells
	 * when exiting DC5/6.
	 */
	if (IS_GEN(dev_priv, 9) && !IS_GEN9_LP(dev_priv) &&
	    (id == SKL_DISP_PW_1 || id == SKL_DISP_PW_MISC_IO))
		val |= I915_READ(regs->bios);

	return (val & mask) == mask;
}

static void assert_can_enable_dc9(struct drm_i915_private *dev_priv)
{
	WARN_ONCE((I915_READ(DC_STATE_EN) & DC_STATE_EN_DC9),
		  "DC9 already programmed to be enabled.\n");
	WARN_ONCE(I915_READ(DC_STATE_EN) & DC_STATE_EN_UPTO_DC5,
		  "DC5 still not disabled to enable DC9.\n");
	WARN_ONCE(I915_READ(HSW_PWR_WELL_CTL2) &
		  HSW_PWR_WELL_CTL_REQ(SKL_PW_CTL_IDX_PW_2),
		  "Power well 2 on.\n");
	WARN_ONCE(intel_irqs_enabled(dev_priv),
		  "Interrupts not disabled yet.\n");

	 /*
	  * TODO: check for the following to verify the conditions to enter DC9
	  * state are satisfied:
	  * 1] Check relevant display engine registers to verify if mode set
	  * disable sequence was followed.
	  * 2] Check if display uninitialize sequence is initialized.
	  */
}

static void assert_can_disable_dc9(struct drm_i915_private *dev_priv)
{
	WARN_ONCE(intel_irqs_enabled(dev_priv),
		  "Interrupts not disabled yet.\n");
	WARN_ONCE(I915_READ(DC_STATE_EN) & DC_STATE_EN_UPTO_DC5,
		  "DC5 still not disabled.\n");

	 /*
	  * TODO: check for the following to verify DC9 state was indeed
	  * entered before programming to disable it:
	  * 1] Check relevant display engine registers to verify if mode
	  *  set disable sequence was followed.
	  * 2] Check if display uninitialize sequence is initialized.
	  */
}

static void gen9_write_dc_state(struct drm_i915_private *dev_priv,
				u32 state)
{
	int rewrites = 0;
	int rereads = 0;
	u32 v;

	I915_WRITE(DC_STATE_EN, state);

	/* It has been observed that disabling the dc6 state sometimes
	 * doesn't stick and dmc keeps returning old value. Make sure
	 * the write really sticks enough times and also force rewrite until
	 * we are confident that state is exactly what we want.
	 */
	do  {
		v = I915_READ(DC_STATE_EN);

		if (v != state) {
			I915_WRITE(DC_STATE_EN, state);
			rewrites++;
			rereads = 0;
		} else if (rereads++ > 5) {
			break;
		}

	} while (rewrites < 100);

	if (v != state)
		DRM_ERROR("Writing dc state to 0x%x failed, now 0x%x\n",
			  state, v);

	/* Most of the times we need one retry, avoid spam */
	if (rewrites > 1)
		DRM_DEBUG_KMS("Rewrote dc state to 0x%x %d times\n",
			      state, rewrites);
}

static u32 gen9_dc_mask(struct drm_i915_private *dev_priv)
{
	u32 mask;

	mask = DC_STATE_EN_UPTO_DC5;

	if (INTEL_GEN(dev_priv) >= 12)
		mask |= DC_STATE_EN_DC3CO | DC_STATE_EN_UPTO_DC6
					  | DC_STATE_EN_DC9;
	else if (IS_GEN(dev_priv, 11))
		mask |= DC_STATE_EN_UPTO_DC6 | DC_STATE_EN_DC9;
	else if (IS_GEN9_LP(dev_priv))
		mask |= DC_STATE_EN_DC9;
	else
		mask |= DC_STATE_EN_UPTO_DC6;

	return mask;
}

static void gen9_sanitize_dc_state(struct drm_i915_private *dev_priv)
{
	u32 val;

	val = I915_READ(DC_STATE_EN) & gen9_dc_mask(dev_priv);

	DRM_DEBUG_KMS("Resetting DC state tracking from %02x to %02x\n",
		      dev_priv->csr.dc_state, val);
	dev_priv->csr.dc_state = val;
}

/**
 * gen9_set_dc_state - set target display C power state
 * @dev_priv: i915 device instance
 * @state: target DC power state
 * - DC_STATE_DISABLE
 * - DC_STATE_EN_UPTO_DC5
 * - DC_STATE_EN_UPTO_DC6
 * - DC_STATE_EN_DC9
 *
 * Signal to DMC firmware/HW the target DC power state passed in @state.
 * DMC/HW can turn off individual display clocks and power rails when entering
 * a deeper DC power state (higher in number) and turns these back when exiting
 * that state to a shallower power state (lower in number). The HW will decide
 * when to actually enter a given state on an on-demand basis, for instance
 * depending on the active state of display pipes. The state of display
 * registers backed by affected power rails are saved/restored as needed.
 *
 * Based on the above enabling a deeper DC power state is asynchronous wrt.
 * enabling it. Disabling a deeper power state is synchronous: for instance
 * setting %DC_STATE_DISABLE won't complete until all HW resources are turned
 * back on and register state is restored. This is guaranteed by the MMIO write
 * to DC_STATE_EN blocking until the state is restored.
 */
static void gen9_set_dc_state(struct drm_i915_private *dev_priv, u32 state)
{
	u32 val;
	u32 mask;

	if (WARN_ON_ONCE(state & ~dev_priv->csr.allowed_dc_mask))
		state &= dev_priv->csr.allowed_dc_mask;

	val = I915_READ(DC_STATE_EN);
	mask = gen9_dc_mask(dev_priv);
	DRM_DEBUG_KMS("Setting DC state from %02x to %02x\n",
		      val & mask, state);

	/* Check if DMC is ignoring our DC state requests */
	if ((val & mask) != dev_priv->csr.dc_state)
		DRM_ERROR("DC state mismatch (0x%x -> 0x%x)\n",
			  dev_priv->csr.dc_state, val & mask);

	val &= ~mask;
	val |= state;

	gen9_write_dc_state(dev_priv, val);

	dev_priv->csr.dc_state = val & mask;
}

=======
>>>>>>> vendor/linux-drm-v6.6.35
static u32
sanitize_target_dc_state(struct drm_i915_private *i915,
			 u32 target_dc_state)
{
	struct i915_power_domains *power_domains = &i915->display.power.domains;
	static const u32 states[] = {
		DC_STATE_EN_UPTO_DC6,
		DC_STATE_EN_UPTO_DC5,
		DC_STATE_EN_DC3CO,
		DC_STATE_DISABLE,
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(states) - 1; i++) {
		if (target_dc_state != states[i])
			continue;

		if (power_domains->allowed_dc_mask & target_dc_state)
			break;

		target_dc_state = states[i + 1];
	}

	return target_dc_state;
}

/**
 * intel_display_power_set_target_dc_state - Set target dc state.
 * @dev_priv: i915 device
 * @state: state which needs to be set as target_dc_state.
 *
 * This function set the "DC off" power well target_dc_state,
 * based upon this target_dc_stste, "DC off" power well will
 * enable desired DC state.
 */
void intel_display_power_set_target_dc_state(struct drm_i915_private *dev_priv,
					     u32 state)
{
	struct i915_power_well *power_well;
	bool dc_off_enabled;
	struct i915_power_domains *power_domains = &dev_priv->display.power.domains;

	mutex_lock(&power_domains->lock);
	power_well = lookup_power_well(dev_priv, SKL_DISP_DC_OFF);

	if (drm_WARN_ON(&dev_priv->drm, !power_well))
		goto unlock;

	state = sanitize_target_dc_state(dev_priv, state);

	if (state == power_domains->target_dc_state)
		goto unlock;

	dc_off_enabled = intel_power_well_is_enabled(dev_priv, power_well);
	/*
	 * If DC off power well is disabled, need to enable and disable the
	 * DC off power well to effect target DC state.
	 */
	if (!dc_off_enabled)
		intel_power_well_enable(dev_priv, power_well);

	power_domains->target_dc_state = state;

	if (!dc_off_enabled)
		intel_power_well_disable(dev_priv, power_well);

unlock:
	mutex_unlock(&power_domains->lock);
}

<<<<<<< HEAD
static void assert_can_enable_dc5(struct drm_i915_private *dev_priv)
{
	bool pg2_enabled = intel_display_power_well_is_enabled(dev_priv,
					SKL_DISP_PW_2);

	WARN_ONCE(pg2_enabled, "PG2 not disabled to enable DC5.\n");

	WARN_ONCE((I915_READ(DC_STATE_EN) & DC_STATE_EN_UPTO_DC5),
		  "DC5 already programmed to be enabled.\n");
	assert_rpm_wakelock_held(&dev_priv->runtime_pm);

	assert_csr_loaded(dev_priv);
}

static void gen9_enable_dc5(struct drm_i915_private *dev_priv)
{
	assert_can_enable_dc5(dev_priv);

	DRM_DEBUG_KMS("Enabling DC5\n");

	/* Wa Display #1183: skl,kbl,cfl */
	if (IS_GEN9_BC(dev_priv))
		I915_WRITE(GEN8_CHICKEN_DCPR_1, I915_READ(GEN8_CHICKEN_DCPR_1) |
			   SKL_SELECT_ALTERNATE_DC_EXIT);

	gen9_set_dc_state(dev_priv, DC_STATE_EN_UPTO_DC5);
}

static void assert_can_enable_dc6(struct drm_i915_private *dev_priv)
{
	WARN_ONCE(I915_READ(UTIL_PIN_CTL) & UTIL_PIN_ENABLE,
		  "Backlight is not disabled.\n");
	WARN_ONCE((I915_READ(DC_STATE_EN) & DC_STATE_EN_UPTO_DC6),
		  "DC6 already programmed to be enabled.\n");

	assert_csr_loaded(dev_priv);
}

static void skl_enable_dc6(struct drm_i915_private *dev_priv)
{
	assert_can_enable_dc6(dev_priv);

	DRM_DEBUG_KMS("Enabling DC6\n");

	/* Wa Display #1183: skl,kbl,cfl */
	if (IS_GEN9_BC(dev_priv))
		I915_WRITE(GEN8_CHICKEN_DCPR_1, I915_READ(GEN8_CHICKEN_DCPR_1) |
			   SKL_SELECT_ALTERNATE_DC_EXIT);

	gen9_set_dc_state(dev_priv, DC_STATE_EN_UPTO_DC6);
}

static void hsw_power_well_sync_hw(struct drm_i915_private *dev_priv,
				   struct i915_power_well *power_well)
{
	const struct i915_power_well_regs *regs = power_well->desc->hsw.regs;
	int pw_idx = power_well->desc->hsw.idx;
	u32 mask = HSW_PWR_WELL_CTL_REQ(pw_idx);
	u32 bios_req = I915_READ(regs->bios);

	/* Take over the request bit if set by BIOS. */
	if (bios_req & mask) {
		u32 drv_req = I915_READ(regs->driver);

		if (!(drv_req & mask))
			I915_WRITE(regs->driver, drv_req | mask);
		I915_WRITE(regs->bios, bios_req & ~mask);
	}
}

static void bxt_dpio_cmn_power_well_enable(struct drm_i915_private *dev_priv,
					   struct i915_power_well *power_well)
{
	bxt_ddi_phy_init(dev_priv, power_well->desc->bxt.phy);
}

static void bxt_dpio_cmn_power_well_disable(struct drm_i915_private *dev_priv,
					    struct i915_power_well *power_well)
{
	bxt_ddi_phy_uninit(dev_priv, power_well->desc->bxt.phy);
}

static bool bxt_dpio_cmn_power_well_enabled(struct drm_i915_private *dev_priv,
					    struct i915_power_well *power_well)
{
	return bxt_ddi_phy_is_enabled(dev_priv, power_well->desc->bxt.phy);
}

static void bxt_verify_ddi_phy_power_wells(struct drm_i915_private *dev_priv)
{
	struct i915_power_well *power_well;

	power_well = lookup_power_well(dev_priv, BXT_DISP_PW_DPIO_CMN_A);
	if (power_well->count > 0)
		bxt_ddi_phy_verify_state(dev_priv, power_well->desc->bxt.phy);

	power_well = lookup_power_well(dev_priv, VLV_DISP_PW_DPIO_CMN_BC);
	if (power_well->count > 0)
		bxt_ddi_phy_verify_state(dev_priv, power_well->desc->bxt.phy);

	if (IS_GEMINILAKE(dev_priv)) {
		power_well = lookup_power_well(dev_priv,
					       GLK_DISP_PW_DPIO_CMN_C);
		if (power_well->count > 0)
			bxt_ddi_phy_verify_state(dev_priv,
						 power_well->desc->bxt.phy);
	}
}

static bool gen9_dc_off_power_well_enabled(struct drm_i915_private *dev_priv,
					   struct i915_power_well *power_well)
{
	return ((I915_READ(DC_STATE_EN) & DC_STATE_EN_DC3CO) == 0 &&
		(I915_READ(DC_STATE_EN) & DC_STATE_EN_UPTO_DC5_DC6_MASK) == 0);
}

static void gen9_assert_dbuf_enabled(struct drm_i915_private *dev_priv)
{
	u32 tmp = I915_READ(DBUF_CTL);

	WARN((tmp & (DBUF_POWER_STATE | DBUF_POWER_REQUEST)) !=
	     (DBUF_POWER_STATE | DBUF_POWER_REQUEST),
	     "Unexpected DBuf power power state (0x%08x)\n", tmp);
}

static void gen9_disable_dc_states(struct drm_i915_private *dev_priv)
{
	struct intel_cdclk_state cdclk_state = {};

	if (dev_priv->csr.target_dc_state == DC_STATE_EN_DC3CO) {
		tgl_disable_dc3co(dev_priv);
		return;
	}

	gen9_set_dc_state(dev_priv, DC_STATE_DISABLE);

	dev_priv->display.get_cdclk(dev_priv, &cdclk_state);
	/* Can't read out voltage_level so can't use intel_cdclk_changed() */
	WARN_ON(intel_cdclk_needs_modeset(&dev_priv->cdclk.hw, &cdclk_state));

	gen9_assert_dbuf_enabled(dev_priv);

	if (IS_GEN9_LP(dev_priv))
		bxt_verify_ddi_phy_power_wells(dev_priv);

	if (INTEL_GEN(dev_priv) >= 11)
		/*
		 * DMC retains HW context only for port A, the other combo
		 * PHY's HW context for port B is lost after DC transitions,
		 * so we need to restore it manually.
		 */
		intel_combo_phy_init(dev_priv);
}

static void gen9_dc_off_power_well_enable(struct drm_i915_private *dev_priv,
					  struct i915_power_well *power_well)
{
	gen9_disable_dc_states(dev_priv);
}

static void gen9_dc_off_power_well_disable(struct drm_i915_private *dev_priv,
					   struct i915_power_well *power_well)
{
	if (!dev_priv->csr.dmc_payload)
		return;

	switch (dev_priv->csr.target_dc_state) {
	case DC_STATE_EN_DC3CO:
		tgl_enable_dc3co(dev_priv);
		break;
	case DC_STATE_EN_UPTO_DC6:
		skl_enable_dc6(dev_priv);
		break;
	case DC_STATE_EN_UPTO_DC5:
		gen9_enable_dc5(dev_priv);
		break;
	}
}

static void i9xx_power_well_sync_hw_noop(struct drm_i915_private *dev_priv,
					 struct i915_power_well *power_well)
{
}

static void i9xx_always_on_power_well_noop(struct drm_i915_private *dev_priv,
					   struct i915_power_well *power_well)
{
}

static bool i9xx_always_on_power_well_enabled(struct drm_i915_private *dev_priv,
					     struct i915_power_well *power_well)
{
	return true;
}

static void i830_pipes_power_well_enable(struct drm_i915_private *dev_priv,
					 struct i915_power_well *power_well)
{
	if ((I915_READ(PIPECONF(PIPE_A)) & PIPECONF_ENABLE) == 0)
		i830_enable_pipe(dev_priv, PIPE_A);
	if ((I915_READ(PIPECONF(PIPE_B)) & PIPECONF_ENABLE) == 0)
		i830_enable_pipe(dev_priv, PIPE_B);
}

static void i830_pipes_power_well_disable(struct drm_i915_private *dev_priv,
					  struct i915_power_well *power_well)
{
	i830_disable_pipe(dev_priv, PIPE_B);
	i830_disable_pipe(dev_priv, PIPE_A);
}

static bool i830_pipes_power_well_enabled(struct drm_i915_private *dev_priv,
					  struct i915_power_well *power_well)
{
	return I915_READ(PIPECONF(PIPE_A)) & PIPECONF_ENABLE &&
		I915_READ(PIPECONF(PIPE_B)) & PIPECONF_ENABLE;
}

static void i830_pipes_power_well_sync_hw(struct drm_i915_private *dev_priv,
					  struct i915_power_well *power_well)
{
	if (power_well->count > 0)
		i830_pipes_power_well_enable(dev_priv, power_well);
	else
		i830_pipes_power_well_disable(dev_priv, power_well);
}

static void vlv_set_power_well(struct drm_i915_private *dev_priv,
			       struct i915_power_well *power_well, bool enable)
{
	int pw_idx = power_well->desc->vlv.idx;
	u32 mask;
	u32 state;
	u32 ctrl;

	mask = PUNIT_PWRGT_MASK(pw_idx);
	state = enable ? PUNIT_PWRGT_PWR_ON(pw_idx) :
			 PUNIT_PWRGT_PWR_GATE(pw_idx);

	vlv_punit_get(dev_priv);

#define COND \
	((vlv_punit_read(dev_priv, PUNIT_REG_PWRGT_STATUS) & mask) == state)

	if (COND)
		goto out;

	ctrl = vlv_punit_read(dev_priv, PUNIT_REG_PWRGT_CTRL);
	ctrl &= ~mask;
	ctrl |= state;
	vlv_punit_write(dev_priv, PUNIT_REG_PWRGT_CTRL, ctrl);

	if (wait_for(COND, 100))
		DRM_ERROR("timeout setting power well state %08x (%08x)\n",
			  state,
			  vlv_punit_read(dev_priv, PUNIT_REG_PWRGT_CTRL));

#undef COND

out:
	vlv_punit_put(dev_priv);
}

static void vlv_power_well_enable(struct drm_i915_private *dev_priv,
				  struct i915_power_well *power_well)
{
	vlv_set_power_well(dev_priv, power_well, true);
}

static void vlv_power_well_disable(struct drm_i915_private *dev_priv,
				   struct i915_power_well *power_well)
{
	vlv_set_power_well(dev_priv, power_well, false);
}

static bool vlv_power_well_enabled(struct drm_i915_private *dev_priv,
				   struct i915_power_well *power_well)
{
	int pw_idx = power_well->desc->vlv.idx;
	bool enabled = false;
	u32 mask;
	u32 state;
	u32 ctrl;

	mask = PUNIT_PWRGT_MASK(pw_idx);
	ctrl = PUNIT_PWRGT_PWR_ON(pw_idx);

	vlv_punit_get(dev_priv);

	state = vlv_punit_read(dev_priv, PUNIT_REG_PWRGT_STATUS) & mask;
	/*
	 * We only ever set the power-on and power-gate states, anything
	 * else is unexpected.
	 */
	WARN_ON(state != PUNIT_PWRGT_PWR_ON(pw_idx) &&
		state != PUNIT_PWRGT_PWR_GATE(pw_idx));
	if (state == ctrl)
		enabled = true;

	/*
	 * A transient state at this point would mean some unexpected party
	 * is poking at the power controls too.
	 */
	ctrl = vlv_punit_read(dev_priv, PUNIT_REG_PWRGT_CTRL) & mask;
	WARN_ON(ctrl != state);

	vlv_punit_put(dev_priv);

	return enabled;
}

static void vlv_init_display_clock_gating(struct drm_i915_private *dev_priv)
{
	u32 val;

	/*
	 * On driver load, a pipe may be active and driving a DSI display.
	 * Preserve DPOUNIT_CLOCK_GATE_DISABLE to avoid the pipe getting stuck
	 * (and never recovering) in this case. intel_dsi_post_disable() will
	 * clear it when we turn off the display.
	 */
	val = I915_READ(DSPCLK_GATE_D);
	val &= DPOUNIT_CLOCK_GATE_DISABLE;
	val |= VRHUNIT_CLOCK_GATE_DISABLE;
	I915_WRITE(DSPCLK_GATE_D, val);

	/*
	 * Disable trickle feed and enable pnd deadline calculation
	 */
	I915_WRITE(MI_ARB_VLV, MI_ARB_DISPLAY_TRICKLE_FEED_DISABLE);
	I915_WRITE(CBR1_VLV, 0);

	WARN_ON(dev_priv->rawclk_freq == 0);

	I915_WRITE(RAWCLK_FREQ_VLV,
		   DIV_ROUND_CLOSEST(dev_priv->rawclk_freq, 1000));
}

static void vlv_display_power_well_init(struct drm_i915_private *dev_priv)
{
	struct intel_encoder *encoder;
	enum pipe pipe;

	/*
	 * Enable the CRI clock source so we can get at the
	 * display and the reference clock for VGA
	 * hotplug / manual detection. Supposedly DSI also
	 * needs the ref clock up and running.
	 *
	 * CHV DPLL B/C have some issues if VGA mode is enabled.
	 */
	for_each_pipe(dev_priv, pipe) {
		u32 val = I915_READ(DPLL(pipe));

		val |= DPLL_REF_CLK_ENABLE_VLV | DPLL_VGA_MODE_DIS;
		if (pipe != PIPE_A)
			val |= DPLL_INTEGRATED_CRI_CLK_VLV;

		I915_WRITE(DPLL(pipe), val);
	}

	vlv_init_display_clock_gating(dev_priv);

	spin_lock_irq(&dev_priv->irq_lock);
	valleyview_enable_display_irqs(dev_priv);
	spin_unlock_irq(&dev_priv->irq_lock);

	/*
	 * During driver initialization/resume we can avoid restoring the
	 * part of the HW/SW state that will be inited anyway explicitly.
	 */
	if (dev_priv->power_domains.initializing)
		return;

	intel_hpd_init(dev_priv);

	/* Re-enable the ADPA, if we have one */
	for_each_intel_encoder(&dev_priv->drm, encoder) {
		if (encoder->type == INTEL_OUTPUT_ANALOG)
			intel_crt_reset(&encoder->base);
	}

	intel_vga_redisable_power_on(dev_priv);

	intel_pps_unlock_regs_wa(dev_priv);
}

static void vlv_display_power_well_deinit(struct drm_i915_private *dev_priv)
{
	spin_lock_irq(&dev_priv->irq_lock);
	valleyview_disable_display_irqs(dev_priv);
	spin_unlock_irq(&dev_priv->irq_lock);

	/* make sure we're done processing display irqs */
	intel_synchronize_irq(dev_priv);

	intel_power_sequencer_reset(dev_priv);

	/* Prevent us from re-enabling polling on accident in late suspend */
#ifdef __NetBSD__
	if (device_activation(dev_priv->drm.dev, DEVACT_LEVEL_FULL))
#else
	if (!dev_priv->drm.dev->power.is_suspended)
#endif
		intel_hpd_poll_init(dev_priv);
}

static void vlv_display_power_well_enable(struct drm_i915_private *dev_priv,
					  struct i915_power_well *power_well)
{
	vlv_set_power_well(dev_priv, power_well, true);

	vlv_display_power_well_init(dev_priv);
}

static void vlv_display_power_well_disable(struct drm_i915_private *dev_priv,
					   struct i915_power_well *power_well)
{
	vlv_display_power_well_deinit(dev_priv);

	vlv_set_power_well(dev_priv, power_well, false);
}

static void vlv_dpio_cmn_power_well_enable(struct drm_i915_private *dev_priv,
					   struct i915_power_well *power_well)
{
	/* since ref/cri clock was enabled */
	udelay(1); /* >10ns for cmnreset, >0ns for sidereset */

	vlv_set_power_well(dev_priv, power_well, true);

	/*
	 * From VLV2A0_DP_eDP_DPIO_driver_vbios_notes_10.docx -
	 *  6.	De-assert cmn_reset/side_reset. Same as VLV X0.
	 *   a.	GUnit 0x2110 bit[0] set to 1 (def 0)
	 *   b.	The other bits such as sfr settings / modesel may all
	 *	be set to 0.
	 *
	 * This should only be done on init and resume from S3 with
	 * both PLLs disabled, or we risk losing DPIO and PLL
	 * synchronization.
	 */
	I915_WRITE(DPIO_CTL, I915_READ(DPIO_CTL) | DPIO_CMNRST);
}

static void vlv_dpio_cmn_power_well_disable(struct drm_i915_private *dev_priv,
					    struct i915_power_well *power_well)
{
	enum pipe pipe;

	for_each_pipe(dev_priv, pipe)
		assert_pll_disabled(dev_priv, pipe);

	/* Assert common reset */
	I915_WRITE(DPIO_CTL, I915_READ(DPIO_CTL) & ~DPIO_CMNRST);

	vlv_set_power_well(dev_priv, power_well, false);
}

=======
>>>>>>> vendor/linux-drm-v6.6.35
#define POWER_DOMAIN_MASK (GENMASK_ULL(POWER_DOMAIN_NUM - 1, 0))

static void __async_put_domains_mask(struct i915_power_domains *power_domains,
				     struct intel_power_domain_mask *mask)
{
	bitmap_or(mask->bits,
		  power_domains->async_put_domains[0].bits,
		  power_domains->async_put_domains[1].bits,
		  POWER_DOMAIN_NUM);
}

#if IS_ENABLED(CONFIG_DRM_I915_DEBUG_RUNTIME_PM)

static bool
assert_async_put_domain_masks_disjoint(struct i915_power_domains *power_domains)
{
	struct drm_i915_private *i915 = container_of(power_domains,
						     struct drm_i915_private,
						     display.power.domains);

	return !drm_WARN_ON(&i915->drm,
			    bitmap_intersects(power_domains->async_put_domains[0].bits,
					      power_domains->async_put_domains[1].bits,
					      POWER_DOMAIN_NUM));
}

static bool
__async_put_domains_state_ok(struct i915_power_domains *power_domains)
{
	struct drm_i915_private *i915 = container_of(power_domains,
						     struct drm_i915_private,
						     display.power.domains);
	struct intel_power_domain_mask async_put_mask;
	enum intel_display_power_domain domain;
	bool err = false;

	err |= !assert_async_put_domain_masks_disjoint(power_domains);
	__async_put_domains_mask(power_domains, &async_put_mask);
	err |= drm_WARN_ON(&i915->drm,
			   !!power_domains->async_put_wakeref !=
			   !bitmap_empty(async_put_mask.bits, POWER_DOMAIN_NUM));

	for_each_power_domain(domain, &async_put_mask)
		err |= drm_WARN_ON(&i915->drm,
				   power_domains->domain_use_count[domain] != 1);

	return !err;
}

static void print_power_domains(struct i915_power_domains *power_domains,
				const char *prefix, struct intel_power_domain_mask *mask)
{
	struct drm_i915_private *i915 = container_of(power_domains,
						     struct drm_i915_private,
						     display.power.domains);
	enum intel_display_power_domain domain;

	drm_dbg(&i915->drm, "%s (%d):\n", prefix, bitmap_weight(mask->bits, POWER_DOMAIN_NUM));
	for_each_power_domain(domain, mask)
		drm_dbg(&i915->drm, "%s use_count %d\n",
			intel_display_power_domain_str(domain),
			power_domains->domain_use_count[domain]);
}

static void
print_async_put_domains_state(struct i915_power_domains *power_domains)
{
	struct drm_i915_private *i915 = container_of(power_domains,
						     struct drm_i915_private,
						     display.power.domains);

	drm_dbg(&i915->drm, "async_put_wakeref %u\n",
		power_domains->async_put_wakeref);

	print_power_domains(power_domains, "async_put_domains[0]",
			    &power_domains->async_put_domains[0]);
	print_power_domains(power_domains, "async_put_domains[1]",
			    &power_domains->async_put_domains[1]);
}

static void
verify_async_put_domains_state(struct i915_power_domains *power_domains)
{
	if (!__async_put_domains_state_ok(power_domains))
		print_async_put_domains_state(power_domains);
}

#else

static void
assert_async_put_domain_masks_disjoint(struct i915_power_domains *power_domains)
{
}

static void
verify_async_put_domains_state(struct i915_power_domains *power_domains)
{
}

#endif /* CONFIG_DRM_I915_DEBUG_RUNTIME_PM */

static void async_put_domains_mask(struct i915_power_domains *power_domains,
				   struct intel_power_domain_mask *mask)

{
	assert_async_put_domain_masks_disjoint(power_domains);

	__async_put_domains_mask(power_domains, mask);
}

static void
async_put_domains_clear_domain(struct i915_power_domains *power_domains,
			       enum intel_display_power_domain domain)
{
	assert_async_put_domain_masks_disjoint(power_domains);

	clear_bit(domain, power_domains->async_put_domains[0].bits);
	clear_bit(domain, power_domains->async_put_domains[1].bits);
}

static void
cancel_async_put_work(struct i915_power_domains *power_domains, bool sync)
{
	if (sync)
		cancel_delayed_work_sync(&power_domains->async_put_work);
	else
		cancel_delayed_work(&power_domains->async_put_work);

	power_domains->async_put_next_delay = 0;
}

static bool
intel_display_power_grab_async_put_ref(struct drm_i915_private *dev_priv,
				       enum intel_display_power_domain domain)
{
	struct i915_power_domains *power_domains = &dev_priv->display.power.domains;
	struct intel_power_domain_mask async_put_mask;
	bool ret = false;

	async_put_domains_mask(power_domains, &async_put_mask);
	if (!test_bit(domain, async_put_mask.bits))
		goto out_verify;

	async_put_domains_clear_domain(power_domains, domain);

	ret = true;

	async_put_domains_mask(power_domains, &async_put_mask);
	if (!bitmap_empty(async_put_mask.bits, POWER_DOMAIN_NUM))
		goto out_verify;

	cancel_async_put_work(power_domains, false);
	intel_runtime_pm_put_raw(&dev_priv->runtime_pm,
				 fetch_and_zero(&power_domains->async_put_wakeref));
out_verify:
	verify_async_put_domains_state(power_domains);

	return ret;
}

static void
__intel_display_power_get_domain(struct drm_i915_private *dev_priv,
				 enum intel_display_power_domain domain)
{
	struct i915_power_domains *power_domains = &dev_priv->display.power.domains;
	struct i915_power_well *power_well;

	if (intel_display_power_grab_async_put_ref(dev_priv, domain))
		return;

	for_each_power_domain_well(dev_priv, power_well, domain)
		intel_power_well_get(dev_priv, power_well);

	power_domains->domain_use_count[domain]++;
}

/**
 * intel_display_power_get - grab a power domain reference
 * @dev_priv: i915 device instance
 * @domain: power domain to reference
 *
 * This function grabs a power domain reference for @domain and ensures that the
 * power domain and all its parents are powered up. Therefore users should only
 * grab a reference to the innermost power domain they need.
 *
 * Any power domain reference obtained by this function must have a symmetric
 * call to intel_display_power_put() to release the reference again.
 */
intel_wakeref_t intel_display_power_get(struct drm_i915_private *dev_priv,
					enum intel_display_power_domain domain)
{
	struct i915_power_domains *power_domains = &dev_priv->display.power.domains;
	intel_wakeref_t wakeref = intel_runtime_pm_get(&dev_priv->runtime_pm);

	mutex_lock(&power_domains->lock);
	__intel_display_power_get_domain(dev_priv, domain);
	mutex_unlock(&power_domains->lock);

	return wakeref;
}

/**
 * intel_display_power_get_if_enabled - grab a reference for an enabled display power domain
 * @dev_priv: i915 device instance
 * @domain: power domain to reference
 *
 * This function grabs a power domain reference for @domain and ensures that the
 * power domain and all its parents are powered up. Therefore users should only
 * grab a reference to the innermost power domain they need.
 *
 * Any power domain reference obtained by this function must have a symmetric
 * call to intel_display_power_put() to release the reference again.
 */
intel_wakeref_t
intel_display_power_get_if_enabled(struct drm_i915_private *dev_priv,
				   enum intel_display_power_domain domain)
{
	struct i915_power_domains *power_domains = &dev_priv->display.power.domains;
	intel_wakeref_t wakeref;
	bool is_enabled;

	wakeref = intel_runtime_pm_get_if_in_use(&dev_priv->runtime_pm);
	if (!wakeref)
		return false;

	mutex_lock(&power_domains->lock);

	if (__intel_display_power_is_enabled(dev_priv, domain)) {
		__intel_display_power_get_domain(dev_priv, domain);
		is_enabled = true;
	} else {
		is_enabled = false;
	}

	mutex_unlock(&power_domains->lock);

	if (!is_enabled) {
		intel_runtime_pm_put(&dev_priv->runtime_pm, wakeref);
		wakeref = 0;
	}

	return wakeref;
}

static void
__intel_display_power_put_domain(struct drm_i915_private *dev_priv,
				 enum intel_display_power_domain domain)
{
	struct i915_power_domains *power_domains;
	struct i915_power_well *power_well;
	const char *name = intel_display_power_domain_str(domain);
	struct intel_power_domain_mask async_put_mask;

	power_domains = &dev_priv->display.power.domains;

	drm_WARN(&dev_priv->drm, !power_domains->domain_use_count[domain],
		 "Use count on domain %s is already zero\n",
		 name);
	async_put_domains_mask(power_domains, &async_put_mask);
	drm_WARN(&dev_priv->drm,
		 test_bit(domain, async_put_mask.bits),
		 "Async disabling of domain %s is pending\n",
		 name);

	power_domains->domain_use_count[domain]--;

	for_each_power_domain_well_reverse(dev_priv, power_well, domain)
		intel_power_well_put(dev_priv, power_well);
}

static void __intel_display_power_put(struct drm_i915_private *dev_priv,
				      enum intel_display_power_domain domain)
{
	struct i915_power_domains *power_domains = &dev_priv->display.power.domains;

	mutex_lock(&power_domains->lock);
	__intel_display_power_put_domain(dev_priv, domain);
	mutex_unlock(&power_domains->lock);
}

static void
queue_async_put_domains_work(struct i915_power_domains *power_domains,
			     intel_wakeref_t wakeref,
			     int delay_ms)
{
	struct drm_i915_private *i915 = container_of(power_domains,
						     struct drm_i915_private,
						     display.power.domains);
	drm_WARN_ON(&i915->drm, power_domains->async_put_wakeref);
	power_domains->async_put_wakeref = wakeref;
	drm_WARN_ON(&i915->drm, !queue_delayed_work(system_unbound_wq,
						    &power_domains->async_put_work,
						    msecs_to_jiffies(delay_ms)));
}

static void
release_async_put_domains(struct i915_power_domains *power_domains,
			  struct intel_power_domain_mask *mask)
{
	struct drm_i915_private *dev_priv =
		container_of(power_domains, struct drm_i915_private,
			     display.power.domains);
	struct intel_runtime_pm *rpm = &dev_priv->runtime_pm;
	enum intel_display_power_domain domain;
	intel_wakeref_t wakeref;

	/*
	 * The caller must hold already raw wakeref, upgrade that to a proper
	 * wakeref to make the state checker happy about the HW access during
	 * power well disabling.
	 */
	assert_rpm_raw_wakeref_held(rpm);
	wakeref = intel_runtime_pm_get(rpm);

	for_each_power_domain(domain, mask) {
		/* Clear before put, so put's sanity check is happy. */
		async_put_domains_clear_domain(power_domains, domain);
		__intel_display_power_put_domain(dev_priv, domain);
	}

	intel_runtime_pm_put(rpm, wakeref);
}

static void
intel_display_power_put_async_work(struct work_struct *work)
{
	struct drm_i915_private *dev_priv =
		container_of(work, struct drm_i915_private,
			     display.power.domains.async_put_work.work);
	struct i915_power_domains *power_domains = &dev_priv->display.power.domains;
	struct intel_runtime_pm *rpm = &dev_priv->runtime_pm;
	intel_wakeref_t new_work_wakeref = intel_runtime_pm_get_raw(rpm);
	intel_wakeref_t old_work_wakeref = 0;

	mutex_lock(&power_domains->lock);

	/*
	 * Bail out if all the domain refs pending to be released were grabbed
	 * by subsequent gets or a flush_work.
	 */
	old_work_wakeref = fetch_and_zero(&power_domains->async_put_wakeref);
	if (!old_work_wakeref)
		goto out_verify;

	release_async_put_domains(power_domains,
				  &power_domains->async_put_domains[0]);

	/* Requeue the work if more domains were async put meanwhile. */
	if (!bitmap_empty(power_domains->async_put_domains[1].bits, POWER_DOMAIN_NUM)) {
		bitmap_copy(power_domains->async_put_domains[0].bits,
			    power_domains->async_put_domains[1].bits,
			    POWER_DOMAIN_NUM);
		bitmap_zero(power_domains->async_put_domains[1].bits,
			    POWER_DOMAIN_NUM);
		queue_async_put_domains_work(power_domains,
					     fetch_and_zero(&new_work_wakeref),
					     power_domains->async_put_next_delay);
		power_domains->async_put_next_delay = 0;
	} else {
		/*
		 * Cancel the work that got queued after this one got dequeued,
		 * since here we released the corresponding async-put reference.
		 */
		cancel_async_put_work(power_domains, false);
	}

out_verify:
	verify_async_put_domains_state(power_domains);

	mutex_unlock(&power_domains->lock);

	if (old_work_wakeref)
		intel_runtime_pm_put_raw(rpm, old_work_wakeref);
	if (new_work_wakeref)
		intel_runtime_pm_put_raw(rpm, new_work_wakeref);
}

/**
 * __intel_display_power_put_async - release a power domain reference asynchronously
 * @i915: i915 device instance
 * @domain: power domain to reference
 * @wakeref: wakeref acquired for the reference that is being released
 * @delay_ms: delay of powering down the power domain
 *
 * This function drops the power domain reference obtained by
 * intel_display_power_get*() and schedules a work to power down the
 * corresponding hardware block if this is the last reference.
 * The power down is delayed by @delay_ms if this is >= 0, or by a default
 * 100 ms otherwise.
 */
void __intel_display_power_put_async(struct drm_i915_private *i915,
				     enum intel_display_power_domain domain,
				     intel_wakeref_t wakeref,
				     int delay_ms)
{
	struct i915_power_domains *power_domains = &i915->display.power.domains;
	struct intel_runtime_pm *rpm = &i915->runtime_pm;
	intel_wakeref_t work_wakeref = intel_runtime_pm_get_raw(rpm);

	delay_ms = delay_ms >= 0 ? delay_ms : 100;

	mutex_lock(&power_domains->lock);

	if (power_domains->domain_use_count[domain] > 1) {
		__intel_display_power_put_domain(i915, domain);

		goto out_verify;
	}

	drm_WARN_ON(&i915->drm, power_domains->domain_use_count[domain] != 1);

	/* Let a pending work requeue itself or queue a new one. */
	if (power_domains->async_put_wakeref) {
		set_bit(domain, power_domains->async_put_domains[1].bits);
		power_domains->async_put_next_delay = max(power_domains->async_put_next_delay,
							  delay_ms);
	} else {
		set_bit(domain, power_domains->async_put_domains[0].bits);
		queue_async_put_domains_work(power_domains,
					     fetch_and_zero(&work_wakeref),
					     delay_ms);
	}

out_verify:
	verify_async_put_domains_state(power_domains);

	mutex_unlock(&power_domains->lock);

	if (work_wakeref)
		intel_runtime_pm_put_raw(rpm, work_wakeref);

	intel_runtime_pm_put(rpm, wakeref);
}

/**
 * intel_display_power_flush_work - flushes the async display power disabling work
 * @i915: i915 device instance
 *
 * Flushes any pending work that was scheduled by a preceding
 * intel_display_power_put_async() call, completing the disabling of the
 * corresponding power domains.
 *
 * Note that the work handler function may still be running after this
 * function returns; to ensure that the work handler isn't running use
 * intel_display_power_flush_work_sync() instead.
 */
void intel_display_power_flush_work(struct drm_i915_private *i915)
{
	struct i915_power_domains *power_domains = &i915->display.power.domains;
	struct intel_power_domain_mask async_put_mask;
	intel_wakeref_t work_wakeref;

	mutex_lock(&power_domains->lock);

	work_wakeref = fetch_and_zero(&power_domains->async_put_wakeref);
	if (!work_wakeref)
		goto out_verify;

	async_put_domains_mask(power_domains, &async_put_mask);
	release_async_put_domains(power_domains, &async_put_mask);
	cancel_async_put_work(power_domains, false);

out_verify:
	verify_async_put_domains_state(power_domains);

	mutex_unlock(&power_domains->lock);

	if (work_wakeref)
		intel_runtime_pm_put_raw(&i915->runtime_pm, work_wakeref);
}

/**
 * intel_display_power_flush_work_sync - flushes and syncs the async display power disabling work
 * @i915: i915 device instance
 *
 * Like intel_display_power_flush_work(), but also ensure that the work
 * handler function is not running any more when this function returns.
 */
static void
intel_display_power_flush_work_sync(struct drm_i915_private *i915)
{
	struct i915_power_domains *power_domains = &i915->display.power.domains;

	intel_display_power_flush_work(i915);
	cancel_async_put_work(power_domains, true);

	verify_async_put_domains_state(power_domains);

	drm_WARN_ON(&i915->drm, power_domains->async_put_wakeref);
}

#if IS_ENABLED(CONFIG_DRM_I915_DEBUG_RUNTIME_PM)
/**
 * intel_display_power_put - release a power domain reference
 * @dev_priv: i915 device instance
 * @domain: power domain to reference
 * @wakeref: wakeref acquired for the reference that is being released
 *
 * This function drops the power domain reference obtained by
 * intel_display_power_get() and might power down the corresponding hardware
 * block right away if this is the last reference.
 */
void intel_display_power_put(struct drm_i915_private *dev_priv,
			     enum intel_display_power_domain domain,
			     intel_wakeref_t wakeref)
{
	__intel_display_power_put(dev_priv, domain);
	intel_runtime_pm_put(&dev_priv->runtime_pm, wakeref);
}
#else
/**
 * intel_display_power_put_unchecked - release an unchecked power domain reference
 * @dev_priv: i915 device instance
 * @domain: power domain to reference
 *
 * This function drops the power domain reference obtained by
 * intel_display_power_get() and might power down the corresponding hardware
 * block right away if this is the last reference.
 *
 * This function is only for the power domain code's internal use to suppress wakeref
 * tracking when the correspondig debug kconfig option is disabled, should not
 * be used otherwise.
 */
void intel_display_power_put_unchecked(struct drm_i915_private *dev_priv,
				       enum intel_display_power_domain domain)
{
	__intel_display_power_put(dev_priv, domain);
	intel_runtime_pm_put_unchecked(&dev_priv->runtime_pm);
}
#endif

void
intel_display_power_get_in_set(struct drm_i915_private *i915,
			       struct intel_display_power_domain_set *power_domain_set,
			       enum intel_display_power_domain domain)
{
	intel_wakeref_t __maybe_unused wf;

	drm_WARN_ON(&i915->drm, test_bit(domain, power_domain_set->mask.bits));

	wf = intel_display_power_get(i915, domain);
#if IS_ENABLED(CONFIG_DRM_I915_DEBUG_RUNTIME_PM)
	power_domain_set->wakerefs[domain] = wf;
#endif
	set_bit(domain, power_domain_set->mask.bits);
}

bool
intel_display_power_get_in_set_if_enabled(struct drm_i915_private *i915,
					  struct intel_display_power_domain_set *power_domain_set,
					  enum intel_display_power_domain domain)
{
	intel_wakeref_t wf;

	drm_WARN_ON(&i915->drm, test_bit(domain, power_domain_set->mask.bits));

	wf = intel_display_power_get_if_enabled(i915, domain);
	if (!wf)
		return false;

#if IS_ENABLED(CONFIG_DRM_I915_DEBUG_RUNTIME_PM)
	power_domain_set->wakerefs[domain] = wf;
#endif
	set_bit(domain, power_domain_set->mask.bits);

	return true;
}

void
intel_display_power_put_mask_in_set(struct drm_i915_private *i915,
				    struct intel_display_power_domain_set *power_domain_set,
				    struct intel_power_domain_mask *mask)
{
	enum intel_display_power_domain domain;

	drm_WARN_ON(&i915->drm,
		    !bitmap_subset(mask->bits, power_domain_set->mask.bits, POWER_DOMAIN_NUM));

	for_each_power_domain(domain, mask) {
		intel_wakeref_t __maybe_unused wf = -1;

#if IS_ENABLED(CONFIG_DRM_I915_DEBUG_RUNTIME_PM)
		wf = fetch_and_zero(&power_domain_set->wakerefs[domain]);
#endif
		intel_display_power_put(i915, domain, wf);
		clear_bit(domain, power_domain_set->mask.bits);
	}
}

static int
sanitize_disable_power_well_option(const struct drm_i915_private *dev_priv,
				   int disable_power_well)
{
	if (disable_power_well >= 0)
		return !!disable_power_well;

	return 1;
}

static u32 get_allowed_dc_mask(const struct drm_i915_private *dev_priv,
			       int enable_dc)
{
	u32 mask;
	int requested_dc;
	int max_dc;

	if (!HAS_DISPLAY(dev_priv))
		return 0;

	if (IS_DG2(dev_priv))
		max_dc = 1;
	else if (IS_DG1(dev_priv))
		max_dc = 3;
	else if (DISPLAY_VER(dev_priv) >= 12)
		max_dc = 4;
	else if (IS_GEMINILAKE(dev_priv) || IS_BROXTON(dev_priv))
		max_dc = 1;
	else if (DISPLAY_VER(dev_priv) >= 9)
		max_dc = 2;
	else
		max_dc = 0;

	/*
	 * DC9 has a separate HW flow from the rest of the DC states,
	 * not depending on the DMC firmware. It's needed by system
	 * suspend/resume, so allow it unconditionally.
	 */
	mask = IS_GEMINILAKE(dev_priv) || IS_BROXTON(dev_priv) ||
		DISPLAY_VER(dev_priv) >= 11 ?
	       DC_STATE_EN_DC9 : 0;

	if (!dev_priv->params.disable_power_well)
		max_dc = 0;

	if (enable_dc >= 0 && enable_dc <= max_dc) {
		requested_dc = enable_dc;
	} else if (enable_dc == -1) {
		requested_dc = max_dc;
	} else if (enable_dc > max_dc && enable_dc <= 4) {
		drm_dbg_kms(&dev_priv->drm,
			    "Adjusting requested max DC state (%d->%d)\n",
			    enable_dc, max_dc);
		requested_dc = max_dc;
	} else {
		drm_err(&dev_priv->drm,
			"Unexpected value for enable_dc (%d)\n", enable_dc);
		requested_dc = max_dc;
	}

	switch (requested_dc) {
	case 4:
		mask |= DC_STATE_EN_DC3CO | DC_STATE_EN_UPTO_DC6;
		break;
	case 3:
		mask |= DC_STATE_EN_DC3CO | DC_STATE_EN_UPTO_DC5;
		break;
	case 2:
		mask |= DC_STATE_EN_UPTO_DC6;
		break;
	case 1:
		mask |= DC_STATE_EN_UPTO_DC5;
		break;
	}

	drm_dbg_kms(&dev_priv->drm, "Allowed DC state mask %02x\n", mask);

	return mask;
}

/**
 * intel_power_domains_init - initializes the power domain structures
 * @dev_priv: i915 device instance
 *
 * Initializes the power domain structures for @dev_priv depending upon the
 * supported platform.
 */
int intel_power_domains_init(struct drm_i915_private *dev_priv)
{
	struct i915_power_domains *power_domains = &dev_priv->display.power.domains;

	dev_priv->params.disable_power_well =
		sanitize_disable_power_well_option(dev_priv,
						   dev_priv->params.disable_power_well);
	power_domains->allowed_dc_mask =
		get_allowed_dc_mask(dev_priv, dev_priv->params.enable_dc);

	power_domains->target_dc_state =
		sanitize_target_dc_state(dev_priv, DC_STATE_EN_UPTO_DC6);

	mutex_init(&power_domains->lock);

	INIT_DELAYED_WORK(&power_domains->async_put_work,
			  intel_display_power_put_async_work);

	return intel_display_power_map_init(power_domains);
}

/**
 * intel_power_domains_cleanup - clean up power domains resources
 * @dev_priv: i915 device instance
 *
 * Release any resources acquired by intel_power_domains_init()
 */
void intel_power_domains_cleanup(struct drm_i915_private *dev_priv)
{
<<<<<<< HEAD
	kfree(dev_priv->power_domains.power_wells);
	mutex_destroy(&dev_priv->power_domains.lock);
=======
	intel_display_power_map_cleanup(&dev_priv->display.power.domains);
>>>>>>> vendor/linux-drm-v6.6.35
}

static void intel_power_domains_sync_hw(struct drm_i915_private *dev_priv)
{
	struct i915_power_domains *power_domains = &dev_priv->display.power.domains;
	struct i915_power_well *power_well;

	mutex_lock(&power_domains->lock);
	for_each_power_well(dev_priv, power_well)
		intel_power_well_sync_hw(dev_priv, power_well);
	mutex_unlock(&power_domains->lock);
}

static void gen9_dbuf_slice_set(struct drm_i915_private *dev_priv,
				enum dbuf_slice slice, bool enable)
{
	i915_reg_t reg = DBUF_CTL_S(slice);
	bool state;

	intel_de_rmw(dev_priv, reg, DBUF_POWER_REQUEST,
		     enable ? DBUF_POWER_REQUEST : 0);
	intel_de_posting_read(dev_priv, reg);
	udelay(10);

	state = intel_de_read(dev_priv, reg) & DBUF_POWER_STATE;
	drm_WARN(&dev_priv->drm, enable != state,
		 "DBuf slice %d power %s timeout!\n",
		 slice, str_enable_disable(enable));
}

void gen9_dbuf_slices_update(struct drm_i915_private *dev_priv,
			     u8 req_slices)
{
	struct i915_power_domains *power_domains = &dev_priv->display.power.domains;
	u8 slice_mask = DISPLAY_INFO(dev_priv)->dbuf.slice_mask;
	enum dbuf_slice slice;

	drm_WARN(&dev_priv->drm, req_slices & ~slice_mask,
		 "Invalid set of dbuf slices (0x%x) requested (total dbuf slices 0x%x)\n",
		 req_slices, slice_mask);

	drm_dbg_kms(&dev_priv->drm, "Updating dbuf slices to 0x%x\n",
		    req_slices);

	/*
	 * Might be running this in parallel to gen9_dc_off_power_well_enable
	 * being called from intel_dp_detect for instance,
	 * which causes assertion triggered by race condition,
	 * as gen9_assert_dbuf_enabled might preempt this when registers
	 * were already updated, while dev_priv was not.
	 */
	mutex_lock(&power_domains->lock);

	for_each_dbuf_slice(dev_priv, slice)
		gen9_dbuf_slice_set(dev_priv, slice, req_slices & BIT(slice));

	dev_priv->display.dbuf.enabled_slices = req_slices;

	mutex_unlock(&power_domains->lock);
}

static void gen9_dbuf_enable(struct drm_i915_private *dev_priv)
{
	u8 slices_mask;

	dev_priv->display.dbuf.enabled_slices =
		intel_enabled_dbuf_slices_mask(dev_priv);

	slices_mask = BIT(DBUF_S1) | dev_priv->display.dbuf.enabled_slices;

	if (DISPLAY_VER(dev_priv) >= 14)
		intel_pmdemand_program_dbuf(dev_priv, slices_mask);

	/*
	 * Just power up at least 1 slice, we will
	 * figure out later which slices we have and what we need.
	 */
	gen9_dbuf_slices_update(dev_priv, slices_mask);
}

static void gen9_dbuf_disable(struct drm_i915_private *dev_priv)
{
	gen9_dbuf_slices_update(dev_priv, 0);

	if (DISPLAY_VER(dev_priv) >= 14)
		intel_pmdemand_program_dbuf(dev_priv, 0);
}

static void gen12_dbuf_slices_config(struct drm_i915_private *dev_priv)
{
	enum dbuf_slice slice;

	if (IS_ALDERLAKE_P(dev_priv))
		return;

	for_each_dbuf_slice(dev_priv, slice)
		intel_de_rmw(dev_priv, DBUF_CTL_S(slice),
			     DBUF_TRACKER_STATE_SERVICE_MASK,
			     DBUF_TRACKER_STATE_SERVICE(8));
}

static void icl_mbus_init(struct drm_i915_private *dev_priv)
{
	unsigned long abox_regs = DISPLAY_INFO(dev_priv)->abox_mask;
	u32 mask, val, i;

	if (IS_ALDERLAKE_P(dev_priv) || DISPLAY_VER(dev_priv) >= 14)
		return;

	mask = MBUS_ABOX_BT_CREDIT_POOL1_MASK |
		MBUS_ABOX_BT_CREDIT_POOL2_MASK |
		MBUS_ABOX_B_CREDIT_MASK |
		MBUS_ABOX_BW_CREDIT_MASK;
	val = MBUS_ABOX_BT_CREDIT_POOL1(16) |
		MBUS_ABOX_BT_CREDIT_POOL2(16) |
		MBUS_ABOX_B_CREDIT(1) |
		MBUS_ABOX_BW_CREDIT(1);

	/*
	 * gen12 platforms that use abox1 and abox2 for pixel data reads still
	 * expect us to program the abox_ctl0 register as well, even though
	 * we don't have to program other instance-0 registers like BW_BUDDY.
	 */
	if (DISPLAY_VER(dev_priv) == 12)
		abox_regs |= BIT(0);

	for_each_set_bit(i, &abox_regs, sizeof(abox_regs))
		intel_de_rmw(dev_priv, MBUS_ABOX_CTL(i), mask, val);
}

static void hsw_assert_cdclk(struct drm_i915_private *dev_priv)
{
	u32 val = intel_de_read(dev_priv, LCPLL_CTL);

	/*
	 * The LCPLL register should be turned on by the BIOS. For now
	 * let's just check its state and print errors in case
	 * something is wrong.  Don't even try to turn it on.
	 */

	if (val & LCPLL_CD_SOURCE_FCLK)
		drm_err(&dev_priv->drm, "CDCLK source is not LCPLL\n");

	if (val & LCPLL_PLL_DISABLE)
		drm_err(&dev_priv->drm, "LCPLL is disabled\n");

	if ((val & LCPLL_REF_MASK) != LCPLL_REF_NON_SSC)
		drm_err(&dev_priv->drm, "LCPLL not using non-SSC reference\n");
}

static void assert_can_disable_lcpll(struct drm_i915_private *dev_priv)
{
	struct intel_crtc *crtc;

	for_each_intel_crtc(&dev_priv->drm, crtc)
		I915_STATE_WARN(dev_priv, crtc->active,
				"CRTC for pipe %c enabled\n",
				pipe_name(crtc->pipe));

	I915_STATE_WARN(dev_priv, intel_de_read(dev_priv, HSW_PWR_WELL_CTL2),
			"Display power well on\n");
	I915_STATE_WARN(dev_priv,
			intel_de_read(dev_priv, SPLL_CTL) & SPLL_PLL_ENABLE,
			"SPLL enabled\n");
	I915_STATE_WARN(dev_priv,
			intel_de_read(dev_priv, WRPLL_CTL(0)) & WRPLL_PLL_ENABLE,
			"WRPLL1 enabled\n");
	I915_STATE_WARN(dev_priv,
			intel_de_read(dev_priv, WRPLL_CTL(1)) & WRPLL_PLL_ENABLE,
			"WRPLL2 enabled\n");
	I915_STATE_WARN(dev_priv,
			intel_de_read(dev_priv, PP_STATUS(0)) & PP_ON,
			"Panel power on\n");
	I915_STATE_WARN(dev_priv,
			intel_de_read(dev_priv, BLC_PWM_CPU_CTL2) & BLM_PWM_ENABLE,
			"CPU PWM1 enabled\n");
	if (IS_HASWELL(dev_priv))
		I915_STATE_WARN(dev_priv,
				intel_de_read(dev_priv, HSW_BLC_PWM2_CTL) & BLM_PWM_ENABLE,
				"CPU PWM2 enabled\n");
	I915_STATE_WARN(dev_priv,
			intel_de_read(dev_priv, BLC_PWM_PCH_CTL1) & BLM_PCH_PWM_ENABLE,
			"PCH PWM1 enabled\n");
	I915_STATE_WARN(dev_priv,
			(intel_de_read(dev_priv, UTIL_PIN_CTL) & (UTIL_PIN_ENABLE | UTIL_PIN_MODE_MASK)) == (UTIL_PIN_ENABLE | UTIL_PIN_MODE_PWM),
			"Utility pin enabled in PWM mode\n");
	I915_STATE_WARN(dev_priv,
			intel_de_read(dev_priv, PCH_GTC_CTL) & PCH_GTC_ENABLE,
			"PCH GTC enabled\n");

	/*
	 * In theory we can still leave IRQs enabled, as long as only the HPD
	 * interrupts remain enabled. We used to check for that, but since it's
	 * gen-specific and since we only disable LCPLL after we fully disable
	 * the interrupts, the check below should be enough.
	 */
	I915_STATE_WARN(dev_priv, intel_irqs_enabled(dev_priv),
			"IRQs enabled\n");
}

static u32 hsw_read_dcomp(struct drm_i915_private *dev_priv)
{
	if (IS_HASWELL(dev_priv))
		return intel_de_read(dev_priv, D_COMP_HSW);
	else
		return intel_de_read(dev_priv, D_COMP_BDW);
}

static void hsw_write_dcomp(struct drm_i915_private *dev_priv, u32 val)
{
	if (IS_HASWELL(dev_priv)) {
		if (snb_pcode_write(&dev_priv->uncore, GEN6_PCODE_WRITE_D_COMP, val))
			drm_dbg_kms(&dev_priv->drm,
				    "Failed to write to D_COMP\n");
	} else {
		intel_de_write(dev_priv, D_COMP_BDW, val);
		intel_de_posting_read(dev_priv, D_COMP_BDW);
	}
}

/*
 * This function implements pieces of two sequences from BSpec:
 * - Sequence for display software to disable LCPLL
 * - Sequence for display software to allow package C8+
 * The steps implemented here are just the steps that actually touch the LCPLL
 * register. Callers should take care of disabling all the display engine
 * functions, doing the mode unset, fixing interrupts, etc.
 */
static void hsw_disable_lcpll(struct drm_i915_private *dev_priv,
			      bool switch_to_fclk, bool allow_power_down)
{
	u32 val;

	assert_can_disable_lcpll(dev_priv);

	val = intel_de_read(dev_priv, LCPLL_CTL);

	if (switch_to_fclk) {
		val |= LCPLL_CD_SOURCE_FCLK;
		intel_de_write(dev_priv, LCPLL_CTL, val);

		if (wait_for_us(intel_de_read(dev_priv, LCPLL_CTL) &
				LCPLL_CD_SOURCE_FCLK_DONE, 1))
			drm_err(&dev_priv->drm, "Switching to FCLK failed\n");

		val = intel_de_read(dev_priv, LCPLL_CTL);
	}

	val |= LCPLL_PLL_DISABLE;
	intel_de_write(dev_priv, LCPLL_CTL, val);
	intel_de_posting_read(dev_priv, LCPLL_CTL);

	if (intel_de_wait_for_clear(dev_priv, LCPLL_CTL, LCPLL_PLL_LOCK, 1))
		drm_err(&dev_priv->drm, "LCPLL still locked\n");

	val = hsw_read_dcomp(dev_priv);
	val |= D_COMP_COMP_DISABLE;
	hsw_write_dcomp(dev_priv, val);
	ndelay(100);

	if (wait_for((hsw_read_dcomp(dev_priv) &
		      D_COMP_RCOMP_IN_PROGRESS) == 0, 1))
		drm_err(&dev_priv->drm, "D_COMP RCOMP still in progress\n");

	if (allow_power_down) {
		intel_de_rmw(dev_priv, LCPLL_CTL, 0, LCPLL_POWER_DOWN_ALLOW);
		intel_de_posting_read(dev_priv, LCPLL_CTL);
	}
}

/*
 * Fully restores LCPLL, disallowing power down and switching back to LCPLL
 * source.
 */
static void hsw_restore_lcpll(struct drm_i915_private *dev_priv)
{
	u32 val;

	val = intel_de_read(dev_priv, LCPLL_CTL);

	if ((val & (LCPLL_PLL_LOCK | LCPLL_PLL_DISABLE | LCPLL_CD_SOURCE_FCLK |
		    LCPLL_POWER_DOWN_ALLOW)) == LCPLL_PLL_LOCK)
		return;

	/*
	 * Make sure we're not on PC8 state before disabling PC8, otherwise
	 * we'll hang the machine. To prevent PC8 state, just enable force_wake.
	 */
	intel_uncore_forcewake_get(&dev_priv->uncore, FORCEWAKE_ALL);

	if (val & LCPLL_POWER_DOWN_ALLOW) {
		val &= ~LCPLL_POWER_DOWN_ALLOW;
		intel_de_write(dev_priv, LCPLL_CTL, val);
		intel_de_posting_read(dev_priv, LCPLL_CTL);
	}

	val = hsw_read_dcomp(dev_priv);
	val |= D_COMP_COMP_FORCE;
	val &= ~D_COMP_COMP_DISABLE;
	hsw_write_dcomp(dev_priv, val);

	val = intel_de_read(dev_priv, LCPLL_CTL);
	val &= ~LCPLL_PLL_DISABLE;
	intel_de_write(dev_priv, LCPLL_CTL, val);

	if (intel_de_wait_for_set(dev_priv, LCPLL_CTL, LCPLL_PLL_LOCK, 5))
		drm_err(&dev_priv->drm, "LCPLL not locked yet\n");

	if (val & LCPLL_CD_SOURCE_FCLK) {
		intel_de_rmw(dev_priv, LCPLL_CTL, LCPLL_CD_SOURCE_FCLK, 0);

		if (wait_for_us((intel_de_read(dev_priv, LCPLL_CTL) &
				 LCPLL_CD_SOURCE_FCLK_DONE) == 0, 1))
			drm_err(&dev_priv->drm,
				"Switching back to LCPLL failed\n");
	}

	intel_uncore_forcewake_put(&dev_priv->uncore, FORCEWAKE_ALL);

	intel_update_cdclk(dev_priv);
	intel_cdclk_dump_config(dev_priv, &dev_priv->display.cdclk.hw, "Current CDCLK");
}

/*
 * Package states C8 and deeper are really deep PC states that can only be
 * reached when all the devices on the system allow it, so even if the graphics
 * device allows PC8+, it doesn't mean the system will actually get to these
 * states. Our driver only allows PC8+ when going into runtime PM.
 *
 * The requirements for PC8+ are that all the outputs are disabled, the power
 * well is disabled and most interrupts are disabled, and these are also
 * requirements for runtime PM. When these conditions are met, we manually do
 * the other conditions: disable the interrupts, clocks and switch LCPLL refclk
 * to Fclk. If we're in PC8+ and we get an non-hotplug interrupt, we can hard
 * hang the machine.
 *
 * When we really reach PC8 or deeper states (not just when we allow it) we lose
 * the state of some registers, so when we come back from PC8+ we need to
 * restore this state. We don't get into PC8+ if we're not in RC6, so we don't
 * need to take care of the registers kept by RC6. Notice that this happens even
 * if we don't put the device in PCI D3 state (which is what currently happens
 * because of the runtime PM support).
 *
 * For more, read "Display Sequences for Package C8" on the hardware
 * documentation.
 */
static void hsw_enable_pc8(struct drm_i915_private *dev_priv)
{
	drm_dbg_kms(&dev_priv->drm, "Enabling package C8+\n");

	if (HAS_PCH_LPT_LP(dev_priv))
		intel_de_rmw(dev_priv, SOUTH_DSPCLK_GATE_D,
			     PCH_LP_PARTITION_LEVEL_DISABLE, 0);

	lpt_disable_clkout_dp(dev_priv);
	hsw_disable_lcpll(dev_priv, true, true);
}

static void hsw_disable_pc8(struct drm_i915_private *dev_priv)
{
	drm_dbg_kms(&dev_priv->drm, "Disabling package C8+\n");

	hsw_restore_lcpll(dev_priv);
	intel_init_pch_refclk(dev_priv);

	/* Many display registers don't survive PC8+ */
	intel_clock_gating_init(dev_priv);
}

static void intel_pch_reset_handshake(struct drm_i915_private *dev_priv,
				      bool enable)
{
	i915_reg_t reg;
	u32 reset_bits;

	if (IS_IVYBRIDGE(dev_priv)) {
		reg = GEN7_MSG_CTL;
		reset_bits = WAIT_FOR_PCH_FLR_ACK | WAIT_FOR_PCH_RESET_ACK;
	} else {
		reg = HSW_NDE_RSTWRN_OPT;
		reset_bits = RESET_PCH_HANDSHAKE_ENABLE;
	}

	if (DISPLAY_VER(dev_priv) >= 14)
		reset_bits |= MTL_RESET_PICA_HANDSHAKE_EN;

	intel_de_rmw(dev_priv, reg, reset_bits, enable ? reset_bits : 0);
}

static void skl_display_core_init(struct drm_i915_private *dev_priv,
				  bool resume)
{
	struct i915_power_domains *power_domains = &dev_priv->display.power.domains;
	struct i915_power_well *well;

	gen9_set_dc_state(dev_priv, DC_STATE_DISABLE);

	/* enable PCH reset handshake */
	intel_pch_reset_handshake(dev_priv, !HAS_PCH_NOP(dev_priv));

	if (!HAS_DISPLAY(dev_priv))
		return;

	/* enable PG1 and Misc I/O */
	mutex_lock(&power_domains->lock);

	well = lookup_power_well(dev_priv, SKL_DISP_PW_1);
	intel_power_well_enable(dev_priv, well);

	well = lookup_power_well(dev_priv, SKL_DISP_PW_MISC_IO);
	intel_power_well_enable(dev_priv, well);

	mutex_unlock(&power_domains->lock);

	intel_cdclk_init_hw(dev_priv);

	gen9_dbuf_enable(dev_priv);

	if (resume)
		intel_dmc_load_program(dev_priv);
}

static void skl_display_core_uninit(struct drm_i915_private *dev_priv)
{
	struct i915_power_domains *power_domains = &dev_priv->display.power.domains;
	struct i915_power_well *well;

	if (!HAS_DISPLAY(dev_priv))
		return;

	gen9_disable_dc_states(dev_priv);
	/* TODO: disable DMC program */

	gen9_dbuf_disable(dev_priv);

	intel_cdclk_uninit_hw(dev_priv);

	/* The spec doesn't call for removing the reset handshake flag */
	/* disable PG1 and Misc I/O */

	mutex_lock(&power_domains->lock);

	/*
	 * BSpec says to keep the MISC IO power well enabled here, only
	 * remove our request for power well 1.
	 * Note that even though the driver's request is removed power well 1
	 * may stay enabled after this due to DMC's own request on it.
	 */
	well = lookup_power_well(dev_priv, SKL_DISP_PW_1);
	intel_power_well_disable(dev_priv, well);

	mutex_unlock(&power_domains->lock);

	usleep_range(10, 30);		/* 10 us delay per Bspec */
}

static void bxt_display_core_init(struct drm_i915_private *dev_priv, bool resume)
{
	struct i915_power_domains *power_domains = &dev_priv->display.power.domains;
	struct i915_power_well *well;

	gen9_set_dc_state(dev_priv, DC_STATE_DISABLE);

	/*
	 * NDE_RSTWRN_OPT RST PCH Handshake En must always be 0b on BXT
	 * or else the reset will hang because there is no PCH to respond.
	 * Move the handshake programming to initialization sequence.
	 * Previously was left up to BIOS.
	 */
	intel_pch_reset_handshake(dev_priv, false);

	if (!HAS_DISPLAY(dev_priv))
		return;

	/* Enable PG1 */
	mutex_lock(&power_domains->lock);

	well = lookup_power_well(dev_priv, SKL_DISP_PW_1);
	intel_power_well_enable(dev_priv, well);

	mutex_unlock(&power_domains->lock);

	intel_cdclk_init_hw(dev_priv);

	gen9_dbuf_enable(dev_priv);

	if (resume)
		intel_dmc_load_program(dev_priv);
}

static void bxt_display_core_uninit(struct drm_i915_private *dev_priv)
{
	struct i915_power_domains *power_domains = &dev_priv->display.power.domains;
	struct i915_power_well *well;

	if (!HAS_DISPLAY(dev_priv))
		return;

	gen9_disable_dc_states(dev_priv);
	/* TODO: disable DMC program */

	gen9_dbuf_disable(dev_priv);

	intel_cdclk_uninit_hw(dev_priv);

	/* The spec doesn't call for removing the reset handshake flag */

	/*
	 * Disable PW1 (PG1).
	 * Note that even though the driver's request is removed power well 1
	 * may stay enabled after this due to DMC's own request on it.
	 */
	mutex_lock(&power_domains->lock);

	well = lookup_power_well(dev_priv, SKL_DISP_PW_1);
	intel_power_well_disable(dev_priv, well);

	mutex_unlock(&power_domains->lock);

	usleep_range(10, 30);		/* 10 us delay per Bspec */
}

struct buddy_page_mask {
	u32 page_mask;
	u8 type;
	u8 num_channels;
};

static const struct buddy_page_mask tgl_buddy_page_masks[] = {
	{ .num_channels = 1, .type = INTEL_DRAM_DDR4,   .page_mask = 0xF },
	{ .num_channels = 1, .type = INTEL_DRAM_DDR5,	.page_mask = 0xF },
	{ .num_channels = 2, .type = INTEL_DRAM_LPDDR4, .page_mask = 0x1C },
	{ .num_channels = 2, .type = INTEL_DRAM_LPDDR5, .page_mask = 0x1C },
	{ .num_channels = 2, .type = INTEL_DRAM_DDR4,   .page_mask = 0x1F },
	{ .num_channels = 2, .type = INTEL_DRAM_DDR5,   .page_mask = 0x1E },
	{ .num_channels = 4, .type = INTEL_DRAM_LPDDR4, .page_mask = 0x38 },
	{ .num_channels = 4, .type = INTEL_DRAM_LPDDR5, .page_mask = 0x38 },
	{}
};

static const struct buddy_page_mask wa_1409767108_buddy_page_masks[] = {
	{ .num_channels = 1, .type = INTEL_DRAM_LPDDR4, .page_mask = 0x1 },
	{ .num_channels = 1, .type = INTEL_DRAM_DDR4,   .page_mask = 0x1 },
	{ .num_channels = 1, .type = INTEL_DRAM_DDR5,   .page_mask = 0x1 },
	{ .num_channels = 1, .type = INTEL_DRAM_LPDDR5, .page_mask = 0x1 },
	{ .num_channels = 2, .type = INTEL_DRAM_LPDDR4, .page_mask = 0x3 },
	{ .num_channels = 2, .type = INTEL_DRAM_DDR4,   .page_mask = 0x3 },
	{ .num_channels = 2, .type = INTEL_DRAM_DDR5,   .page_mask = 0x3 },
	{ .num_channels = 2, .type = INTEL_DRAM_LPDDR5, .page_mask = 0x3 },
	{}
};

static void tgl_bw_buddy_init(struct drm_i915_private *dev_priv)
{
	enum intel_dram_type type = dev_priv->dram_info.type;
	u8 num_channels = dev_priv->dram_info.num_channels;
	const struct buddy_page_mask *table;
	unsigned long abox_mask = DISPLAY_INFO(dev_priv)->abox_mask;
	int config, i;

	/* BW_BUDDY registers are not used on dgpu's beyond DG1 */
	if (IS_DGFX(dev_priv) && !IS_DG1(dev_priv))
		return;

	if (IS_ALDERLAKE_S(dev_priv) ||
	    (IS_ROCKETLAKE(dev_priv) && IS_DISPLAY_STEP(dev_priv, STEP_A0, STEP_B0)))
		/* Wa_1409767108 */
		table = wa_1409767108_buddy_page_masks;
	else
		table = tgl_buddy_page_masks;

	for (config = 0; table[config].page_mask != 0; config++)
		if (table[config].num_channels == num_channels &&
		    table[config].type == type)
			break;

	if (table[config].page_mask == 0) {
		drm_dbg(&dev_priv->drm,
			"Unknown memory configuration; disabling address buddy logic.\n");
		for_each_set_bit(i, &abox_mask, sizeof(abox_mask))
			intel_de_write(dev_priv, BW_BUDDY_CTL(i),
				       BW_BUDDY_DISABLE);
	} else {
		for_each_set_bit(i, &abox_mask, sizeof(abox_mask)) {
			intel_de_write(dev_priv, BW_BUDDY_PAGE_MASK(i),
				       table[config].page_mask);

			/* Wa_22010178259:tgl,dg1,rkl,adl-s */
			if (DISPLAY_VER(dev_priv) == 12)
				intel_de_rmw(dev_priv, BW_BUDDY_CTL(i),
					     BW_BUDDY_TLB_REQ_TIMER_MASK,
					     BW_BUDDY_TLB_REQ_TIMER(0x8));
		}
	}
}

static void icl_display_core_init(struct drm_i915_private *dev_priv,
				  bool resume)
{
	struct i915_power_domains *power_domains = &dev_priv->display.power.domains;
	struct i915_power_well *well;

	gen9_set_dc_state(dev_priv, DC_STATE_DISABLE);

	/* Wa_14011294188:ehl,jsl,tgl,rkl,adl-s */
	if (INTEL_PCH_TYPE(dev_priv) >= PCH_TGP &&
	    INTEL_PCH_TYPE(dev_priv) < PCH_DG1)
		intel_de_rmw(dev_priv, SOUTH_DSPCLK_GATE_D, 0,
			     PCH_DPMGUNIT_CLOCK_GATE_DISABLE);

	/* 1. Enable PCH reset handshake. */
	intel_pch_reset_handshake(dev_priv, !HAS_PCH_NOP(dev_priv));

	if (!HAS_DISPLAY(dev_priv))
		return;

	/* 2. Initialize all combo phys */
	intel_combo_phy_init(dev_priv);

	/*
	 * 3. Enable Power Well 1 (PG1).
	 *    The AUX IO power wells will be enabled on demand.
	 */
	mutex_lock(&power_domains->lock);
	well = lookup_power_well(dev_priv, SKL_DISP_PW_1);
	intel_power_well_enable(dev_priv, well);
	mutex_unlock(&power_domains->lock);

	if (DISPLAY_VER(dev_priv) == 14)
		intel_de_rmw(dev_priv, DC_STATE_EN,
			     HOLD_PHY_PG1_LATCH | HOLD_PHY_CLKREQ_PG1_LATCH, 0);

	/* 4. Enable CDCLK. */
	intel_cdclk_init_hw(dev_priv);

	if (DISPLAY_VER(dev_priv) >= 12)
		gen12_dbuf_slices_config(dev_priv);

	/* 5. Enable DBUF. */
	gen9_dbuf_enable(dev_priv);

	/* 6. Setup MBUS. */
	icl_mbus_init(dev_priv);

	/* 7. Program arbiter BW_BUDDY registers */
	if (DISPLAY_VER(dev_priv) >= 12)
		tgl_bw_buddy_init(dev_priv);

	/* 8. Ensure PHYs have completed calibration and adaptation */
	if (IS_DG2(dev_priv))
		intel_snps_phy_wait_for_calibration(dev_priv);

	if (resume)
		intel_dmc_load_program(dev_priv);

	/* Wa_14011508470:tgl,dg1,rkl,adl-s,adl-p */
	if (DISPLAY_VER(dev_priv) >= 12)
		intel_de_rmw(dev_priv, GEN11_CHICKEN_DCPR_2, 0,
			     DCPR_CLEAR_MEMSTAT_DIS | DCPR_SEND_RESP_IMM |
			     DCPR_MASK_LPMODE | DCPR_MASK_MAXLATENCY_MEMUP_CLR);

	/* Wa_14011503030:xelpd */
	if (DISPLAY_VER(dev_priv) >= 13)
		intel_de_write(dev_priv, XELPD_DISPLAY_ERR_FATAL_MASK, ~0);
}

static void icl_display_core_uninit(struct drm_i915_private *dev_priv)
{
	struct i915_power_domains *power_domains = &dev_priv->display.power.domains;
	struct i915_power_well *well;

	if (!HAS_DISPLAY(dev_priv))
		return;

	gen9_disable_dc_states(dev_priv);
	intel_dmc_disable_program(dev_priv);

	/* 1. Disable all display engine functions -> aready done */

	/* 2. Disable DBUF */
	gen9_dbuf_disable(dev_priv);

	/* 3. Disable CD clock */
	intel_cdclk_uninit_hw(dev_priv);

	if (DISPLAY_VER(dev_priv) == 14)
		intel_de_rmw(dev_priv, DC_STATE_EN, 0,
			     HOLD_PHY_PG1_LATCH | HOLD_PHY_CLKREQ_PG1_LATCH);

	/*
	 * 4. Disable Power Well 1 (PG1).
	 *    The AUX IO power wells are toggled on demand, so they are already
	 *    disabled at this point.
	 */
	mutex_lock(&power_domains->lock);
	well = lookup_power_well(dev_priv, SKL_DISP_PW_1);
	intel_power_well_disable(dev_priv, well);
	mutex_unlock(&power_domains->lock);

	/* 5. */
	intel_combo_phy_uninit(dev_priv);
}

static void chv_phy_control_init(struct drm_i915_private *dev_priv)
{
	struct i915_power_well *cmn_bc =
		lookup_power_well(dev_priv, VLV_DISP_PW_DPIO_CMN_BC);
	struct i915_power_well *cmn_d =
		lookup_power_well(dev_priv, CHV_DISP_PW_DPIO_CMN_D);

	/*
	 * DISPLAY_PHY_CONTROL can get corrupted if read. As a
	 * workaround never ever read DISPLAY_PHY_CONTROL, and
	 * instead maintain a shadow copy ourselves. Use the actual
	 * power well state and lane status to reconstruct the
	 * expected initial value.
	 */
	dev_priv->display.power.chv_phy_control =
		PHY_LDO_SEQ_DELAY(PHY_LDO_DELAY_600NS, DPIO_PHY0) |
		PHY_LDO_SEQ_DELAY(PHY_LDO_DELAY_600NS, DPIO_PHY1) |
		PHY_CH_POWER_MODE(PHY_CH_DEEP_PSR, DPIO_PHY0, DPIO_CH0) |
		PHY_CH_POWER_MODE(PHY_CH_DEEP_PSR, DPIO_PHY0, DPIO_CH1) |
		PHY_CH_POWER_MODE(PHY_CH_DEEP_PSR, DPIO_PHY1, DPIO_CH0);

	/*
	 * If all lanes are disabled we leave the override disabled
	 * with all power down bits cleared to match the state we
	 * would use after disabling the port. Otherwise enable the
	 * override and set the lane powerdown bits accding to the
	 * current lane status.
	 */
	if (intel_power_well_is_enabled(dev_priv, cmn_bc)) {
		u32 status = intel_de_read(dev_priv, DPLL(PIPE_A));
		unsigned int mask;

		mask = status & DPLL_PORTB_READY_MASK;
		if (mask == 0xf)
			mask = 0x0;
		else
			dev_priv->display.power.chv_phy_control |=
				PHY_CH_POWER_DOWN_OVRD_EN(DPIO_PHY0, DPIO_CH0);

		dev_priv->display.power.chv_phy_control |=
			PHY_CH_POWER_DOWN_OVRD(mask, DPIO_PHY0, DPIO_CH0);

		mask = (status & DPLL_PORTC_READY_MASK) >> 4;
		if (mask == 0xf)
			mask = 0x0;
		else
			dev_priv->display.power.chv_phy_control |=
				PHY_CH_POWER_DOWN_OVRD_EN(DPIO_PHY0, DPIO_CH1);

		dev_priv->display.power.chv_phy_control |=
			PHY_CH_POWER_DOWN_OVRD(mask, DPIO_PHY0, DPIO_CH1);

		dev_priv->display.power.chv_phy_control |= PHY_COM_LANE_RESET_DEASSERT(DPIO_PHY0);

		dev_priv->display.power.chv_phy_assert[DPIO_PHY0] = false;
	} else {
		dev_priv->display.power.chv_phy_assert[DPIO_PHY0] = true;
	}

	if (intel_power_well_is_enabled(dev_priv, cmn_d)) {
		u32 status = intel_de_read(dev_priv, DPIO_PHY_STATUS);
		unsigned int mask;

		mask = status & DPLL_PORTD_READY_MASK;

		if (mask == 0xf)
			mask = 0x0;
		else
			dev_priv->display.power.chv_phy_control |=
				PHY_CH_POWER_DOWN_OVRD_EN(DPIO_PHY1, DPIO_CH0);

		dev_priv->display.power.chv_phy_control |=
			PHY_CH_POWER_DOWN_OVRD(mask, DPIO_PHY1, DPIO_CH0);

		dev_priv->display.power.chv_phy_control |= PHY_COM_LANE_RESET_DEASSERT(DPIO_PHY1);

		dev_priv->display.power.chv_phy_assert[DPIO_PHY1] = false;
	} else {
		dev_priv->display.power.chv_phy_assert[DPIO_PHY1] = true;
	}

	drm_dbg_kms(&dev_priv->drm, "Initial PHY_CONTROL=0x%08x\n",
		    dev_priv->display.power.chv_phy_control);

	/* Defer application of initial phy_control to enabling the powerwell */
}

static void vlv_cmnlane_wa(struct drm_i915_private *dev_priv)
{
	struct i915_power_well *cmn =
		lookup_power_well(dev_priv, VLV_DISP_PW_DPIO_CMN_BC);
	struct i915_power_well *disp2d =
		lookup_power_well(dev_priv, VLV_DISP_PW_DISP2D);

	/* If the display might be already active skip this */
	if (intel_power_well_is_enabled(dev_priv, cmn) &&
	    intel_power_well_is_enabled(dev_priv, disp2d) &&
	    intel_de_read(dev_priv, DPIO_CTL) & DPIO_CMNRST)
		return;

	drm_dbg_kms(&dev_priv->drm, "toggling display PHY side reset\n");

	/* cmnlane needs DPLL registers */
	intel_power_well_enable(dev_priv, disp2d);

	/*
	 * From VLV2A0_DP_eDP_HDMI_DPIO_driver_vbios_notes_11.docx:
	 * Need to assert and de-assert PHY SB reset by gating the
	 * common lane power, then un-gating it.
	 * Simply ungating isn't enough to reset the PHY enough to get
	 * ports and lanes running.
	 */
	intel_power_well_disable(dev_priv, cmn);
}

static bool vlv_punit_is_power_gated(struct drm_i915_private *dev_priv, u32 reg0)
{
	bool ret;

	vlv_punit_get(dev_priv);
	ret = (vlv_punit_read(dev_priv, reg0) & SSPM0_SSC_MASK) == SSPM0_SSC_PWR_GATE;
	vlv_punit_put(dev_priv);

	return ret;
}

static void assert_ved_power_gated(struct drm_i915_private *dev_priv)
{
	drm_WARN(&dev_priv->drm,
		 !vlv_punit_is_power_gated(dev_priv, PUNIT_REG_VEDSSPM0),
		 "VED not power gated\n");
}

static void assert_isp_power_gated(struct drm_i915_private *dev_priv)
{
	static const struct pci_device_id isp_ids[] = {
		{PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x0f38)},
		{PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x22b8)},
		{}
	};

	drm_WARN(&dev_priv->drm, !pci_dev_present(isp_ids) &&
		 !vlv_punit_is_power_gated(dev_priv, PUNIT_REG_ISPSSPM0),
		 "ISP not power gated\n");
}

static void intel_power_domains_verify_state(struct drm_i915_private *dev_priv);

/**
 * intel_power_domains_init_hw - initialize hardware power domain state
 * @i915: i915 device instance
 * @resume: Called from resume code paths or not
 *
 * This function initializes the hardware power domain state and enables all
 * power wells belonging to the INIT power domain. Power wells in other
 * domains (and not in the INIT domain) are referenced or disabled by
 * intel_modeset_readout_hw_state(). After that the reference count of each
 * power well must match its HW enabled state, see
 * intel_power_domains_verify_state().
 *
 * It will return with power domains disabled (to be enabled later by
 * intel_power_domains_enable()) and must be paired with
 * intel_power_domains_driver_remove().
 */
void intel_power_domains_init_hw(struct drm_i915_private *i915, bool resume)
{
	struct i915_power_domains *power_domains = &i915->display.power.domains;

	power_domains->initializing = true;

	if (DISPLAY_VER(i915) >= 11) {
		icl_display_core_init(i915, resume);
	} else if (IS_GEMINILAKE(i915) || IS_BROXTON(i915)) {
		bxt_display_core_init(i915, resume);
	} else if (DISPLAY_VER(i915) == 9) {
		skl_display_core_init(i915, resume);
	} else if (IS_CHERRYVIEW(i915)) {
		mutex_lock(&power_domains->lock);
		chv_phy_control_init(i915);
		mutex_unlock(&power_domains->lock);
		assert_isp_power_gated(i915);
	} else if (IS_VALLEYVIEW(i915)) {
		mutex_lock(&power_domains->lock);
		vlv_cmnlane_wa(i915);
		mutex_unlock(&power_domains->lock);
		assert_ved_power_gated(i915);
		assert_isp_power_gated(i915);
	} else if (IS_BROADWELL(i915) || IS_HASWELL(i915)) {
		hsw_assert_cdclk(i915);
		intel_pch_reset_handshake(i915, !HAS_PCH_NOP(i915));
	} else if (IS_IVYBRIDGE(i915)) {
		intel_pch_reset_handshake(i915, !HAS_PCH_NOP(i915));
	}

	/*
	 * Keep all power wells enabled for any dependent HW access during
	 * initialization and to make sure we keep BIOS enabled display HW
	 * resources powered until display HW readout is complete. We drop
	 * this reference in intel_power_domains_enable().
	 */
	drm_WARN_ON(&i915->drm, power_domains->init_wakeref);
	power_domains->init_wakeref =
		intel_display_power_get(i915, POWER_DOMAIN_INIT);

	/* Disable power support if the user asked so. */
	if (!i915->params.disable_power_well) {
		drm_WARN_ON(&i915->drm, power_domains->disable_wakeref);
		i915->display.power.domains.disable_wakeref = intel_display_power_get(i915,
										      POWER_DOMAIN_INIT);
	}
	intel_power_domains_sync_hw(i915);

	power_domains->initializing = false;
}

/**
 * intel_power_domains_driver_remove - deinitialize hw power domain state
 * @i915: i915 device instance
 *
 * De-initializes the display power domain HW state. It also ensures that the
 * device stays powered up so that the driver can be reloaded.
 *
 * It must be called with power domains already disabled (after a call to
 * intel_power_domains_disable()) and must be paired with
 * intel_power_domains_init_hw().
 */
void intel_power_domains_driver_remove(struct drm_i915_private *i915)
{
	intel_wakeref_t wakeref __maybe_unused =
		fetch_and_zero(&i915->display.power.domains.init_wakeref);

	/* Remove the refcount we took to keep power well support disabled. */
	if (!i915->params.disable_power_well)
		intel_display_power_put(i915, POWER_DOMAIN_INIT,
					fetch_and_zero(&i915->display.power.domains.disable_wakeref));

	intel_display_power_flush_work_sync(i915);

	intel_power_domains_verify_state(i915);

	/* Keep the power well enabled, but cancel its rpm wakeref. */
	intel_runtime_pm_put(&i915->runtime_pm, wakeref);
}

/**
 * intel_power_domains_sanitize_state - sanitize power domains state
 * @i915: i915 device instance
 *
 * Sanitize the power domains state during driver loading and system resume.
 * The function will disable all display power wells that BIOS has enabled
 * without a user for it (any user for a power well has taken a reference
 * on it by the time this function is called, after the state of all the
 * pipe, encoder, etc. HW resources have been sanitized).
 */
void intel_power_domains_sanitize_state(struct drm_i915_private *i915)
{
	struct i915_power_domains *power_domains = &i915->display.power.domains;
	struct i915_power_well *power_well;

	mutex_lock(&power_domains->lock);

	for_each_power_well_reverse(i915, power_well) {
		if (power_well->desc->always_on || power_well->count ||
		    !intel_power_well_is_enabled(i915, power_well))
			continue;

		drm_dbg_kms(&i915->drm,
			    "BIOS left unused %s power well enabled, disabling it\n",
			    intel_power_well_name(power_well));
		intel_power_well_disable(i915, power_well);
	}

	mutex_unlock(&power_domains->lock);
}

/**
 * intel_power_domains_enable - enable toggling of display power wells
 * @i915: i915 device instance
 *
 * Enable the ondemand enabling/disabling of the display power wells. Note that
 * power wells not belonging to POWER_DOMAIN_INIT are allowed to be toggled
 * only at specific points of the display modeset sequence, thus they are not
 * affected by the intel_power_domains_enable()/disable() calls. The purpose
 * of these function is to keep the rest of power wells enabled until the end
 * of display HW readout (which will acquire the power references reflecting
 * the current HW state).
 */
void intel_power_domains_enable(struct drm_i915_private *i915)
{
	intel_wakeref_t wakeref __maybe_unused =
		fetch_and_zero(&i915->display.power.domains.init_wakeref);

	intel_display_power_put(i915, POWER_DOMAIN_INIT, wakeref);
	intel_power_domains_verify_state(i915);
}

/**
 * intel_power_domains_disable - disable toggling of display power wells
 * @i915: i915 device instance
 *
 * Disable the ondemand enabling/disabling of the display power wells. See
 * intel_power_domains_enable() for which power wells this call controls.
 */
void intel_power_domains_disable(struct drm_i915_private *i915)
{
	struct i915_power_domains *power_domains = &i915->display.power.domains;

	drm_WARN_ON(&i915->drm, power_domains->init_wakeref);
	power_domains->init_wakeref =
		intel_display_power_get(i915, POWER_DOMAIN_INIT);

	intel_power_domains_verify_state(i915);
}

/**
 * intel_power_domains_suspend - suspend power domain state
 * @i915: i915 device instance
 * @s2idle: specifies whether we go to idle, or deeper sleep
 *
 * This function prepares the hardware power domain state before entering
 * system suspend.
 *
 * It must be called with power domains already disabled (after a call to
 * intel_power_domains_disable()) and paired with intel_power_domains_resume().
 */
void intel_power_domains_suspend(struct drm_i915_private *i915, bool s2idle)
{
	struct i915_power_domains *power_domains = &i915->display.power.domains;
	intel_wakeref_t wakeref __maybe_unused =
		fetch_and_zero(&power_domains->init_wakeref);

	intel_display_power_put(i915, POWER_DOMAIN_INIT, wakeref);

	/*
	 * In case of suspend-to-idle (aka S0ix) on a DMC platform without DC9
	 * support don't manually deinit the power domains. This also means the
	 * DMC firmware will stay active, it will power down any HW
	 * resources as required and also enable deeper system power states
	 * that would be blocked if the firmware was inactive.
	 */
	if (!(power_domains->allowed_dc_mask & DC_STATE_EN_DC9) && s2idle &&
	    intel_dmc_has_payload(i915)) {
		intel_display_power_flush_work(i915);
		intel_power_domains_verify_state(i915);
		return;
	}

	/*
	 * Even if power well support was disabled we still want to disable
	 * power wells if power domains must be deinitialized for suspend.
	 */
	if (!i915->params.disable_power_well)
		intel_display_power_put(i915, POWER_DOMAIN_INIT,
					fetch_and_zero(&i915->display.power.domains.disable_wakeref));

	intel_display_power_flush_work(i915);
	intel_power_domains_verify_state(i915);

	if (DISPLAY_VER(i915) >= 11)
		icl_display_core_uninit(i915);
	else if (IS_GEMINILAKE(i915) || IS_BROXTON(i915))
		bxt_display_core_uninit(i915);
	else if (DISPLAY_VER(i915) == 9)
		skl_display_core_uninit(i915);

	power_domains->display_core_suspended = true;
}

/**
 * intel_power_domains_resume - resume power domain state
 * @i915: i915 device instance
 *
 * This function resume the hardware power domain state during system resume.
 *
 * It will return with power domain support disabled (to be enabled later by
 * intel_power_domains_enable()) and must be paired with
 * intel_power_domains_suspend().
 */
void intel_power_domains_resume(struct drm_i915_private *i915)
{
	struct i915_power_domains *power_domains = &i915->display.power.domains;

	if (power_domains->display_core_suspended) {
		intel_power_domains_init_hw(i915, true);
		power_domains->display_core_suspended = false;
	} else {
		drm_WARN_ON(&i915->drm, power_domains->init_wakeref);
		power_domains->init_wakeref =
			intel_display_power_get(i915, POWER_DOMAIN_INIT);
	}

	intel_power_domains_verify_state(i915);
}

#if IS_ENABLED(CONFIG_DRM_I915_DEBUG_RUNTIME_PM)

static void intel_power_domains_dump_info(struct drm_i915_private *i915)
{
	struct i915_power_domains *power_domains = &i915->display.power.domains;
	struct i915_power_well *power_well;

	for_each_power_well(i915, power_well) {
		enum intel_display_power_domain domain;

		drm_dbg(&i915->drm, "%-25s %d\n",
			intel_power_well_name(power_well), intel_power_well_refcount(power_well));

		for_each_power_domain(domain, intel_power_well_domains(power_well))
			drm_dbg(&i915->drm, "  %-23s %d\n",
				intel_display_power_domain_str(domain),
				power_domains->domain_use_count[domain]);
	}
}

/**
 * intel_power_domains_verify_state - verify the HW/SW state for all power wells
 * @i915: i915 device instance
 *
 * Verify if the reference count of each power well matches its HW enabled
 * state and the total refcount of the domains it belongs to. This must be
 * called after modeset HW state sanitization, which is responsible for
 * acquiring reference counts for any power wells in use and disabling the
 * ones left on by BIOS but not required by any active output.
 */
static void intel_power_domains_verify_state(struct drm_i915_private *i915)
{
	struct i915_power_domains *power_domains = &i915->display.power.domains;
	struct i915_power_well *power_well;
	bool dump_domain_info;

	mutex_lock(&power_domains->lock);

	verify_async_put_domains_state(power_domains);

	dump_domain_info = false;
	for_each_power_well(i915, power_well) {
		enum intel_display_power_domain domain;
		int domains_count;
		bool enabled;

		enabled = intel_power_well_is_enabled(i915, power_well);
		if ((intel_power_well_refcount(power_well) ||
		     intel_power_well_is_always_on(power_well)) !=
		    enabled)
			drm_err(&i915->drm,
				"power well %s state mismatch (refcount %d/enabled %d)",
				intel_power_well_name(power_well),
				intel_power_well_refcount(power_well), enabled);

		domains_count = 0;
		for_each_power_domain(domain, intel_power_well_domains(power_well))
			domains_count += power_domains->domain_use_count[domain];

		if (intel_power_well_refcount(power_well) != domains_count) {
			drm_err(&i915->drm,
				"power well %s refcount/domain refcount mismatch "
				"(refcount %d/domains refcount %d)\n",
				intel_power_well_name(power_well),
				intel_power_well_refcount(power_well),
				domains_count);
			dump_domain_info = true;
		}
	}

	if (dump_domain_info) {
		static bool dumped;

		if (!dumped) {
			intel_power_domains_dump_info(i915);
			dumped = true;
		}
	}

	mutex_unlock(&power_domains->lock);
}

#else

static void intel_power_domains_verify_state(struct drm_i915_private *i915)
{
}

#endif

void intel_display_power_suspend_late(struct drm_i915_private *i915)
{
	if (DISPLAY_VER(i915) >= 11 || IS_GEMINILAKE(i915) ||
	    IS_BROXTON(i915)) {
		bxt_enable_dc9(i915);
	} else if (IS_HASWELL(i915) || IS_BROADWELL(i915)) {
		hsw_enable_pc8(i915);
	}

	/* Tweaked Wa_14010685332:cnp,icp,jsp,mcc,tgp,adp */
	if (INTEL_PCH_TYPE(i915) >= PCH_CNP && INTEL_PCH_TYPE(i915) < PCH_DG1)
		intel_de_rmw(i915, SOUTH_CHICKEN1, SBCLK_RUN_REFCLK_DIS, SBCLK_RUN_REFCLK_DIS);
}

void intel_display_power_resume_early(struct drm_i915_private *i915)
{
	if (DISPLAY_VER(i915) >= 11 || IS_GEMINILAKE(i915) ||
	    IS_BROXTON(i915)) {
		gen9_sanitize_dc_state(i915);
		bxt_disable_dc9(i915);
	} else if (IS_HASWELL(i915) || IS_BROADWELL(i915)) {
		hsw_disable_pc8(i915);
	}

	/* Tweaked Wa_14010685332:cnp,icp,jsp,mcc,tgp,adp */
	if (INTEL_PCH_TYPE(i915) >= PCH_CNP && INTEL_PCH_TYPE(i915) < PCH_DG1)
		intel_de_rmw(i915, SOUTH_CHICKEN1, SBCLK_RUN_REFCLK_DIS, 0);
}

void intel_display_power_suspend(struct drm_i915_private *i915)
{
	if (DISPLAY_VER(i915) >= 11) {
		icl_display_core_uninit(i915);
		bxt_enable_dc9(i915);
	} else if (IS_GEMINILAKE(i915) || IS_BROXTON(i915)) {
		bxt_display_core_uninit(i915);
		bxt_enable_dc9(i915);
	} else if (IS_HASWELL(i915) || IS_BROADWELL(i915)) {
		hsw_enable_pc8(i915);
	}
}

void intel_display_power_resume(struct drm_i915_private *i915)
{
	struct i915_power_domains *power_domains = &i915->display.power.domains;

	if (DISPLAY_VER(i915) >= 11) {
		bxt_disable_dc9(i915);
		icl_display_core_init(i915, true);
		if (intel_dmc_has_payload(i915)) {
			if (power_domains->allowed_dc_mask & DC_STATE_EN_UPTO_DC6)
				skl_enable_dc6(i915);
			else if (power_domains->allowed_dc_mask & DC_STATE_EN_UPTO_DC5)
				gen9_enable_dc5(i915);
		}
	} else if (IS_GEMINILAKE(i915) || IS_BROXTON(i915)) {
		bxt_disable_dc9(i915);
		bxt_display_core_init(i915, true);
		if (intel_dmc_has_payload(i915) &&
		    (power_domains->allowed_dc_mask & DC_STATE_EN_UPTO_DC5))
			gen9_enable_dc5(i915);
	} else if (IS_HASWELL(i915) || IS_BROADWELL(i915)) {
		hsw_disable_pc8(i915);
	}
}

void intel_display_power_debug(struct drm_i915_private *i915, struct seq_file *m)
{
	struct i915_power_domains *power_domains = &i915->display.power.domains;
	int i;

	mutex_lock(&power_domains->lock);

	seq_printf(m, "%-25s %s\n", "Power well/domain", "Use count");
	for (i = 0; i < power_domains->power_well_count; i++) {
		struct i915_power_well *power_well;
		enum intel_display_power_domain power_domain;

		power_well = &power_domains->power_wells[i];
		seq_printf(m, "%-25s %d\n", intel_power_well_name(power_well),
			   intel_power_well_refcount(power_well));

		for_each_power_domain(power_domain, intel_power_well_domains(power_well))
			seq_printf(m, "  %-23s %d\n",
				   intel_display_power_domain_str(power_domain),
				   power_domains->domain_use_count[power_domain]);
	}

	mutex_unlock(&power_domains->lock);
}

struct intel_ddi_port_domains {
	enum port port_start;
	enum port port_end;
	enum aux_ch aux_ch_start;
	enum aux_ch aux_ch_end;

	enum intel_display_power_domain ddi_lanes;
	enum intel_display_power_domain ddi_io;
	enum intel_display_power_domain aux_io;
	enum intel_display_power_domain aux_legacy_usbc;
	enum intel_display_power_domain aux_tbt;
};

static const struct intel_ddi_port_domains
i9xx_port_domains[] = {
	{
		.port_start = PORT_A,
		.port_end = PORT_F,
		.aux_ch_start = AUX_CH_A,
		.aux_ch_end = AUX_CH_F,

		.ddi_lanes = POWER_DOMAIN_PORT_DDI_LANES_A,
		.ddi_io = POWER_DOMAIN_PORT_DDI_IO_A,
		.aux_io = POWER_DOMAIN_AUX_IO_A,
		.aux_legacy_usbc = POWER_DOMAIN_AUX_A,
		.aux_tbt = POWER_DOMAIN_INVALID,
	},
};

static const struct intel_ddi_port_domains
d11_port_domains[] = {
	{
		.port_start = PORT_A,
		.port_end = PORT_B,
		.aux_ch_start = AUX_CH_A,
		.aux_ch_end = AUX_CH_B,

		.ddi_lanes = POWER_DOMAIN_PORT_DDI_LANES_A,
		.ddi_io = POWER_DOMAIN_PORT_DDI_IO_A,
		.aux_io = POWER_DOMAIN_AUX_IO_A,
		.aux_legacy_usbc = POWER_DOMAIN_AUX_A,
		.aux_tbt = POWER_DOMAIN_INVALID,
	}, {
		.port_start = PORT_C,
		.port_end = PORT_F,
		.aux_ch_start = AUX_CH_C,
		.aux_ch_end = AUX_CH_F,

		.ddi_lanes = POWER_DOMAIN_PORT_DDI_LANES_C,
		.ddi_io = POWER_DOMAIN_PORT_DDI_IO_C,
		.aux_io = POWER_DOMAIN_AUX_IO_C,
		.aux_legacy_usbc = POWER_DOMAIN_AUX_C,
		.aux_tbt = POWER_DOMAIN_AUX_TBT1,
	},
};

static const struct intel_ddi_port_domains
d12_port_domains[] = {
	{
		.port_start = PORT_A,
		.port_end = PORT_C,
		.aux_ch_start = AUX_CH_A,
		.aux_ch_end = AUX_CH_C,

		.ddi_lanes = POWER_DOMAIN_PORT_DDI_LANES_A,
		.ddi_io = POWER_DOMAIN_PORT_DDI_IO_A,
		.aux_io = POWER_DOMAIN_AUX_IO_A,
		.aux_legacy_usbc = POWER_DOMAIN_AUX_A,
		.aux_tbt = POWER_DOMAIN_INVALID,
	}, {
		.port_start = PORT_TC1,
		.port_end = PORT_TC6,
		.aux_ch_start = AUX_CH_USBC1,
		.aux_ch_end = AUX_CH_USBC6,

		.ddi_lanes = POWER_DOMAIN_PORT_DDI_LANES_TC1,
		.ddi_io = POWER_DOMAIN_PORT_DDI_IO_TC1,
		.aux_io = POWER_DOMAIN_INVALID,
		.aux_legacy_usbc = POWER_DOMAIN_AUX_USBC1,
		.aux_tbt = POWER_DOMAIN_AUX_TBT1,
	},
};

static const struct intel_ddi_port_domains
d13_port_domains[] = {
	{
		.port_start = PORT_A,
		.port_end = PORT_C,
		.aux_ch_start = AUX_CH_A,
		.aux_ch_end = AUX_CH_C,

		.ddi_lanes = POWER_DOMAIN_PORT_DDI_LANES_A,
		.ddi_io = POWER_DOMAIN_PORT_DDI_IO_A,
		.aux_io = POWER_DOMAIN_AUX_IO_A,
		.aux_legacy_usbc = POWER_DOMAIN_AUX_A,
		.aux_tbt = POWER_DOMAIN_INVALID,
	}, {
		.port_start = PORT_TC1,
		.port_end = PORT_TC4,
		.aux_ch_start = AUX_CH_USBC1,
		.aux_ch_end = AUX_CH_USBC4,

		.ddi_lanes = POWER_DOMAIN_PORT_DDI_LANES_TC1,
		.ddi_io = POWER_DOMAIN_PORT_DDI_IO_TC1,
		.aux_io = POWER_DOMAIN_INVALID,
		.aux_legacy_usbc = POWER_DOMAIN_AUX_USBC1,
		.aux_tbt = POWER_DOMAIN_AUX_TBT1,
	}, {
		.port_start = PORT_D_XELPD,
		.port_end = PORT_E_XELPD,
		.aux_ch_start = AUX_CH_D_XELPD,
		.aux_ch_end = AUX_CH_E_XELPD,

		.ddi_lanes = POWER_DOMAIN_PORT_DDI_LANES_D,
		.ddi_io = POWER_DOMAIN_PORT_DDI_IO_D,
		.aux_io = POWER_DOMAIN_AUX_IO_D,
		.aux_legacy_usbc = POWER_DOMAIN_AUX_D,
		.aux_tbt = POWER_DOMAIN_INVALID,
	},
};

static void
intel_port_domains_for_platform(struct drm_i915_private *i915,
				const struct intel_ddi_port_domains **domains,
				int *domains_size)
{
	if (DISPLAY_VER(i915) >= 13) {
		*domains = d13_port_domains;
		*domains_size = ARRAY_SIZE(d13_port_domains);
	} else if (DISPLAY_VER(i915) >= 12) {
		*domains = d12_port_domains;
		*domains_size = ARRAY_SIZE(d12_port_domains);
	} else if (DISPLAY_VER(i915) >= 11) {
		*domains = d11_port_domains;
		*domains_size = ARRAY_SIZE(d11_port_domains);
	} else {
		*domains = i9xx_port_domains;
		*domains_size = ARRAY_SIZE(i9xx_port_domains);
	}
}

static const struct intel_ddi_port_domains *
intel_port_domains_for_port(struct drm_i915_private *i915, enum port port)
{
	const struct intel_ddi_port_domains *domains;
	int domains_size;
	int i;

	intel_port_domains_for_platform(i915, &domains, &domains_size);
	for (i = 0; i < domains_size; i++)
		if (port >= domains[i].port_start && port <= domains[i].port_end)
			return &domains[i];

	return NULL;
}

enum intel_display_power_domain
intel_display_power_ddi_io_domain(struct drm_i915_private *i915, enum port port)
{
	const struct intel_ddi_port_domains *domains = intel_port_domains_for_port(i915, port);

	if (drm_WARN_ON(&i915->drm, !domains || domains->ddi_io == POWER_DOMAIN_INVALID))
		return POWER_DOMAIN_PORT_DDI_IO_A;

	return domains->ddi_io + (int)(port - domains->port_start);
}

enum intel_display_power_domain
intel_display_power_ddi_lanes_domain(struct drm_i915_private *i915, enum port port)
{
	const struct intel_ddi_port_domains *domains = intel_port_domains_for_port(i915, port);

	if (drm_WARN_ON(&i915->drm, !domains || domains->ddi_lanes == POWER_DOMAIN_INVALID))
		return POWER_DOMAIN_PORT_DDI_LANES_A;

	return domains->ddi_lanes + (int)(port - domains->port_start);
}

static const struct intel_ddi_port_domains *
intel_port_domains_for_aux_ch(struct drm_i915_private *i915, enum aux_ch aux_ch)
{
	const struct intel_ddi_port_domains *domains;
	int domains_size;
	int i;

	intel_port_domains_for_platform(i915, &domains, &domains_size);
	for (i = 0; i < domains_size; i++)
		if (aux_ch >= domains[i].aux_ch_start && aux_ch <= domains[i].aux_ch_end)
			return &domains[i];

	return NULL;
}

enum intel_display_power_domain
intel_display_power_aux_io_domain(struct drm_i915_private *i915, enum aux_ch aux_ch)
{
	const struct intel_ddi_port_domains *domains = intel_port_domains_for_aux_ch(i915, aux_ch);

	if (drm_WARN_ON(&i915->drm, !domains || domains->aux_io == POWER_DOMAIN_INVALID))
		return POWER_DOMAIN_AUX_IO_A;

	return domains->aux_io + (int)(aux_ch - domains->aux_ch_start);
}

enum intel_display_power_domain
intel_display_power_legacy_aux_domain(struct drm_i915_private *i915, enum aux_ch aux_ch)
{
	const struct intel_ddi_port_domains *domains = intel_port_domains_for_aux_ch(i915, aux_ch);

	if (drm_WARN_ON(&i915->drm, !domains || domains->aux_legacy_usbc == POWER_DOMAIN_INVALID))
		return POWER_DOMAIN_AUX_A;

	return domains->aux_legacy_usbc + (int)(aux_ch - domains->aux_ch_start);
}

enum intel_display_power_domain
intel_display_power_tbt_aux_domain(struct drm_i915_private *i915, enum aux_ch aux_ch)
{
	const struct intel_ddi_port_domains *domains = intel_port_domains_for_aux_ch(i915, aux_ch);

	if (drm_WARN_ON(&i915->drm, !domains || domains->aux_tbt == POWER_DOMAIN_INVALID))
		return POWER_DOMAIN_AUX_TBT1;

	return domains->aux_tbt + (int)(aux_ch - domains->aux_ch_start);
}
