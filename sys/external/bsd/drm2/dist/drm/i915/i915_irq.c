/*	$NetBSD: i915_irq.c,v 1.25 2021/12/19 11:45:01 riastradh Exp $	*/

/* i915_irq.c -- IRQ support for the I915 -*- linux-c -*-
 */
/*
 * Copyright 2003 Tungsten Graphics, Inc., Cedar Park, Texas.
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sub license, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * IN NO EVENT SHALL TUNGSTEN GRAPHICS AND/OR ITS SUPPLIERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: i915_irq.c,v 1.25 2021/12/19 11:45:01 riastradh Exp $");

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/slab.h>
#include <linux/sysrq.h>

#include <drm/drm_drv.h>

#include "display/intel_display_irq.h"
#include "display/intel_display_types.h"
#include "display/intel_hotplug.h"
#include "display/intel_hotplug_irq.h"
#include "display/intel_lpe_audio.h"
#include "display/intel_psr_regs.h"

#include "gt/intel_breadcrumbs.h"
#include "gt/intel_gt.h"
#include "gt/intel_gt_irq.h"
#include "gt/intel_gt_pm_irq.h"
#include "gt/intel_gt_regs.h"
#include "gt/intel_rps.h"

#include "i915_driver.h"
#include "i915_drv.h"
#include "i915_irq.h"
#include "i915_reg.h"

/**
 * DOC: interrupt handling
 *
 * These functions provide the basic support for enabling and disabling the
 * interrupt handling support. There's a lot more functionality in i915_irq.c
 * and related files, but that will be described in separate chapters.
 */

/*
 * Interrupt statistic for PMU. Increments the counter only if the
 * interrupt originated from the GPU so interrupts from a device which
 * shares the interrupt line are not accounted.
 */
static inline void pmu_irq_stats(struct drm_i915_private *i915,
				 irqreturn_t res)
{
	if (unlikely(res != IRQ_HANDLED))
		return;

	/*
	 * A clever compiler translates that into INC. A not so clever one
	 * should at least prevent store tearing.
	 */
	WRITE_ONCE(i915->pmu.irq_count, i915->pmu.irq_count + 1);
}

void gen3_irq_reset(struct intel_uncore *uncore, i915_reg_t imr,
		    i915_reg_t iir, i915_reg_t ier)
{
	intel_uncore_write(uncore, imr, 0xffffffff);
	intel_uncore_posting_read(uncore, imr);

	intel_uncore_write(uncore, ier, 0);

	/* IIR can theoretically queue up two events. Be paranoid. */
	intel_uncore_write(uncore, iir, 0xffffffff);
	intel_uncore_posting_read(uncore, iir);
	intel_uncore_write(uncore, iir, 0xffffffff);
	intel_uncore_posting_read(uncore, iir);
}

static void gen2_irq_reset(struct intel_uncore *uncore)
{
	intel_uncore_write16(uncore, GEN2_IMR, 0xffff);
	intel_uncore_posting_read16(uncore, GEN2_IMR);

	intel_uncore_write16(uncore, GEN2_IER, 0);

	/* IIR can theoretically queue up two events. Be paranoid. */
	intel_uncore_write16(uncore, GEN2_IIR, 0xffff);
	intel_uncore_posting_read16(uncore, GEN2_IIR);
	intel_uncore_write16(uncore, GEN2_IIR, 0xffff);
	intel_uncore_posting_read16(uncore, GEN2_IIR);
}

/*
 * We should clear IMR at preinstall/uninstall, and just check at postinstall.
 */
void gen3_assert_iir_is_zero(struct intel_uncore *uncore, i915_reg_t reg)
{
	u32 val = intel_uncore_read(uncore, reg);

	if (val == 0)
		return;

	drm_WARN(&uncore->i915->drm, 1,
		 "Interrupt register 0x%x is not zero: 0x%08x\n",
		 i915_mmio_reg_offset(reg), val);
	intel_uncore_write(uncore, reg, 0xffffffff);
	intel_uncore_posting_read(uncore, reg);
	intel_uncore_write(uncore, reg, 0xffffffff);
	intel_uncore_posting_read(uncore, reg);
}

static void gen2_assert_iir_is_zero(struct intel_uncore *uncore)
{
	u16 val = intel_uncore_read16(uncore, GEN2_IIR);

	if (val == 0)
		return;

	drm_WARN(&uncore->i915->drm, 1,
		 "Interrupt register 0x%x is not zero: 0x%08x\n",
		 i915_mmio_reg_offset(GEN2_IIR), val);
	intel_uncore_write16(uncore, GEN2_IIR, 0xffff);
	intel_uncore_posting_read16(uncore, GEN2_IIR);
	intel_uncore_write16(uncore, GEN2_IIR, 0xffff);
	intel_uncore_posting_read16(uncore, GEN2_IIR);
}

void gen3_irq_init(struct intel_uncore *uncore,
		   i915_reg_t imr, u32 imr_val,
		   i915_reg_t ier, u32 ier_val,
		   i915_reg_t iir)
{
	gen3_assert_iir_is_zero(uncore, iir);

	intel_uncore_write(uncore, ier, ier_val);
	intel_uncore_write(uncore, imr, imr_val);
	intel_uncore_posting_read(uncore, imr);
}

static void gen2_irq_init(struct intel_uncore *uncore,
			  u32 imr_val, u32 ier_val)
{
	gen2_assert_iir_is_zero(uncore);

	intel_uncore_write16(uncore, GEN2_IER, ier_val);
	intel_uncore_write16(uncore, GEN2_IMR, imr_val);
	intel_uncore_posting_read16(uncore, GEN2_IMR);
}

/**
 * ivb_parity_work - Workqueue called when a parity error interrupt
 * occurred.
 * @work: workqueue struct
 *
 * Doesn't actually do anything except notify userspace. As a consequence of
 * this event, userspace should try to remap the bad rows since statistically
 * it is likely the same row is more likely to go bad again.
 */
static void ivb_parity_work(struct work_struct *work)
{
	struct drm_i915_private *dev_priv =
		container_of(work, typeof(*dev_priv), l3_parity.error_work);
	struct intel_gt *gt = to_gt(dev_priv);
	u32 error_status, row, bank, subbank;
#ifndef __NetBSD__		/* XXX kobject uevent...? */
	char *parity_event[6];
#endif
	u32 misccpctl;
	u8 slice = 0;

	/* We must turn off DOP level clock gating to access the L3 registers.
	 * In order to prevent a get/put style interface, acquire struct mutex
	 * any time we access those registers.
	 */
	mutex_lock(&dev_priv->drm.struct_mutex);

	/* If we've screwed up tracking, just let the interrupt fire again */
	if (drm_WARN_ON(&dev_priv->drm, !dev_priv->l3_parity.which_slice))
		goto out;

	misccpctl = intel_uncore_rmw(&dev_priv->uncore, GEN7_MISCCPCTL,
				     GEN7_DOP_CLOCK_GATE_ENABLE, 0);
	intel_uncore_posting_read(&dev_priv->uncore, GEN7_MISCCPCTL);

	while ((slice = ffs(dev_priv->l3_parity.which_slice)) != 0) {
		i915_reg_t reg;

		slice--;
		if (drm_WARN_ON_ONCE(&dev_priv->drm,
				     slice >= NUM_L3_SLICES(dev_priv)))
			break;

		dev_priv->l3_parity.which_slice &= ~(1<<slice);

		reg = GEN7_L3CDERRST1(slice);

		error_status = intel_uncore_read(&dev_priv->uncore, reg);
		row = GEN7_PARITY_ERROR_ROW(error_status);
		bank = GEN7_PARITY_ERROR_BANK(error_status);
		subbank = GEN7_PARITY_ERROR_SUBBANK(error_status);

		intel_uncore_write(&dev_priv->uncore, reg, GEN7_PARITY_ERROR_VALID | GEN7_L3CDERRST1_ENABLE);
		intel_uncore_posting_read(&dev_priv->uncore, reg);

#ifndef __NetBSD__		/* XXX kobject uevent...? */
		parity_event[0] = I915_L3_PARITY_UEVENT "=1";
		parity_event[1] = kasprintf(GFP_KERNEL, "ROW=%d", row);
		parity_event[2] = kasprintf(GFP_KERNEL, "BANK=%d", bank);
		parity_event[3] = kasprintf(GFP_KERNEL, "SUBBANK=%d", subbank);
		parity_event[4] = kasprintf(GFP_KERNEL, "SLICE=%d", slice);
		parity_event[5] = NULL;

		kobject_uevent_env(&dev_priv->drm.primary->kdev->kobj,
				   KOBJ_CHANGE, parity_event);
#endif

		drm_dbg(&dev_priv->drm,
			"Parity error: Slice = %d, Row = %d, Bank = %d, Sub bank = %d.\n",
			slice, row, bank, subbank);

#ifndef __NetBSD__		/* XXX kobject uevent...? */
		kfree(parity_event[4]);
		kfree(parity_event[3]);
		kfree(parity_event[2]);
		kfree(parity_event[1]);
#endif
	}

	intel_uncore_write(&dev_priv->uncore, GEN7_MISCCPCTL, misccpctl);

out:
	drm_WARN_ON(&dev_priv->drm, dev_priv->l3_parity.which_slice);
	spin_lock_irq(gt->irq_lock);
	gen5_gt_enable_irq(gt, GT_PARITY_ERROR(dev_priv));
	spin_unlock_irq(gt->irq_lock);

	mutex_unlock(&dev_priv->drm.struct_mutex);
}

<<<<<<< HEAD
static bool gen11_port_hotplug_long_detect(enum hpd_pin pin, u32 val)
{
	switch (pin) {
	case HPD_PORT_C:
		return val & GEN11_HOTPLUG_CTL_LONG_DETECT(PORT_TC1);
	case HPD_PORT_D:
		return val & GEN11_HOTPLUG_CTL_LONG_DETECT(PORT_TC2);
	case HPD_PORT_E:
		return val & GEN11_HOTPLUG_CTL_LONG_DETECT(PORT_TC3);
	case HPD_PORT_F:
		return val & GEN11_HOTPLUG_CTL_LONG_DETECT(PORT_TC4);
	default:
		return false;
	}
}

static bool gen12_port_hotplug_long_detect(enum hpd_pin pin, u32 val)
{
	switch (pin) {
	case HPD_PORT_D:
		return val & GEN11_HOTPLUG_CTL_LONG_DETECT(PORT_TC1);
	case HPD_PORT_E:
		return val & GEN11_HOTPLUG_CTL_LONG_DETECT(PORT_TC2);
	case HPD_PORT_F:
		return val & GEN11_HOTPLUG_CTL_LONG_DETECT(PORT_TC3);
	case HPD_PORT_G:
		return val & GEN11_HOTPLUG_CTL_LONG_DETECT(PORT_TC4);
	case HPD_PORT_H:
		return val & GEN11_HOTPLUG_CTL_LONG_DETECT(PORT_TC5);
	case HPD_PORT_I:
		return val & GEN11_HOTPLUG_CTL_LONG_DETECT(PORT_TC6);
	default:
		return false;
	}
}

static bool bxt_port_hotplug_long_detect(enum hpd_pin pin, u32 val)
{
	switch (pin) {
	case HPD_PORT_A:
		return val & PORTA_HOTPLUG_LONG_DETECT;
	case HPD_PORT_B:
		return val & PORTB_HOTPLUG_LONG_DETECT;
	case HPD_PORT_C:
		return val & PORTC_HOTPLUG_LONG_DETECT;
	default:
		return false;
	}
}

static bool icp_ddi_port_hotplug_long_detect(enum hpd_pin pin, u32 val)
{
	switch (pin) {
	case HPD_PORT_A:
		return val & SHOTPLUG_CTL_DDI_HPD_LONG_DETECT(PORT_A);
	case HPD_PORT_B:
		return val & SHOTPLUG_CTL_DDI_HPD_LONG_DETECT(PORT_B);
	case HPD_PORT_C:
		return val & SHOTPLUG_CTL_DDI_HPD_LONG_DETECT(PORT_C);
	default:
		return false;
	}
}

static bool icp_tc_port_hotplug_long_detect(enum hpd_pin pin, u32 val)
{
	switch (pin) {
	case HPD_PORT_C:
		return val & ICP_TC_HPD_LONG_DETECT(PORT_TC1);
	case HPD_PORT_D:
		return val & ICP_TC_HPD_LONG_DETECT(PORT_TC2);
	case HPD_PORT_E:
		return val & ICP_TC_HPD_LONG_DETECT(PORT_TC3);
	case HPD_PORT_F:
		return val & ICP_TC_HPD_LONG_DETECT(PORT_TC4);
	default:
		return false;
	}
}

static bool tgp_tc_port_hotplug_long_detect(enum hpd_pin pin, u32 val)
{
	switch (pin) {
	case HPD_PORT_D:
		return val & ICP_TC_HPD_LONG_DETECT(PORT_TC1);
	case HPD_PORT_E:
		return val & ICP_TC_HPD_LONG_DETECT(PORT_TC2);
	case HPD_PORT_F:
		return val & ICP_TC_HPD_LONG_DETECT(PORT_TC3);
	case HPD_PORT_G:
		return val & ICP_TC_HPD_LONG_DETECT(PORT_TC4);
	case HPD_PORT_H:
		return val & ICP_TC_HPD_LONG_DETECT(PORT_TC5);
	case HPD_PORT_I:
		return val & ICP_TC_HPD_LONG_DETECT(PORT_TC6);
	default:
		return false;
	}
}

static bool spt_port_hotplug2_long_detect(enum hpd_pin pin, u32 val)
{
	switch (pin) {
	case HPD_PORT_E:
		return val & PORTE_HOTPLUG_LONG_DETECT;
	default:
		return false;
	}
}

static bool spt_port_hotplug_long_detect(enum hpd_pin pin, u32 val)
{
	switch (pin) {
	case HPD_PORT_A:
		return val & PORTA_HOTPLUG_LONG_DETECT;
	case HPD_PORT_B:
		return val & PORTB_HOTPLUG_LONG_DETECT;
	case HPD_PORT_C:
		return val & PORTC_HOTPLUG_LONG_DETECT;
	case HPD_PORT_D:
		return val & PORTD_HOTPLUG_LONG_DETECT;
	default:
		return false;
	}
}

static bool ilk_port_hotplug_long_detect(enum hpd_pin pin, u32 val)
{
	switch (pin) {
	case HPD_PORT_A:
		return val & DIGITAL_PORTA_HOTPLUG_LONG_DETECT;
	default:
		return false;
	}
}

static bool pch_port_hotplug_long_detect(enum hpd_pin pin, u32 val)
{
	switch (pin) {
	case HPD_PORT_B:
		return val & PORTB_HOTPLUG_LONG_DETECT;
	case HPD_PORT_C:
		return val & PORTC_HOTPLUG_LONG_DETECT;
	case HPD_PORT_D:
		return val & PORTD_HOTPLUG_LONG_DETECT;
	default:
		return false;
	}
}

static bool i9xx_port_hotplug_long_detect(enum hpd_pin pin, u32 val)
{
	switch (pin) {
	case HPD_PORT_B:
		return val & PORTB_HOTPLUG_INT_LONG_PULSE;
	case HPD_PORT_C:
		return val & PORTC_HOTPLUG_INT_LONG_PULSE;
	case HPD_PORT_D:
		return val & PORTD_HOTPLUG_INT_LONG_PULSE;
	default:
		return false;
	}
}

/*
 * Get a bit mask of pins that have triggered, and which ones may be long.
 * This can be called multiple times with the same masks to accumulate
 * hotplug detection results from several registers.
 *
 * Note that the caller is expected to zero out the masks initially.
 */
static void intel_get_hpd_pins(struct drm_i915_private *dev_priv,
			       u32 *pin_mask, u32 *long_mask,
			       u32 hotplug_trigger, u32 dig_hotplug_reg,
			       const u32 hpd[HPD_NUM_PINS],
			       bool long_pulse_detect(enum hpd_pin pin, u32 val))
{
	enum hpd_pin pin;

	BUILD_BUG_ON(BITS_PER_TYPE(*pin_mask) < HPD_NUM_PINS);

	for_each_hpd_pin(pin) {
		if ((hpd[pin] & hotplug_trigger) == 0)
			continue;

		*pin_mask |= BIT(pin);

		if (long_pulse_detect(pin, dig_hotplug_reg))
			*long_mask |= BIT(pin);
	}

	DRM_DEBUG_DRIVER("hotplug event received, stat 0x%08x, dig 0x%08x, pins 0x%08x, long 0x%08x\n",
			 hotplug_trigger, dig_hotplug_reg, *pin_mask, *long_mask);

}

static void gmbus_irq_handler(struct drm_i915_private *dev_priv)
{

	spin_lock(&dev_priv->gmbus_wait_lock);
	DRM_SPIN_WAKEUP_ALL(&dev_priv->gmbus_wait_queue,
	    &dev_priv->gmbus_wait_lock);
	spin_unlock(&dev_priv->gmbus_wait_lock);
}

static void dp_aux_irq_handler(struct drm_i915_private *dev_priv)
{

	spin_lock(&dev_priv->gmbus_wait_lock);
	DRM_SPIN_WAKEUP_ALL(&dev_priv->gmbus_wait_queue,
	    &dev_priv->gmbus_wait_lock);
	spin_unlock(&dev_priv->gmbus_wait_lock);
}

#if defined(CONFIG_DEBUG_FS)
static void display_pipe_crc_irq_handler(struct drm_i915_private *dev_priv,
					 enum pipe pipe,
					 u32 crc0, u32 crc1,
					 u32 crc2, u32 crc3,
					 u32 crc4)
{
	struct intel_pipe_crc *pipe_crc = &dev_priv->pipe_crc[pipe];
	struct intel_crtc *crtc = intel_get_crtc_for_pipe(dev_priv, pipe);
	u32 crcs[5] = { crc0, crc1, crc2, crc3, crc4 };

	trace_intel_pipe_crc(crtc, crcs);

	spin_lock(&pipe_crc->lock);
	/*
	 * For some not yet identified reason, the first CRC is
	 * bonkers. So let's just wait for the next vblank and read
	 * out the buggy result.
	 *
	 * On GEN8+ sometimes the second CRC is bonkers as well, so
	 * don't trust that one either.
	 */
	if (pipe_crc->skipped <= 0 ||
	    (INTEL_GEN(dev_priv) >= 8 && pipe_crc->skipped == 1)) {
		pipe_crc->skipped++;
		spin_unlock(&pipe_crc->lock);
		return;
	}
	spin_unlock(&pipe_crc->lock);

	drm_crtc_add_crc_entry(&crtc->base, true,
				drm_crtc_accurate_vblank_count(&crtc->base),
				crcs);
}
#else
static inline void
display_pipe_crc_irq_handler(struct drm_i915_private *dev_priv,
			     enum pipe pipe,
			     u32 crc0, u32 crc1,
			     u32 crc2, u32 crc3,
			     u32 crc4) {}
#endif


static void hsw_pipe_crc_irq_handler(struct drm_i915_private *dev_priv,
				     enum pipe pipe)
{
	display_pipe_crc_irq_handler(dev_priv, pipe,
				     I915_READ(PIPE_CRC_RES_1_IVB(pipe)),
				     0, 0, 0, 0);
}

static void ivb_pipe_crc_irq_handler(struct drm_i915_private *dev_priv,
				     enum pipe pipe)
{
	display_pipe_crc_irq_handler(dev_priv, pipe,
				     I915_READ(PIPE_CRC_RES_1_IVB(pipe)),
				     I915_READ(PIPE_CRC_RES_2_IVB(pipe)),
				     I915_READ(PIPE_CRC_RES_3_IVB(pipe)),
				     I915_READ(PIPE_CRC_RES_4_IVB(pipe)),
				     I915_READ(PIPE_CRC_RES_5_IVB(pipe)));
}

static void i9xx_pipe_crc_irq_handler(struct drm_i915_private *dev_priv,
				      enum pipe pipe)
{
	u32 res1, res2;

	if (INTEL_GEN(dev_priv) >= 3)
		res1 = I915_READ(PIPE_CRC_RES_RES1_I915(pipe));
	else
		res1 = 0;

	if (INTEL_GEN(dev_priv) >= 5 || IS_G4X(dev_priv))
		res2 = I915_READ(PIPE_CRC_RES_RES2_G4X(pipe));
	else
		res2 = 0;

	display_pipe_crc_irq_handler(dev_priv, pipe,
				     I915_READ(PIPE_CRC_RES_RED(pipe)),
				     I915_READ(PIPE_CRC_RES_GREEN(pipe)),
				     I915_READ(PIPE_CRC_RES_BLUE(pipe)),
				     res1, res2);
}

static void i9xx_pipestat_irq_reset(struct drm_i915_private *dev_priv)
{
	enum pipe pipe;

	for_each_pipe(dev_priv, pipe) {
		I915_WRITE(PIPESTAT(pipe),
			   PIPESTAT_INT_STATUS_MASK |
			   PIPE_FIFO_UNDERRUN_STATUS);

		dev_priv->pipestat_irq_mask[pipe] = 0;
	}
}

static void i9xx_pipestat_irq_ack(struct drm_i915_private *dev_priv,
				  u32 iir, u32 pipe_stats[I915_MAX_PIPES])
{
	enum pipe pipe;

	spin_lock(&dev_priv->irq_lock);

	if (!dev_priv->display_irqs_enabled) {
		spin_unlock(&dev_priv->irq_lock);
		return;
	}

	for_each_pipe(dev_priv, pipe) {
		i915_reg_t reg;
		u32 status_mask, enable_mask, iir_bit = 0;

		/*
		 * PIPESTAT bits get signalled even when the interrupt is
		 * disabled with the mask bits, and some of the status bits do
		 * not generate interrupts at all (like the underrun bit). Hence
		 * we need to be careful that we only handle what we want to
		 * handle.
		 */

		/* fifo underruns are filterered in the underrun handler. */
		status_mask = PIPE_FIFO_UNDERRUN_STATUS;

		switch (pipe) {
		default:
		case PIPE_A:
			iir_bit = I915_DISPLAY_PIPE_A_EVENT_INTERRUPT;
			break;
		case PIPE_B:
			iir_bit = I915_DISPLAY_PIPE_B_EVENT_INTERRUPT;
			break;
		case PIPE_C:
			iir_bit = I915_DISPLAY_PIPE_C_EVENT_INTERRUPT;
			break;
		}
		if (iir & iir_bit)
			status_mask |= dev_priv->pipestat_irq_mask[pipe];

		if (!status_mask)
			continue;

		reg = PIPESTAT(pipe);
		pipe_stats[pipe] = I915_READ(reg) & status_mask;
		enable_mask = i915_pipestat_enable_mask(dev_priv, pipe);

		/*
		 * Clear the PIPE*STAT regs before the IIR
		 *
		 * Toggle the enable bits to make sure we get an
		 * edge in the ISR pipe event bit if we don't clear
		 * all the enabled status bits. Otherwise the edge
		 * triggered IIR on i965/g4x wouldn't notice that
		 * an interrupt is still pending.
		 */
		if (pipe_stats[pipe]) {
			I915_WRITE(reg, pipe_stats[pipe]);
			I915_WRITE(reg, enable_mask);
		}
	}
	spin_unlock(&dev_priv->irq_lock);
}

static void i8xx_pipestat_irq_handler(struct drm_i915_private *dev_priv,
				      u16 iir, u32 pipe_stats[I915_MAX_PIPES])
{
	enum pipe pipe;

	for_each_pipe(dev_priv, pipe) {
		if (pipe_stats[pipe] & PIPE_VBLANK_INTERRUPT_STATUS)
			drm_handle_vblank(&dev_priv->drm, pipe);

		if (pipe_stats[pipe] & PIPE_CRC_DONE_INTERRUPT_STATUS)
			i9xx_pipe_crc_irq_handler(dev_priv, pipe);

		if (pipe_stats[pipe] & PIPE_FIFO_UNDERRUN_STATUS)
			intel_cpu_fifo_underrun_irq_handler(dev_priv, pipe);
	}
}

static void i915_pipestat_irq_handler(struct drm_i915_private *dev_priv,
				      u32 iir, u32 pipe_stats[I915_MAX_PIPES])
{
	bool blc_event = false;
	enum pipe pipe;

	for_each_pipe(dev_priv, pipe) {
		if (pipe_stats[pipe] & PIPE_VBLANK_INTERRUPT_STATUS)
			drm_handle_vblank(&dev_priv->drm, pipe);

		if (pipe_stats[pipe] & PIPE_LEGACY_BLC_EVENT_STATUS)
			blc_event = true;

		if (pipe_stats[pipe] & PIPE_CRC_DONE_INTERRUPT_STATUS)
			i9xx_pipe_crc_irq_handler(dev_priv, pipe);

		if (pipe_stats[pipe] & PIPE_FIFO_UNDERRUN_STATUS)
			intel_cpu_fifo_underrun_irq_handler(dev_priv, pipe);
	}

	if (blc_event || (iir & I915_ASLE_INTERRUPT))
		intel_opregion_asle_intr(dev_priv);
}

static void i965_pipestat_irq_handler(struct drm_i915_private *dev_priv,
				      u32 iir, u32 pipe_stats[I915_MAX_PIPES])
{
	bool blc_event = false;
	enum pipe pipe;

	for_each_pipe(dev_priv, pipe) {
		if (pipe_stats[pipe] & PIPE_START_VBLANK_INTERRUPT_STATUS)
			drm_handle_vblank(&dev_priv->drm, pipe);

		if (pipe_stats[pipe] & PIPE_LEGACY_BLC_EVENT_STATUS)
			blc_event = true;

		if (pipe_stats[pipe] & PIPE_CRC_DONE_INTERRUPT_STATUS)
			i9xx_pipe_crc_irq_handler(dev_priv, pipe);

		if (pipe_stats[pipe] & PIPE_FIFO_UNDERRUN_STATUS)
			intel_cpu_fifo_underrun_irq_handler(dev_priv, pipe);
	}

	if (blc_event || (iir & I915_ASLE_INTERRUPT))
		intel_opregion_asle_intr(dev_priv);

	if (pipe_stats[0] & PIPE_GMBUS_INTERRUPT_STATUS)
		gmbus_irq_handler(dev_priv);
}

static void valleyview_pipestat_irq_handler(struct drm_i915_private *dev_priv,
					    u32 pipe_stats[I915_MAX_PIPES])
{
	enum pipe pipe;

	for_each_pipe(dev_priv, pipe) {
		if (pipe_stats[pipe] & PIPE_START_VBLANK_INTERRUPT_STATUS)
			drm_handle_vblank(&dev_priv->drm, pipe);

		if (pipe_stats[pipe] & PIPE_CRC_DONE_INTERRUPT_STATUS)
			i9xx_pipe_crc_irq_handler(dev_priv, pipe);

		if (pipe_stats[pipe] & PIPE_FIFO_UNDERRUN_STATUS)
			intel_cpu_fifo_underrun_irq_handler(dev_priv, pipe);
	}

	if (pipe_stats[0] & PIPE_GMBUS_INTERRUPT_STATUS)
		gmbus_irq_handler(dev_priv);
}

static u32 i9xx_hpd_irq_ack(struct drm_i915_private *dev_priv)
{
	u32 hotplug_status = 0, hotplug_status_mask;
	int i;

	if (IS_G4X(dev_priv) ||
	    IS_VALLEYVIEW(dev_priv) || IS_CHERRYVIEW(dev_priv))
		hotplug_status_mask = HOTPLUG_INT_STATUS_G4X |
			DP_AUX_CHANNEL_MASK_INT_STATUS_G4X;
	else
		hotplug_status_mask = HOTPLUG_INT_STATUS_I915;

	/*
	 * We absolutely have to clear all the pending interrupt
	 * bits in PORT_HOTPLUG_STAT. Otherwise the ISR port
	 * interrupt bit won't have an edge, and the i965/g4x
	 * edge triggered IIR will not notice that an interrupt
	 * is still pending. We can't use PORT_HOTPLUG_EN to
	 * guarantee the edge as the act of toggling the enable
	 * bits can itself generate a new hotplug interrupt :(
	 */
	for (i = 0; i < 10; i++) {
		u32 tmp = I915_READ(PORT_HOTPLUG_STAT) & hotplug_status_mask;

		if (tmp == 0)
			return hotplug_status;

		hotplug_status |= tmp;
		I915_WRITE(PORT_HOTPLUG_STAT, hotplug_status);
	}

	WARN_ONCE(1,
		  "PORT_HOTPLUG_STAT did not clear (0x%08x)\n",
		  I915_READ(PORT_HOTPLUG_STAT));

	return hotplug_status;
}

static void i9xx_hpd_irq_handler(struct drm_i915_private *dev_priv,
				 u32 hotplug_status)
{
	u32 pin_mask = 0, long_mask = 0;

	if (IS_G4X(dev_priv) || IS_VALLEYVIEW(dev_priv) ||
	    IS_CHERRYVIEW(dev_priv)) {
		u32 hotplug_trigger = hotplug_status & HOTPLUG_INT_STATUS_G4X;

		if (hotplug_trigger) {
			intel_get_hpd_pins(dev_priv, &pin_mask, &long_mask,
					   hotplug_trigger, hotplug_trigger,
					   hpd_status_g4x,
					   i9xx_port_hotplug_long_detect);

			intel_hpd_irq_handler(dev_priv, pin_mask, long_mask);
		}

		if (hotplug_status & DP_AUX_CHANNEL_MASK_INT_STATUS_G4X)
			dp_aux_irq_handler(dev_priv);
	} else {
		u32 hotplug_trigger = hotplug_status & HOTPLUG_INT_STATUS_I915;

		if (hotplug_trigger) {
			intel_get_hpd_pins(dev_priv, &pin_mask, &long_mask,
					   hotplug_trigger, hotplug_trigger,
					   hpd_status_i915,
					   i9xx_port_hotplug_long_detect);
			intel_hpd_irq_handler(dev_priv, pin_mask, long_mask);
		}
	}
}

static irqreturn_t valleyview_irq_handler(DRM_IRQ_ARGS)
=======
static irqreturn_t valleyview_irq_handler(int irq, void *arg)
>>>>>>> vendor/linux-drm-v6.6.35
{
	struct drm_i915_private *dev_priv = arg;
	irqreturn_t ret = IRQ_NONE;

	if (!intel_irqs_enabled(dev_priv))
		return IRQ_NONE;

	/* IRQs are synced during runtime_suspend, we don't require a wakeref */
	disable_rpm_wakeref_asserts(&dev_priv->runtime_pm);

	do {
		u32 iir, gt_iir, pm_iir;
		u32 pipe_stats[I915_MAX_PIPES] = {};
		u32 hotplug_status = 0;
		u32 ier = 0;

		gt_iir = intel_uncore_read(&dev_priv->uncore, GTIIR);
		pm_iir = intel_uncore_read(&dev_priv->uncore, GEN6_PMIIR);
		iir = intel_uncore_read(&dev_priv->uncore, VLV_IIR);

		if (gt_iir == 0 && pm_iir == 0 && iir == 0)
			break;

		ret = IRQ_HANDLED;

		/*
		 * Theory on interrupt generation, based on empirical evidence:
		 *
		 * x = ((VLV_IIR & VLV_IER) ||
		 *      (((GT_IIR & GT_IER) || (GEN6_PMIIR & GEN6_PMIER)) &&
		 *       (VLV_MASTER_IER & MASTER_INTERRUPT_ENABLE)));
		 *
		 * A CPU interrupt will only be raised when 'x' has a 0->1 edge.
		 * Hence we clear MASTER_INTERRUPT_ENABLE and VLV_IER to
		 * guarantee the CPU interrupt will be raised again even if we
		 * don't end up clearing all the VLV_IIR, GT_IIR, GEN6_PMIIR
		 * bits this time around.
		 */
		intel_uncore_write(&dev_priv->uncore, VLV_MASTER_IER, 0);
		ier = intel_uncore_rmw(&dev_priv->uncore, VLV_IER, ~0, 0);

		if (gt_iir)
			intel_uncore_write(&dev_priv->uncore, GTIIR, gt_iir);
		if (pm_iir)
			intel_uncore_write(&dev_priv->uncore, GEN6_PMIIR, pm_iir);

		if (iir & I915_DISPLAY_PORT_INTERRUPT)
			hotplug_status = i9xx_hpd_irq_ack(dev_priv);

		/* Call regardless, as some status bits might not be
		 * signalled in iir */
		i9xx_pipestat_irq_ack(dev_priv, iir, pipe_stats);

		if (iir & (I915_LPE_PIPE_A_INTERRUPT |
			   I915_LPE_PIPE_B_INTERRUPT))
			intel_lpe_audio_irq_handler(dev_priv);

		/*
		 * VLV_IIR is single buffered, and reflects the level
		 * from PIPESTAT/PORT_HOTPLUG_STAT, hence clear it last.
		 */
		if (iir)
			intel_uncore_write(&dev_priv->uncore, VLV_IIR, iir);

		intel_uncore_write(&dev_priv->uncore, VLV_IER, ier);
		intel_uncore_write(&dev_priv->uncore, VLV_MASTER_IER, MASTER_INTERRUPT_ENABLE);

		if (gt_iir)
			gen6_gt_irq_handler(to_gt(dev_priv), gt_iir);
		if (pm_iir)
			gen6_rps_irq_handler(&to_gt(dev_priv)->rps, pm_iir);

		if (hotplug_status)
			i9xx_hpd_irq_handler(dev_priv, hotplug_status);

		valleyview_pipestat_irq_handler(dev_priv, pipe_stats);
	} while (0);

	pmu_irq_stats(dev_priv, ret);

	enable_rpm_wakeref_asserts(&dev_priv->runtime_pm);

	return ret;
}

static irqreturn_t cherryview_irq_handler(DRM_IRQ_ARGS)
{
	struct drm_i915_private *dev_priv = arg;
	irqreturn_t ret = IRQ_NONE;

	if (!intel_irqs_enabled(dev_priv))
		return IRQ_NONE;

	/* IRQs are synced during runtime_suspend, we don't require a wakeref */
	disable_rpm_wakeref_asserts(&dev_priv->runtime_pm);

	do {
		u32 master_ctl, iir;
		u32 pipe_stats[I915_MAX_PIPES] = {};
		u32 hotplug_status = 0;
		u32 ier = 0;

		master_ctl = intel_uncore_read(&dev_priv->uncore, GEN8_MASTER_IRQ) & ~GEN8_MASTER_IRQ_CONTROL;
		iir = intel_uncore_read(&dev_priv->uncore, VLV_IIR);

		if (master_ctl == 0 && iir == 0)
			break;

		ret = IRQ_HANDLED;

		/*
		 * Theory on interrupt generation, based on empirical evidence:
		 *
		 * x = ((VLV_IIR & VLV_IER) ||
		 *      ((GEN8_MASTER_IRQ & ~GEN8_MASTER_IRQ_CONTROL) &&
		 *       (GEN8_MASTER_IRQ & GEN8_MASTER_IRQ_CONTROL)));
		 *
		 * A CPU interrupt will only be raised when 'x' has a 0->1 edge.
		 * Hence we clear GEN8_MASTER_IRQ_CONTROL and VLV_IER to
		 * guarantee the CPU interrupt will be raised again even if we
		 * don't end up clearing all the VLV_IIR and GEN8_MASTER_IRQ_CONTROL
		 * bits this time around.
		 */
		intel_uncore_write(&dev_priv->uncore, GEN8_MASTER_IRQ, 0);
		ier = intel_uncore_rmw(&dev_priv->uncore, VLV_IER, ~0, 0);

		gen8_gt_irq_handler(to_gt(dev_priv), master_ctl);

		if (iir & I915_DISPLAY_PORT_INTERRUPT)
			hotplug_status = i9xx_hpd_irq_ack(dev_priv);

		/* Call regardless, as some status bits might not be
		 * signalled in iir */
		i9xx_pipestat_irq_ack(dev_priv, iir, pipe_stats);

		if (iir & (I915_LPE_PIPE_A_INTERRUPT |
			   I915_LPE_PIPE_B_INTERRUPT |
			   I915_LPE_PIPE_C_INTERRUPT))
			intel_lpe_audio_irq_handler(dev_priv);

		/*
		 * VLV_IIR is single buffered, and reflects the level
		 * from PIPESTAT/PORT_HOTPLUG_STAT, hence clear it last.
		 */
		if (iir)
			intel_uncore_write(&dev_priv->uncore, VLV_IIR, iir);

		intel_uncore_write(&dev_priv->uncore, VLV_IER, ier);
		intel_uncore_write(&dev_priv->uncore, GEN8_MASTER_IRQ, GEN8_MASTER_IRQ_CONTROL);

		if (hotplug_status)
			i9xx_hpd_irq_handler(dev_priv, hotplug_status);

		valleyview_pipestat_irq_handler(dev_priv, pipe_stats);
	} while (0);

	pmu_irq_stats(dev_priv, ret);

	enable_rpm_wakeref_asserts(&dev_priv->runtime_pm);

	return ret;
}

/*
 * To handle irqs with the minimum potential races with fresh interrupts, we:
 * 1 - Disable Master Interrupt Control.
 * 2 - Find the source(s) of the interrupt.
 * 3 - Clear the Interrupt Identity bits (IIR).
 * 4 - Process the interrupt(s) that had bits set in the IIRs.
 * 5 - Re-enable Master Interrupt Control.
 */
static irqreturn_t ilk_irq_handler(DRM_IRQ_ARGS)
{
	struct drm_i915_private *i915 = arg;
	void __iomem * const regs = intel_uncore_regs(&i915->uncore);
	u32 de_iir, gt_iir, de_ier, sde_ier = 0;
	irqreturn_t ret = IRQ_NONE;

	if (unlikely(!intel_irqs_enabled(i915)))
		return IRQ_NONE;

	/* IRQs are synced during runtime_suspend, we don't require a wakeref */
	disable_rpm_wakeref_asserts(&i915->runtime_pm);

	/* disable master interrupt before clearing iir  */
	de_ier = raw_reg_read(regs, DEIER);
	raw_reg_write(regs, DEIER, de_ier & ~DE_MASTER_IRQ_CONTROL);

	/* Disable south interrupts. We'll only write to SDEIIR once, so further
	 * interrupts will will be stored on its back queue, and then we'll be
	 * able to process them after we restore SDEIER (as soon as we restore
	 * it, we'll get an interrupt if SDEIIR still has something to process
	 * due to its back queue). */
	if (!HAS_PCH_NOP(i915)) {
		sde_ier = raw_reg_read(regs, SDEIER);
		raw_reg_write(regs, SDEIER, 0);
	}

	/* Find, clear, then process each source of interrupt */

	gt_iir = raw_reg_read(regs, GTIIR);
	if (gt_iir) {
		raw_reg_write(regs, GTIIR, gt_iir);
		if (GRAPHICS_VER(i915) >= 6)
			gen6_gt_irq_handler(to_gt(i915), gt_iir);
		else
			gen5_gt_irq_handler(to_gt(i915), gt_iir);
		ret = IRQ_HANDLED;
	}

	de_iir = raw_reg_read(regs, DEIIR);
	if (de_iir) {
		raw_reg_write(regs, DEIIR, de_iir);
		if (DISPLAY_VER(i915) >= 7)
			ivb_display_irq_handler(i915, de_iir);
		else
			ilk_display_irq_handler(i915, de_iir);
		ret = IRQ_HANDLED;
	}

	if (GRAPHICS_VER(i915) >= 6) {
		u32 pm_iir = raw_reg_read(regs, GEN6_PMIIR);
		if (pm_iir) {
			raw_reg_write(regs, GEN6_PMIIR, pm_iir);
			gen6_rps_irq_handler(&to_gt(i915)->rps, pm_iir);
			ret = IRQ_HANDLED;
		}
	}

	raw_reg_write(regs, DEIER, de_ier);
	if (sde_ier)
		raw_reg_write(regs, SDEIER, sde_ier);

	pmu_irq_stats(i915, ret);

	/* IRQs are synced during runtime_suspend, we don't require a wakeref */
	enable_rpm_wakeref_asserts(&i915->runtime_pm);

	return ret;
}

static inline u32 gen8_master_intr_disable(struct intel_uncore *regs)
{
	raw_reg_write(regs, GEN8_MASTER_IRQ, 0);

	/*
	 * Now with master disabled, get a sample of level indications
	 * for this interrupt. Indications will be cleared on related acks.
	 * New indications can and will light up during processing,
	 * and will generate new interrupt after enabling master.
	 */
	return raw_reg_read(regs, GEN8_MASTER_IRQ);
}

static inline void gen8_master_intr_enable(struct intel_uncore *regs)
{
	raw_reg_write(regs, GEN8_MASTER_IRQ, GEN8_MASTER_IRQ_CONTROL);
}

static irqreturn_t gen8_irq_handler(DRM_IRQ_ARGS)
{
	struct drm_i915_private *dev_priv = arg;
<<<<<<< HEAD
=======
	void __iomem * const regs = intel_uncore_regs(&dev_priv->uncore);
>>>>>>> vendor/linux-drm-v6.6.35
	u32 master_ctl;

	if (!intel_irqs_enabled(dev_priv))
		return IRQ_NONE;

	master_ctl = gen8_master_intr_disable(&dev_priv->uncore);
	if (!master_ctl) {
		gen8_master_intr_enable(&dev_priv->uncore);
		return IRQ_NONE;
	}

	/* Find, queue (onto bottom-halves), then clear each source */
	gen8_gt_irq_handler(to_gt(dev_priv), master_ctl);

	/* IRQs are synced during runtime_suspend, we don't require a wakeref */
	if (master_ctl & ~GEN8_GT_IRQS) {
		disable_rpm_wakeref_asserts(&dev_priv->runtime_pm);
		gen8_de_irq_handler(dev_priv, master_ctl);
		enable_rpm_wakeref_asserts(&dev_priv->runtime_pm);
	}

	gen8_master_intr_enable(&dev_priv->uncore);

	pmu_irq_stats(dev_priv, IRQ_HANDLED);

	return IRQ_HANDLED;
}

<<<<<<< HEAD
static u32
gen11_gu_misc_irq_ack(struct intel_gt *gt, const u32 master_ctl)
{
	u32 iir;

	if (!(master_ctl & GEN11_GU_MISC_IRQ))
		return 0;

	iir = raw_reg_read(gt->uncore, GEN11_GU_MISC_IIR);
	if (likely(iir))
		raw_reg_write(gt->uncore, GEN11_GU_MISC_IIR, iir);

	return iir;
}

static void
gen11_gu_misc_irq_handler(struct intel_gt *gt, const u32 iir)
{
	if (iir & GEN11_GU_MISC_GSE)
		intel_opregion_asle_intr(gt->i915);
}

static inline u32 gen11_master_intr_disable(struct intel_uncore *regs)
=======
static inline u32 gen11_master_intr_disable(void __iomem * const regs)
>>>>>>> vendor/linux-drm-v6.6.35
{
	raw_reg_write(regs, GEN11_GFX_MSTR_IRQ, 0);

	/*
	 * Now with master disabled, get a sample of level indications
	 * for this interrupt. Indications will be cleared on related acks.
	 * New indications can and will light up during processing,
	 * and will generate new interrupt after enabling master.
	 */
	return raw_reg_read(regs, GEN11_GFX_MSTR_IRQ);
}

static inline void gen11_master_intr_enable(struct intel_uncore *regs)
{
	raw_reg_write(regs, GEN11_GFX_MSTR_IRQ, GEN11_MASTER_IRQ);
}

static irqreturn_t gen11_irq_handler(int irq, void *arg)
{
<<<<<<< HEAD
	const u32 disp_ctl = raw_reg_read(&i915->uncore, GEN11_DISPLAY_INT_CTL);

	disable_rpm_wakeref_asserts(&i915->runtime_pm);
	/*
	 * GEN11_DISPLAY_INT_CTL has same format as GEN8_MASTER_IRQ
	 * for the display related bits.
	 */
	raw_reg_write(&i915->uncore, GEN11_DISPLAY_INT_CTL, 0x0);
	gen8_de_irq_handler(i915, disp_ctl);
	raw_reg_write(&i915->uncore, GEN11_DISPLAY_INT_CTL,
		      GEN11_DISPLAY_IRQ_ENABLE);

	enable_rpm_wakeref_asserts(&i915->runtime_pm);
}

static __always_inline irqreturn_t
__gen11_irq_handler(struct drm_i915_private * const i915,
		    u32 (*intr_disable)(struct intel_uncore *regs),
		    void (*intr_enable)(struct intel_uncore *regs))
{
	struct intel_gt *gt = &i915->gt;
=======
	struct drm_i915_private *i915 = arg;
	void __iomem * const regs = intel_uncore_regs(&i915->uncore);
	struct intel_gt *gt = to_gt(i915);
>>>>>>> vendor/linux-drm-v6.6.35
	u32 master_ctl;
	u32 gu_misc_iir;

	if (!intel_irqs_enabled(i915))
		return IRQ_NONE;

<<<<<<< HEAD
	master_ctl = intr_disable(&i915->uncore);
	if (!master_ctl) {
		intr_enable(&i915->uncore);
=======
	master_ctl = gen11_master_intr_disable(regs);
	if (!master_ctl) {
		gen11_master_intr_enable(regs);
>>>>>>> vendor/linux-drm-v6.6.35
		return IRQ_NONE;
	}

	/* Find, queue (onto bottom-halves), then clear each source */
	gen11_gt_irq_handler(gt, master_ctl);

	/* IRQs are synced during runtime_suspend, we don't require a wakeref */
	if (master_ctl & GEN11_DISPLAY_IRQ)
		gen11_display_irq_handler(i915);

	gu_misc_iir = gen11_gu_misc_irq_ack(i915, master_ctl);

<<<<<<< HEAD
	intr_enable(&i915->uncore);
=======
	gen11_master_intr_enable(regs);
>>>>>>> vendor/linux-drm-v6.6.35

	gen11_gu_misc_irq_handler(i915, gu_misc_iir);

	pmu_irq_stats(i915, IRQ_HANDLED);

	return IRQ_HANDLED;
}

<<<<<<< HEAD
static irqreturn_t gen11_irq_handler(DRM_IRQ_ARGS)
=======
static inline u32 dg1_master_intr_disable(void __iomem * const regs)
>>>>>>> vendor/linux-drm-v6.6.35
{
	u32 val;

	/* First disable interrupts */
	raw_reg_write(regs, DG1_MSTR_TILE_INTR, 0);

	/* Get the indication levels and ack the master unit */
	val = raw_reg_read(regs, DG1_MSTR_TILE_INTR);
	if (unlikely(!val))
		return 0;

	raw_reg_write(regs, DG1_MSTR_TILE_INTR, val);

	return val;
}

static inline void dg1_master_intr_enable(void __iomem * const regs)
{
	raw_reg_write(regs, DG1_MSTR_TILE_INTR, DG1_MSTR_IRQ);
}

static irqreturn_t dg1_irq_handler(int irq, void *arg)
{
	struct drm_i915_private * const i915 = arg;
	struct intel_gt *gt = to_gt(i915);
	void __iomem * const regs = intel_uncore_regs(gt->uncore);
	u32 master_tile_ctl, master_ctl;
	u32 gu_misc_iir;

	if (!intel_irqs_enabled(i915))
		return IRQ_NONE;

	master_tile_ctl = dg1_master_intr_disable(regs);
	if (!master_tile_ctl) {
		dg1_master_intr_enable(regs);
		return IRQ_NONE;
	}

	/* FIXME: we only support tile 0 for now. */
	if (master_tile_ctl & DG1_MSTR_TILE(0)) {
		master_ctl = raw_reg_read(regs, GEN11_GFX_MSTR_IRQ);
		raw_reg_write(regs, GEN11_GFX_MSTR_IRQ, master_ctl);
	} else {
		drm_err(&i915->drm, "Tile not supported: 0x%08x\n",
			master_tile_ctl);
		dg1_master_intr_enable(regs);
		return IRQ_NONE;
	}

	gen11_gt_irq_handler(gt, master_ctl);

	if (master_ctl & GEN11_DISPLAY_IRQ)
		gen11_display_irq_handler(i915);

	gu_misc_iir = gen11_gu_misc_irq_ack(i915, master_ctl);

	dg1_master_intr_enable(regs);

	gen11_gu_misc_irq_handler(i915, gu_misc_iir);

	pmu_irq_stats(i915, IRQ_HANDLED);

	return IRQ_HANDLED;
}

static void ibx_irq_reset(struct drm_i915_private *dev_priv)
{
	struct intel_uncore *uncore = &dev_priv->uncore;

	if (HAS_PCH_NOP(dev_priv))
		return;

	GEN3_IRQ_RESET(uncore, SDE);

	if (HAS_PCH_CPT(dev_priv) || HAS_PCH_LPT(dev_priv))
		intel_uncore_write(&dev_priv->uncore, SERR_INT, 0xffffffff);
}

/* drm_dma.h hooks
*/
static void ilk_irq_reset(struct drm_i915_private *dev_priv)
{
	struct intel_uncore *uncore = &dev_priv->uncore;

	GEN3_IRQ_RESET(uncore, DE);
	dev_priv->irq_mask = ~0u;

	if (GRAPHICS_VER(dev_priv) == 7)
		intel_uncore_write(uncore, GEN7_ERR_INT, 0xffffffff);

	if (IS_HASWELL(dev_priv)) {
		intel_uncore_write(uncore, EDP_PSR_IMR, 0xffffffff);
		intel_uncore_write(uncore, EDP_PSR_IIR, 0xffffffff);
	}

	gen5_gt_irq_reset(to_gt(dev_priv));

	ibx_irq_reset(dev_priv);
}

static void valleyview_irq_reset(struct drm_i915_private *dev_priv)
{
	intel_uncore_write(&dev_priv->uncore, VLV_MASTER_IER, 0);
	intel_uncore_posting_read(&dev_priv->uncore, VLV_MASTER_IER);

	gen5_gt_irq_reset(to_gt(dev_priv));

	spin_lock_irq(&dev_priv->irq_lock);
	if (dev_priv->display_irqs_enabled)
		vlv_display_irq_reset(dev_priv);
	spin_unlock_irq(&dev_priv->irq_lock);
}

static void gen8_irq_reset(struct drm_i915_private *dev_priv)
{
	struct intel_uncore *uncore = &dev_priv->uncore;

<<<<<<< HEAD
	gen8_master_intr_disable(&dev_priv->uncore);
=======
	gen8_master_intr_disable(intel_uncore_regs(uncore));
>>>>>>> vendor/linux-drm-v6.6.35

	gen8_gt_irq_reset(to_gt(dev_priv));
	gen8_display_irq_reset(dev_priv);
	GEN3_IRQ_RESET(uncore, GEN8_PCU_);

	if (HAS_PCH_SPLIT(dev_priv))
		ibx_irq_reset(dev_priv);

}

static void gen11_irq_reset(struct drm_i915_private *dev_priv)
{
	struct intel_gt *gt = to_gt(dev_priv);
	struct intel_uncore *uncore = gt->uncore;

<<<<<<< HEAD
	gen11_master_intr_disable(&dev_priv->uncore);
=======
	gen11_master_intr_disable(intel_uncore_regs(&dev_priv->uncore));
>>>>>>> vendor/linux-drm-v6.6.35

	gen11_gt_irq_reset(gt);
	gen11_display_irq_reset(dev_priv);

	GEN3_IRQ_RESET(uncore, GEN11_GU_MISC_);
	GEN3_IRQ_RESET(uncore, GEN8_PCU_);
}

static void dg1_irq_reset(struct drm_i915_private *dev_priv)
{
	struct intel_uncore *uncore = &dev_priv->uncore;
	struct intel_gt *gt;
	unsigned int i;

	dg1_master_intr_disable(intel_uncore_regs(&dev_priv->uncore));

	for_each_gt(gt, dev_priv, i)
		gen11_gt_irq_reset(gt);

	gen11_display_irq_reset(dev_priv);

	GEN3_IRQ_RESET(uncore, GEN11_GU_MISC_);
	GEN3_IRQ_RESET(uncore, GEN8_PCU_);
}

static void cherryview_irq_reset(struct drm_i915_private *dev_priv)
{
	struct intel_uncore *uncore = &dev_priv->uncore;

	intel_uncore_write(uncore, GEN8_MASTER_IRQ, 0);
	intel_uncore_posting_read(&dev_priv->uncore, GEN8_MASTER_IRQ);

	gen8_gt_irq_reset(to_gt(dev_priv));

	GEN3_IRQ_RESET(uncore, GEN8_PCU_);

	spin_lock_irq(&dev_priv->irq_lock);
	if (dev_priv->display_irqs_enabled)
		vlv_display_irq_reset(dev_priv);
	spin_unlock_irq(&dev_priv->irq_lock);
}

<<<<<<< HEAD
static u32 intel_hpd_enabled_irqs(struct drm_i915_private *dev_priv,
				  const u32 hpd[HPD_NUM_PINS])
{
	struct intel_encoder *encoder;
	u32 enabled_irqs = 0;

	for_each_intel_encoder(&dev_priv->drm, encoder)
		if (dev_priv->hotplug.stats[encoder->hpd_pin].state == HPD_ENABLED)
			enabled_irqs |= hpd[encoder->hpd_pin];

	return enabled_irqs;
}

static void ibx_hpd_detection_setup(struct drm_i915_private *dev_priv)
{
	u32 hotplug;

	/*
	 * Enable digital hotplug on the PCH, and configure the DP short pulse
	 * duration to 2ms (which is the minimum in the Display Port spec).
	 * The pulse duration bits are reserved on LPT+.
	 */
	hotplug = I915_READ(PCH_PORT_HOTPLUG);
	hotplug &= ~(PORTB_PULSE_DURATION_MASK |
		     PORTC_PULSE_DURATION_MASK |
		     PORTD_PULSE_DURATION_MASK);
	hotplug |= PORTB_HOTPLUG_ENABLE | PORTB_PULSE_DURATION_2ms;
	hotplug |= PORTC_HOTPLUG_ENABLE | PORTC_PULSE_DURATION_2ms;
	hotplug |= PORTD_HOTPLUG_ENABLE | PORTD_PULSE_DURATION_2ms;
	/*
	 * When CPU and PCH are on the same package, port A
	 * HPD must be enabled in both north and south.
	 */
	if (HAS_PCH_LPT_LP(dev_priv))
		hotplug |= PORTA_HOTPLUG_ENABLE;
	I915_WRITE(PCH_PORT_HOTPLUG, hotplug);
}

static void ibx_hpd_irq_setup(struct drm_i915_private *dev_priv)
{
	u32 hotplug_irqs, enabled_irqs;

	if (HAS_PCH_IBX(dev_priv)) {
		hotplug_irqs = SDE_HOTPLUG_MASK;
		enabled_irqs = intel_hpd_enabled_irqs(dev_priv, hpd_ibx);
	} else {
		hotplug_irqs = SDE_HOTPLUG_MASK_CPT;
		enabled_irqs = intel_hpd_enabled_irqs(dev_priv, hpd_cpt);
	}

	ibx_display_interrupt_update(dev_priv, hotplug_irqs, enabled_irqs);

	ibx_hpd_detection_setup(dev_priv);
}

static void icp_hpd_detection_setup(struct drm_i915_private *dev_priv,
				    u32 ddi_hotplug_enable_mask,
				    u32 tc_hotplug_enable_mask)
{
	u32 hotplug;

	hotplug = I915_READ(SHOTPLUG_CTL_DDI);
	hotplug |= ddi_hotplug_enable_mask;
	I915_WRITE(SHOTPLUG_CTL_DDI, hotplug);

	if (tc_hotplug_enable_mask) {
		hotplug = I915_READ(SHOTPLUG_CTL_TC);
		hotplug |= tc_hotplug_enable_mask;
		I915_WRITE(SHOTPLUG_CTL_TC, hotplug);
	}
}

static void icp_hpd_irq_setup(struct drm_i915_private *dev_priv,
			      u32 sde_ddi_mask, u32 sde_tc_mask,
			      u32 ddi_enable_mask, u32 tc_enable_mask,
			      const u32 *pins)
{
	u32 hotplug_irqs, enabled_irqs;

	hotplug_irqs = sde_ddi_mask | sde_tc_mask;
	enabled_irqs = intel_hpd_enabled_irqs(dev_priv, pins);

	I915_WRITE(SHPD_FILTER_CNT, SHPD_FILTER_CNT_500_ADJ);

	ibx_display_interrupt_update(dev_priv, hotplug_irqs, enabled_irqs);

	icp_hpd_detection_setup(dev_priv, ddi_enable_mask, tc_enable_mask);
}

/*
 * EHL doesn't need most of gen11_hpd_irq_setup, it's handling only the
 * equivalent of SDE.
 */
static void mcc_hpd_irq_setup(struct drm_i915_private *dev_priv)
{
	icp_hpd_irq_setup(dev_priv,
			  SDE_DDI_MASK_ICP, SDE_TC_HOTPLUG_ICP(PORT_TC1),
			  ICP_DDI_HPD_ENABLE_MASK, ICP_TC_HPD_ENABLE(PORT_TC1),
			  hpd_icp);
}

/*
 * JSP behaves exactly the same as MCC above except that port C is mapped to
 * the DDI-C pins instead of the TC1 pins.  This means we should follow TGP's
 * masks & tables rather than ICP's masks & tables.
 */
static void jsp_hpd_irq_setup(struct drm_i915_private *dev_priv)
{
	icp_hpd_irq_setup(dev_priv,
			  SDE_DDI_MASK_TGP, 0,
			  TGP_DDI_HPD_ENABLE_MASK, 0,
			  hpd_tgp);
}

static void gen11_hpd_detection_setup(struct drm_i915_private *dev_priv)
{
	u32 hotplug;

	hotplug = I915_READ(GEN11_TC_HOTPLUG_CTL);
	hotplug |= GEN11_HOTPLUG_CTL_ENABLE(PORT_TC1) |
		   GEN11_HOTPLUG_CTL_ENABLE(PORT_TC2) |
		   GEN11_HOTPLUG_CTL_ENABLE(PORT_TC3) |
		   GEN11_HOTPLUG_CTL_ENABLE(PORT_TC4);
	I915_WRITE(GEN11_TC_HOTPLUG_CTL, hotplug);

	hotplug = I915_READ(GEN11_TBT_HOTPLUG_CTL);
	hotplug |= GEN11_HOTPLUG_CTL_ENABLE(PORT_TC1) |
		   GEN11_HOTPLUG_CTL_ENABLE(PORT_TC2) |
		   GEN11_HOTPLUG_CTL_ENABLE(PORT_TC3) |
		   GEN11_HOTPLUG_CTL_ENABLE(PORT_TC4);
	I915_WRITE(GEN11_TBT_HOTPLUG_CTL, hotplug);
}

static void gen11_hpd_irq_setup(struct drm_i915_private *dev_priv)
{
	u32 hotplug_irqs, enabled_irqs __unused;
	const u32 *hpd;
	u32 val;

	hpd = INTEL_GEN(dev_priv) >= 12 ? hpd_gen12 : hpd_gen11;
	enabled_irqs = intel_hpd_enabled_irqs(dev_priv, hpd);
	hotplug_irqs = GEN11_DE_TC_HOTPLUG_MASK | GEN11_DE_TBT_HOTPLUG_MASK;

	val = I915_READ(GEN11_DE_HPD_IMR);
	val &= ~hotplug_irqs;
	I915_WRITE(GEN11_DE_HPD_IMR, val);
	POSTING_READ(GEN11_DE_HPD_IMR);

	gen11_hpd_detection_setup(dev_priv);

	if (INTEL_PCH_TYPE(dev_priv) >= PCH_TGP)
		icp_hpd_irq_setup(dev_priv, SDE_DDI_MASK_TGP, SDE_TC_MASK_TGP,
				  TGP_DDI_HPD_ENABLE_MASK,
				  TGP_TC_HPD_ENABLE_MASK, hpd_tgp);
	else if (INTEL_PCH_TYPE(dev_priv) >= PCH_ICP)
		icp_hpd_irq_setup(dev_priv, SDE_DDI_MASK_ICP, SDE_TC_MASK_ICP,
				  ICP_DDI_HPD_ENABLE_MASK,
				  ICP_TC_HPD_ENABLE_MASK, hpd_icp);
}

static void spt_hpd_detection_setup(struct drm_i915_private *dev_priv)
{
	u32 val, hotplug;

	/* Display WA #1179 WaHardHangonHotPlug: cnp */
	if (HAS_PCH_CNP(dev_priv)) {
		val = I915_READ(SOUTH_CHICKEN1);
		val &= ~CHASSIS_CLK_REQ_DURATION_MASK;
		val |= CHASSIS_CLK_REQ_DURATION(0xf);
		I915_WRITE(SOUTH_CHICKEN1, val);
	}

	/* Enable digital hotplug on the PCH */
	hotplug = I915_READ(PCH_PORT_HOTPLUG);
	hotplug |= PORTA_HOTPLUG_ENABLE |
		   PORTB_HOTPLUG_ENABLE |
		   PORTC_HOTPLUG_ENABLE |
		   PORTD_HOTPLUG_ENABLE;
	I915_WRITE(PCH_PORT_HOTPLUG, hotplug);

	hotplug = I915_READ(PCH_PORT_HOTPLUG2);
	hotplug |= PORTE_HOTPLUG_ENABLE;
	I915_WRITE(PCH_PORT_HOTPLUG2, hotplug);
}

static void spt_hpd_irq_setup(struct drm_i915_private *dev_priv)
{
	u32 hotplug_irqs, enabled_irqs;

	if (INTEL_PCH_TYPE(dev_priv) >= PCH_CNP)
		I915_WRITE(SHPD_FILTER_CNT, SHPD_FILTER_CNT_500_ADJ);

	hotplug_irqs = SDE_HOTPLUG_MASK_SPT;
	enabled_irqs = intel_hpd_enabled_irqs(dev_priv, hpd_spt);

	ibx_display_interrupt_update(dev_priv, hotplug_irqs, enabled_irqs);

	spt_hpd_detection_setup(dev_priv);
}

static void ilk_hpd_detection_setup(struct drm_i915_private *dev_priv)
{
	u32 hotplug;

	/*
	 * Enable digital hotplug on the CPU, and configure the DP short pulse
	 * duration to 2ms (which is the minimum in the Display Port spec)
	 * The pulse duration bits are reserved on HSW+.
	 */
	hotplug = I915_READ(DIGITAL_PORT_HOTPLUG_CNTRL);
	hotplug &= ~DIGITAL_PORTA_PULSE_DURATION_MASK;
	hotplug |= DIGITAL_PORTA_HOTPLUG_ENABLE |
		   DIGITAL_PORTA_PULSE_DURATION_2ms;
	I915_WRITE(DIGITAL_PORT_HOTPLUG_CNTRL, hotplug);
}

static void ilk_hpd_irq_setup(struct drm_i915_private *dev_priv)
{
	u32 hotplug_irqs, enabled_irqs;

	if (INTEL_GEN(dev_priv) >= 8) {
		hotplug_irqs = GEN8_PORT_DP_A_HOTPLUG;
		enabled_irqs = intel_hpd_enabled_irqs(dev_priv, hpd_bdw);

		bdw_update_port_irq(dev_priv, hotplug_irqs, enabled_irqs);
	} else if (INTEL_GEN(dev_priv) >= 7) {
		hotplug_irqs = DE_DP_A_HOTPLUG_IVB;
		enabled_irqs = intel_hpd_enabled_irqs(dev_priv, hpd_ivb);

		ilk_update_display_irq(dev_priv, hotplug_irqs, enabled_irqs);
	} else {
		hotplug_irqs = DE_DP_A_HOTPLUG;
		enabled_irqs = intel_hpd_enabled_irqs(dev_priv, hpd_ilk);

		ilk_update_display_irq(dev_priv, hotplug_irqs, enabled_irqs);
	}

	ilk_hpd_detection_setup(dev_priv);

	ibx_hpd_irq_setup(dev_priv);
}

static void __bxt_hpd_detection_setup(struct drm_i915_private *dev_priv,
				      u32 enabled_irqs)
{
	u32 hotplug;

	hotplug = I915_READ(PCH_PORT_HOTPLUG);
	hotplug |= PORTA_HOTPLUG_ENABLE |
		   PORTB_HOTPLUG_ENABLE |
		   PORTC_HOTPLUG_ENABLE;

	DRM_DEBUG_KMS("Invert bit setting: hp_ctl:%x hp_port:%x\n",
		      hotplug, enabled_irqs);
	hotplug &= ~BXT_DDI_HPD_INVERT_MASK;

	/*
	 * For BXT invert bit has to be set based on AOB design
	 * for HPD detection logic, update it based on VBT fields.
	 */
	if ((enabled_irqs & BXT_DE_PORT_HP_DDIA) &&
	    intel_bios_is_port_hpd_inverted(dev_priv, PORT_A))
		hotplug |= BXT_DDIA_HPD_INVERT;
	if ((enabled_irqs & BXT_DE_PORT_HP_DDIB) &&
	    intel_bios_is_port_hpd_inverted(dev_priv, PORT_B))
		hotplug |= BXT_DDIB_HPD_INVERT;
	if ((enabled_irqs & BXT_DE_PORT_HP_DDIC) &&
	    intel_bios_is_port_hpd_inverted(dev_priv, PORT_C))
		hotplug |= BXT_DDIC_HPD_INVERT;

	I915_WRITE(PCH_PORT_HOTPLUG, hotplug);
}

static void bxt_hpd_detection_setup(struct drm_i915_private *dev_priv)
{
	__bxt_hpd_detection_setup(dev_priv, BXT_DE_PORT_HOTPLUG_MASK);
}

static void bxt_hpd_irq_setup(struct drm_i915_private *dev_priv)
{
	u32 hotplug_irqs, enabled_irqs;

	enabled_irqs = intel_hpd_enabled_irqs(dev_priv, hpd_bxt);
	hotplug_irqs = BXT_DE_PORT_HOTPLUG_MASK;

	bdw_update_port_irq(dev_priv, hotplug_irqs, enabled_irqs);

	__bxt_hpd_detection_setup(dev_priv, enabled_irqs);
}

static void ibx_irq_postinstall(struct drm_i915_private *dev_priv)
{
	u32 mask;

	if (HAS_PCH_NOP(dev_priv))
		return;

	if (HAS_PCH_IBX(dev_priv))
		mask = SDE_GMBUS | SDE_AUX_MASK | SDE_POISON;
	else if (HAS_PCH_CPT(dev_priv) || HAS_PCH_LPT(dev_priv))
		mask = SDE_GMBUS_CPT | SDE_AUX_MASK_CPT;
	else
		mask = SDE_GMBUS_CPT;

	gen3_assert_iir_is_zero(&dev_priv->uncore, SDEIIR);
	I915_WRITE(SDEIMR, ~mask);

	if (HAS_PCH_IBX(dev_priv) || HAS_PCH_CPT(dev_priv) ||
	    HAS_PCH_LPT(dev_priv))
		ibx_hpd_detection_setup(dev_priv);
	else
		spt_hpd_detection_setup(dev_priv);
}

=======
>>>>>>> vendor/linux-drm-v6.6.35
static void ilk_irq_postinstall(struct drm_i915_private *dev_priv)
{
	gen5_gt_irq_postinstall(to_gt(dev_priv));

	ilk_de_irq_postinstall(dev_priv);
}

static void valleyview_irq_postinstall(struct drm_i915_private *dev_priv)
{
	gen5_gt_irq_postinstall(to_gt(dev_priv));

	spin_lock_irq(&dev_priv->irq_lock);
	if (dev_priv->display_irqs_enabled)
		vlv_display_irq_postinstall(dev_priv);
	spin_unlock_irq(&dev_priv->irq_lock);

	intel_uncore_write(&dev_priv->uncore, VLV_MASTER_IER, MASTER_INTERRUPT_ENABLE);
	intel_uncore_posting_read(&dev_priv->uncore, VLV_MASTER_IER);
}

static void gen8_irq_postinstall(struct drm_i915_private *dev_priv)
{
	gen8_gt_irq_postinstall(to_gt(dev_priv));
	gen8_de_irq_postinstall(dev_priv);

<<<<<<< HEAD
	if (HAS_PCH_SPLIT(dev_priv))
		ibx_irq_postinstall(dev_priv);

	gen8_master_intr_enable(&dev_priv->uncore);
}

static void icp_irq_postinstall(struct drm_i915_private *dev_priv)
{
	u32 mask = SDE_GMBUS_ICP;

	WARN_ON(I915_READ(SDEIER) != 0);
	I915_WRITE(SDEIER, 0xffffffff);
	POSTING_READ(SDEIER);

	gen3_assert_iir_is_zero(&dev_priv->uncore, SDEIIR);
	I915_WRITE(SDEIMR, ~mask);

	if (HAS_PCH_TGP(dev_priv))
		icp_hpd_detection_setup(dev_priv, TGP_DDI_HPD_ENABLE_MASK,
					TGP_TC_HPD_ENABLE_MASK);
	else if (HAS_PCH_JSP(dev_priv))
		icp_hpd_detection_setup(dev_priv, TGP_DDI_HPD_ENABLE_MASK, 0);
	else if (HAS_PCH_MCC(dev_priv))
		icp_hpd_detection_setup(dev_priv, ICP_DDI_HPD_ENABLE_MASK,
					ICP_TC_HPD_ENABLE(PORT_TC1));
	else
		icp_hpd_detection_setup(dev_priv, ICP_DDI_HPD_ENABLE_MASK,
					ICP_TC_HPD_ENABLE_MASK);
=======
	gen8_master_intr_enable(intel_uncore_regs(&dev_priv->uncore));
>>>>>>> vendor/linux-drm-v6.6.35
}

static void gen11_irq_postinstall(struct drm_i915_private *dev_priv)
{
	struct intel_gt *gt = to_gt(dev_priv);
	struct intel_uncore *uncore = gt->uncore;
	u32 gu_misc_masked = GEN11_GU_MISC_GSE;

	gen11_gt_irq_postinstall(gt);
	gen11_de_irq_postinstall(dev_priv);

	GEN3_IRQ_INIT(uncore, GEN11_GU_MISC_, ~gu_misc_masked, gu_misc_masked);

	gen11_master_intr_enable(intel_uncore_regs(uncore));
	intel_uncore_posting_read(&dev_priv->uncore, GEN11_GFX_MSTR_IRQ);
}

<<<<<<< HEAD
	gen11_master_intr_enable(uncore);
	POSTING_READ(GEN11_GFX_MSTR_IRQ);
=======
static void dg1_irq_postinstall(struct drm_i915_private *dev_priv)
{
	struct intel_uncore *uncore = &dev_priv->uncore;
	u32 gu_misc_masked = GEN11_GU_MISC_GSE;
	struct intel_gt *gt;
	unsigned int i;

	for_each_gt(gt, dev_priv, i)
		gen11_gt_irq_postinstall(gt);

	GEN3_IRQ_INIT(uncore, GEN11_GU_MISC_, ~gu_misc_masked, gu_misc_masked);

	dg1_de_irq_postinstall(dev_priv);

	dg1_master_intr_enable(intel_uncore_regs(uncore));
	intel_uncore_posting_read(uncore, DG1_MSTR_TILE_INTR);
>>>>>>> vendor/linux-drm-v6.6.35
}

static void cherryview_irq_postinstall(struct drm_i915_private *dev_priv)
{
	gen8_gt_irq_postinstall(to_gt(dev_priv));

	spin_lock_irq(&dev_priv->irq_lock);
	if (dev_priv->display_irqs_enabled)
		vlv_display_irq_postinstall(dev_priv);
	spin_unlock_irq(&dev_priv->irq_lock);

	intel_uncore_write(&dev_priv->uncore, GEN8_MASTER_IRQ, GEN8_MASTER_IRQ_CONTROL);
	intel_uncore_posting_read(&dev_priv->uncore, GEN8_MASTER_IRQ);
}

static void i8xx_irq_reset(struct drm_i915_private *dev_priv)
{
	struct intel_uncore *uncore = &dev_priv->uncore;

	i9xx_pipestat_irq_reset(dev_priv);

	gen2_irq_reset(uncore);
	dev_priv->irq_mask = ~0u;
}

static u32 i9xx_error_mask(struct drm_i915_private *i915)
{
	/*
	 * On gen2/3 FBC generates (seemingly spurious)
	 * display INVALID_GTT/INVALID_GTT_PTE table errors.
	 *
	 * Also gen3 bspec has this to say:
	 * "DISPA_INVALID_GTT_PTE
	 "  [DevNapa] : Reserved. This bit does not reflect the page
	 "              table error for the display plane A."
	 *
	 * Unfortunately we can't mask off individual PGTBL_ER bits,
	 * so we just have to mask off all page table errors via EMR.
	 */
	if (HAS_FBC(i915))
		return ~I915_ERROR_MEMORY_REFRESH;
	else
		return ~(I915_ERROR_PAGE_TABLE |
			 I915_ERROR_MEMORY_REFRESH);
}

static void i8xx_irq_postinstall(struct drm_i915_private *dev_priv)
{
	struct intel_uncore *uncore = &dev_priv->uncore;
	u16 enable_mask;

	intel_uncore_write16(uncore, EMR, i9xx_error_mask(dev_priv));

	/* Unmask the interrupts that we always want on. */
	dev_priv->irq_mask =
		~(I915_DISPLAY_PIPE_A_EVENT_INTERRUPT |
		  I915_DISPLAY_PIPE_B_EVENT_INTERRUPT |
		  I915_MASTER_ERROR_INTERRUPT);

	enable_mask =
		I915_DISPLAY_PIPE_A_EVENT_INTERRUPT |
		I915_DISPLAY_PIPE_B_EVENT_INTERRUPT |
		I915_MASTER_ERROR_INTERRUPT |
		I915_USER_INTERRUPT;

	gen2_irq_init(uncore, dev_priv->irq_mask, enable_mask);

	/* Interrupt setup is already guaranteed to be single-threaded, this is
	 * just to make the assert_spin_locked check happy. */
	spin_lock_irq(&dev_priv->irq_lock);
	i915_enable_pipestat(dev_priv, PIPE_A, PIPE_CRC_DONE_INTERRUPT_STATUS);
	i915_enable_pipestat(dev_priv, PIPE_B, PIPE_CRC_DONE_INTERRUPT_STATUS);
	spin_unlock_irq(&dev_priv->irq_lock);
}

static void i8xx_error_irq_ack(struct drm_i915_private *i915,
			       u16 *eir, u16 *eir_stuck)
{
	struct intel_uncore *uncore = &i915->uncore;
	u16 emr;

	*eir = intel_uncore_read16(uncore, EIR);
	intel_uncore_write16(uncore, EIR, *eir);

	*eir_stuck = intel_uncore_read16(uncore, EIR);
	if (*eir_stuck == 0)
		return;

	/*
	 * Toggle all EMR bits to make sure we get an edge
	 * in the ISR master error bit if we don't clear
	 * all the EIR bits. Otherwise the edge triggered
	 * IIR on i965/g4x wouldn't notice that an interrupt
	 * is still pending. Also some EIR bits can't be
	 * cleared except by handling the underlying error
	 * (or by a GPU reset) so we mask any bit that
	 * remains set.
	 */
	emr = intel_uncore_read16(uncore, EMR);
	intel_uncore_write16(uncore, EMR, 0xffff);
	intel_uncore_write16(uncore, EMR, emr | *eir_stuck);
}

static void i8xx_error_irq_handler(struct drm_i915_private *dev_priv,
				   u16 eir, u16 eir_stuck)
{
	drm_dbg(&dev_priv->drm, "Master Error: EIR 0x%04x\n", eir);

	if (eir_stuck)
		drm_dbg(&dev_priv->drm, "EIR stuck: 0x%04x, masked\n",
			eir_stuck);

	drm_dbg(&dev_priv->drm, "PGTBL_ER: 0x%08x\n",
		intel_uncore_read(&dev_priv->uncore, PGTBL_ER));
}

static void i9xx_error_irq_ack(struct drm_i915_private *dev_priv,
			       u32 *eir, u32 *eir_stuck)
{
	u32 emr;

	*eir = intel_uncore_read(&dev_priv->uncore, EIR);
	intel_uncore_write(&dev_priv->uncore, EIR, *eir);

	*eir_stuck = intel_uncore_read(&dev_priv->uncore, EIR);
	if (*eir_stuck == 0)
		return;

	/*
	 * Toggle all EMR bits to make sure we get an edge
	 * in the ISR master error bit if we don't clear
	 * all the EIR bits. Otherwise the edge triggered
	 * IIR on i965/g4x wouldn't notice that an interrupt
	 * is still pending. Also some EIR bits can't be
	 * cleared except by handling the underlying error
	 * (or by a GPU reset) so we mask any bit that
	 * remains set.
	 */
	emr = intel_uncore_read(&dev_priv->uncore, EMR);
	intel_uncore_write(&dev_priv->uncore, EMR, 0xffffffff);
	intel_uncore_write(&dev_priv->uncore, EMR, emr | *eir_stuck);
}

static void i9xx_error_irq_handler(struct drm_i915_private *dev_priv,
				   u32 eir, u32 eir_stuck)
{
	drm_dbg(&dev_priv->drm, "Master Error, EIR 0x%08x\n", eir);

	if (eir_stuck)
		drm_dbg(&dev_priv->drm, "EIR stuck: 0x%08x, masked\n",
			eir_stuck);

	drm_dbg(&dev_priv->drm, "PGTBL_ER: 0x%08x\n",
		intel_uncore_read(&dev_priv->uncore, PGTBL_ER));
}

static irqreturn_t i8xx_irq_handler(DRM_IRQ_ARGS)
{
	struct drm_i915_private *dev_priv = arg;
	irqreturn_t ret = IRQ_NONE;

	if (!intel_irqs_enabled(dev_priv))
		return IRQ_NONE;

	/* IRQs are synced during runtime_suspend, we don't require a wakeref */
	disable_rpm_wakeref_asserts(&dev_priv->runtime_pm);

	do {
		u32 pipe_stats[I915_MAX_PIPES] = {};
		u16 eir = 0, eir_stuck = 0;
		u16 iir;

		iir = intel_uncore_read16(&dev_priv->uncore, GEN2_IIR);
		if (iir == 0)
			break;

		ret = IRQ_HANDLED;

		/* Call regardless, as some status bits might not be
		 * signalled in iir */
		i9xx_pipestat_irq_ack(dev_priv, iir, pipe_stats);

		if (iir & I915_MASTER_ERROR_INTERRUPT)
			i8xx_error_irq_ack(dev_priv, &eir, &eir_stuck);

		intel_uncore_write16(&dev_priv->uncore, GEN2_IIR, iir);

		if (iir & I915_USER_INTERRUPT)
			intel_engine_cs_irq(to_gt(dev_priv)->engine[RCS0], iir);

		if (iir & I915_MASTER_ERROR_INTERRUPT)
			i8xx_error_irq_handler(dev_priv, eir, eir_stuck);

		i8xx_pipestat_irq_handler(dev_priv, iir, pipe_stats);
	} while (0);

	pmu_irq_stats(dev_priv, ret);

	enable_rpm_wakeref_asserts(&dev_priv->runtime_pm);

	return ret;
}

static void i915_irq_reset(struct drm_i915_private *dev_priv)
{
	struct intel_uncore *uncore = &dev_priv->uncore;

	if (I915_HAS_HOTPLUG(dev_priv)) {
		i915_hotplug_interrupt_update(dev_priv, 0xffffffff, 0);
		intel_uncore_rmw(&dev_priv->uncore, PORT_HOTPLUG_STAT, 0, 0);
	}

	i9xx_pipestat_irq_reset(dev_priv);

	GEN3_IRQ_RESET(uncore, GEN2_);
	dev_priv->irq_mask = ~0u;
}

static void i915_irq_postinstall(struct drm_i915_private *dev_priv)
{
	struct intel_uncore *uncore = &dev_priv->uncore;
	u32 enable_mask;

	intel_uncore_write(uncore, EMR, i9xx_error_mask(dev_priv));

	/* Unmask the interrupts that we always want on. */
	dev_priv->irq_mask =
		~(I915_ASLE_INTERRUPT |
		  I915_DISPLAY_PIPE_A_EVENT_INTERRUPT |
		  I915_DISPLAY_PIPE_B_EVENT_INTERRUPT |
		  I915_MASTER_ERROR_INTERRUPT);

	enable_mask =
		I915_ASLE_INTERRUPT |
		I915_DISPLAY_PIPE_A_EVENT_INTERRUPT |
		I915_DISPLAY_PIPE_B_EVENT_INTERRUPT |
		I915_MASTER_ERROR_INTERRUPT |
		I915_USER_INTERRUPT;

	if (I915_HAS_HOTPLUG(dev_priv)) {
		/* Enable in IER... */
		enable_mask |= I915_DISPLAY_PORT_INTERRUPT;
		/* and unmask in IMR */
		dev_priv->irq_mask &= ~I915_DISPLAY_PORT_INTERRUPT;
	}

	GEN3_IRQ_INIT(uncore, GEN2_, dev_priv->irq_mask, enable_mask);

	/* Interrupt setup is already guaranteed to be single-threaded, this is
	 * just to make the assert_spin_locked check happy. */
	spin_lock_irq(&dev_priv->irq_lock);
	i915_enable_pipestat(dev_priv, PIPE_A, PIPE_CRC_DONE_INTERRUPT_STATUS);
	i915_enable_pipestat(dev_priv, PIPE_B, PIPE_CRC_DONE_INTERRUPT_STATUS);
	spin_unlock_irq(&dev_priv->irq_lock);

	i915_enable_asle_pipestat(dev_priv);
}

static irqreturn_t i915_irq_handler(DRM_IRQ_ARGS)
{
	struct drm_i915_private *dev_priv = arg;
	irqreturn_t ret = IRQ_NONE;

	if (!intel_irqs_enabled(dev_priv))
		return IRQ_NONE;

	/* IRQs are synced during runtime_suspend, we don't require a wakeref */
	disable_rpm_wakeref_asserts(&dev_priv->runtime_pm);

	do {
		u32 pipe_stats[I915_MAX_PIPES] = {};
		u32 eir = 0, eir_stuck = 0;
		u32 hotplug_status = 0;
		u32 iir;

		iir = intel_uncore_read(&dev_priv->uncore, GEN2_IIR);
		if (iir == 0)
			break;

		ret = IRQ_HANDLED;

		if (I915_HAS_HOTPLUG(dev_priv) &&
		    iir & I915_DISPLAY_PORT_INTERRUPT)
			hotplug_status = i9xx_hpd_irq_ack(dev_priv);

		/* Call regardless, as some status bits might not be
		 * signalled in iir */
		i9xx_pipestat_irq_ack(dev_priv, iir, pipe_stats);

		if (iir & I915_MASTER_ERROR_INTERRUPT)
			i9xx_error_irq_ack(dev_priv, &eir, &eir_stuck);

		intel_uncore_write(&dev_priv->uncore, GEN2_IIR, iir);

		if (iir & I915_USER_INTERRUPT)
			intel_engine_cs_irq(to_gt(dev_priv)->engine[RCS0], iir);

		if (iir & I915_MASTER_ERROR_INTERRUPT)
			i9xx_error_irq_handler(dev_priv, eir, eir_stuck);

		if (hotplug_status)
			i9xx_hpd_irq_handler(dev_priv, hotplug_status);

		i915_pipestat_irq_handler(dev_priv, iir, pipe_stats);
	} while (0);

	pmu_irq_stats(dev_priv, ret);

	enable_rpm_wakeref_asserts(&dev_priv->runtime_pm);

	return ret;
}

static void i965_irq_reset(struct drm_i915_private *dev_priv)
{
	struct intel_uncore *uncore = &dev_priv->uncore;

	i915_hotplug_interrupt_update(dev_priv, 0xffffffff, 0);
	intel_uncore_rmw(uncore, PORT_HOTPLUG_STAT, 0, 0);

	i9xx_pipestat_irq_reset(dev_priv);

	GEN3_IRQ_RESET(uncore, GEN2_);
	dev_priv->irq_mask = ~0u;
}

static u32 i965_error_mask(struct drm_i915_private *i915)
{
	/*
	 * Enable some error detection, note the instruction error mask
	 * bit is reserved, so we leave it masked.
	 *
	 * i965 FBC no longer generates spurious GTT errors,
	 * so we can always enable the page table errors.
	 */
	if (IS_G4X(i915))
		return ~(GM45_ERROR_PAGE_TABLE |
			 GM45_ERROR_MEM_PRIV |
			 GM45_ERROR_CP_PRIV |
			 I915_ERROR_MEMORY_REFRESH);
	else
		return ~(I915_ERROR_PAGE_TABLE |
			 I915_ERROR_MEMORY_REFRESH);
}

static void i965_irq_postinstall(struct drm_i915_private *dev_priv)
{
	struct intel_uncore *uncore = &dev_priv->uncore;
	u32 enable_mask;

	intel_uncore_write(uncore, EMR, i965_error_mask(dev_priv));

	/* Unmask the interrupts that we always want on. */
	dev_priv->irq_mask =
		~(I915_ASLE_INTERRUPT |
		  I915_DISPLAY_PORT_INTERRUPT |
		  I915_DISPLAY_PIPE_A_EVENT_INTERRUPT |
		  I915_DISPLAY_PIPE_B_EVENT_INTERRUPT |
		  I915_MASTER_ERROR_INTERRUPT);

	enable_mask =
		I915_ASLE_INTERRUPT |
		I915_DISPLAY_PORT_INTERRUPT |
		I915_DISPLAY_PIPE_A_EVENT_INTERRUPT |
		I915_DISPLAY_PIPE_B_EVENT_INTERRUPT |
		I915_MASTER_ERROR_INTERRUPT |
		I915_USER_INTERRUPT;

	if (IS_G4X(dev_priv))
		enable_mask |= I915_BSD_USER_INTERRUPT;

	GEN3_IRQ_INIT(uncore, GEN2_, dev_priv->irq_mask, enable_mask);

	/* Interrupt setup is already guaranteed to be single-threaded, this is
	 * just to make the assert_spin_locked check happy. */
	spin_lock_irq(&dev_priv->irq_lock);
	i915_enable_pipestat(dev_priv, PIPE_A, PIPE_GMBUS_INTERRUPT_STATUS);
	i915_enable_pipestat(dev_priv, PIPE_A, PIPE_CRC_DONE_INTERRUPT_STATUS);
	i915_enable_pipestat(dev_priv, PIPE_B, PIPE_CRC_DONE_INTERRUPT_STATUS);
	spin_unlock_irq(&dev_priv->irq_lock);

	i915_enable_asle_pipestat(dev_priv);
}

<<<<<<< HEAD
static void i915_hpd_irq_setup(struct drm_i915_private *dev_priv)
{
	u32 hotplug_en;

	lockdep_assert_held(&dev_priv->irq_lock);

	/* Note HDMI and DP share hotplug bits */
	/* enable bits are the same for all generations */
	hotplug_en = intel_hpd_enabled_irqs(dev_priv, hpd_mask_i915);
	/* Programming the CRT detection parameters tends
	   to generate a spurious hotplug event about three
	   seconds later.  So just do it once.
	*/
	if (IS_G4X(dev_priv))
		hotplug_en |= CRT_HOTPLUG_ACTIVATION_PERIOD_64;
	hotplug_en |= CRT_HOTPLUG_VOLTAGE_COMPARE_50;

	/* Ignore TV since it's buggy */
	i915_hotplug_interrupt_update_locked(dev_priv,
					     HOTPLUG_INT_EN_MASK |
					     CRT_HOTPLUG_VOLTAGE_COMPARE_MASK |
					     CRT_HOTPLUG_ACTIVATION_PERIOD_64,
					     hotplug_en);
}

static irqreturn_t i965_irq_handler(DRM_IRQ_ARGS)
=======
static irqreturn_t i965_irq_handler(int irq, void *arg)
>>>>>>> vendor/linux-drm-v6.6.35
{
	struct drm_i915_private *dev_priv = arg;
	irqreturn_t ret = IRQ_NONE;

	if (!intel_irqs_enabled(dev_priv))
		return IRQ_NONE;

	/* IRQs are synced during runtime_suspend, we don't require a wakeref */
	disable_rpm_wakeref_asserts(&dev_priv->runtime_pm);

	do {
		u32 pipe_stats[I915_MAX_PIPES] = {};
		u32 eir = 0, eir_stuck = 0;
		u32 hotplug_status = 0;
		u32 iir;

		iir = intel_uncore_read(&dev_priv->uncore, GEN2_IIR);
		if (iir == 0)
			break;

		ret = IRQ_HANDLED;

		if (iir & I915_DISPLAY_PORT_INTERRUPT)
			hotplug_status = i9xx_hpd_irq_ack(dev_priv);

		/* Call regardless, as some status bits might not be
		 * signalled in iir */
		i9xx_pipestat_irq_ack(dev_priv, iir, pipe_stats);

		if (iir & I915_MASTER_ERROR_INTERRUPT)
			i9xx_error_irq_ack(dev_priv, &eir, &eir_stuck);

		intel_uncore_write(&dev_priv->uncore, GEN2_IIR, iir);

		if (iir & I915_USER_INTERRUPT)
			intel_engine_cs_irq(to_gt(dev_priv)->engine[RCS0],
					    iir);

		if (iir & I915_BSD_USER_INTERRUPT)
			intel_engine_cs_irq(to_gt(dev_priv)->engine[VCS0],
					    iir >> 25);

		if (iir & I915_MASTER_ERROR_INTERRUPT)
			i9xx_error_irq_handler(dev_priv, eir, eir_stuck);

		if (hotplug_status)
			i9xx_hpd_irq_handler(dev_priv, hotplug_status);

		i965_pipestat_irq_handler(dev_priv, iir, pipe_stats);
	} while (0);

	pmu_irq_stats(dev_priv, IRQ_HANDLED);

	enable_rpm_wakeref_asserts(&dev_priv->runtime_pm);

	return ret;
}

/**
 * intel_irq_init - initializes irq support
 * @dev_priv: i915 device instance
 *
 * This function initializes all the irq support including work items, timers
 * and all the vtables. It does not setup the interrupt itself though.
 */
void intel_irq_init(struct drm_i915_private *dev_priv)
{
	int i;

	INIT_WORK(&dev_priv->l3_parity.error_work, ivb_parity_work);
	for (i = 0; i < MAX_L3_SLICES; ++i)
		dev_priv->l3_parity.remap_info[i] = NULL;

	/* pre-gen11 the guc irqs bits are in the upper 16 bits of the pm reg */
	if (HAS_GT_UC(dev_priv) && GRAPHICS_VER(dev_priv) < 11)
		to_gt(dev_priv)->pm_guc_events = GUC_INTR_GUC2HOST << 16;
}

/**
 * intel_irq_fini - deinitializes IRQ support
 * @i915: i915 device instance
 *
 * This function deinitializes all the IRQ support.
 */
void intel_irq_fini(struct drm_i915_private *i915)
{
	int i;

	for (i = 0; i < MAX_L3_SLICES; ++i)
		kfree(i915->l3_parity.remap_info[i]);
}

static irq_handler_t intel_irq_handler(struct drm_i915_private *dev_priv)
{
	if (HAS_GMCH(dev_priv)) {
		if (IS_CHERRYVIEW(dev_priv))
			return cherryview_irq_handler;
		else if (IS_VALLEYVIEW(dev_priv))
			return valleyview_irq_handler;
		else if (GRAPHICS_VER(dev_priv) == 4)
			return i965_irq_handler;
		else if (GRAPHICS_VER(dev_priv) == 3)
			return i915_irq_handler;
		else
			return i8xx_irq_handler;
	} else {
		if (GRAPHICS_VER_FULL(dev_priv) >= IP_VER(12, 10))
			return dg1_irq_handler;
		else if (GRAPHICS_VER(dev_priv) >= 11)
			return gen11_irq_handler;
		else if (GRAPHICS_VER(dev_priv) >= 8)
			return gen8_irq_handler;
		else
			return ilk_irq_handler;
	}
}

static void intel_irq_reset(struct drm_i915_private *dev_priv)
{
	if (HAS_GMCH(dev_priv)) {
		if (IS_CHERRYVIEW(dev_priv))
			cherryview_irq_reset(dev_priv);
		else if (IS_VALLEYVIEW(dev_priv))
			valleyview_irq_reset(dev_priv);
		else if (GRAPHICS_VER(dev_priv) == 4)
			i965_irq_reset(dev_priv);
		else if (GRAPHICS_VER(dev_priv) == 3)
			i915_irq_reset(dev_priv);
		else
			i8xx_irq_reset(dev_priv);
	} else {
		if (GRAPHICS_VER_FULL(dev_priv) >= IP_VER(12, 10))
			dg1_irq_reset(dev_priv);
		else if (GRAPHICS_VER(dev_priv) >= 11)
			gen11_irq_reset(dev_priv);
		else if (GRAPHICS_VER(dev_priv) >= 8)
			gen8_irq_reset(dev_priv);
		else
			ilk_irq_reset(dev_priv);
	}
}

static void intel_irq_postinstall(struct drm_i915_private *dev_priv)
{
	if (HAS_GMCH(dev_priv)) {
		if (IS_CHERRYVIEW(dev_priv))
			cherryview_irq_postinstall(dev_priv);
		else if (IS_VALLEYVIEW(dev_priv))
			valleyview_irq_postinstall(dev_priv);
		else if (GRAPHICS_VER(dev_priv) == 4)
			i965_irq_postinstall(dev_priv);
		else if (GRAPHICS_VER(dev_priv) == 3)
			i915_irq_postinstall(dev_priv);
		else
			i8xx_irq_postinstall(dev_priv);
	} else {
		if (GRAPHICS_VER_FULL(dev_priv) >= IP_VER(12, 10))
			dg1_irq_postinstall(dev_priv);
		else if (GRAPHICS_VER(dev_priv) >= 11)
			gen11_irq_postinstall(dev_priv);
		else if (GRAPHICS_VER(dev_priv) >= 8)
			gen8_irq_postinstall(dev_priv);
		else
			ilk_irq_postinstall(dev_priv);
	}
}

/**
 * intel_irq_install - enables the hardware interrupt
 * @dev_priv: i915 device instance
 *
 * This function enables the hardware interrupt handling, but leaves the hotplug
 * handling still disabled. It is called after intel_irq_init().
 *
 * In the driver load and resume code we need working interrupts in a few places
 * but don't want to deal with the hassle of concurrent probe and hotplug
 * workers. Hence the split into this two-stage approach.
 */
int intel_irq_install(struct drm_i915_private *dev_priv)
{
<<<<<<< HEAD
#ifndef __NetBSD__
	int irq = dev_priv->drm.pdev->irq;
#endif
=======
	int irq = to_pci_dev(dev_priv->drm.dev)->irq;
>>>>>>> vendor/linux-drm-v6.6.35
	int ret;

	/*
	 * We enable some interrupt sources in our postinstall hooks, so mark
	 * interrupts as enabled _before_ actually enabling them to avoid
	 * special cases in our ordering checks.
	 */
	dev_priv->runtime_pm.irqs_enabled = true;

	dev_priv->irq_enabled = true;

	intel_irq_reset(dev_priv);

#ifdef __NetBSD__
    {
	struct pci_dev *const pdev = dev_priv->drm.pdev;
	const char *const name = device_xname(pci_dev_dev(pdev));
	const struct pci_attach_args *pa = &pdev->pd_pa;
	const char *intrstr;
	char intrbuf[PCI_INTRSTR_LEN];

	if (pdev->msi_enabled) {
		if (pdev->pd_intr_handles == NULL) {
			/* XXX errno NetBSD->Linux */
			if ((ret = -pci_msi_alloc_exact(pa, &dev_priv->pci_ihp,
			    1))) {
				aprint_error_dev(pci_dev_dev(pdev),
				    "couldn't allocate MSI (%s)\n", name);
				goto out;
			}
		} else {
			dev_priv->pci_ihp = pdev->pd_intr_handles;
			pdev->pd_intr_handles = NULL;
		}
	} else {
		/* XXX errno NetBSD->Linux */
		if ((ret = -pci_intx_alloc(pa, &dev_priv->pci_ihp))) {
			aprint_error_dev(pci_dev_dev(pdev),
			    "couldn't allocate INTx interrupt (%s)\n",
			    name);
			goto out;
		}
	}

	intrstr = pci_intr_string(pa->pa_pc, dev_priv->pci_ihp[0],
	    intrbuf, sizeof(intrbuf));
	dev_priv->pci_intrcookie = pci_intr_establish_xname(pa->pa_pc,
	    dev_priv->pci_ihp[0], IPL_DRM, intel_irq_handler(dev_priv),
	    dev_priv, name);
	if (dev_priv->pci_intrcookie == NULL) {
		aprint_error_dev(pci_dev_dev(pdev),
		    "couldn't establish interrupt at %s (%s)\n", intrstr, name);
		pci_intr_release(pa->pa_pc, dev_priv->pci_ihp, 1);
		dev_priv->pci_ihp = NULL;
		ret = -EIO;	/* XXX er? */
		goto out;
	}

	/* Success!  */
	aprint_normal_dev(pci_dev_dev(pdev), "interrupting at %s (%s)\n",
	    intrstr, name);
	ret = 0;
out:;
    }
#else
	ret = request_irq(irq, intel_irq_handler(dev_priv),
			  IRQF_SHARED, DRIVER_NAME, dev_priv);
#endif
	if (ret < 0) {
		dev_priv->irq_enabled = false;
		return ret;
	}

	intel_irq_postinstall(dev_priv);

	return ret;
}

/**
 * intel_irq_uninstall - finilizes all irq handling
 * @dev_priv: i915 device instance
 *
 * This stops interrupt and hotplug handling and unregisters and frees all
 * resources acquired in the init functions.
 */
void intel_irq_uninstall(struct drm_i915_private *dev_priv)
{
<<<<<<< HEAD
#ifndef __NetBSD__
	int irq = dev_priv->drm.pdev->irq;
#endif
=======
	int irq = to_pci_dev(dev_priv->drm.dev)->irq;
>>>>>>> vendor/linux-drm-v6.6.35

	/*
	 * FIXME we can get called twice during driver probe
	 * error handling as well as during driver remove due to
	 * intel_display_driver_remove() calling us out of sequence.
	 * Would be nice if it didn't do that...
	 */
	if (!dev_priv->irq_enabled)
		return;

	dev_priv->irq_enabled = false;

	intel_irq_reset(dev_priv);

#ifdef __NetBSD__
	const struct pci_attach_args *pa = &dev_priv->drm.pdev->pd_pa;
	if (dev_priv->pci_intrcookie != NULL) {
		pci_intr_disestablish(pa->pa_pc, dev_priv->pci_intrcookie);
		dev_priv->pci_intrcookie = NULL;
	}
	if (dev_priv->pci_ihp != NULL) {
		pci_intr_release(pa->pa_pc, dev_priv->pci_ihp, 1);
		dev_priv->pci_ihp = NULL;
	}
#else
	free_irq(irq, dev_priv);
#endif

	intel_hpd_cancel_work(dev_priv);
	dev_priv->runtime_pm.irqs_enabled = false;
}

/**
 * intel_runtime_pm_disable_interrupts - runtime interrupt disabling
 * @dev_priv: i915 device instance
 *
 * This function is used to disable interrupts at runtime, both in the runtime
 * pm and the system suspend/resume code.
 */
void intel_runtime_pm_disable_interrupts(struct drm_i915_private *dev_priv)
{
	intel_irq_reset(dev_priv);
	dev_priv->runtime_pm.irqs_enabled = false;
	intel_synchronize_irq(dev_priv);
}

/**
 * intel_runtime_pm_enable_interrupts - runtime interrupt enabling
 * @dev_priv: i915 device instance
 *
 * This function is used to enable interrupts at runtime, both in the runtime
 * pm and the system suspend/resume code.
 */
void intel_runtime_pm_enable_interrupts(struct drm_i915_private *dev_priv)
{
	dev_priv->runtime_pm.irqs_enabled = true;
	intel_irq_reset(dev_priv);
	intel_irq_postinstall(dev_priv);
}

bool intel_irqs_enabled(struct drm_i915_private *dev_priv)
{
	return dev_priv->runtime_pm.irqs_enabled;
}

void intel_synchronize_irq(struct drm_i915_private *i915)
{
<<<<<<< HEAD
#ifdef __NetBSD__
	xc_barrier(0);
#else
	synchronize_irq(i915->drm.pdev->irq);
#endif
=======
	synchronize_irq(to_pci_dev(i915->drm.dev)->irq);
}

void intel_synchronize_hardirq(struct drm_i915_private *i915)
{
	synchronize_hardirq(to_pci_dev(i915->drm.dev)->irq);
>>>>>>> vendor/linux-drm-v6.6.35
}
