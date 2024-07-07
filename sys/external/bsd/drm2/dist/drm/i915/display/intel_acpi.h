/*	$NetBSD: intel_acpi.h,v 1.5 2024/04/16 14:34:01 riastradh Exp $	*/

/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2019 Intel Corporation
 */

#ifndef __INTEL_ACPI_H__
#define __INTEL_ACPI_H__

<<<<<<< HEAD
#include <linux/acpi.h>

=======
>>>>>>> vendor/linux-drm-v6.6.35
struct drm_i915_private;

#ifdef CONFIG_ACPI
#ifdef __NetBSD__
void intel_register_dsm_handler(struct drm_i915_private *);
#else
void intel_register_dsm_handler(void);
#endif
void intel_unregister_dsm_handler(void);
void intel_dsm_get_bios_data_funcs_supported(struct drm_i915_private *i915);
void intel_acpi_device_id_update(struct drm_i915_private *i915);
void intel_acpi_assign_connector_fwnodes(struct drm_i915_private *i915);
void intel_acpi_video_register(struct drm_i915_private *i915);
#else
#ifdef __NetBSD__
static inline void intel_register_dsm_handler(struct drm_i915_private *i915) { return; }
#else
static inline void intel_register_dsm_handler(void) { return; }
#endif
static inline void intel_unregister_dsm_handler(void) { return; }
static inline
void intel_dsm_get_bios_data_funcs_supported(struct drm_i915_private *i915) { return; }
static inline
void intel_acpi_device_id_update(struct drm_i915_private *i915) { return; }
static inline
void intel_acpi_assign_connector_fwnodes(struct drm_i915_private *i915) { return; }
static inline
void intel_acpi_video_register(struct drm_i915_private *i915) { return; }
#endif /* CONFIG_ACPI */

#endif /* __INTEL_ACPI_H__ */
