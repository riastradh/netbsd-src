/*	$NetBSD: intel_gtt.c,v 1.9 2021/12/19 12:10:42 riastradh Exp $	*/

// SPDX-License-Identifier: MIT
/*
 * Copyright © 2020 Intel Corporation
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: intel_gtt.c,v 1.9 2021/12/19 12:10:42 riastradh Exp $");

#include <linux/slab.h> /* fault-inject.h is not standalone! */

#include <linux/fault-inject.h>
#include <linux/sched/mm.h>

#include <drm/drm_cache.h>

#include "gem/i915_gem_internal.h"
#include "gem/i915_gem_lmem.h"
#include "i915_reg.h"
#include "i915_trace.h"
#include "i915_utils.h"
#include "intel_gt.h"
#include "intel_gt_mcr.h"
#include "intel_gt_print.h"
#include "intel_gt_regs.h"
#include "intel_gtt.h"

<<<<<<< HEAD
#include <linux/nbsd-namespace.h>

#ifndef __NetBSD__
void stash_init(struct pagestash *stash)
=======

static bool intel_ggtt_update_needs_vtd_wa(struct drm_i915_private *i915)
>>>>>>> vendor/linux-drm-v6.6.35
{
	return IS_BROXTON(i915) && i915_vtd_active(i915);
}

bool intel_vm_no_concurrent_access_wa(struct drm_i915_private *i915)
{
	return IS_CHERRYVIEW(i915) || intel_ggtt_update_needs_vtd_wa(i915);
}

struct drm_i915_gem_object *alloc_pt_lmem(struct i915_address_space *vm, int sz)
{
	struct drm_i915_gem_object *obj;

	/*
	 * To avoid severe over-allocation when dealing with min_page_size
	 * restrictions, we override that behaviour here by allowing an object
	 * size and page layout which can be smaller. In practice this should be
	 * totally fine, since GTT paging structures are not typically inserted
	 * into the GTT.
	 *
	 * Note that we also hit this path for the scratch page, and for this
	 * case it might need to be 64K, but that should work fine here since we
	 * used the passed in size for the page size, which should ensure it
	 * also has the same alignment.
	 */
	obj = __i915_gem_object_create_lmem_with_ps(vm->i915, sz, sz,
						    vm->lmem_pt_obj_flags);
	/*
	 * Ensure all paging structures for this vm share the same dma-resv
	 * object underneath, with the idea that one object_lock() will lock
	 * them all at once.
	 */
	if (!IS_ERR(obj)) {
		obj->base.resv = i915_vm_resv_get(vm);
		obj->shares_resv_from = vm;
	}

	return obj;
}

struct drm_i915_gem_object *alloc_pt_dma(struct i915_address_space *vm, int sz)
{
	struct drm_i915_gem_object *obj;

	if (I915_SELFTEST_ONLY(should_fail(&vm->fault_attr, 1)))
		i915_gem_shrink_all(vm->i915);

	obj = i915_gem_object_create_internal(vm->i915, sz);
	/*
	 * Ensure all paging structures for this vm share the same dma-resv
	 * object underneath, with the idea that one object_lock() will lock
	 * them all at once.
	 */
	if (!IS_ERR(obj)) {
		obj->base.resv = i915_vm_resv_get(vm);
		obj->shares_resv_from = vm;
	}

	return obj;
}

int map_pt_dma(struct i915_address_space *vm, struct drm_i915_gem_object *obj)
{
	enum i915_map_type type;
	void *vaddr;

	type = intel_gt_coherent_map_type(vm->gt, obj, true);
	vaddr = i915_gem_object_pin_map_unlocked(obj, type);
	if (IS_ERR(vaddr))
		return PTR_ERR(vaddr);

	i915_gem_object_make_unshrinkable(obj);
	return 0;
}

int map_pt_dma_locked(struct i915_address_space *vm, struct drm_i915_gem_object *obj)
{
	enum i915_map_type type;
	void *vaddr;

	type = intel_gt_coherent_map_type(vm->gt, obj, true);
	vaddr = i915_gem_object_pin_map(obj, type);
	if (IS_ERR(vaddr))
		return PTR_ERR(vaddr);

	i915_gem_object_make_unshrinkable(obj);
	return 0;
}
#endif

static void clear_vm_list(struct list_head *list)
{
	struct i915_vma *vma, *vn;

	list_for_each_entry_safe(vma, vn, list, vm_link) {
		struct drm_i915_gem_object *obj = vma->obj;

		if (!i915_gem_object_get_rcu(obj)) {
			/*
			 * Object is dying, but has not yet cleared its
			 * vma list.
			 * Unbind the dying vma to ensure our list
			 * is completely drained. We leave the destruction to
			 * the object destructor to avoid the vma
			 * disappearing under it.
			 */
			atomic_and(~I915_VMA_PIN_MASK, &vma->flags);
			WARN_ON(__i915_vma_unbind(vma));

			/* Remove from the unbound list */
			list_del_init(&vma->vm_link);

			/*
			 * Delay the vm and vm mutex freeing until the
			 * object is done with destruction.
			 */
			i915_vm_resv_get(vma->vm);
			vma->vm_ddestroy = true;
		} else {
			i915_vma_destroy_locked(vma);
			i915_gem_object_put(obj);
		}

	}
}

static void __i915_vm_close(struct i915_address_space *vm)
{
	mutex_lock(&vm->mutex);

	clear_vm_list(&vm->bound_list);
	clear_vm_list(&vm->unbound_list);

	/* Check for must-fix unanticipated side-effects */
	GEM_BUG_ON(!list_empty(&vm->bound_list));
	GEM_BUG_ON(!list_empty(&vm->unbound_list));

	mutex_unlock(&vm->mutex);
}

/* lock the vm into the current ww, if we lock one, we lock all */
int i915_vm_lock_objects(struct i915_address_space *vm,
			 struct i915_gem_ww_ctx *ww)
{
	if (vm->scratch[0]->base.resv == &vm->_resv) {
		return i915_gem_object_lock(vm->scratch[0], ww);
	} else {
		struct i915_ppgtt *ppgtt = i915_vm_to_ppgtt(vm);

		/* We borrowed the scratch page from ggtt, take the top level object */
		return i915_gem_object_lock(ppgtt->pd->pt.base, ww);
	}
}

void i915_address_space_fini(struct i915_address_space *vm)
{
<<<<<<< HEAD
#ifndef __NetBSD__
	spin_lock(&vm->free_pages.lock);
	if (pagevec_count(&vm->free_pages.pvec))
		vm_free_pages_release(vm, true);
	GEM_BUG_ON(pagevec_count(&vm->free_pages.pvec));
	spin_unlock(&vm->free_pages.lock);
#endif

=======
>>>>>>> vendor/linux-drm-v6.6.35
	drm_mm_takedown(&vm->mm);
}

/**
 * i915_vm_resv_release - Final struct i915_address_space destructor
 * @kref: Pointer to the &i915_address_space.resv_ref member.
 *
 * This function is called when the last lock sharer no longer shares the
 * &i915_address_space._resv lock, and also if we raced when
 * destroying a vma by the vma destruction
 */
void i915_vm_resv_release(struct kref *kref)
{
	struct i915_address_space *vm =
		container_of(kref, typeof(*vm), resv_ref);

	dma_resv_fini(&vm->_resv);
	mutex_destroy(&vm->mutex);

	kfree(vm);
}

static void __i915_vm_release(struct work_struct *work)
{
	struct i915_address_space *vm =
		container_of(work, struct i915_address_space, release_work);

	__i915_vm_close(vm);

	/* Synchronize async unbinds. */
	i915_vma_resource_bind_dep_sync_all(vm);

	vm->cleanup(vm);
	i915_address_space_fini(vm);

	i915_vm_resv_put(vm);
}

void i915_vm_release(struct kref *kref)
{
	struct i915_address_space *vm =
		container_of(kref, struct i915_address_space, ref);

	GEM_BUG_ON(i915_is_ggtt(vm));
	trace_i915_ppgtt_release(vm);

	queue_work(vm->i915->wq, &vm->release_work);
}

void i915_address_space_init(struct i915_address_space *vm, int subclass)
{
	kref_init(&vm->ref);

	/*
	 * Special case for GGTT that has already done an early
	 * kref_init here.
	 */
	if (!kref_read(&vm->resv_ref))
		kref_init(&vm->resv_ref);

	vm->pending_unbind = RB_ROOT_CACHED;
	INIT_WORK(&vm->release_work, __i915_vm_release);

	/*
	 * The vm->mutex must be reclaim safe (for use in the shrinker).
	 * Do a dummy acquire now under fs_reclaim so that any allocation
	 * attempt holding the lock is immediately reported by lockdep.
	 */
	mutex_init(&vm->mutex);
	lockdep_set_subclass(&vm->mutex, subclass);

	if (!intel_vm_no_concurrent_access_wa(vm->i915)) {
		i915_gem_shrinker_taints_mutex(vm->i915, &vm->mutex);
	} else {
		/*
		 * CHV + BXT VTD workaround use stop_machine(),
		 * which is allowed to allocate memory. This means &vm->mutex
		 * is the outer lock, and in theory we can allocate memory inside
		 * it through stop_machine().
		 *
		 * Add the annotation for this, we use trylock in shrinker.
		 */
		mutex_acquire(&vm->mutex.dep_map, 0, 0, _THIS_IP_);
		might_alloc(GFP_KERNEL);
		mutex_release(&vm->mutex.dep_map, _THIS_IP_);
	}
	dma_resv_init(&vm->_resv);

	GEM_BUG_ON(!vm->total);
	drm_mm_init(&vm->mm, 0, vm->total);

	memset64(vm->min_alignment, I915_GTT_MIN_ALIGNMENT,
		 ARRAY_SIZE(vm->min_alignment));

	if (HAS_64K_PAGES(vm->i915)) {
		vm->min_alignment[INTEL_MEMORY_LOCAL] = I915_GTT_PAGE_SIZE_64K;
		vm->min_alignment[INTEL_MEMORY_STOLEN_LOCAL] = I915_GTT_PAGE_SIZE_64K;
	}

	vm->mm.head_node.color = I915_COLOR_UNEVICTABLE;

<<<<<<< HEAD
#ifdef __NetBSD__
	vm->dmat = vm->i915->drm.dmat;
#else
	stash_init(&vm->free_pages);
#endif

=======
>>>>>>> vendor/linux-drm-v6.6.35
	INIT_LIST_HEAD(&vm->bound_list);
	INIT_LIST_HEAD(&vm->unbound_list);
}

void *__px_vaddr(struct drm_i915_gem_object *p)
{
	enum i915_map_type type;

	GEM_BUG_ON(!i915_gem_object_has_pages(p));
	return page_unpack_bits(p->mm.mapping, &type);
}

dma_addr_t __px_dma(struct drm_i915_gem_object *p)
{
<<<<<<< HEAD
#ifdef __NetBSD__
	int busdmaflags = 0;
	int error;
	int nseg = 1;

	if (gfp & __GFP_WAIT)
		busdmaflags |= BUS_DMA_WAITOK;
	else
		busdmaflags |= BUS_DMA_NOWAIT;

	error = bus_dmamem_alloc(vm->dmat, PAGE_SIZE, PAGE_SIZE, 0, &p->seg,
	    nseg, &nseg, busdmaflags);
	if (error) {
fail0:		p->map = NULL;
		return -error;	/* XXX errno NetBSD->Linux */
	}
	KASSERT(nseg == 1);
	error = bus_dmamap_create(vm->dmat, PAGE_SIZE, 1, PAGE_SIZE, 0,
	    busdmaflags, &p->map);
	if (error) {
fail1:		bus_dmamem_free(vm->dmat, &p->seg, 1);
		goto fail0;
	}
	error = bus_dmamap_load_raw(vm->dmat, p->map, &p->seg, 1, PAGE_SIZE,
	    busdmaflags);
	if (error) {
fail2: __unused
		bus_dmamap_destroy(vm->dmat, p->map);
		goto fail1;
	}

	p->page = container_of(PHYS_TO_VM_PAGE(p->seg.ds_addr), struct page,
	    p_vmp);

	if (gfp & __GFP_ZERO) {
		void *va = kmap_atomic(p->page);
		memset(va, 0, PAGE_SIZE);
		kunmap_atomic(va);
	}
#else
	p->page = vm_alloc_page(vm, gfp | I915_GFP_ALLOW_FAIL);
	if (unlikely(!p->page))
		return -ENOMEM;

	p->daddr = dma_map_page_attrs(vm->dma,
				      p->page, 0, PAGE_SIZE,
				      PCI_DMA_BIDIRECTIONAL,
				      DMA_ATTR_SKIP_CPU_SYNC |
				      DMA_ATTR_NO_WARN);
	if (unlikely(dma_mapping_error(vm->dma, p->daddr))) {
		vm_free_page(vm, p->page);
		return -ENOMEM;
	}
#endif

	return 0;
=======
	GEM_BUG_ON(!i915_gem_object_has_pages(p));
	return sg_dma_address(p->mm.pages->sgl);
>>>>>>> vendor/linux-drm-v6.6.35
}

struct page *__px_page(struct drm_i915_gem_object *p)
{
<<<<<<< HEAD
	return __setup_page_dma(vm, p, __GFP_HIGHMEM);
}

void cleanup_page_dma(struct i915_address_space *vm, struct i915_page_dma *p)
{
#ifdef __NetBSD__
	bus_dmamap_unload(vm->dmat, p->map);
	bus_dmamap_destroy(vm->dmat, p->map);
	bus_dmamem_free(vm->dmat, &p->seg, 1);
#else
	dma_unmap_page(vm->dma, p->daddr, PAGE_SIZE, PCI_DMA_BIDIRECTIONAL);
	vm_free_page(vm, p->page);
#endif
=======
	GEM_BUG_ON(!i915_gem_object_has_pages(p));
	return sg_page(p->mm.pages->sgl);
>>>>>>> vendor/linux-drm-v6.6.35
}

void
fill_page_dma(struct drm_i915_gem_object *p, const u64 val, unsigned int count)
{
	void *vaddr = __px_vaddr(p);

	memset64(vaddr, val, count);
	drm_clflush_virt_range(vaddr, PAGE_SIZE);
}

static void poison_scratch_page(struct drm_i915_gem_object *scratch)
{
	void *vaddr = __px_vaddr(scratch);
	u8 val;

	val = 0;
	if (IS_ENABLED(CONFIG_DRM_I915_DEBUG_GEM))
		val = POISON_FREE;

	memset(vaddr, val, scratch->base.size);
	drm_clflush_virt_range(vaddr, scratch->base.size);
}

int setup_scratch_page(struct i915_address_space *vm)
{
	unsigned long size;

	/*
	 * In order to utilize 64K pages for an object with a size < 2M, we will
	 * need to support a 64K scratch page, given that every 16th entry for a
	 * page-table operating in 64K mode must point to a properly aligned 64K
	 * region, including any PTEs which happen to point to scratch.
	 *
	 * This is only relevant for the 48b PPGTT where we support
	 * huge-gtt-pages, see also i915_vma_insert(). However, as we share the
	 * scratch (read-only) between all vm, we create one 64k scratch page
	 * for all.
	 */
	size = I915_GTT_PAGE_SIZE_4K;
	if (i915_vm_is_4lvl(vm) &&
	    HAS_PAGE_SIZES(vm->i915, I915_GTT_PAGE_SIZE_64K) &&
	    !HAS_64K_PAGES(vm->i915))
		size = I915_GTT_PAGE_SIZE_64K;

	do {
<<<<<<< HEAD
		unsigned int order = get_order(size);
#ifdef __NetBSD__
		struct vm_page *vm_page;
		void *kva;
		int nseg;
		int ret;

		/* Allocate a scratch page.  */
		/* XXX errno NetBSD->Linux */
		ret = -bus_dmamem_alloc(vm->dmat, size, size, 0,
		    &vm->scratch[0].base.seg, 1, &nseg, BUS_DMA_NOWAIT);
		if (ret)
			goto skip;
		KASSERT(nseg == 1);
		KASSERT(vm->scratch[0].base.seg.ds_len == size);

		/* Create a DMA map.  */
		ret = -bus_dmamap_create(vm->dmat, size, 1, size, 0,
		    BUS_DMA_NOWAIT, &vm->scratch[0].base.map);
		if (ret)
			goto free_dmamem;

		/* Load the segment into the DMA map.  */
		ret = -bus_dmamap_load_raw(vm->dmat, vm->scratch[0].base.map,
		    &vm->scratch[0].base.seg, 1, size, BUS_DMA_NOWAIT);
		if (ret)
			goto destroy_dmamap;
		KASSERT(vm->scratch[0].base.map->dm_nsegs == 1);
		KASSERT(vm->scratch[0].base.map->dm_segs[0].ds_len == size);

		/* Zero the page.  */
		ret = -bus_dmamem_map(vm->dmat, &vm->scratch[0].base.seg, 1,
		    size, &kva, BUS_DMA_NOWAIT|BUS_DMA_NOCACHE);
		if (ret)
			goto unload_dmamap;
		memset(kva, 0, size);
		bus_dmamap_sync(vm->dmat, vm->scratch[0].base.map, 0, size,
		    BUS_DMASYNC_PREREAD|BUS_DMASYNC_PREWRITE);
		bus_dmamem_unmap(vm->dmat, kva, size);

		/* XXX Is this page guaranteed to work as a huge page?  */
		vm_page = PHYS_TO_VM_PAGE(vm->scratch[0].base.seg.ds_addr);
		vm->scratch[0].base.page = container_of(vm_page, struct page,
		    p_vmp);
#else
		struct page *page;
		dma_addr_t addr;
=======
		struct drm_i915_gem_object *obj;
>>>>>>> vendor/linux-drm-v6.6.35

		obj = vm->alloc_scratch_dma(vm, size);
		if (IS_ERR(obj))
			goto skip;

		if (map_pt_dma(vm, obj))
			goto skip_obj;

		/* We need a single contiguous page for our scratch */
		if (obj->mm.page_sizes.sg < size)
			goto skip_obj;

<<<<<<< HEAD
		vm->scratch[0].base.page = page;
		vm->scratch[0].base.daddr = addr;
#endif
		vm->scratch_order = order;
		return 0;

#ifdef __NetBSD__
unload_dmamap:	bus_dmamap_unload(vm->dmat, vm->scratch[0].base.map);
destroy_dmamap:	bus_dmamap_destroy(vm->dmat, vm->scratch[0].base.map);
		vm->scratch[0].base.map = NULL; /* paranoia */
free_dmamem:	bus_dmamem_free(vm->dmat, &vm->scratch[0].base.seg, 1);
#else
unmap_page:
		dma_unmap_page(vm->dma, addr, size, PCI_DMA_BIDIRECTIONAL);
free_page:
		__free_pages(page, order);
#endif
=======
		/* And it needs to be correspondingly aligned */
		if (__px_dma(obj) & (size - 1))
			goto skip_obj;

		/*
		 * Use a non-zero scratch page for debugging.
		 *
		 * We want a value that should be reasonably obvious
		 * to spot in the error state, while also causing a GPU hang
		 * if executed. We prefer using a clear page in production, so
		 * should it ever be accidentally used, the effect should be
		 * fairly benign.
		 */
		poison_scratch_page(obj);

		vm->scratch[0] = obj;
		vm->scratch_order = get_order(size);
		return 0;

skip_obj:
		i915_gem_object_put(obj);
>>>>>>> vendor/linux-drm-v6.6.35
skip:
		if (size == I915_GTT_PAGE_SIZE_4K)
			return -ENOMEM;

		size = I915_GTT_PAGE_SIZE_4K;
	} while (1);
}

<<<<<<< HEAD
void cleanup_scratch_page(struct i915_address_space *vm)
{
	struct i915_page_dma *p = px_base(&vm->scratch[0]);
#ifdef __NetBSD__
	bus_dmamap_unload(vm->dmat, p->map);
	bus_dmamap_destroy(vm->dmat, p->map);
	vm->scratch[0].base.map = NULL; /* paranoia */
	bus_dmamem_free(vm->dmat, &p->seg, 1);
#else
	unsigned int order = vm->scratch_order;

	dma_unmap_page(vm->dma, p->daddr, BIT(order) << PAGE_SHIFT,
		       PCI_DMA_BIDIRECTIONAL);
	__free_pages(p->page, order);
#endif
}

=======
>>>>>>> vendor/linux-drm-v6.6.35
void free_scratch(struct i915_address_space *vm)
{
	int i;

	if (!vm->scratch[0])
		return;

	for (i = 0; i <= vm->top; i++)
		i915_gem_object_put(vm->scratch[i]);
}

void gtt_write_workarounds(struct intel_gt *gt)
{
	struct drm_i915_private *i915 = gt->i915;
	struct intel_uncore *uncore = gt->uncore;

	/*
	 * This function is for gtt related workarounds. This function is
	 * called on driver load and after a GPU reset, so you can place
	 * workarounds here even if they get overwritten by GPU reset.
	 */
	/* WaIncreaseDefaultTLBEntries:chv,bdw,skl,bxt,kbl,glk,cfl,cnl,icl */
	if (IS_BROADWELL(i915))
		intel_uncore_write(uncore,
				   GEN8_L3_LRA_1_GPGPU,
				   GEN8_L3_LRA_1_GPGPU_DEFAULT_VALUE_BDW);
	else if (IS_CHERRYVIEW(i915))
		intel_uncore_write(uncore,
				   GEN8_L3_LRA_1_GPGPU,
				   GEN8_L3_LRA_1_GPGPU_DEFAULT_VALUE_CHV);
	else if (IS_GEN9_LP(i915))
		intel_uncore_write(uncore,
				   GEN8_L3_LRA_1_GPGPU,
				   GEN9_L3_LRA_1_GPGPU_DEFAULT_VALUE_BXT);
	else if (GRAPHICS_VER(i915) >= 9 && GRAPHICS_VER(i915) <= 11)
		intel_uncore_write(uncore,
				   GEN8_L3_LRA_1_GPGPU,
				   GEN9_L3_LRA_1_GPGPU_DEFAULT_VALUE_SKL);

	/*
	 * To support 64K PTEs we need to first enable the use of the
	 * Intermediate-Page-Size(IPS) bit of the PDE field via some magical
	 * mmio, otherwise the page-walker will simply ignore the IPS bit. This
	 * shouldn't be needed after GEN10.
	 *
	 * 64K pages were first introduced from BDW+, although technically they
	 * only *work* from gen9+. For pre-BDW we instead have the option for
	 * 32K pages, but we don't currently have any support for it in our
	 * driver.
	 */
	if (HAS_PAGE_SIZES(i915, I915_GTT_PAGE_SIZE_64K) &&
	    GRAPHICS_VER(i915) <= 10)
		intel_uncore_rmw(uncore,
				 GEN8_GAMW_ECO_DEV_RW_IA,
				 0,
				 GAMW_ECO_ENABLE_64K_IPS_FIELD);

	if (IS_GRAPHICS_VER(i915, 8, 11)) {
		bool can_use_gtt_cache = true;

		/*
		 * According to the BSpec if we use 2M/1G pages then we also
		 * need to disable the GTT cache. At least on BDW we can see
		 * visual corruption when using 2M pages, and not disabling the
		 * GTT cache.
		 */
		if (HAS_PAGE_SIZES(i915, I915_GTT_PAGE_SIZE_2M))
			can_use_gtt_cache = false;

		/* WaGttCachingOffByDefault */
		intel_uncore_write(uncore,
				   HSW_GTT_CACHE_EN,
				   can_use_gtt_cache ? GTT_CACHE_EN_ALL : 0);
		gt_WARN_ON_ONCE(gt, can_use_gtt_cache &&
				intel_uncore_read(uncore,
						  HSW_GTT_CACHE_EN) == 0);
	}
}

static void xelpmp_setup_private_ppat(struct intel_uncore *uncore)
{
	intel_uncore_write(uncore, XELPMP_PAT_INDEX(0),
			   MTL_PPAT_L4_0_WB);
	intel_uncore_write(uncore, XELPMP_PAT_INDEX(1),
			   MTL_PPAT_L4_1_WT);
	intel_uncore_write(uncore, XELPMP_PAT_INDEX(2),
			   MTL_PPAT_L4_3_UC);
	intel_uncore_write(uncore, XELPMP_PAT_INDEX(3),
			   MTL_PPAT_L4_0_WB | MTL_2_COH_1W);
	intel_uncore_write(uncore, XELPMP_PAT_INDEX(4),
			   MTL_PPAT_L4_0_WB | MTL_3_COH_2W);

	/*
	 * Remaining PAT entries are left at the hardware-default
	 * fully-cached setting
	 */
}

static void xelpg_setup_private_ppat(struct intel_gt *gt)
{
	intel_gt_mcr_multicast_write(gt, XEHP_PAT_INDEX(0),
				     MTL_PPAT_L4_0_WB);
	intel_gt_mcr_multicast_write(gt, XEHP_PAT_INDEX(1),
				     MTL_PPAT_L4_1_WT);
	intel_gt_mcr_multicast_write(gt, XEHP_PAT_INDEX(2),
				     MTL_PPAT_L4_3_UC);
	intel_gt_mcr_multicast_write(gt, XEHP_PAT_INDEX(3),
				     MTL_PPAT_L4_0_WB | MTL_2_COH_1W);
	intel_gt_mcr_multicast_write(gt, XEHP_PAT_INDEX(4),
				     MTL_PPAT_L4_0_WB | MTL_3_COH_2W);

	/*
	 * Remaining PAT entries are left at the hardware-default
	 * fully-cached setting
	 */
}

static void tgl_setup_private_ppat(struct intel_uncore *uncore)
{
	/* TGL doesn't support LLC or AGE settings */
	intel_uncore_write(uncore, GEN12_PAT_INDEX(0), GEN8_PPAT_WB);
	intel_uncore_write(uncore, GEN12_PAT_INDEX(1), GEN8_PPAT_WC);
	intel_uncore_write(uncore, GEN12_PAT_INDEX(2), GEN8_PPAT_WT);
	intel_uncore_write(uncore, GEN12_PAT_INDEX(3), GEN8_PPAT_UC);
	intel_uncore_write(uncore, GEN12_PAT_INDEX(4), GEN8_PPAT_WB);
	intel_uncore_write(uncore, GEN12_PAT_INDEX(5), GEN8_PPAT_WB);
	intel_uncore_write(uncore, GEN12_PAT_INDEX(6), GEN8_PPAT_WB);
	intel_uncore_write(uncore, GEN12_PAT_INDEX(7), GEN8_PPAT_WB);
}

static void xehp_setup_private_ppat(struct intel_gt *gt)
{
	enum forcewake_domains fw;
	unsigned long flags;

	fw = intel_uncore_forcewake_for_reg(gt->uncore, _MMIO(XEHP_PAT_INDEX(0).reg),
					    FW_REG_WRITE);
	intel_uncore_forcewake_get(gt->uncore, fw);

	intel_gt_mcr_lock(gt, &flags);
	intel_gt_mcr_multicast_write_fw(gt, XEHP_PAT_INDEX(0), GEN8_PPAT_WB);
	intel_gt_mcr_multicast_write_fw(gt, XEHP_PAT_INDEX(1), GEN8_PPAT_WC);
	intel_gt_mcr_multicast_write_fw(gt, XEHP_PAT_INDEX(2), GEN8_PPAT_WT);
	intel_gt_mcr_multicast_write_fw(gt, XEHP_PAT_INDEX(3), GEN8_PPAT_UC);
	intel_gt_mcr_multicast_write_fw(gt, XEHP_PAT_INDEX(4), GEN8_PPAT_WB);
	intel_gt_mcr_multicast_write_fw(gt, XEHP_PAT_INDEX(5), GEN8_PPAT_WB);
	intel_gt_mcr_multicast_write_fw(gt, XEHP_PAT_INDEX(6), GEN8_PPAT_WB);
	intel_gt_mcr_multicast_write_fw(gt, XEHP_PAT_INDEX(7), GEN8_PPAT_WB);
	intel_gt_mcr_unlock(gt, flags);

	intel_uncore_forcewake_put(gt->uncore, fw);
}

static void icl_setup_private_ppat(struct intel_uncore *uncore)
{
	intel_uncore_write(uncore,
			   GEN10_PAT_INDEX(0),
			   GEN8_PPAT_WB | GEN8_PPAT_LLC);
	intel_uncore_write(uncore,
			   GEN10_PAT_INDEX(1),
			   GEN8_PPAT_WC | GEN8_PPAT_LLCELLC);
	intel_uncore_write(uncore,
			   GEN10_PAT_INDEX(2),
			   GEN8_PPAT_WB | GEN8_PPAT_ELLC_OVERRIDE);
	intel_uncore_write(uncore,
			   GEN10_PAT_INDEX(3),
			   GEN8_PPAT_UC);
	intel_uncore_write(uncore,
			   GEN10_PAT_INDEX(4),
			   GEN8_PPAT_WB | GEN8_PPAT_LLCELLC | GEN8_PPAT_AGE(0));
	intel_uncore_write(uncore,
			   GEN10_PAT_INDEX(5),
			   GEN8_PPAT_WB | GEN8_PPAT_LLCELLC | GEN8_PPAT_AGE(1));
	intel_uncore_write(uncore,
			   GEN10_PAT_INDEX(6),
			   GEN8_PPAT_WB | GEN8_PPAT_LLCELLC | GEN8_PPAT_AGE(2));
	intel_uncore_write(uncore,
			   GEN10_PAT_INDEX(7),
			   GEN8_PPAT_WB | GEN8_PPAT_LLCELLC | GEN8_PPAT_AGE(3));
}

/*
 * The GGTT and PPGTT need a private PPAT setup in order to handle cacheability
 * bits. When using advanced contexts each context stores its own PAT, but
 * writing this data shouldn't be harmful even in those cases.
 */
static void bdw_setup_private_ppat(struct intel_uncore *uncore)
{
	struct drm_i915_private *i915 = uncore->i915;
	u64 pat;

	pat = GEN8_PPAT(0, GEN8_PPAT_WB | GEN8_PPAT_LLC) |	/* for normal objects, no eLLC */
	      GEN8_PPAT(1, GEN8_PPAT_WC | GEN8_PPAT_LLCELLC) |	/* for something pointing to ptes? */
	      GEN8_PPAT(3, GEN8_PPAT_UC) |			/* Uncached objects, mostly for scanout */
	      GEN8_PPAT(4, GEN8_PPAT_WB | GEN8_PPAT_LLCELLC | GEN8_PPAT_AGE(0)) |
	      GEN8_PPAT(5, GEN8_PPAT_WB | GEN8_PPAT_LLCELLC | GEN8_PPAT_AGE(1)) |
	      GEN8_PPAT(6, GEN8_PPAT_WB | GEN8_PPAT_LLCELLC | GEN8_PPAT_AGE(2)) |
	      GEN8_PPAT(7, GEN8_PPAT_WB | GEN8_PPAT_LLCELLC | GEN8_PPAT_AGE(3));

	/* for scanout with eLLC */
	if (GRAPHICS_VER(i915) >= 9)
		pat |= GEN8_PPAT(2, GEN8_PPAT_WB | GEN8_PPAT_ELLC_OVERRIDE);
	else
		pat |= GEN8_PPAT(2, GEN8_PPAT_WT | GEN8_PPAT_LLCELLC);

	intel_uncore_write(uncore, GEN8_PRIVATE_PAT_LO, lower_32_bits(pat));
	intel_uncore_write(uncore, GEN8_PRIVATE_PAT_HI, upper_32_bits(pat));
}

static void chv_setup_private_ppat(struct intel_uncore *uncore)
{
	u64 pat;

	/*
	 * Map WB on BDW to snooped on CHV.
	 *
	 * Only the snoop bit has meaning for CHV, the rest is
	 * ignored.
	 *
	 * The hardware will never snoop for certain types of accesses:
	 * - CPU GTT (GMADR->GGTT->no snoop->memory)
	 * - PPGTT page tables
	 * - some other special cycles
	 *
	 * As with BDW, we also need to consider the following for GT accesses:
	 * "For GGTT, there is NO pat_sel[2:0] from the entry,
	 * so RTL will always use the value corresponding to
	 * pat_sel = 000".
	 * Which means we must set the snoop bit in PAT entry 0
	 * in order to keep the global status page working.
	 */

	pat = GEN8_PPAT(0, CHV_PPAT_SNOOP) |
	      GEN8_PPAT(1, 0) |
	      GEN8_PPAT(2, 0) |
	      GEN8_PPAT(3, 0) |
	      GEN8_PPAT(4, CHV_PPAT_SNOOP) |
	      GEN8_PPAT(5, CHV_PPAT_SNOOP) |
	      GEN8_PPAT(6, CHV_PPAT_SNOOP) |
	      GEN8_PPAT(7, CHV_PPAT_SNOOP);

	intel_uncore_write(uncore, GEN8_PRIVATE_PAT_LO, lower_32_bits(pat));
	intel_uncore_write(uncore, GEN8_PRIVATE_PAT_HI, upper_32_bits(pat));
}

void setup_private_pat(struct intel_gt *gt)
{
	struct intel_uncore *uncore = gt->uncore;
	struct drm_i915_private *i915 = gt->i915;

	GEM_BUG_ON(GRAPHICS_VER(i915) < 8);

	if (gt->type == GT_MEDIA) {
		xelpmp_setup_private_ppat(gt->uncore);
		return;
	}

	if (GRAPHICS_VER_FULL(i915) >= IP_VER(12, 70))
		xelpg_setup_private_ppat(gt);
	else if (GRAPHICS_VER_FULL(i915) >= IP_VER(12, 50))
		xehp_setup_private_ppat(gt);
	else if (GRAPHICS_VER(i915) >= 12)
		tgl_setup_private_ppat(uncore);
	else if (GRAPHICS_VER(i915) >= 11)
		icl_setup_private_ppat(uncore);
	else if (IS_CHERRYVIEW(i915) || IS_GEN9_LP(i915))
		chv_setup_private_ppat(uncore);
	else
		bdw_setup_private_ppat(uncore);
}

struct i915_vma *
__vm_create_scratch_for_read(struct i915_address_space *vm, unsigned long size)
{
	struct drm_i915_gem_object *obj;
	struct i915_vma *vma;

	obj = i915_gem_object_create_internal(vm->i915, PAGE_ALIGN(size));
	if (IS_ERR(obj))
		return ERR_CAST(obj);

	i915_gem_object_set_cache_coherency(obj, I915_CACHE_LLC);

	vma = i915_vma_instance(obj, vm, NULL);
	if (IS_ERR(vma)) {
		i915_gem_object_put(obj);
		return vma;
	}

	return vma;
}

struct i915_vma *
__vm_create_scratch_for_read_pinned(struct i915_address_space *vm, unsigned long size)
{
	struct i915_vma *vma;
	int err;

	vma = __vm_create_scratch_for_read(vm, size);
	if (IS_ERR(vma))
		return vma;

	err = i915_vma_pin(vma, 0, 0,
			   i915_vma_is_ggtt(vma) ? PIN_GLOBAL : PIN_USER);
	if (err) {
		i915_vma_put(vma);
		return ERR_PTR(err);
	}

	return vma;
}

#if IS_ENABLED(CONFIG_DRM_I915_SELFTEST)
#include "selftests/mock_gtt.c"
#endif
