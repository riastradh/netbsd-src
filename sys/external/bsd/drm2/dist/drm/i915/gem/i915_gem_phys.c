/*	$NetBSD: i915_gem_phys.c,v 1.9 2024/01/19 22:23:04 riastradh Exp $	*/

/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2014-2016 Intel Corporation
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: i915_gem_phys.c,v 1.9 2024/01/19 22:23:04 riastradh Exp $");

#ifdef __NetBSD__
/*
 * Make sure this block comes before any linux includes, so we don't
 * get mixed up by the PAGE_MASK complementation.
 */

#include <sys/bus.h>

#include <uvm/uvm.h>
#include <uvm/uvm_extern.h>

#include <machine/pmap_private.h> /* kvtopte, pmap_pte_clearbits */

/*
 * Version of bus_dmamem_map that uses pmap_kenter_pa, not pmap_enter,
 * so that it isn't affected by pmap_page_protect on the physical
 * address.  Adapted from sys/arch/x86/x86/bus_dma.c.
 */
static int
bus_dmamem_kmap(bus_dma_tag_t t, bus_dma_segment_t *segs, int nsegs,
    size_t size, void **kvap, int flags)
{
	vaddr_t va;
	bus_addr_t addr;
	int curseg;
	const uvm_flag_t kmflags =
	    (flags & BUS_DMA_NOWAIT) != 0 ? UVM_KMF_NOWAIT : 0;
	u_int pmapflags = PMAP_WIRED | VM_PROT_READ | VM_PROT_WRITE;

	size = round_page(size);
	if (flags & BUS_DMA_NOCACHE)
		pmapflags |= PMAP_NOCACHE;

	va = uvm_km_alloc(kernel_map, size, 0, UVM_KMF_VAONLY | kmflags);

	if (va == 0)
		return ENOMEM;

	*kvap = (void *)va;

	for (curseg = 0; curseg < nsegs; curseg++) {
		for (addr = segs[curseg].ds_addr;
		    addr < (segs[curseg].ds_addr + segs[curseg].ds_len);
		    addr += PAGE_SIZE, va += PAGE_SIZE, size -= PAGE_SIZE) {
			if (size == 0)
				panic("bus_dmamem_kmap: size botch");
			pmap_kenter_pa(va, addr,
			    VM_PROT_READ | VM_PROT_WRITE,
			    pmapflags);
		}
	}
	pmap_update(pmap_kernel());

	return 0;
}

static void
bus_dmamem_kunmap(bus_dma_tag_t t, void *kva, size_t size)
{
	pt_entry_t *pte, opte;
	vaddr_t va, sva, eva;

	KASSERTMSG(((uintptr_t)kva & PGOFSET) == 0, "kva=%p", kva);

	size = round_page(size);
	sva = (vaddr_t)kva;
	eva = sva + size;

	/*
	 * mark pages cacheable again.
	 */
	for (va = sva; va < eva; va += PAGE_SIZE) {
		pte = kvtopte(va);
		opte = *pte;
		if ((opte & PTE_PCD) != 0)
			pmap_pte_clearbits(pte, PTE_PCD);
	}
	pmap_kremove((vaddr_t)kva, size);
	pmap_update(pmap_kernel());
	uvm_km_free(kernel_map, (vaddr_t)kva, size, UVM_KMF_VAONLY);
}

#endif

#include <linux/highmem.h>
#include <linux/shmem_fs.h>
#include <linux/swap.h>

#include <drm/drm_cache.h>

#include "gt/intel_gt.h"
#include "i915_drv.h"
#include "i915_gem_object.h"
#include "i915_gem_region.h"
#include "i915_gem_tiling.h"
#include "i915_scatterlist.h"

#include <linux/nbsd-namespace.h>

static int i915_gem_object_get_pages_phys(struct drm_i915_gem_object *obj)
{
#ifdef __NetBSD__
	struct uvm_object *mapping = obj->base.filp;
#else
	struct address_space *mapping = obj->base.filp->f_mapping;
#endif
	struct drm_i915_private *i915 = to_i915(obj->base.dev);
	struct scatterlist *sg;
	struct sg_table *st;
	dma_addr_t dma;
	void *vaddr;
	void *dst;
	int i;

	/* Contiguous chunk, with a single scatterlist element */
	if (overflows_type(obj->base.size, sg->length))
		return -E2BIG;

	if (GEM_WARN_ON(i915_gem_object_needs_bit17_swizzle(obj)))
		return -EINVAL;


	/*
	 * Always aligning to the object size, allows a single allocation
	 * to handle all possible callers, and given typical object sizes,
	 * the alignment of the buddy allocation will naturally match.
	 */
#ifdef __NetBSD__
	__USE(dma);
	bus_dma_tag_t dmat = obj->base.dev->dmat;
	bool loaded = false;
	int rsegs = 0;
	int ret;

	vaddr = NULL;

	/* XXX errno NetBSD->Linux */
	ret = -bus_dmamem_alloc(dmat, roundup_pow_of_two(obj->base.size),
	    roundup_pow_of_two(obj->base.size), 0, &obj->mm.u.phys.seg, 1,
	    &rsegs, BUS_DMA_WAITOK);
	if (ret)
		return -ENOMEM;
	KASSERT(rsegs == 1);
	ret = -bus_dmamem_kmap(dmat, &obj->mm.u.phys.seg, 1,
	    roundup_pow_of_two(obj->base.size), &vaddr,
	    BUS_DMA_WAITOK|BUS_DMA_COHERENT);
	if (ret)
		goto err_pci;
	obj->mm.u.phys.kva = vaddr;
#else
	vaddr = dma_alloc_coherent(obj->base.dev->dev,
				   roundup_pow_of_two(obj->base.size),
				   &dma, GFP_KERNEL);
	if (!vaddr)
		return -ENOMEM;
#endif

	st = kmalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		goto err_pci;

#ifdef __NetBSD__
	if (sg_alloc_table_from_bus_dmamem(st, dmat, &obj->mm.u.phys.seg, 1,
		GFP_KERNEL))
#else
	if (sg_alloc_table(st, 1, GFP_KERNEL))
#endif
		goto err_st;

	sg = st->sgl;
#ifdef __NetBSD__
	/* XXX errno NetBSD->Linux */
	ret = -bus_dmamap_create(dmat, roundup_pow_of_two(obj->base.size), 1,
	    roundup_pow_of_two(obj->base.size), 0, BUS_DMA_WAITOK,
	    &sg->sg_dmamap);
	if (ret) {
		sg->sg_dmamap = NULL;
		goto err_st1;
	}
	sg->sg_dmat = dmat;
	/* XXX errno NetBSD->Linux */
	ret = -bus_dmamap_load_raw(dmat, sg->sg_dmamap, &obj->mm.u.phys.seg, 1,
	    roundup_pow_of_two(obj->base.size), BUS_DMA_WAITOK);
	if (ret)
		goto err_st1;
	loaded = true;
#else
	sg->offset = 0;
	sg->length = obj->base.size;

	sg_assign_page(sg, (struct page *)vaddr);
	sg_dma_address(sg) = dma;
	sg_dma_len(sg) = obj->base.size;
#endif

	dst = vaddr;
	for (i = 0; i < obj->base.size / PAGE_SIZE; i++) {
		struct page *page;
		void *src;

		page = shmem_read_mapping_page(mapping, i);
		if (IS_ERR(page))
			goto err_st;

		src = kmap_atomic(page);
		memcpy(dst, src, PAGE_SIZE);
		drm_clflush_virt_range(dst, PAGE_SIZE);
		kunmap_atomic(src);

#ifdef __NetBSD__
		uvm_obj_unwirepages(mapping, i*PAGE_SIZE, (i + 1)*PAGE_SIZE);
#else
		put_page(page);
#endif
		dst += PAGE_SIZE;
	}

	intel_gt_chipset_flush(to_gt(i915));

	/* We're no longer struct page backed */
	obj->mem_flags &= ~I915_BO_FLAG_STRUCT_PAGE;
	__i915_gem_object_set_pages(obj, st, obj->base.size);

	return 0;

#ifdef __NetBSD__
err_st1:
	if (loaded)
		bus_dmamap_unload(dmat, st->sgl->sg_dmamap);
	sg_free_table(st);
#endif
err_st:
	kfree(st);
err_pci:
#ifdef __NetBSD__
	if (vaddr) {
		bus_dmamem_kunmap(dmat, vaddr,
		    roundup_pow_of_two(obj->base.size));
	}
	obj->mm.u.phys.kva = NULL;
	if (rsegs)
		bus_dmamem_free(dmat, &obj->mm.u.phys.seg, rsegs);
#else
	dma_free_coherent(obj->base.dev->dev,
			  roundup_pow_of_two(obj->base.size),
			  vaddr, dma);
#endif
	return -ENOMEM;
}

void
i915_gem_object_put_pages_phys(struct drm_i915_gem_object *obj,
			       struct sg_table *pages)
{
#ifdef __NetBSD__
	bus_dma_tag_t dmat = obj->base.dev->dmat;
	void *vaddr = obj->mm.u.phys.kva;
#else
	dma_addr_t dma = sg_dma_address(pages->sgl);
	void *vaddr = sg_page(pages->sgl);
#endif

	__i915_gem_object_release_shmem(obj, pages, false);

	if (obj->mm.dirty) {
#ifdef __NetBSD__
		struct uvm_object *mapping = obj->base.filp;
#else
		struct address_space *mapping = obj->base.filp->f_mapping;
#endif
		void *src = vaddr;
		int i;

		for (i = 0; i < obj->base.size / PAGE_SIZE; i++) {
			struct page *page;
			char *dst;

			page = shmem_read_mapping_page(mapping, i);
			if (IS_ERR(page))
				continue;

			dst = kmap_atomic(page);
			drm_clflush_virt_range(src, PAGE_SIZE);
			memcpy(dst, src, PAGE_SIZE);
			kunmap_atomic(dst);

			set_page_dirty(page);
#ifdef __NetBSD__
			/* XXX mark_page_accessed */
			uvm_obj_unwirepages(mapping, i*PAGE_SIZE,
			    (i + 1)*PAGE_SIZE);
#else
			if (obj->mm.madv == I915_MADV_WILLNEED)
				mark_page_accessed(page);
			put_page(page);
#endif

			src += PAGE_SIZE;
		}
		obj->mm.dirty = false;
	}

#ifdef __NetBSD__
	bus_dmamap_unload(dmat, pages->sgl->sg_dmamap);
#endif

	sg_free_table(pages);
	kfree(pages);

#ifdef __NetBSD__
	bus_dmamem_kunmap(dmat, obj->mm.u.phys.kva,
	    roundup_pow_of_two(obj->base.size));
	obj->mm.u.phys.kva = NULL;
	bus_dmamem_free(dmat, &obj->mm.u.phys.seg, 1);
#else
	dma_free_coherent(obj->base.dev->dev,
			  roundup_pow_of_two(obj->base.size),
			  vaddr, dma);
#endif
}

int i915_gem_object_pwrite_phys(struct drm_i915_gem_object *obj,
				const struct drm_i915_gem_pwrite *args)
{
#ifdef __NetBSD__
	void *vaddr = obj->mm.u.phys.kva + args->offset;
#else
	void *vaddr = sg_page(obj->mm.pages->sgl) + args->offset;
#endif
	char __user *user_data = u64_to_user_ptr(args->data_ptr);
	struct drm_i915_private *i915 = to_i915(obj->base.dev);
	int err;

	err = i915_gem_object_wait(obj,
				   I915_WAIT_INTERRUPTIBLE |
				   I915_WAIT_ALL,
				   MAX_SCHEDULE_TIMEOUT);
	if (err)
		return err;

	/*
	 * We manually control the domain here and pretend that it
	 * remains coherent i.e. in the GTT domain, like shmem_pwrite.
	 */
	i915_gem_object_invalidate_frontbuffer(obj, ORIGIN_CPU);

	if (copy_from_user(vaddr, user_data, args->size))
		return -EFAULT;

	drm_clflush_virt_range(vaddr, args->size);
	intel_gt_chipset_flush(to_gt(i915));

	i915_gem_object_flush_frontbuffer(obj, ORIGIN_CPU);
	return 0;
}

int i915_gem_object_pread_phys(struct drm_i915_gem_object *obj,
			       const struct drm_i915_gem_pread *args)
{
#ifdef __NetBSD__
	void *vaddr = obj->mm.u.phys.kva + args->offset;
#else
	void *vaddr = sg_page(obj->mm.pages->sgl) + args->offset;
#endif
	char __user *user_data = u64_to_user_ptr(args->data_ptr);
	int err;

	err = i915_gem_object_wait(obj,
				   I915_WAIT_INTERRUPTIBLE,
				   MAX_SCHEDULE_TIMEOUT);
	if (err)
		return err;

	drm_clflush_virt_range(vaddr, args->size);
	if (copy_to_user(user_data, vaddr, args->size))
		return -EFAULT;

	return 0;
}

static int i915_gem_object_shmem_to_phys(struct drm_i915_gem_object *obj)
{
	struct sg_table *pages;
	int err;

	pages = __i915_gem_object_unset_pages(obj);

	err = i915_gem_object_get_pages_phys(obj);
	if (err)
		goto err_xfer;

	/* Perma-pin (until release) the physical set of pages */
	__i915_gem_object_pin_pages(obj);

	if (!IS_ERR_OR_NULL(pages))
		i915_gem_object_put_pages_shmem(obj, pages);

	i915_gem_object_release_memory_region(obj);
	return 0;

err_xfer:
	if (!IS_ERR_OR_NULL(pages))
		__i915_gem_object_set_pages(obj, pages);
	return err;
}

int i915_gem_object_attach_phys(struct drm_i915_gem_object *obj, int align)
{
	int err;

	assert_object_held(obj);

	if (align > obj->base.size)
		return -EINVAL;

	if (!i915_gem_object_is_shmem(obj))
		return -EINVAL;

	if (!i915_gem_object_has_struct_page(obj))
		return 0;

	err = i915_gem_object_unbind(obj, I915_GEM_OBJECT_UNBIND_ACTIVE);
	if (err)
		return err;

	if (obj->mm.madv != I915_MADV_WILLNEED)
		return -EFAULT;

	if (i915_gem_object_has_tiling_quirk(obj))
		return -EFAULT;

	if (obj->mm.mapping || i915_gem_object_has_pinned_pages(obj))
		return -EBUSY;

	if (unlikely(obj->mm.madv != I915_MADV_WILLNEED)) {
		drm_dbg(obj->base.dev,
			"Attempting to obtain a purgeable object\n");
		return -EFAULT;
	}

	return i915_gem_object_shmem_to_phys(obj);
}

#if IS_ENABLED(CONFIG_DRM_I915_SELFTEST)
#include "selftests/i915_gem_phys.c"
#endif
