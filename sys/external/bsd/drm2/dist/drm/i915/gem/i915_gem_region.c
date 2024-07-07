/*	$NetBSD: i915_gem_region.c,v 1.6 2024/01/19 22:23:19 riastradh Exp $	*/

// SPDX-License-Identifier: MIT
/*
 * Copyright © 2019 Intel Corporation
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: i915_gem_region.c,v 1.6 2024/01/19 22:23:19 riastradh Exp $");

#include <uapi/drm/i915_drm.h>

#include "intel_memory_region.h"
#include "i915_gem_region.h"
#include "i915_drv.h"
#include "i915_trace.h"

<<<<<<< HEAD
void
i915_gem_object_put_pages_buddy(struct drm_i915_gem_object *obj,
				struct sg_table *pages)
{
	__intel_memory_region_put_pages_buddy(obj->mm.region, &obj->mm.blocks);

	obj->mm.dirty = false;
#ifdef __NetBSD__
	bus_dmamap_unload(obj->base.dev->dmat, pages->sgl->sg_dmamap);
#endif
	sg_free_table(pages);
	kfree(pages);
}

int
i915_gem_object_get_pages_buddy(struct drm_i915_gem_object *obj)
{
	struct intel_memory_region *mem = obj->mm.region;
	struct list_head *blocks = &obj->mm.blocks;
	resource_size_t size = obj->base.size;
	resource_size_t prev_end;
	struct i915_buddy_block *block;
	unsigned int flags;
	struct sg_table *st;
	struct scatterlist *sg;
	unsigned int sg_page_sizes;
	int ret;

	st = kmalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

#ifndef __NetBSD__
	if (sg_alloc_table(st, size >> ilog2(mem->mm.chunk_size), GFP_KERNEL)) {
		kfree(st);
		return -ENOMEM;
	}
#endif

	flags = I915_ALLOC_MIN_PAGE_SIZE;
	if (obj->flags & I915_BO_ALLOC_CONTIGUOUS)
		flags |= I915_ALLOC_CONTIGUOUS;

	ret = __intel_memory_region_get_pages_buddy(mem, size, flags, blocks);
	if (ret)
		goto err_free_sg;

	GEM_BUG_ON(list_empty(blocks));

#ifdef __NetBSD__
	__USE(prev_end);
	bus_dma_tag_t dmat = obj->base.dev->dmat;
	bus_dma_segment_t *segs = NULL;
	int i = 0, nsegs = 0;
	bool loaded = false;

	sg = NULL;

	list_for_each_entry(block, blocks, link) {
		if (nsegs >= INT_MAX ||
		    nsegs >= SIZE_MAX/sizeof(segs[0]))
			goto err;
		nsegs++;
	}
	segs = kmem_zalloc(nsegs * sizeof(segs[0]), KM_SLEEP);
	list_for_each_entry(block, blocks, link) {
		u64 block_size, offset;

		block_size = min_t(u64, size,
				   i915_buddy_block_size(&mem->mm, block));
		offset = i915_buddy_block_offset(block);

		segs[i].ds_addr = mem->region.start + offset;
		segs[i].ds_len = block_size;
		i++;
	}
	KASSERT(i == nsegs);

	ret = sg_alloc_table_from_bus_dmamem(st, dmat, segs, nsegs,
	    GFP_KERNEL);
	if (ret)
		goto err;
	sg = st->sgl;

	/* XXX errno NetBSD->Linux */
	ret = -bus_dmamap_create(dmat, size, nsegs, size, 0, BUS_DMA_WAITOK,
	    &sg->sg_dmamap);
	if (ret) {
		sg->sg_dmamap = NULL;
		goto err;
	}
	sg->sg_dmat = dmat;

	/* XXX errno NetBSD->Linux */
	ret = -bus_dmamap_load_raw(dmat, sg->sg_dmamap, segs, nsegs, size,
	    BUS_DMA_WAITOK);
	if (ret)
		goto err;
	loaded = true;

	kmem_free(segs, nsegs * sizeof(segs[0]));
	segs = NULL;

	sg_page_sizes = i915_sg_page_sizes(sg);
#else
	sg = st->sgl;
	st->nents = 0;
	sg_page_sizes = 0;
	prev_end = (resource_size_t)-1;

	list_for_each_entry(block, blocks, link) {
		u64 block_size, offset;

		block_size = min_t(u64, size,
				   i915_buddy_block_size(&mem->mm, block));
		offset = i915_buddy_block_offset(block);

		GEM_BUG_ON(overflows_type(block_size, sg->length));

		if (offset != prev_end ||
		    add_overflows_t(typeof(sg->length), sg->length, block_size)) {
			if (st->nents) {
				sg_page_sizes |= sg->length;
				sg = __sg_next(sg);
			}

			sg_dma_address(sg) = mem->region.start + offset;
			sg_dma_len(sg) = block_size;

			sg->length = block_size;

			st->nents++;
		} else {
			sg->length += block_size;
			sg_dma_len(sg) += block_size;
		}

		prev_end = offset + block_size;
	}

	sg_page_sizes |= sg->length;
	sg_mark_end(sg);
	i915_sg_trim(st);
#endif

	__i915_gem_object_set_pages(obj, st, sg_page_sizes);

	return 0;

#ifdef __NetBSD__
err:
	if (loaded)
		bus_dmamap_unload(dmat, st->sgl->sg_dmamap);
	if (sg && sg->sg_dmamap)
		bus_dmamap_destroy(dmat, sg->sg_dmamap);
	if (segs)
		kmem_free(segs, nsegs * sizeof(segs[0]));
	__intel_memory_region_put_pages_buddy(mem, blocks);
#endif
err_free_sg:
	sg_free_table(st);
	kfree(st);
	return ret;
}

=======
>>>>>>> vendor/linux-drm-v6.6.35
void i915_gem_object_init_memory_region(struct drm_i915_gem_object *obj,
					struct intel_memory_region *mem)
{
	obj->mm.region = mem;

	mutex_lock(&mem->objects.lock);
	list_add(&obj->mm.region_link, &mem->objects.list);
	mutex_unlock(&mem->objects.lock);
}

void i915_gem_object_release_memory_region(struct drm_i915_gem_object *obj)
{
	struct intel_memory_region *mem = obj->mm.region;

	mutex_lock(&mem->objects.lock);
	list_del(&obj->mm.region_link);
	mutex_unlock(&mem->objects.lock);
}

static struct drm_i915_gem_object *
__i915_gem_object_create_region(struct intel_memory_region *mem,
				resource_size_t offset,
				resource_size_t size,
				resource_size_t page_size,
				unsigned int flags)
{
	struct drm_i915_gem_object *obj;
	resource_size_t default_page_size;
	int err;

	/*
	 * NB: Our use of resource_size_t for the size stems from using struct
	 * resource for the mem->region. We might need to revisit this in the
	 * future.
	 */

	GEM_BUG_ON(flags & ~I915_BO_ALLOC_FLAGS);

	if (WARN_ON_ONCE(flags & I915_BO_ALLOC_GPU_ONLY &&
			 (flags & I915_BO_ALLOC_CPU_CLEAR ||
			  flags & I915_BO_ALLOC_PM_EARLY)))
		return ERR_PTR(-EINVAL);

	if (!mem)
		return ERR_PTR(-ENODEV);

	default_page_size = mem->min_page_size;
	if (page_size)
		default_page_size = page_size;

	/* We should be able to fit a page within an sg entry */
	GEM_BUG_ON(overflows_type(default_page_size, u32));
	GEM_BUG_ON(!is_power_of_2_u64(default_page_size));
	GEM_BUG_ON(default_page_size < PAGE_SIZE);

	size = round_up(size, default_page_size);

	if (default_page_size == size)
		flags |= I915_BO_ALLOC_CONTIGUOUS;

	GEM_BUG_ON(!size);
	GEM_BUG_ON(!IS_ALIGNED(size, I915_GTT_MIN_ALIGNMENT));

	if (i915_gem_object_size_2big(size))
		return ERR_PTR(-E2BIG);

	obj = i915_gem_object_alloc();
	if (!obj)
		return ERR_PTR(-ENOMEM);

	/*
	 * Anything smaller than the min_page_size can't be freely inserted into
	 * the GTT, due to alignemnt restrictions. For such special objects,
	 * make sure we force memcpy based suspend-resume. In the future we can
	 * revisit this, either by allowing special mis-aligned objects in the
	 * migration path, or by mapping all of LMEM upfront using cheap 1G
	 * GTT entries.
	 */
	if (default_page_size < mem->min_page_size)
		flags |= I915_BO_ALLOC_PM_EARLY;

	err = mem->ops->init_object(mem, obj, offset, size, page_size, flags);
	if (err)
		goto err_object_free;

	trace_i915_gem_object_create(obj);
	return obj;

err_object_free:
	i915_gem_object_free(obj);
	return ERR_PTR(err);
}

struct drm_i915_gem_object *
i915_gem_object_create_region(struct intel_memory_region *mem,
			      resource_size_t size,
			      resource_size_t page_size,
			      unsigned int flags)
{
	return __i915_gem_object_create_region(mem, I915_BO_INVALID_OFFSET,
					       size, page_size, flags);
}

struct drm_i915_gem_object *
i915_gem_object_create_region_at(struct intel_memory_region *mem,
				 resource_size_t offset,
				 resource_size_t size,
				 unsigned int flags)
{
	GEM_BUG_ON(offset == I915_BO_INVALID_OFFSET);

	if (GEM_WARN_ON(!IS_ALIGNED(size, mem->min_page_size)) ||
	    GEM_WARN_ON(!IS_ALIGNED(offset, mem->min_page_size)))
		return ERR_PTR(-EINVAL);

	if (range_overflows(offset, size, resource_size(&mem->region)))
		return ERR_PTR(-EINVAL);

	if (!(flags & I915_BO_ALLOC_GPU_ONLY) &&
	    offset + size > mem->io_size &&
	    !i915_ggtt_has_aperture(to_gt(mem->i915)->ggtt))
		return ERR_PTR(-ENOSPC);

	return __i915_gem_object_create_region(mem, offset, size, 0,
					       flags | I915_BO_ALLOC_CONTIGUOUS);
}

/**
 * i915_gem_process_region - Iterate over all objects of a region using ops
 * to process and optionally skip objects
 * @mr: The memory region
 * @apply: ops and private data
 *
 * This function can be used to iterate over the regions object list,
 * checking whether to skip objects, and, if not, lock the objects and
 * process them using the supplied ops. Note that this function temporarily
 * removes objects from the region list while iterating, so that if run
 * concurrently with itself may not iterate over all objects.
 *
 * Return: 0 if successful, negative error code on failure.
 */
int i915_gem_process_region(struct intel_memory_region *mr,
			    struct i915_gem_apply_to_region *apply)
{
	const struct i915_gem_apply_to_region_ops *ops = apply->ops;
	struct drm_i915_gem_object *obj;
	struct list_head still_in_list;
	int ret = 0;

	/*
	 * In the future, a non-NULL apply->ww could mean the caller is
	 * already in a locking transaction and provides its own context.
	 */
	GEM_WARN_ON(apply->ww);

	INIT_LIST_HEAD(&still_in_list);
	mutex_lock(&mr->objects.lock);
	for (;;) {
		struct i915_gem_ww_ctx ww;

		obj = list_first_entry_or_null(&mr->objects.list, typeof(*obj),
					       mm.region_link);
		if (!obj)
			break;

		list_move_tail(&obj->mm.region_link, &still_in_list);
		if (!kref_get_unless_zero(&obj->base.refcount))
			continue;

		/*
		 * Note: Someone else might be migrating the object at this
		 * point. The object's region is not stable until we lock
		 * the object.
		 */
		mutex_unlock(&mr->objects.lock);
		apply->ww = &ww;
		for_i915_gem_ww(&ww, ret, apply->interruptible) {
			ret = i915_gem_object_lock(obj, apply->ww);
			if (ret)
				continue;

			if (obj->mm.region == mr)
				ret = ops->process_obj(apply, obj);
			/* Implicit object unlock */
		}

		i915_gem_object_put(obj);
		mutex_lock(&mr->objects.lock);
		if (ret)
			break;
	}
	list_splice_tail(&still_in_list, &mr->objects.list);
	mutex_unlock(&mr->objects.lock);

	return ret;
}
