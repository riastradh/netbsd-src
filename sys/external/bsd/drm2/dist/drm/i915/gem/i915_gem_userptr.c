/*	$NetBSD: i915_gem_userptr.c,v 1.5 2021/12/19 12:32:15 riastradh Exp $	*/

/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2012-2014 Intel Corporation
 *
  * Based on amdgpu_mn, which bears the following notice:
 *
 * Copyright 2014 Advanced Micro Devices, Inc.
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
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS, AUTHORS AND/OR ITS SUPPLIERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
 * USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 */
/*
 * Authors:
 *    Christian König <christian.koenig@amd.com>
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: i915_gem_userptr.c,v 1.5 2021/12/19 12:32:15 riastradh Exp $");

#include <linux/mmu_context.h>
#include <linux/mempolicy.h>
#include <linux/swap.h>
#include <linux/sched/mm.h>

#include "i915_drv.h"
#include "i915_gem_ioctls.h"
#include "i915_gem_object.h"
#include "i915_gem_userptr.h"
#include "i915_scatterlist.h"

<<<<<<< HEAD
#include <linux/nbsd-namespace.h>

struct i915_mm_struct {
#ifdef __NetBSD__
	struct vmspace *mm;
#else
	struct mm_struct *mm;
#endif
	struct drm_i915_private *i915;
	struct i915_mmu_notifier *mn;
	struct hlist_node node;
	struct kref kref;
	struct work_struct work;
};
=======
#ifdef CONFIG_MMU_NOTIFIER
>>>>>>> vendor/linux-drm-v6.6.35

/**
 * i915_gem_userptr_invalidate - callback to notify about mm change
 *
 * @mni: the range (mm) is about to update
 * @range: details on the invalidation
 * @cur_seq: Value to pass to mmu_interval_set_seq()
 *
 * Block for operations on BOs to finish and mark pages as accessed and
 * potentially dirty.
 */
static bool i915_gem_userptr_invalidate(struct mmu_interval_notifier *mni,
					const struct mmu_notifier_range *range,
					unsigned long cur_seq)
{
	struct drm_i915_gem_object *obj = container_of(mni, struct drm_i915_gem_object, userptr.notifier);
	struct drm_i915_private *i915 = to_i915(obj->base.dev);
	long r;

	if (!mmu_notifier_range_blockable(range))
		return false;

	write_lock(&i915->mm.notifier_lock);

	mmu_interval_set_seq(mni, cur_seq);

	write_unlock(&i915->mm.notifier_lock);

	/*
	 * We don't wait when the process is exiting. This is valid
	 * because the object will be cleaned up anyway.
	 *
	 * This is also temporarily required as a hack, because we
	 * cannot currently force non-consistent batch buffers to preempt
	 * and reschedule by waiting on it, hanging processes on exit.
	 */
	if (current->flags & PF_EXITING)
		return true;

	/* we will unbind on next submission, still have userptr pins */
	r = dma_resv_wait_timeout(obj->base.resv, DMA_RESV_USAGE_BOOKKEEP, false,
				  MAX_SCHEDULE_TIMEOUT);
	if (r <= 0)
		drm_err(&i915->drm, "(%ld) failed to wait for idle\n", r);

	return true;
}

static const struct mmu_interval_notifier_ops i915_gem_userptr_notifier_ops = {
	.invalidate = i915_gem_userptr_invalidate,
};

static int
i915_gem_userptr_init__mmu_notifier(struct drm_i915_gem_object *obj)
{
	return mmu_interval_notifier_insert(&obj->userptr.notifier, current->mm,
					    obj->userptr.ptr, obj->base.size,
					    &i915_gem_userptr_notifier_ops);
}

static void i915_gem_object_userptr_drop_ref(struct drm_i915_gem_object *obj)
{
	struct page **pvec = NULL;

	assert_object_held_shared(obj);

	if (!--obj->userptr.page_ref) {
		pvec = obj->userptr.pvec;
		obj->userptr.pvec = NULL;
	}
	GEM_BUG_ON(obj->userptr.page_ref < 0);

<<<<<<< HEAD
	if (mn && !IS_ERR(mn)) {
		spin_lock_destroy(&mn->lock);
		kfree(mn);
	}
=======
	if (pvec) {
		const unsigned long num_pages = obj->base.size >> PAGE_SHIFT;
>>>>>>> vendor/linux-drm-v6.6.35

		unpin_user_pages(pvec, num_pages);
		kvfree(pvec);
	}
}

static int i915_gem_userptr_get_pages(struct drm_i915_gem_object *obj)
{
<<<<<<< HEAD
	struct i915_mmu_notifier *mn;
	struct i915_mmu_object *mo;

	if (flags & I915_USERPTR_UNSYNCHRONIZED)
		return capable(CAP_SYS_ADMIN) ? 0 : -EPERM;

	if (WARN_ON(obj->userptr.mm == NULL))
		return -EINVAL;

	mn = i915_mmu_notifier_find(obj->userptr.mm);
	if (IS_ERR(mn))
		return PTR_ERR(mn);

	mo = kzalloc(sizeof(*mo), GFP_KERNEL);
	if (!mo)
		return -ENOMEM;

	mo->mn = mn;
	mo->obj = obj;
	mo->it.start = obj->userptr.ptr;
	mo->it.last = obj->userptr.ptr + obj->base.size - 1;
	RB_CLEAR_NODE(&mo->it.rb);

	obj->userptr.mmu_object = mo;
	return 0;
}

static void
#ifdef __NetBSD__
i915_mmu_notifier_free(struct i915_mmu_notifier *mn,
		       struct vmspace *mm)
#else
i915_mmu_notifier_free(struct i915_mmu_notifier *mn,
		       struct mm_struct *mm)
#endif
{
	if (mn == NULL)
		return;

	mmu_notifier_unregister(&mn->mn, mm);
	spin_lock_destroy(&mn->lock);
	kfree(mn);
}

#else

static void
__i915_gem_userptr_set_active(struct drm_i915_gem_object *obj, bool value)
{
}

static void
i915_gem_userptr_release__mmu_notifier(struct drm_i915_gem_object *obj)
{
}

static int
i915_gem_userptr_init__mmu_notifier(struct drm_i915_gem_object *obj,
				    unsigned flags)
{
	if ((flags & I915_USERPTR_UNSYNCHRONIZED) == 0)
		return -ENODEV;

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	return 0;
}

static void
#ifdef __NetBSD__
i915_mmu_notifier_free(struct i915_mmu_notifier *mn,
		       struct vmspace *mm)
#else
i915_mmu_notifier_free(struct i915_mmu_notifier *mn,
		       struct mm_struct *mm)
#endif
{
}

#endif

static struct i915_mm_struct *
#ifdef __NetBSD__
__i915_mm_struct_find(struct drm_i915_private *dev_priv, struct vmspace *real)
#else
__i915_mm_struct_find(struct drm_i915_private *dev_priv, struct mm_struct *real)
#endif
{
	struct i915_mm_struct *mm;

	/* Protected by dev_priv->mm_lock */
	hash_for_each_possible(dev_priv->mm_structs, mm, node, (unsigned long)real)
		if (mm->mm == real)
			return mm;

	return NULL;
}

static int
i915_gem_userptr_init__mm_struct(struct drm_i915_gem_object *obj)
{
	struct drm_i915_private *dev_priv = to_i915(obj->base.dev);
	struct i915_mm_struct *mm;
	int ret = 0;

	/* During release of the GEM object we hold the struct_mutex. This
	 * precludes us from calling mmput() at that time as that may be
	 * the last reference and so call exit_mmap(). exit_mmap() will
	 * attempt to reap the vma, and if we were holding a GTT mmap
	 * would then call drm_gem_vm_close() and attempt to reacquire
	 * the struct mutex. So in order to avoid that recursion, we have
	 * to defer releasing the mm reference until after we drop the
	 * struct_mutex, i.e. we need to schedule a worker to do the clean
	 * up.
	 */
	mutex_lock(&dev_priv->mm_lock);
#ifdef __NetBSD__
	mm = __i915_mm_struct_find(dev_priv, curproc->p_vmspace);
#else
	mm = __i915_mm_struct_find(dev_priv, current->mm);
#endif
	if (mm == NULL) {
		mm = kmalloc(sizeof(*mm), GFP_KERNEL);
		if (mm == NULL) {
			ret = -ENOMEM;
			goto out;
		}

		kref_init(&mm->kref);
		mm->i915 = to_i915(obj->base.dev);

#ifdef __NetBSD__
		mm->mm = curproc->p_vmspace;
#else
		mm->mm = current->mm;
#endif
		mmgrab(mm->mm);

		mm->mn = NULL;

		/* Protected by dev_priv->mm_lock */
		hash_add(dev_priv->mm_structs,
			 &mm->node, (unsigned long)mm->mm);
	} else
		kref_get(&mm->kref);

	obj->userptr.mm = mm;
out:
	mutex_unlock(&dev_priv->mm_lock);
	return ret;
}

static void
__i915_mm_struct_free__worker(struct work_struct *work)
{
	struct i915_mm_struct *mm = container_of(work, typeof(*mm), work);
	i915_mmu_notifier_free(mm->mn, mm->mm);
	mmdrop(mm->mm);
	kfree(mm);
}

static void
__i915_mm_struct_free(struct kref *kref)
{
	struct i915_mm_struct *mm = container_of(kref, typeof(*mm), kref);

	/* Protected by dev_priv->mm_lock */
	hash_del(&mm->node);
	mutex_unlock(&mm->i915->mm_lock);

	INIT_WORK(&mm->work, __i915_mm_struct_free__worker);
	queue_work(mm->i915->mm.userptr_wq, &mm->work);
}

static void
i915_gem_userptr_release__mm_struct(struct drm_i915_gem_object *obj)
{
	if (obj->userptr.mm == NULL)
		return;

	kref_put_mutex(&obj->userptr.mm->kref,
		       __i915_mm_struct_free,
		       &to_i915(obj->base.dev)->mm_lock);
	obj->userptr.mm = NULL;
}

struct get_pages_work {
	struct work_struct work;
	struct drm_i915_gem_object *obj;
	struct task_struct *task;
};

static struct sg_table *
__i915_gem_userptr_alloc_pages(struct drm_i915_gem_object *obj,
			       struct page **pvec, unsigned long num_pages)
{
	unsigned int max_segment = i915_sg_segment_size();
=======
	unsigned int max_segment = i915_sg_segment_size(obj->base.dev->dev);
>>>>>>> vendor/linux-drm-v6.6.35
	struct sg_table *st;
	struct page **pvec;
	unsigned int num_pages; /* limited by sg_alloc_table_from_pages_segment */
	int ret;

	if (overflows_type(obj->base.size >> PAGE_SHIFT, num_pages))
		return -E2BIG;

	num_pages = obj->base.size >> PAGE_SHIFT;
	st = kmalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	if (!obj->userptr.page_ref) {
		ret = -EAGAIN;
		goto err_free;
	}

	obj->userptr.page_ref++;
	pvec = obj->userptr.pvec;

alloc_table:
	ret = sg_alloc_table_from_pages_segment(st, pvec, num_pages, 0,
						num_pages << PAGE_SHIFT,
						max_segment, GFP_KERNEL);
	if (ret)
		goto err;

	ret = i915_gem_gtt_prepare_pages(obj, st);
	if (ret) {
		sg_free_table(st);

		if (max_segment > PAGE_SIZE) {
			max_segment = PAGE_SIZE;
			goto alloc_table;
		}

		goto err;
	}

	WARN_ON_ONCE(!(obj->cache_coherent & I915_BO_CACHE_COHERENT_FOR_WRITE));
	if (i915_gem_object_can_bypass_llc(obj))
		obj->cache_dirty = true;

	__i915_gem_object_set_pages(obj, st);

	return 0;

<<<<<<< HEAD
static void
__i915_gem_userptr_get_pages_worker(struct work_struct *_work)
{
	struct get_pages_work *work = container_of(_work, typeof(*work), work);
	struct drm_i915_gem_object *obj = work->obj;
	const unsigned long npages = obj->base.size >> PAGE_SHIFT;
	unsigned long pinned;
	struct page **pvec;
	int ret;

	ret = -ENOMEM;
	pinned = 0;

	pvec = kvmalloc_array(npages, sizeof(struct page *), GFP_KERNEL);
	if (pvec != NULL) {
#ifdef __NetBSD__
		struct vmspace *mm = obj->userptr.mm->mm;
#else
		struct mm_struct *mm = obj->userptr.mm->mm;
#endif
		unsigned int flags = 0;
		int locked = 0;

		if (!i915_gem_object_is_readonly(obj))
			flags |= FOLL_WRITE;

		ret = -EFAULT;
		if (mmget_not_zero(mm)) {
			while (pinned < npages) {
				if (!locked) {
					down_read(&mm->mmap_sem);
					locked = 1;
				}
				ret = get_user_pages_remote
					(work->task, mm,
					 obj->userptr.ptr + pinned * PAGE_SIZE,
					 npages - pinned,
					 flags,
					 pvec + pinned, NULL, &locked);
				if (ret < 0)
					break;

				pinned += ret;
			}
			if (locked)
				up_read(&mm->mmap_sem);
			mmput(mm);
		}
	}

	mutex_lock_nested(&obj->mm.lock, I915_MM_GET_PAGES);
	if (obj->userptr.work == &work->work) {
		struct sg_table *pages = ERR_PTR(ret);

		if (pinned == npages) {
			pages = __i915_gem_userptr_alloc_pages(obj, pvec,
							       npages);
			if (!IS_ERR(pages)) {
				pinned = 0;
				pages = NULL;
			}
		}

		obj->userptr.work = ERR_CAST(pages);
		if (IS_ERR(pages))
			__i915_gem_userptr_set_active(obj, false);
	}
	mutex_unlock(&obj->mm.lock);

	release_pages(pvec, pinned);
	kvfree(pvec);

	i915_gem_object_put(obj);
	put_task_struct(work->task);
	kfree(work);
}

static struct sg_table *
__i915_gem_userptr_get_pages_schedule(struct drm_i915_gem_object *obj)
{
	struct get_pages_work *work;

	/* Spawn a worker so that we can acquire the
	 * user pages without holding our mutex. Access
	 * to the user pages requires mmap_sem, and we have
	 * a strict lock ordering of mmap_sem, struct_mutex -
	 * we already hold struct_mutex here and so cannot
	 * call gup without encountering a lock inversion.
	 *
	 * Userspace will keep on repeating the operation
	 * (thanks to EAGAIN) until either we hit the fast
	 * path or the worker completes. If the worker is
	 * cancelled or superseded, the task is still run
	 * but the results ignored. (This leads to
	 * complications that we may have a stray object
	 * refcount that we need to be wary of when
	 * checking for existing objects during creation.)
	 * If the worker encounters an error, it reports
	 * that error back to this function through
	 * obj->userptr.work = ERR_PTR.
	 */
	work = kmalloc(sizeof(*work), GFP_KERNEL);
	if (work == NULL)
		return ERR_PTR(-ENOMEM);

	obj->userptr.work = &work->work;

	work->obj = i915_gem_object_get(obj);

	work->task = current;
	get_task_struct(work->task);

	INIT_WORK(&work->work, __i915_gem_userptr_get_pages_worker);
	queue_work(to_i915(obj->base.dev)->mm.userptr_wq, &work->work);

	return ERR_PTR(-EAGAIN);
}

static int i915_gem_userptr_get_pages(struct drm_i915_gem_object *obj)
{
	const unsigned long num_pages = obj->base.size >> PAGE_SHIFT;
#ifdef __NetBSD__
	struct vmspace *mm = obj->userptr.mm->mm;
#else
	struct mm_struct *mm = obj->userptr.mm->mm;
#endif
	struct page **pvec;
	struct sg_table *pages;
	bool active;
	int pinned;

	/* If userspace should engineer that these pages are replaced in
	 * the vma between us binding this page into the GTT and completion
	 * of rendering... Their loss. If they change the mapping of their
	 * pages they need to create a new bo to point to the new vma.
	 *
	 * However, that still leaves open the possibility of the vma
	 * being copied upon fork. Which falls under the same userspace
	 * synchronisation issue as a regular bo, except that this time
	 * the process may not be expecting that a particular piece of
	 * memory is tied to the GPU.
	 *
	 * Fortunately, we can hook into the mmu_notifier in order to
	 * discard the page references prior to anything nasty happening
	 * to the vma (discard or cloning) which should prevent the more
	 * egregious cases from causing harm.
	 */

	if (obj->userptr.work) {
		/* active flag should still be held for the pending work */
		if (IS_ERR(obj->userptr.work))
			return PTR_ERR(obj->userptr.work);
		else
			return -EAGAIN;
	}

	pvec = NULL;
	pinned = 0;

#ifdef __NetBSD__
	if (mm == curproc->p_vmspace)
#else
	if (mm == current->mm)
#endif
	{
		pvec = kvmalloc_array(num_pages, sizeof(struct page *),
				      GFP_KERNEL |
				      __GFP_NORETRY |
				      __GFP_NOWARN);
		if (pvec) /* defer to worker if malloc fails */
			pinned = __get_user_pages_fast(obj->userptr.ptr,
						       num_pages,
						       !i915_gem_object_is_readonly(obj),
						       pvec);
	}

	active = false;
	if (pinned < 0) {
		pages = ERR_PTR(pinned);
		pinned = 0;
	} else if (pinned < num_pages) {
		pages = __i915_gem_userptr_get_pages_schedule(obj);
		active = pages == ERR_PTR(-EAGAIN);
	} else {
		pages = __i915_gem_userptr_alloc_pages(obj, pvec, num_pages);
		active = !IS_ERR(pages);
	}
	if (active)
		__i915_gem_userptr_set_active(obj, true);

	if (IS_ERR(pages))
		release_pages(pvec, pinned);
	kvfree(pvec);

	return PTR_ERR_OR_ZERO(pages);
=======
err:
	i915_gem_object_userptr_drop_ref(obj);
err_free:
	kfree(st);
	return ret;
>>>>>>> vendor/linux-drm-v6.6.35
}

static void
i915_gem_userptr_put_pages(struct drm_i915_gem_object *obj,
			   struct sg_table *pages)
{
	struct sgt_iter sgt_iter;
	struct page *page;

	if (!pages)
		return;

	__i915_gem_object_release_shmem(obj, pages, true);
	i915_gem_gtt_finish_pages(obj, pages);

	/*
	 * We always mark objects as dirty when they are used by the GPU,
	 * just in case. However, if we set the vma as being read-only we know
	 * that the object will never have been written to.
	 */
	if (i915_gem_object_is_readonly(obj))
		obj->mm.dirty = false;

	for_each_sgt_page(page, sgt_iter, pages) {
		if (obj->mm.dirty && trylock_page(page)) {
			/*
			 * As this may not be anonymous memory (e.g. shmem)
			 * but exist on a real mapping, we have to lock
			 * the page in order to dirty it -- holding
			 * the page reference is not sufficient to
			 * prevent the inode from being truncated.
			 * Play safe and take the lock.
			 *
			 * However...!
			 *
			 * The mmu-notifier can be invalidated for a
			 * migrate_folio, that is alreadying holding the lock
			 * on the folio. Such a try_to_unmap() will result
			 * in us calling put_pages() and so recursively try
			 * to lock the page. We avoid that deadlock with
			 * a trylock_page() and in exchange we risk missing
			 * some page dirtying.
			 */
			set_page_dirty(page);
			unlock_page(page);
		}

		mark_page_accessed(page);
	}
	obj->mm.dirty = false;

	sg_free_table(pages);
	kfree(pages);

	i915_gem_object_userptr_drop_ref(obj);
}

static int i915_gem_object_userptr_unbind(struct drm_i915_gem_object *obj)
{
	struct sg_table *pages;
	int err;

	err = i915_gem_object_unbind(obj, I915_GEM_OBJECT_UNBIND_ACTIVE);
	if (err)
		return err;

	if (GEM_WARN_ON(i915_gem_object_has_pinned_pages(obj)))
		return -EBUSY;

	assert_object_held(obj);

	pages = __i915_gem_object_unset_pages(obj);
	if (!IS_ERR_OR_NULL(pages))
		i915_gem_userptr_put_pages(obj, pages);

	return err;
}

int i915_gem_object_userptr_submit_init(struct drm_i915_gem_object *obj)
{
	const unsigned long num_pages = obj->base.size >> PAGE_SHIFT;
	struct page **pvec;
	unsigned int gup_flags = 0;
	unsigned long notifier_seq;
	int pinned, ret;

	if (obj->userptr.notifier.mm != current->mm)
		return -EFAULT;

	notifier_seq = mmu_interval_read_begin(&obj->userptr.notifier);

	ret = i915_gem_object_lock_interruptible(obj, NULL);
	if (ret)
		return ret;

	if (notifier_seq == obj->userptr.notifier_seq && obj->userptr.pvec) {
		i915_gem_object_unlock(obj);
		return 0;
	}

	ret = i915_gem_object_userptr_unbind(obj);
	i915_gem_object_unlock(obj);
	if (ret)
		return ret;

	pvec = kvmalloc_array(num_pages, sizeof(struct page *), GFP_KERNEL);
	if (!pvec)
		return -ENOMEM;

	if (!i915_gem_object_is_readonly(obj))
		gup_flags |= FOLL_WRITE;

	pinned = 0;
	while (pinned < num_pages) {
		ret = pin_user_pages_fast(obj->userptr.ptr + pinned * PAGE_SIZE,
					  num_pages - pinned, gup_flags,
					  &pvec[pinned]);
		if (ret < 0)
			goto out;

		pinned += ret;
	}

	ret = i915_gem_object_lock_interruptible(obj, NULL);
	if (ret)
		goto out;

	if (mmu_interval_read_retry(&obj->userptr.notifier,
		!obj->userptr.page_ref ? notifier_seq :
		obj->userptr.notifier_seq)) {
		ret = -EAGAIN;
		goto out_unlock;
	}

	if (!obj->userptr.page_ref++) {
		obj->userptr.pvec = pvec;
		obj->userptr.notifier_seq = notifier_seq;
		pvec = NULL;
		ret = ____i915_gem_object_get_pages(obj);
	}

	obj->userptr.page_ref--;

out_unlock:
	i915_gem_object_unlock(obj);

out:
	if (pvec) {
		unpin_user_pages(pvec, pinned);
		kvfree(pvec);
	}

	return ret;
}

int i915_gem_object_userptr_submit_done(struct drm_i915_gem_object *obj)
{
	if (mmu_interval_read_retry(&obj->userptr.notifier,
				    obj->userptr.notifier_seq)) {
		/* We collided with the mmu notifier, need to retry */

		return -EAGAIN;
	}

	return 0;
}

int i915_gem_object_userptr_validate(struct drm_i915_gem_object *obj)
{
	int err;

	err = i915_gem_object_userptr_submit_init(obj);
	if (err)
		return err;

	err = i915_gem_object_lock_interruptible(obj, NULL);
	if (!err) {
		/*
		 * Since we only check validity, not use the pages,
		 * it doesn't matter if we collide with the mmu notifier,
		 * and -EAGAIN handling is not required.
		 */
		err = i915_gem_object_pin_pages(obj);
		if (!err)
			i915_gem_object_unpin_pages(obj);

		i915_gem_object_unlock(obj);
	}

	return err;
}

static void
i915_gem_userptr_release(struct drm_i915_gem_object *obj)
{
	GEM_WARN_ON(obj->userptr.page_ref);

	if (!obj->userptr.notifier.mm)
		return;

	mmu_interval_notifier_remove(&obj->userptr.notifier);
	obj->userptr.notifier.mm = NULL;
}

static int
i915_gem_userptr_dmabuf_export(struct drm_i915_gem_object *obj)
{
	drm_dbg(obj->base.dev, "Exporting userptr no longer allowed\n");

	return -EINVAL;
}

static int
i915_gem_userptr_pwrite(struct drm_i915_gem_object *obj,
			const struct drm_i915_gem_pwrite *args)
{
	drm_dbg(obj->base.dev, "pwrite to userptr no longer allowed\n");

	return -EINVAL;
}

static int
i915_gem_userptr_pread(struct drm_i915_gem_object *obj,
		       const struct drm_i915_gem_pread *args)
{
	drm_dbg(obj->base.dev, "pread from userptr no longer allowed\n");

	return -EINVAL;
}

static const struct drm_i915_gem_object_ops i915_gem_userptr_ops = {
	.name = "i915_gem_object_userptr",
	.flags = I915_GEM_OBJECT_IS_SHRINKABLE |
		 I915_GEM_OBJECT_NO_MMAP |
		 I915_GEM_OBJECT_IS_PROXY,
	.get_pages = i915_gem_userptr_get_pages,
	.put_pages = i915_gem_userptr_put_pages,
	.dmabuf_export = i915_gem_userptr_dmabuf_export,
	.pwrite = i915_gem_userptr_pwrite,
	.pread = i915_gem_userptr_pread,
	.release = i915_gem_userptr_release,
};

#endif

static int
probe_range(struct mm_struct *mm, unsigned long addr, unsigned long len)
{
	VMA_ITERATOR(vmi, mm, addr);
	struct vm_area_struct *vma;
	unsigned long end = addr + len;

	mmap_read_lock(mm);
	for_each_vma_range(vmi, vma, end) {
		/* Check for holes, note that we also update the addr below */
		if (vma->vm_start > addr)
			break;

		if (vma->vm_flags & (VM_PFNMAP | VM_MIXEDMAP))
			break;

		addr = vma->vm_end;
	}
	mmap_read_unlock(mm);

	if (vma || addr < end)
		return -EFAULT;
	return 0;
}

/*
 * Creates a new mm object that wraps some normal memory from the process
 * context - user memory.
 *
 * We impose several restrictions upon the memory being mapped
 * into the GPU.
 * 1. It must be page aligned (both start/end addresses, i.e ptr and size).
 * 2. It must be normal system memory, not a pointer into another map of IO
 *    space (e.g. it must not be a GTT mmapping of another object).
 * 3. We only allow a bo as large as we could in theory map into the GTT,
 *    that is we limit the size to the total size of the GTT.
 * 4. The bo is marked as being snoopable. The backing pages are left
 *    accessible directly by the CPU, but reads and writes by the GPU may
 *    incur the cost of a snoop (unless you have an LLC architecture).
 *
 * Synchronisation between multiple users and the GPU is left to userspace
 * through the normal set-domain-ioctl. The kernel will enforce that the
 * GPU relinquishes the VMA before it is returned back to the system
 * i.e. upon free(), munmap() or process termination. However, the userspace
 * malloc() library may not immediately relinquish the VMA after free() and
 * instead reuse it whilst the GPU is still reading and writing to the VMA.
 * Caveat emptor.
 *
 * Also note, that the object created here is not currently a "first class"
 * object, in that several ioctls are banned. These are the CPU access
 * ioctls: mmap(), pwrite and pread. In practice, you are expected to use
 * direct access via your pointer rather than use those ioctls. Another
 * restriction is that we do not allow userptr surfaces to be pinned to the
 * hardware and so we reject any attempt to create a framebuffer out of a
 * userptr.
 *
 * If you think this is a good interface to use to pass GPU memory between
 * drivers, please use dma-buf instead. In fact, wherever possible use
 * dma-buf instead.
 */
int
i915_gem_userptr_ioctl(struct drm_device *dev,
		       void *data,
		       struct drm_file *file)
{
	static struct lock_class_key __maybe_unused lock_class;
	struct drm_i915_private *dev_priv = to_i915(dev);
	struct drm_i915_gem_userptr *args = data;
	struct drm_i915_gem_object __maybe_unused *obj;
	int __maybe_unused ret;
	u32 __maybe_unused handle;

	if (!HAS_LLC(dev_priv) && !HAS_SNOOP(dev_priv)) {
		/* We cannot support coherent userptr objects on hw without
		 * LLC and broken snooping.
		 */
		return -ENODEV;
	}

	if (args->flags & ~(I915_USERPTR_READ_ONLY |
			    I915_USERPTR_UNSYNCHRONIZED |
			    I915_USERPTR_PROBE))
		return -EINVAL;

	if (i915_gem_object_size_2big(args->user_size))
		return -E2BIG;

	if (!args->user_size)
		return -EINVAL;

	if (offset_in_page(args->user_ptr | args->user_size))
		return -EINVAL;

	if (!access_ok((char __user *)(unsigned long)args->user_ptr, args->user_size))
		return -EFAULT;

	if (args->flags & I915_USERPTR_UNSYNCHRONIZED)
		return -ENODEV;

	if (args->flags & I915_USERPTR_READ_ONLY) {
		/*
		 * On almost all of the older hw, we cannot tell the GPU that
		 * a page is readonly.
		 */
		if (!to_gt(dev_priv)->vm->has_read_only)
			return -ENODEV;
	}

	if (args->flags & I915_USERPTR_PROBE) {
		/*
		 * Check that the range pointed to represents real struct
		 * pages and not iomappings (at this moment in time!)
		 */
		ret = probe_range(current->mm, args->user_ptr, args->user_size);
		if (ret)
			return ret;
	}

#ifdef CONFIG_MMU_NOTIFIER
	obj = i915_gem_object_alloc();
	if (obj == NULL)
		return -ENOMEM;

	drm_gem_private_object_init(dev, &obj->base, args->user_size);
	i915_gem_object_init(obj, &i915_gem_userptr_ops, &lock_class,
			     I915_BO_ALLOC_USER);
	obj->mem_flags = I915_BO_FLAG_STRUCT_PAGE;
	obj->read_domains = I915_GEM_DOMAIN_CPU;
	obj->write_domain = I915_GEM_DOMAIN_CPU;
	i915_gem_object_set_cache_coherency(obj, I915_CACHE_LLC);

	obj->userptr.ptr = args->user_ptr;
	obj->userptr.notifier_seq = ULONG_MAX;
	if (args->flags & I915_USERPTR_READ_ONLY)
		i915_gem_object_set_readonly(obj);

	/* And keep a pointer to the current->mm for resolving the user pages
	 * at binding. This means that we need to hook into the mmu_notifier
	 * in order to detect if the mmu is destroyed.
	 */
	ret = i915_gem_userptr_init__mmu_notifier(obj);
	if (ret == 0)
		ret = drm_gem_handle_create(file, &obj->base, &handle);

	/* drop reference from allocate - handle holds it now */
	i915_gem_object_put(obj);
	if (ret)
		return ret;

	args->handle = handle;
	return 0;
#else
	return -ENODEV;
#endif
}

int i915_gem_init_userptr(struct drm_i915_private *dev_priv)
{
#ifdef CONFIG_MMU_NOTIFIER
	rwlock_init(&dev_priv->mm.notifier_lock);
#endif

	return 0;
}

void i915_gem_cleanup_userptr(struct drm_i915_private *dev_priv)
{
<<<<<<< HEAD
	destroy_workqueue(dev_priv->mm.userptr_wq);
	mutex_destroy(&dev_priv->mm_lock);
=======
>>>>>>> vendor/linux-drm-v6.6.35
}
