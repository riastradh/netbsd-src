/*	$NetBSD: drm_prime.c,v 1.20 2022/07/06 01:12:45 riastradh Exp $	*/

/*
 * Copyright © 2012 Red Hat
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
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 * Authors:
 *      Dave Airlie <airlied@redhat.com>
 *      Rob Clark <rob.clark@linaro.org>
 *
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: drm_prime.c,v 1.20 2022/07/06 01:12:45 riastradh Exp $");

#include <linux/export.h>
#include <linux/dma-buf.h>
#include <linux/rbtree.h>
#include <linux/module.h>

#include <drm/drm.h>
#include <drm/drm_drv.h>
#include <drm/drm_file.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem.h>
#include <drm/drm_prime.h>

#include "drm_internal.h"

<<<<<<< HEAD
#ifdef __NetBSD__

#include <sys/file.h>

#include <drm/bus_dma_hacks.h>

#include <linux/nbsd-namespace.h>

#endif	/* __NetBSD__ */
=======
MODULE_IMPORT_NS(DMA_BUF);
>>>>>>> vendor/linux-drm-v6.6.35

/**
 * DOC: overview and lifetime rules
 *
 * Similar to GEM global names, PRIME file descriptors are also used to share
 * buffer objects across processes. They offer additional security: as file
 * descriptors must be explicitly sent over UNIX domain sockets to be shared
 * between applications, they can't be guessed like the globally unique GEM
 * names.
 *
 * Drivers that support the PRIME API implement the drm_gem_object_funcs.export
 * and &drm_driver.gem_prime_import hooks. &dma_buf_ops implementations for
 * drivers are all individually exported for drivers which need to overwrite
 * or reimplement some of them.
 *
 * Reference Counting for GEM Drivers
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * On the export the &dma_buf holds a reference to the exported buffer object,
 * usually a &drm_gem_object. It takes this reference in the PRIME_HANDLE_TO_FD
 * IOCTL, when it first calls &drm_gem_object_funcs.export
 * and stores the exporting GEM object in the &dma_buf.priv field. This
 * reference needs to be released when the final reference to the &dma_buf
 * itself is dropped and its &dma_buf_ops.release function is called.  For
 * GEM-based drivers, the &dma_buf should be exported using
 * drm_gem_dmabuf_export() and then released by drm_gem_dmabuf_release().
 *
 * Thus the chain of references always flows in one direction, avoiding loops:
 * importing GEM object -> dma-buf -> exported GEM bo. A further complication
 * are the lookup caches for import and export. These are required to guarantee
 * that any given object will always have only one unique userspace handle. This
 * is required to allow userspace to detect duplicated imports, since some GEM
 * drivers do fail command submissions if a given buffer object is listed more
 * than once. These import and export caches in &drm_prime_file_private only
 * retain a weak reference, which is cleaned up when the corresponding object is
 * released.
 *
 * Self-importing: If userspace is using PRIME as a replacement for flink then
 * it will get a fd->handle request for a GEM object that it created.  Drivers
 * should detect this situation and return back the underlying object from the
 * dma-buf private. For GEM based drivers this is handled in
 * drm_gem_prime_import() already.
 */

struct drm_prime_member {
	struct dma_buf *dma_buf;
	uint32_t handle;

	struct rb_node dmabuf_rb;
	struct rb_node handle_rb;
};

#ifdef __NetBSD__
static int
compare_dmabufs(void *cookie, const void *va, const void *vb)
{
	const struct drm_prime_member *ma = va;
	const struct drm_prime_member *mb = vb;

	if (ma->dma_buf < mb->dma_buf)
		return -1;
	if (ma->dma_buf > mb->dma_buf)
		return +1;
	return 0;
}

static int
compare_dmabuf_key(void *cookie, const void *vm, const void *vk)
{
	const struct drm_prime_member *m = vm;
	const struct dma_buf *const *kp = vk;

	if (m->dma_buf < *kp)
		return -1;
	if (m->dma_buf > *kp)
		return +1;
	return 0;
}

static int
compare_handles(void *cookie, const void *va, const void *vb)
{
	const struct drm_prime_member *ma = va;
	const struct drm_prime_member *mb = vb;

	if (ma->handle < mb->handle)
		return -1;
	if (ma->handle > mb->handle)
		return +1;
	return 0;
}

static int
compare_handle_key(void *cookie, const void *vm, const void *vk)
{
	const struct drm_prime_member *m = vm;
	const uint32_t *kp = vk;

	if (m->handle < *kp)
		return -1;
	if (m->handle > *kp)
		return +1;
	return 0;
}

static const rb_tree_ops_t dmabuf_ops = {
	.rbto_compare_nodes = compare_dmabufs,
	.rbto_compare_key = compare_dmabuf_key,
	.rbto_node_offset = offsetof(struct drm_prime_member, dmabuf_rb),
};

static const rb_tree_ops_t handle_ops = {
	.rbto_compare_nodes = compare_handles,
	.rbto_compare_key = compare_handle_key,
	.rbto_node_offset = offsetof(struct drm_prime_member, handle_rb),
};
#endif

static int drm_prime_add_buf_handle(struct drm_prime_file_private *prime_fpriv,
				    struct dma_buf *dma_buf, uint32_t handle)
{
	struct drm_prime_member *member;
#ifdef __NetBSD__
	struct drm_prime_member *collision __diagused;
#else
	struct rb_node **p, *rb;
#endif

	member = kmalloc(sizeof(*member), GFP_KERNEL);
	if (!member)
		return -ENOMEM;

	get_dma_buf(dma_buf);
	member->dma_buf = dma_buf;
	member->handle = handle;

#ifdef __NetBSD__
	collision = rb_tree_insert_node(&prime_fpriv->dmabufs.rbr_tree,
	    member);
	KASSERT(collision == member);
#else
	rb = NULL;
	p = &prime_fpriv->dmabufs.rb_node;
	while (*p) {
		struct drm_prime_member *pos;

		rb = *p;
		pos = rb_entry(rb, struct drm_prime_member, dmabuf_rb);
		if (dma_buf > pos->dma_buf)
			p = &rb->rb_right;
		else
			p = &rb->rb_left;
	}
	rb_link_node(&member->dmabuf_rb, rb, p);
	rb_insert_color(&member->dmabuf_rb, &prime_fpriv->dmabufs);
#endif

#ifdef __NetBSD__
	collision = rb_tree_insert_node(&prime_fpriv->handles.rbr_tree,
	    member);
	KASSERT(collision == member);
#else
	rb = NULL;
	p = &prime_fpriv->handles.rb_node;
	while (*p) {
		struct drm_prime_member *pos;

		rb = *p;
		pos = rb_entry(rb, struct drm_prime_member, handle_rb);
		if (handle > pos->handle)
			p = &rb->rb_right;
		else
			p = &rb->rb_left;
	}
	rb_link_node(&member->handle_rb, rb, p);
	rb_insert_color(&member->handle_rb, &prime_fpriv->handles);
#endif

	return 0;
}

static struct dma_buf *drm_prime_lookup_buf_by_handle(struct drm_prime_file_private *prime_fpriv,
						      uint32_t handle)
{
#ifdef __NetBSD__
	struct drm_prime_member *member;

	member = rb_tree_find_node(&prime_fpriv->handles.rbr_tree, &handle);
	if (member == NULL)
		return NULL;
	return member->dma_buf;
#else
	struct rb_node *rb;

	rb = prime_fpriv->handles.rb_node;
	while (rb) {
		struct drm_prime_member *member;

		member = rb_entry(rb, struct drm_prime_member, handle_rb);
		if (member->handle == handle)
			return member->dma_buf;
		else if (member->handle < handle)
			rb = rb->rb_right;
		else
			rb = rb->rb_left;
	}

	return NULL;
#endif
}

static int drm_prime_lookup_buf_handle(struct drm_prime_file_private *prime_fpriv,
				       struct dma_buf *dma_buf,
				       uint32_t *handle)
{
#ifdef __NetBSD__
	struct drm_prime_member *member;

	member = rb_tree_find_node(&prime_fpriv->dmabufs.rbr_tree, &dma_buf);
	if (member == NULL)
		return -ENOENT;
	*handle = member->handle;
	return 0;
#else
	struct rb_node *rb;

	rb = prime_fpriv->dmabufs.rb_node;
	while (rb) {
		struct drm_prime_member *member;

		member = rb_entry(rb, struct drm_prime_member, dmabuf_rb);
		if (member->dma_buf == dma_buf) {
			*handle = member->handle;
			return 0;
		} else if (member->dma_buf < dma_buf) {
			rb = rb->rb_right;
		} else {
			rb = rb->rb_left;
		}
	}

	return -ENOENT;
#endif
}

void drm_prime_remove_buf_handle(struct drm_prime_file_private *prime_fpriv,
				 uint32_t handle)
{
#ifdef __NetBSD__
	struct drm_prime_member *member;

	member = rb_tree_find_node(&prime_fpriv->dmabufs.rbr_tree, &dma_buf);
	if (member != NULL) {
		rb_tree_remove_node(&prime_fpriv->handles.rbr_tree, member);
		rb_tree_remove_node(&prime_fpriv->dmabufs.rbr_tree, member);
		dma_buf_put(dma_buf);
		kfree(member);
	}
#else
	struct rb_node *rb;

	mutex_lock(&prime_fpriv->lock);

	rb = prime_fpriv->handles.rb_node;
	while (rb) {
		struct drm_prime_member *member;

		member = rb_entry(rb, struct drm_prime_member, handle_rb);
		if (member->handle == handle) {
			rb_erase(&member->handle_rb, &prime_fpriv->handles);
			rb_erase(&member->dmabuf_rb, &prime_fpriv->dmabufs);

			dma_buf_put(member->dma_buf);
			kfree(member);
			break;
		} else if (member->handle < handle) {
			rb = rb->rb_right;
		} else {
			rb = rb->rb_left;
		}
	}
<<<<<<< HEAD
#endif
=======

	mutex_unlock(&prime_fpriv->lock);
>>>>>>> vendor/linux-drm-v6.6.35
}

void drm_prime_init_file_private(struct drm_prime_file_private *prime_fpriv)
{
	mutex_init(&prime_fpriv->lock);
#ifdef __NetBSD__
	rb_tree_init(&prime_fpriv->dmabufs.rbr_tree, &dmabuf_ops);
	rb_tree_init(&prime_fpriv->handles.rbr_tree, &handle_ops);
#else
	prime_fpriv->dmabufs = RB_ROOT;
	prime_fpriv->handles = RB_ROOT;
#endif
}

void drm_prime_destroy_file_private(struct drm_prime_file_private *prime_fpriv)
{
	mutex_destroy(&prime_fpriv->lock);
	/* by now drm_gem_release should've made sure the list is empty */
	WARN_ON(!RB_EMPTY_ROOT(&prime_fpriv->dmabufs));
	WARN_ON(!RB_EMPTY_ROOT(&prime_fpriv->handles));
}

/**
 * drm_gem_dmabuf_export - &dma_buf export implementation for GEM
 * @dev: parent device for the exported dmabuf
 * @exp_info: the export information used by dma_buf_export()
 *
 * This wraps dma_buf_export() for use by generic GEM drivers that are using
 * drm_gem_dmabuf_release(). In addition to calling dma_buf_export(), we take
 * a reference to the &drm_device and the exported &drm_gem_object (stored in
 * &dma_buf_export_info.priv) which is released by drm_gem_dmabuf_release().
 *
 * Returns the new dmabuf.
 */
struct dma_buf *drm_gem_dmabuf_export(struct drm_device *dev,
				      struct dma_buf_export_info *exp_info)
{
	struct drm_gem_object *obj = exp_info->priv;
	struct dma_buf *dma_buf;

	dma_buf = dma_buf_export(exp_info);
	if (IS_ERR(dma_buf))
		return dma_buf;

	drm_dev_get(dev);
	drm_gem_object_get(obj);
#ifndef __NetBSD__		/* XXX dmabuf share */
	dma_buf->file->f_mapping = obj->dev->anon_inode->i_mapping;
#endif

	return dma_buf;
}
EXPORT_SYMBOL(drm_gem_dmabuf_export);

/**
 * drm_gem_dmabuf_release - &dma_buf release implementation for GEM
 * @dma_buf: buffer to be released
 *
 * Generic release function for dma_bufs exported as PRIME buffers. GEM drivers
 * must use this in their &dma_buf_ops structure as the release callback.
 * drm_gem_dmabuf_release() should be used in conjunction with
 * drm_gem_dmabuf_export().
 */
void drm_gem_dmabuf_release(struct dma_buf *dma_buf)
{
	struct drm_gem_object *obj = dma_buf->priv;
	struct drm_device *dev = obj->dev;

	/* drop the reference on the export fd holds */
	drm_gem_object_put(obj);

	drm_dev_put(dev);
}
EXPORT_SYMBOL(drm_gem_dmabuf_release);

/**
 * drm_gem_prime_fd_to_handle - PRIME import function for GEM drivers
 * @dev: drm_device to import into
 * @file_priv: drm file-private structure
 * @prime_fd: fd id of the dma-buf which should be imported
 * @handle: pointer to storage for the handle of the imported buffer object
 *
 * This is the PRIME import function which must be used mandatorily by GEM
 * drivers to ensure correct lifetime management of the underlying GEM object.
 * The actual importing of GEM object from the dma-buf is done through the
 * &drm_driver.gem_prime_import driver callback.
 *
 * Returns 0 on success or a negative error code on failure.
 */
int drm_gem_prime_fd_to_handle(struct drm_device *dev,
			       struct drm_file *file_priv, int prime_fd,
			       uint32_t *handle)
{
	struct dma_buf *dma_buf;
	struct drm_gem_object *obj;
	int ret;

	dma_buf = dma_buf_get(prime_fd);
	if (IS_ERR(dma_buf))
		return PTR_ERR(dma_buf);

	mutex_lock(&file_priv->prime.lock);

	ret = drm_prime_lookup_buf_handle(&file_priv->prime,
			dma_buf, handle);
	if (ret == 0)
		goto out_put;

	/* never seen this one, need to import */
	mutex_lock(&dev->object_name_lock);
	if (dev->driver->gem_prime_import)
		obj = dev->driver->gem_prime_import(dev, dma_buf);
	else
		obj = drm_gem_prime_import(dev, dma_buf);
	if (IS_ERR(obj)) {
		ret = PTR_ERR(obj);
		goto out_unlock;
	}

	if (obj->dma_buf) {
		WARN_ON(obj->dma_buf != dma_buf);
	} else {
		obj->dma_buf = dma_buf;
		get_dma_buf(dma_buf);
	}

	/* _handle_create_tail unconditionally unlocks dev->object_name_lock. */
	ret = drm_gem_handle_create_tail(file_priv, obj, handle);
	drm_gem_object_put(obj);
	if (ret)
		goto out_put;

	ret = drm_prime_add_buf_handle(&file_priv->prime,
			dma_buf, *handle);
	mutex_unlock(&file_priv->prime.lock);
	if (ret)
		goto fail;

	dma_buf_put(dma_buf);

	return 0;

fail:
	/* hmm, if driver attached, we are relying on the free-object path
	 * to detach.. which seems ok..
	 */
	drm_gem_handle_delete(file_priv, *handle);
	dma_buf_put(dma_buf);
	return ret;

out_unlock:
	mutex_unlock(&dev->object_name_lock);
out_put:
	mutex_unlock(&file_priv->prime.lock);
	dma_buf_put(dma_buf);
	return ret;
}
EXPORT_SYMBOL(drm_gem_prime_fd_to_handle);

int drm_prime_fd_to_handle_ioctl(struct drm_device *dev, void *data,
				 struct drm_file *file_priv)
{
	struct drm_prime_handle *args = data;

	if (dev->driver->prime_fd_to_handle) {
		return dev->driver->prime_fd_to_handle(dev, file_priv, args->fd,
						       &args->handle);
	}

	return drm_gem_prime_fd_to_handle(dev, file_priv, args->fd, &args->handle);
}

static struct dma_buf *export_and_register_object(struct drm_device *dev,
						  struct drm_gem_object *obj,
						  uint32_t flags)
{
	struct dma_buf *dmabuf;

	/* prevent races with concurrent gem_close. */
	if (obj->handle_count == 0) {
		dmabuf = ERR_PTR(-ENOENT);
		return dmabuf;
	}

	if (obj->funcs && obj->funcs->export)
		dmabuf = obj->funcs->export(obj, flags);
	else
		dmabuf = drm_gem_prime_export(obj, flags);
	if (IS_ERR(dmabuf)) {
		/* normally the created dma-buf takes ownership of the ref,
		 * but if that fails then drop the ref
		 */
		return dmabuf;
	}

	/*
	 * Note that callers do not need to clean up the export cache
	 * since the check for obj->handle_count guarantees that someone
	 * will clean it up.
	 */
	obj->dma_buf = dmabuf;
	get_dma_buf(obj->dma_buf);

	return dmabuf;
}

/**
 * drm_gem_prime_handle_to_fd - PRIME export function for GEM drivers
 * @dev: dev to export the buffer from
 * @file_priv: drm file-private structure
 * @handle: buffer handle to export
 * @flags: flags like DRM_CLOEXEC
 * @prime_fd: pointer to storage for the fd id of the create dma-buf
 *
 * This is the PRIME export function which must be used mandatorily by GEM
 * drivers to ensure correct lifetime management of the underlying GEM object.
 * The actual exporting from GEM object to a dma-buf is done through the
 * &drm_gem_object_funcs.export callback.
 */
int drm_gem_prime_handle_to_fd(struct drm_device *dev,
			       struct drm_file *file_priv, uint32_t handle,
			       uint32_t flags,
			       int *prime_fd)
{
	struct drm_gem_object *obj;
	int ret = 0;
	struct dma_buf *dmabuf;

	mutex_lock(&file_priv->prime.lock);
	obj = drm_gem_object_lookup(file_priv, handle);
	if (!obj)  {
		ret = -ENOENT;
		goto out_unlock;
	}

	dmabuf = drm_prime_lookup_buf_by_handle(&file_priv->prime, handle);
	if (dmabuf) {
		get_dma_buf(dmabuf);
		goto out_have_handle;
	}

	mutex_lock(&dev->object_name_lock);
	/* re-export the original imported object */
	if (obj->import_attach) {
		dmabuf = obj->import_attach->dmabuf;
		get_dma_buf(dmabuf);
		goto out_have_obj;
	}

	if (obj->dma_buf) {
		get_dma_buf(obj->dma_buf);
		dmabuf = obj->dma_buf;
		goto out_have_obj;
	}

	dmabuf = export_and_register_object(dev, obj, flags);
	if (IS_ERR(dmabuf)) {
		/* normally the created dma-buf takes ownership of the ref,
		 * but if that fails then drop the ref
		 */
		ret = PTR_ERR(dmabuf);
		mutex_unlock(&dev->object_name_lock);
		goto out;
	}

out_have_obj:
	/*
	 * If we've exported this buffer then cheat and add it to the import list
	 * so we get the correct handle back. We must do this under the
	 * protection of dev->object_name_lock to ensure that a racing gem close
	 * ioctl doesn't miss to remove this buffer handle from the cache.
	 */
	ret = drm_prime_add_buf_handle(&file_priv->prime,
				       dmabuf, handle);
	mutex_unlock(&dev->object_name_lock);
	if (ret)
		goto fail_put_dmabuf;

out_have_handle:
	ret = dma_buf_fd(dmabuf, flags);
	/*
	 * We must _not_ remove the buffer from the handle cache since the newly
	 * created dma buf is already linked in the global obj->dma_buf pointer,
	 * and that is invariant as long as a userspace gem handle exists.
	 * Closing the handle will clean out the cache anyway, so we don't leak.
	 */
	if (ret < 0) {
		goto fail_put_dmabuf;
	} else {
		*prime_fd = ret;
		ret = 0;
	}

	goto out;

fail_put_dmabuf:
	dma_buf_put(dmabuf);
out:
	drm_gem_object_put(obj);
out_unlock:
	mutex_unlock(&file_priv->prime.lock);

	return ret;
}
EXPORT_SYMBOL(drm_gem_prime_handle_to_fd);

int drm_prime_handle_to_fd_ioctl(struct drm_device *dev, void *data,
				 struct drm_file *file_priv)
{
	struct drm_prime_handle *args = data;

	/* check flags are valid */
	if (args->flags & ~(DRM_CLOEXEC | DRM_RDWR))
		return -EINVAL;

	if (dev->driver->prime_handle_to_fd) {
		return dev->driver->prime_handle_to_fd(dev, file_priv,
						       args->handle, args->flags,
						       &args->fd);
	}
	return drm_gem_prime_handle_to_fd(dev, file_priv, args->handle,
					  args->flags, &args->fd);
}

/**
 * DOC: PRIME Helpers
 *
 * Drivers can implement &drm_gem_object_funcs.export and
 * &drm_driver.gem_prime_import in terms of simpler APIs by using the helper
 * functions drm_gem_prime_export() and drm_gem_prime_import(). These functions
 * implement dma-buf support in terms of some lower-level helpers, which are
 * again exported for drivers to use individually:
 *
 * Exporting buffers
 * ~~~~~~~~~~~~~~~~~
 *
 * Optional pinning of buffers is handled at dma-buf attach and detach time in
 * drm_gem_map_attach() and drm_gem_map_detach(). Backing storage itself is
 * handled by drm_gem_map_dma_buf() and drm_gem_unmap_dma_buf(), which relies on
 * &drm_gem_object_funcs.get_sg_table. If &drm_gem_object_funcs.get_sg_table is
 * unimplemented, exports into another device are rejected.
 *
 * For kernel-internal access there's drm_gem_dmabuf_vmap() and
 * drm_gem_dmabuf_vunmap(). Userspace mmap support is provided by
 * drm_gem_dmabuf_mmap().
 *
 * Note that these export helpers can only be used if the underlying backing
 * storage is fully coherent and either permanently pinned, or it is safe to pin
 * it indefinitely.
 *
 * FIXME: The underlying helper functions are named rather inconsistently.
 *
 * Importing buffers
 * ~~~~~~~~~~~~~~~~~
 *
 * Importing dma-bufs using drm_gem_prime_import() relies on
 * &drm_driver.gem_prime_import_sg_table.
 *
 * Note that similarly to the export helpers this permanently pins the
 * underlying backing storage. Which is ok for scanout, but is not the best
 * option for sharing lots of buffers for rendering.
 */

/**
 * drm_gem_map_attach - dma_buf attach implementation for GEM
 * @dma_buf: buffer to attach device to
 * @attach: buffer attachment data
 *
 * Calls &drm_gem_object_funcs.pin for device specific handling. This can be
 * used as the &dma_buf_ops.attach callback. Must be used together with
 * drm_gem_map_detach().
 *
 * Returns 0 on success, negative error code on failure.
 */
int drm_gem_map_attach(struct dma_buf *dma_buf,
		       struct dma_buf_attachment *attach)
{
	struct drm_gem_object *obj = dma_buf->priv;

	/*
	 * drm_gem_map_dma_buf() requires obj->get_sg_table(), but drivers
	 * that implement their own ->map_dma_buf() do not.
	 */
	if (dma_buf->ops->map_dma_buf == drm_gem_map_dma_buf &&
	    !obj->funcs->get_sg_table)
		return -ENOSYS;

	return drm_gem_pin(obj);
}
EXPORT_SYMBOL(drm_gem_map_attach);

/**
 * drm_gem_map_detach - dma_buf detach implementation for GEM
 * @dma_buf: buffer to detach from
 * @attach: attachment to be detached
 *
 * Calls &drm_gem_object_funcs.pin for device specific handling.  Cleans up
 * &dma_buf_attachment from drm_gem_map_attach(). This can be used as the
 * &dma_buf_ops.detach callback.
 */
void drm_gem_map_detach(struct dma_buf *dma_buf,
			struct dma_buf_attachment *attach)
{
	struct drm_gem_object *obj = dma_buf->priv;

	drm_gem_unpin(obj);
}
EXPORT_SYMBOL(drm_gem_map_detach);

/**
 * drm_gem_map_dma_buf - map_dma_buf implementation for GEM
 * @attach: attachment whose scatterlist is to be returned
 * @dir: direction of DMA transfer
 *
 * Calls &drm_gem_object_funcs.get_sg_table and then maps the scatterlist. This
 * can be used as the &dma_buf_ops.map_dma_buf callback. Should be used together
 * with drm_gem_unmap_dma_buf().
 *
 * Returns:sg_table containing the scatterlist to be returned; returns ERR_PTR
 * on error. May return -EINTR if it is interrupted by a signal.
 */
struct sg_table *drm_gem_map_dma_buf(struct dma_buf_attachment *attach,
				     enum dma_data_direction dir)
{
	struct drm_gem_object *obj = attach->dmabuf->priv;
	struct sg_table *sgt;
	int ret;

	if (WARN_ON(dir == DMA_NONE))
		return ERR_PTR(-EINVAL);

	if (WARN_ON(!obj->funcs->get_sg_table))
		return ERR_PTR(-ENOSYS);

	sgt = obj->funcs->get_sg_table(obj);
	if (IS_ERR(sgt))
		return sgt;

	ret = dma_map_sgtable(attach->dev, sgt, dir,
			      DMA_ATTR_SKIP_CPU_SYNC);
	if (ret) {
		sg_free_table(sgt);
		kfree(sgt);
		sgt = ERR_PTR(ret);
	}

	return sgt;
}
EXPORT_SYMBOL(drm_gem_map_dma_buf);

/**
 * drm_gem_unmap_dma_buf - unmap_dma_buf implementation for GEM
 * @attach: attachment to unmap buffer from
 * @sgt: scatterlist info of the buffer to unmap
 * @dir: direction of DMA transfer
 *
 * This can be used as the &dma_buf_ops.unmap_dma_buf callback.
 */
void drm_gem_unmap_dma_buf(struct dma_buf_attachment *attach,
			   struct sg_table *sgt,
			   enum dma_data_direction dir)
{
	if (!sgt)
		return;

	dma_unmap_sgtable(attach->dev, sgt, dir, DMA_ATTR_SKIP_CPU_SYNC);
	sg_free_table(sgt);
	kfree(sgt);
}
EXPORT_SYMBOL(drm_gem_unmap_dma_buf);

/**
 * drm_gem_dmabuf_vmap - dma_buf vmap implementation for GEM
 * @dma_buf: buffer to be mapped
 * @map: the virtual address of the buffer
 *
 * Sets up a kernel virtual mapping. This can be used as the &dma_buf_ops.vmap
 * callback. Calls into &drm_gem_object_funcs.vmap for device specific handling.
 * The kernel virtual address is returned in map.
 *
 * Returns 0 on success or a negative errno code otherwise.
 */
int drm_gem_dmabuf_vmap(struct dma_buf *dma_buf, struct iosys_map *map)
{
	struct drm_gem_object *obj = dma_buf->priv;

	return drm_gem_vmap(obj, map);
}
EXPORT_SYMBOL(drm_gem_dmabuf_vmap);

/**
 * drm_gem_dmabuf_vunmap - dma_buf vunmap implementation for GEM
 * @dma_buf: buffer to be unmapped
 * @map: the virtual address of the buffer
 *
 * Releases a kernel virtual mapping. This can be used as the
 * &dma_buf_ops.vunmap callback. Calls into &drm_gem_object_funcs.vunmap for device specific handling.
 */
void drm_gem_dmabuf_vunmap(struct dma_buf *dma_buf, struct iosys_map *map)
{
	struct drm_gem_object *obj = dma_buf->priv;

	drm_gem_vunmap(obj, map);
}
EXPORT_SYMBOL(drm_gem_dmabuf_vunmap);

/**
 * drm_gem_prime_mmap - PRIME mmap function for GEM drivers
 * @obj: GEM object
 * @vma: Virtual address range
 *
 * This function sets up a userspace mapping for PRIME exported buffers using
 * the same codepath that is used for regular GEM buffer mapping on the DRM fd.
 * The fake GEM offset is added to vma->vm_pgoff and &drm_driver->fops->mmap is
 * called to set up the mapping.
 */
#ifdef __NetBSD__
int drm_gem_prime_mmap(struct drm_gem_object *obj, off_t *offp, size_t size,
    int prot, int *flagsp, int *advicep, struct uvm_object **uobjp,
    int *maxprotp)
#else
int drm_gem_prime_mmap(struct drm_gem_object *obj, struct vm_area_struct *vma)
#endif
{
	struct drm_file *priv;
	struct file *fil;
	int ret;

	/* Add the fake offset */
#ifdef __NetBSD__
	*offp += drm_vma_node_start(&obj->vma_node);
#else
	vma->vm_pgoff += drm_vma_node_start(&obj->vma_node);
#endif

	if (obj->funcs && obj->funcs->mmap) {
<<<<<<< HEAD
#ifdef __NetBSD__
		ret = obj->funcs->mmap(obj, offp, size, prot, flagsp, advicep,
		    uobjp, maxprotp);
#else
		ret = obj->funcs->mmap(obj, vma);
#endif
		if (ret)
			return ret;
#ifndef __NetBSD__
		vma->vm_private_data = obj;
#endif
=======
		vma->vm_ops = obj->funcs->vm_ops;

>>>>>>> vendor/linux-drm-v6.6.35
		drm_gem_object_get(obj);
		ret = obj->funcs->mmap(obj, vma);
		if (ret) {
			drm_gem_object_put(obj);
			return ret;
		}
		vma->vm_private_data = obj;
		return 0;
	}

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	fil = kzalloc(sizeof(*fil), GFP_KERNEL);
	if (!priv || !fil) {
		ret = -ENOMEM;
		goto out;
	}

	/* Used by drm_gem_mmap() to lookup the GEM object */
	priv->minor = obj->dev->primary;
#ifdef __NetBSD__
	fil->f_data = priv;
#else
	fil->private_data = priv;
#endif

	ret = drm_vma_node_allow(&obj->vma_node, priv);
	if (ret)
		goto out;

#ifdef __NetBSD__
	KASSERT(size > 0);
	ret = obj->dev->driver->mmap_object(obj->dev, *offp, size, prot, uobjp,
	    offp, fil);
#else
	ret = obj->dev->driver->fops->mmap(fil, vma);
#endif

	drm_vma_node_revoke(&obj->vma_node, priv);
out:
	kfree(priv);
	kfree(fil);

	return ret;
}
EXPORT_SYMBOL(drm_gem_prime_mmap);

/**
 * drm_gem_dmabuf_mmap - dma_buf mmap implementation for GEM
 * @dma_buf: buffer to be mapped
 * @vma: virtual address range
 *
 * Provides memory mapping for the buffer. This can be used as the
 * &dma_buf_ops.mmap callback. It just forwards to drm_gem_prime_mmap().
 *
 * Returns 0 on success or a negative error code on failure.
 */
#ifdef __NetBSD__
int
drm_gem_dmabuf_mmap(struct dma_buf *dma_buf, off_t *offp, size_t size,
    int prot, int *flagsp, int *advicep, struct uvm_object **uobjp,
    int *maxprotp)
#else
int drm_gem_dmabuf_mmap(struct dma_buf *dma_buf, struct vm_area_struct *vma)
#endif
{
	struct drm_gem_object *obj = dma_buf->priv;

<<<<<<< HEAD
	if (!dev->driver->gem_prime_mmap)
		return -ENOSYS;

#ifdef __NetBSD__
	KASSERT(size > 0);
	return dev->driver->gem_prime_mmap(obj, offp, size, prot, flagsp,
	    advicep, uobjp, maxprotp);
#else
	return dev->driver->gem_prime_mmap(obj, vma);
#endif
=======
	return drm_gem_prime_mmap(obj, vma);
>>>>>>> vendor/linux-drm-v6.6.35
}
EXPORT_SYMBOL(drm_gem_dmabuf_mmap);

static const struct dma_buf_ops drm_gem_prime_dmabuf_ops =  {
	.cache_sgt_mapping = true,
	.attach = drm_gem_map_attach,
	.detach = drm_gem_map_detach,
	.map_dma_buf = drm_gem_map_dma_buf,
	.unmap_dma_buf = drm_gem_unmap_dma_buf,
	.release = drm_gem_dmabuf_release,
	.mmap = drm_gem_dmabuf_mmap,
	.vmap = drm_gem_dmabuf_vmap,
	.vunmap = drm_gem_dmabuf_vunmap,
};

/**
 * drm_prime_pages_to_sg - converts a page array into an sg list
 * @dev: DRM device
 * @pages: pointer to the array of page pointers to convert
 * @nr_pages: length of the page vector
 *
 * This helper creates an sg table object from a set of pages
 * the driver is responsible for mapping the pages into the
 * importers address space for use with dma_buf itself.
 *
 * This is useful for implementing &drm_gem_object_funcs.get_sg_table.
 */
struct sg_table *drm_prime_pages_to_sg(struct drm_device *dev,
				       struct page **pages, unsigned int nr_pages)
{
	struct sg_table *sg;
	size_t max_segment = 0;
	int err;

	sg = kmalloc(sizeof(struct sg_table), GFP_KERNEL);
	if (!sg)
		return ERR_PTR(-ENOMEM);

	if (dev)
		max_segment = dma_max_mapping_size(dev->dev);
	if (max_segment == 0)
		max_segment = UINT_MAX;
	err = sg_alloc_table_from_pages_segment(sg, pages, nr_pages, 0,
						(unsigned long)nr_pages << PAGE_SHIFT,
						max_segment, GFP_KERNEL);
	if (err) {
		kfree(sg);
		sg = ERR_PTR(err);
	}
	return sg;
}
EXPORT_SYMBOL(drm_prime_pages_to_sg);

/**
 * drm_prime_get_contiguous_size - returns the contiguous size of the buffer
 * @sgt: sg_table describing the buffer to check
 *
 * This helper calculates the contiguous size in the DMA address space
 * of the buffer described by the provided sg_table.
 *
 * This is useful for implementing
 * &drm_gem_object_funcs.gem_prime_import_sg_table.
 */
unsigned long drm_prime_get_contiguous_size(struct sg_table *sgt)
{
	dma_addr_t expected = sg_dma_address(sgt->sgl);
	struct scatterlist *sg;
	unsigned long size = 0;
	int i;

	for_each_sgtable_dma_sg(sgt, sg, i) {
		unsigned int len = sg_dma_len(sg);

		if (!len)
			break;
		if (sg_dma_address(sg) != expected)
			break;
		expected += len;
		size += len;
	}
	return size;
}
EXPORT_SYMBOL(drm_prime_get_contiguous_size);

/**
 * drm_gem_prime_export - helper library implementation of the export callback
 * @obj: GEM object to export
 * @flags: flags like DRM_CLOEXEC and DRM_RDWR
 *
 * This is the implementation of the &drm_gem_object_funcs.export functions for GEM drivers
 * using the PRIME helpers. It is used as the default in
 * drm_gem_prime_handle_to_fd().
 */
struct dma_buf *drm_gem_prime_export(struct drm_gem_object *obj,
				     int flags)
{
	struct drm_device *dev = obj->dev;
	struct dma_buf_export_info exp_info = {
#ifndef __NetBSD__
		.exp_name = KBUILD_MODNAME, /* white lie for debug */
		.owner = dev->driver->fops->owner,
#endif
		.ops = &drm_gem_prime_dmabuf_ops,
		.size = obj->size,
		.flags = flags,
		.priv = obj,
		.resv = obj->resv,
	};

	return drm_gem_dmabuf_export(dev, &exp_info);
}
EXPORT_SYMBOL(drm_gem_prime_export);

/**
 * drm_gem_prime_import_dev - core implementation of the import callback
 * @dev: drm_device to import into
 * @dma_buf: dma-buf object to import
 * @attach_dev: struct device to dma_buf attach
 *
 * This is the core of drm_gem_prime_import(). It's designed to be called by
 * drivers who want to use a different device structure than &drm_device.dev for
 * attaching via dma_buf. This function calls
 * &drm_driver.gem_prime_import_sg_table internally.
 *
 * Drivers must arrange to call drm_prime_gem_destroy() from their
 * &drm_gem_object_funcs.free hook when using this function.
 */
#ifdef __NetBSD__
struct drm_gem_object *drm_gem_prime_import_dev(struct drm_device *dev,
					    struct dma_buf *dma_buf,
					    bus_dma_tag_t attach_dev)
#else
struct drm_gem_object *drm_gem_prime_import_dev(struct drm_device *dev,
					    struct dma_buf *dma_buf,
					    struct device *attach_dev)
#endif
{
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	struct drm_gem_object *obj;
	int ret;

	if (dma_buf->ops == &drm_gem_prime_dmabuf_ops) {
		obj = dma_buf->priv;
		if (obj->dev == dev) {
			/*
			 * Importing dmabuf exported from our own gem increases
			 * refcount on gem itself instead of f_count of dmabuf.
			 */
			drm_gem_object_get(obj);
			return obj;
		}
	}

	if (!dev->driver->gem_prime_import_sg_table)
		return ERR_PTR(-EINVAL);

	attach = dma_buf_attach(dma_buf, attach_dev);
	if (IS_ERR(attach))
		return ERR_CAST(attach);

	get_dma_buf(dma_buf);

	sgt = dma_buf_map_attachment_unlocked(attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		goto fail_detach;
	}

	obj = dev->driver->gem_prime_import_sg_table(dev, attach, sgt);
	if (IS_ERR(obj)) {
		ret = PTR_ERR(obj);
		goto fail_unmap;
	}

	obj->import_attach = attach;
	obj->resv = dma_buf->resv;

	return obj;

fail_unmap:
	dma_buf_unmap_attachment_unlocked(attach, sgt, DMA_BIDIRECTIONAL);
fail_detach:
	dma_buf_detach(dma_buf, attach);
	dma_buf_put(dma_buf);

	return ERR_PTR(ret);
}
EXPORT_SYMBOL(drm_gem_prime_import_dev);

/**
 * drm_gem_prime_import - helper library implementation of the import callback
 * @dev: drm_device to import into
 * @dma_buf: dma-buf object to import
 *
 * This is the implementation of the gem_prime_import functions for GEM drivers
 * using the PRIME helpers. Drivers can use this as their
 * &drm_driver.gem_prime_import implementation. It is used as the default
 * implementation in drm_gem_prime_fd_to_handle().
 *
 * Drivers must arrange to call drm_prime_gem_destroy() from their
 * &drm_gem_object_funcs.free hook when using this function.
 */
struct drm_gem_object *drm_gem_prime_import(struct drm_device *dev,
					    struct dma_buf *dma_buf)
{
#ifdef __NetBSD__
	return drm_gem_prime_import_dev(dev, dma_buf, dev->dmat);
#else
	return drm_gem_prime_import_dev(dev, dma_buf, dev->dev);
#endif
}
EXPORT_SYMBOL(drm_gem_prime_import);

#ifdef __NetBSD__

struct sg_table *
drm_prime_bus_dmamem_to_sg(bus_dma_tag_t dmat, const bus_dma_segment_t *segs,
    int nsegs)
{
	struct sg_table *sg;
	int ret;

	sg = kmalloc(sizeof(*sg), GFP_KERNEL);
	if (sg == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	ret = sg_alloc_table_from_bus_dmamem(sg, dmat, segs, nsegs,
	    GFP_KERNEL);
	if (ret)
		goto out;

	return sg;
out:
	kfree(sg);
	return ERR_PTR(ret);
}

bus_size_t
drm_prime_sg_size(struct sg_table *sg)
{

	return sg->sgl->sg_npgs << PAGE_SHIFT;
}

void
drm_prime_sg_free(struct sg_table *sg)
{

	sg_free_table(sg);
	kfree(sg);
}

int
drm_prime_sg_to_bus_dmamem(bus_dma_tag_t dmat, bus_dma_segment_t *segs,
    int nsegs, int *rsegs, const struct sg_table *sgt)
{

	/* XXX errno NetBSD->Linux */
	return -bus_dmamem_import_pages(dmat, segs, nsegs, rsegs,
	    sgt->sgl->sg_pgs, sgt->sgl->sg_npgs);
}

int
drm_prime_bus_dmamap_load_sgt(bus_dma_tag_t dmat, bus_dmamap_t map,
    struct sg_table *sgt)
{
	bus_dma_segment_t *segs;
	bus_size_t size = drm_prime_sg_size(sgt);
	int nsegs = sgt->sgl->sg_npgs;
	int ret;

	segs = kcalloc(sgt->sgl->sg_npgs, sizeof(segs[0]), GFP_KERNEL);
	if (segs == NULL) {
		ret = -ENOMEM;
		goto out0;
	}

	ret = drm_prime_sg_to_bus_dmamem(dmat, segs, nsegs, &nsegs, sgt);
	if (ret)
		goto out1;
	KASSERT(nsegs <= sgt->sgl->sg_npgs);

	/* XXX errno NetBSD->Linux */
	ret = -bus_dmamap_load_raw(dmat, map, segs, nsegs, size,
	    BUS_DMA_NOWAIT);
	if (ret)
		goto out1;

out1:	kfree(segs);
out0:	return ret;
}

bool
drm_prime_sg_importable(bus_dma_tag_t dmat, struct sg_table *sgt)
{
	unsigned i;

	for (i = 0; i < sgt->sgl->sg_npgs; i++) {
		if (bus_dmatag_bounces_paddr(dmat,
			VM_PAGE_TO_PHYS(&sgt->sgl->sg_pgs[i]->p_vmp)))
			return false;
	}
	return true;
}

#else  /* !__NetBSD__ */

/**
 * drm_prime_sg_to_page_array - convert an sg table into a page array
 * @sgt: scatter-gather table to convert
 * @pages: array of page pointers to store the pages in
 * @max_entries: size of the passed-in array
 *
 * Exports an sg table into an array of pages.
 *
 * This function is deprecated and strongly discouraged to be used.
 * The page array is only useful for page faults and those can corrupt fields
 * in the struct page if they are not handled by the exporting driver.
 */
int __deprecated drm_prime_sg_to_page_array(struct sg_table *sgt,
					    struct page **pages,
					    int max_entries)
{
	struct sg_page_iter page_iter;
	struct page **p = pages;

	for_each_sgtable_page(sgt, &page_iter, 0) {
		if (WARN_ON(p - pages >= max_entries))
			return -1;
		*p++ = sg_page_iter_page(&page_iter);
	}
	return 0;
}
EXPORT_SYMBOL(drm_prime_sg_to_page_array);

/**
 * drm_prime_sg_to_dma_addr_array - convert an sg table into a dma addr array
 * @sgt: scatter-gather table to convert
 * @addrs: array to store the dma bus address of each page
 * @max_entries: size of both the passed-in arrays
 *
 * Exports an sg table into an array of addresses.
 *
 * Drivers should use this in their &drm_driver.gem_prime_import_sg_table
 * implementation.
 */
int drm_prime_sg_to_dma_addr_array(struct sg_table *sgt, dma_addr_t *addrs,
				   int max_entries)
{
	struct sg_dma_page_iter dma_iter;
	dma_addr_t *a = addrs;

	for_each_sgtable_dma_page(sgt, &dma_iter, 0) {
		if (WARN_ON(a - addrs >= max_entries))
			return -1;
		*a++ = sg_page_iter_dma_address(&dma_iter);
	}
	return 0;
}
EXPORT_SYMBOL(drm_prime_sg_to_dma_addr_array);

#endif	/* __NetBSD__ */

/**
 * drm_prime_gem_destroy - helper to clean up a PRIME-imported GEM object
 * @obj: GEM object which was created from a dma-buf
 * @sg: the sg-table which was pinned at import time
 *
 * This is the cleanup functions which GEM drivers need to call when they use
 * drm_gem_prime_import() or drm_gem_prime_import_dev() to import dma-bufs.
 */
void drm_prime_gem_destroy(struct drm_gem_object *obj, struct sg_table *sg)
{
	struct dma_buf_attachment *attach;
	struct dma_buf *dma_buf;

	attach = obj->import_attach;
	if (sg)
		dma_buf_unmap_attachment_unlocked(attach, sg, DMA_BIDIRECTIONAL);
	dma_buf = attach->dmabuf;
	dma_buf_detach(attach->dmabuf, attach);
	/* remove the reference */
	dma_buf_put(dma_buf);
}
EXPORT_SYMBOL(drm_prime_gem_destroy);
