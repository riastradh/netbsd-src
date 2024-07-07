/*	$NetBSD: nouveau_nvkm_engine_fifo_chan.c,v 1.10 2021/12/18 23:45:35 riastradh Exp $	*/

/*
 * Copyright 2012 Red Hat Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Authors: Ben Skeggs
 */
#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: nouveau_nvkm_engine_fifo_chan.c,v 1.10 2021/12/18 23:45:35 riastradh Exp $");

#include "chan.h"
#include "chid.h"
#include "cgrp.h"
#include "chid.h"
#include "runl.h"
#include "priv.h"

#include <core/ramht.h>
#include <subdev/mmu.h>
#include <engine/dma.h>

#include <nvif/if0020.h>

const struct nvkm_event_func
nvkm_chan_event = {
};

void
nvkm_chan_cctx_bind(struct nvkm_chan *chan, struct nvkm_engn *engn, struct nvkm_cctx *cctx)
{
	struct nvkm_cgrp *cgrp = chan->cgrp;
	struct nvkm_runl *runl = cgrp->runl;
	struct nvkm_engine *engine = engn->engine;

	if (!engn->func->bind)
		return;

	CHAN_TRACE(chan, "%sbind cctx %d[%s]", cctx ? "" : "un", engn->id, engine->subdev.name);

	/* Prevent any channel in channel group from being rescheduled, kick them
	 * off host and any engine(s) they're loaded on.
	 */
	if (cgrp->hw)
		nvkm_runl_block(runl);
	else
		nvkm_chan_block(chan);
	nvkm_chan_preempt(chan, true);

	/* Update context pointer. */
	engn->func->bind(engn, cctx, chan);

	/* Resume normal operation. */
	if (cgrp->hw)
		nvkm_runl_allow(runl);
	else
		nvkm_chan_allow(chan);
}

void
nvkm_chan_cctx_put(struct nvkm_chan *chan, struct nvkm_cctx **pcctx)
{
	struct nvkm_cctx *cctx = *pcctx;

	if (cctx) {
		struct nvkm_engn *engn = cctx->vctx->ectx->engn;

		if (refcount_dec_and_mutex_lock(&cctx->refs, &chan->cgrp->mutex)) {
			CHAN_TRACE(chan, "dtor cctx %d[%s]", engn->id, engn->engine->subdev.name);
			nvkm_cgrp_vctx_put(chan->cgrp, &cctx->vctx);
			list_del(&cctx->head);
			kfree(cctx);
			mutex_unlock(&chan->cgrp->mutex);
		}

		*pcctx = NULL;
	}
}

int
nvkm_chan_cctx_get(struct nvkm_chan *chan, struct nvkm_engn *engn, struct nvkm_cctx **pcctx,
		   struct nvkm_client *client)
{
	struct nvkm_cgrp *cgrp = chan->cgrp;
	struct nvkm_vctx *vctx;
	struct nvkm_cctx *cctx;
	int ret;

	/* Look for an existing channel context for this engine+VEID. */
	mutex_lock(&cgrp->mutex);
	cctx = nvkm_list_find(cctx, &chan->cctxs, head,
			      cctx->vctx->ectx->engn == engn && cctx->vctx->vmm == chan->vmm);
	if (cctx) {
		refcount_inc(&cctx->refs);
		*pcctx = cctx;
		mutex_unlock(&chan->cgrp->mutex);
		return 0;
	}

	/* Nope - create a fresh one.  But, sub-context first. */
	ret = nvkm_cgrp_vctx_get(cgrp, engn, chan, &vctx, client);
	if (ret) {
		CHAN_ERROR(chan, "vctx %d[%s]: %d", engn->id, engn->engine->subdev.name, ret);
		goto done;
	}

	/* Now, create the channel context - to track engine binding. */
	CHAN_TRACE(chan, "ctor cctx %d[%s]", engn->id, engn->engine->subdev.name);
	if (!(cctx = *pcctx = kzalloc(sizeof(*cctx), GFP_KERNEL))) {
		nvkm_cgrp_vctx_put(cgrp, &vctx);
		ret = -ENOMEM;
		goto done;
	}

	cctx->vctx = vctx;
	refcount_set(&cctx->refs, 1);
	refcount_set(&cctx->uses, 0);
	list_add_tail(&cctx->head, &chan->cctxs);
done:
	mutex_unlock(&cgrp->mutex);
	return ret;
}

int
nvkm_chan_preempt_locked(struct nvkm_chan *chan, bool wait)
{
	struct nvkm_runl *runl = chan->cgrp->runl;

	CHAN_TRACE(chan, "preempt");
	chan->func->preempt(chan);
	if (!wait)
		return 0;

	return nvkm_runl_preempt_wait(runl);
}

int
nvkm_chan_preempt(struct nvkm_chan *chan, bool wait)
{
	int ret;

	if (!chan->func->preempt)
		return 0;

	mutex_lock(&chan->cgrp->runl->mutex);
	ret = nvkm_chan_preempt_locked(chan, wait);
	mutex_unlock(&chan->cgrp->runl->mutex);
	return ret;
}

void
nvkm_chan_remove_locked(struct nvkm_chan *chan)
{
	struct nvkm_cgrp *cgrp = chan->cgrp;
	struct nvkm_runl *runl = cgrp->runl;

	if (list_empty(&chan->head))
		return;

	CHAN_TRACE(chan, "remove");
	if (!--cgrp->chan_nr) {
		runl->cgrp_nr--;
		list_del(&cgrp->head);
	}
	runl->chan_nr--;
	list_del_init(&chan->head);
	atomic_set(&runl->changed, 1);
}

void
nvkm_chan_remove(struct nvkm_chan *chan, bool preempt)
{
	struct nvkm_runl *runl = chan->cgrp->runl;

	mutex_lock(&runl->mutex);
	if (preempt && chan->func->preempt)
		nvkm_chan_preempt_locked(chan, true);
	nvkm_chan_remove_locked(chan);
	nvkm_runl_update_locked(runl, true);
	mutex_unlock(&runl->mutex);
}

void
nvkm_chan_insert(struct nvkm_chan *chan)
{
	struct nvkm_cgrp *cgrp = chan->cgrp;
	struct nvkm_runl *runl = cgrp->runl;

	mutex_lock(&runl->mutex);
	if (WARN_ON(!list_empty(&chan->head))) {
		mutex_unlock(&runl->mutex);
		return;
	}

	CHAN_TRACE(chan, "insert");
	list_add_tail(&chan->head, &cgrp->chans);
	runl->chan_nr++;
	if (!cgrp->chan_nr++) {
		list_add_tail(&cgrp->head, &cgrp->runl->cgrps);
		runl->cgrp_nr++;
	}
	atomic_set(&runl->changed, 1);
	nvkm_runl_update_locked(runl, true);
	mutex_unlock(&runl->mutex);
}

static void
nvkm_chan_block_locked(struct nvkm_chan *chan)
{
	CHAN_TRACE(chan, "block %d", atomic_read(&chan->blocked));
	if (atomic_inc_return(&chan->blocked) == 1)
		chan->func->stop(chan);
}

void
nvkm_chan_error(struct nvkm_chan *chan, bool preempt)
{
<<<<<<< HEAD
	struct nvkm_engine *engine = oclass->engine;
	struct nvkm_fifo_chan *chan = nvkm_fifo_chan(oclass->parent);
	struct nvkm_fifo_engn *engn = &chan->engn[engine->subdev.index];
	struct nvkm_fifo_chan_object *object;
	int ret = 0;

	if (!(object = kzalloc(sizeof(*object), GFP_KERNEL)))
		return -ENOMEM;
	nvkm_oproxy_ctor(&nvkm_fifo_chan_child_func, oclass, &object->oproxy);
	object->chan = chan;
	*pobject = &object->oproxy.base;

	if (!engn->refcount++) {
		struct nvkm_oclass cclass = {
			.client = oclass->client,
			.engine = oclass->engine,
		};

		if (chan->vmm)
			atomic_inc(&chan->vmm->engref[engine->subdev.index]);

		if (engine->func->fifo.cclass) {
			ret = engine->func->fifo.cclass(chan, &cclass,
							&engn->object);
		} else
		if (engine->func->cclass) {
			ret = nvkm_object_new_(engine->func->cclass, &cclass,
					       NULL, 0, &engn->object);
		}
		if (ret)
			return ret;

		if (chan->func->engine_ctor) {
			ret = chan->func->engine_ctor(chan, oclass->engine,
						      engn->object);
			if (ret)
				return ret;
		}
	}

	ret = oclass->base.ctor(&(const struct nvkm_oclass) {
					.base = oclass->base,
					.engn = oclass->engn,
					.handle = oclass->handle,
					.object = oclass->object,
					.client = oclass->client,
					.parent = engn->object ?
						  engn->object :
						  oclass->parent,
					.engine = engine,
				}, data, size, &object->oproxy.object);
	if (ret)
		return ret;

	if (chan->func->object_ctor) {
		object->hash =
			chan->func->object_ctor(chan, object->oproxy.object);
		if (object->hash < 0)
			return object->hash;
	}

	return 0;
}

static int
nvkm_fifo_chan_child_get(struct nvkm_object *object, int index,
			 struct nvkm_oclass *oclass)
{
	struct nvkm_fifo_chan *chan = nvkm_fifo_chan(object);
	struct nvkm_fifo *fifo = chan->fifo;
	struct nvkm_device *device = fifo->engine.subdev.device;
	struct nvkm_engine *engine;
	u64 mask = chan->engines;
	int ret, i, c;

	for (; c = 0, mask && (i = __ffs64(mask), 1); mask &= ~(1ULL << i)) {
		if (!(engine = nvkm_device_engine(device, i)))
			continue;
		oclass->engine = engine;
		oclass->base.oclass = 0;

		if (engine->func->fifo.sclass) {
			ret = engine->func->fifo.sclass(oclass, index);
			if (oclass->base.oclass) {
				if (!oclass->base.ctor)
					oclass->base.ctor = nvkm_object_new;
				oclass->ctor = nvkm_fifo_chan_child_new;
				return 0;
			}

			index -= ret;
			continue;
		}

		while (engine->func->sclass[c].oclass) {
			if (c++ == index) {
				oclass->base = engine->func->sclass[index];
				if (!oclass->base.ctor)
					oclass->base.ctor = nvkm_object_new;
				oclass->ctor = nvkm_fifo_chan_child_new;
				return 0;
			}
		}
		index -= c;
	}

	return -EINVAL;
}

static int
nvkm_fifo_chan_ntfy(struct nvkm_object *object, u32 type,
		    struct nvkm_event **pevent)
{
	struct nvkm_fifo_chan *chan = nvkm_fifo_chan(object);
	if (chan->func->ntfy)
		return chan->func->ntfy(chan, type, pevent);
	return -ENODEV;
}

static int
#ifdef __NetBSD__
nvkm_fifo_chan_map(struct nvkm_object *object, void *argv, u32 argc,
		   enum nvkm_object_map *type, bus_space_tag_t *tagp,
		   u64 *addr, u64 *size)
#else
nvkm_fifo_chan_map(struct nvkm_object *object, void *argv, u32 argc,
		   enum nvkm_object_map *type, u64 *addr, u64 *size)
#endif
{
	struct nvkm_fifo_chan *chan = nvkm_fifo_chan(object);
#ifdef __NetBSD__
	/* XXX Uh oh.  Can't map this more than once.  OK?  */
	*tagp = chan->bst;
#endif
	*type = NVKM_OBJECT_MAP_IO;
	*addr = chan->addr;
	*size = chan->size;
	return 0;
}

#ifdef __NetBSD__
static int
nvkm_fifo_chan_ensure_mapped(struct nvkm_fifo_chan *chan)
{
	int ret;

	if (likely(chan->mapped))
		goto out;

	/* XXX errno NetBSD->Linux */
	ret = -bus_space_map(chan->bst, chan->addr, chan->size, 0,
	    &chan->bsh);
	if (ret)
		return ret;
	chan->mapped = true;

out:	KASSERT(chan->mapped);
	return 0;
}
#endif

static int
nvkm_fifo_chan_rd32(struct nvkm_object *object, u64 addr, u32 *data)
{
	struct nvkm_fifo_chan *chan = nvkm_fifo_chan(object);
#ifdef __NetBSD__
	int ret = nvkm_fifo_chan_ensure_mapped(chan);
	if (ret)
		return ret;
#else
	if (unlikely(!chan->user)) {
		chan->user = ioremap(chan->addr, chan->size);
		if (!chan->user)
			return -ENOMEM;
	}
#endif
	if (unlikely(addr + 4 > chan->size))
		return -EINVAL;
#ifdef __NetBSD__
	*data = bus_space_read_stream_4(chan->bst, chan->bsh, addr);
#else
	*data = ioread32_native(chan->user + addr);
#endif
	return 0;
}

static int
nvkm_fifo_chan_wr32(struct nvkm_object *object, u64 addr, u32 data)
{
	struct nvkm_fifo_chan *chan = nvkm_fifo_chan(object);
#ifdef __NetBSD__
	int ret = nvkm_fifo_chan_ensure_mapped(chan);
	if (ret)
		return ret;
#else
	if (unlikely(!chan->user)) {
		chan->user = ioremap(chan->addr, chan->size);
		if (!chan->user)
			return -ENOMEM;
	}
#endif
	if (unlikely(addr + 4 > chan->size))
		return -EINVAL;
#ifdef __NetBSD__
	bus_space_write_stream_4(chan->bst, chan->bsh, addr, data);
#else
	iowrite32_native(data, chan->user + addr);
#endif
	return 0;
}

static int
nvkm_fifo_chan_fini(struct nvkm_object *object, bool suspend)
{
	struct nvkm_fifo_chan *chan = nvkm_fifo_chan(object);
	chan->func->fini(chan);
	return 0;
}

static int
nvkm_fifo_chan_init(struct nvkm_object *object)
{
	struct nvkm_fifo_chan *chan = nvkm_fifo_chan(object);
	chan->func->init(chan);
	return 0;
}

static void *
nvkm_fifo_chan_dtor(struct nvkm_object *object)
{
	struct nvkm_fifo_chan *chan = nvkm_fifo_chan(object);
	struct nvkm_fifo *fifo = chan->fifo;
	void *data = chan->func->dtor(chan);
=======
>>>>>>> vendor/linux-drm-v6.6.35
	unsigned long flags;

	spin_lock_irqsave(&chan->lock, flags);
	if (atomic_inc_return(&chan->errored) == 1) {
		CHAN_ERROR(chan, "errored - disabling channel");
		nvkm_chan_block_locked(chan);
		if (preempt)
			chan->func->preempt(chan);
		nvkm_event_ntfy(&chan->cgrp->runl->chid->event, chan->id, NVKM_CHAN_EVENT_ERRORED);
	}
	spin_unlock_irqrestore(&chan->lock, flags);
}

<<<<<<< HEAD
#ifdef __NetBSD__
	if (!chan->subregion && chan->mapped) {
		bus_space_unmap(chan->bst, chan->bsh, chan->size);
		chan->mapped = false;
	}
#else
	if (chan->user)
		iounmap(chan->user);
#endif
=======
void
nvkm_chan_block(struct nvkm_chan *chan)
{
	spin_lock_irq(&chan->lock);
	nvkm_chan_block_locked(chan);
	spin_unlock_irq(&chan->lock);
}

void
nvkm_chan_allow(struct nvkm_chan *chan)
{
	spin_lock_irq(&chan->lock);
	CHAN_TRACE(chan, "allow %d", atomic_read(&chan->blocked));
	if (atomic_dec_and_test(&chan->blocked))
		chan->func->start(chan);
	spin_unlock_irq(&chan->lock);
}

void
nvkm_chan_del(struct nvkm_chan **pchan)
{
	struct nvkm_chan *chan = *pchan;

	if (!chan)
		return;

	if (chan->func->ramfc->clear)
		chan->func->ramfc->clear(chan);

	nvkm_ramht_del(&chan->ramht);
	nvkm_gpuobj_del(&chan->pgd);
	nvkm_gpuobj_del(&chan->eng);
	nvkm_gpuobj_del(&chan->cache);
	nvkm_gpuobj_del(&chan->ramfc);

	nvkm_memory_unref(&chan->userd.mem);

	if (chan->cgrp) {
		nvkm_chid_put(chan->cgrp->runl->chid, chan->id, &chan->cgrp->lock);
		nvkm_cgrp_unref(&chan->cgrp);
	}
>>>>>>> vendor/linux-drm-v6.6.35

	if (chan->vmm) {
		nvkm_vmm_part(chan->vmm, chan->inst->memory);
		nvkm_vmm_unref(&chan->vmm);
	}

	nvkm_gpuobj_del(&chan->push);
	nvkm_gpuobj_del(&chan->inst);
	kfree(chan);
}

void
nvkm_chan_put(struct nvkm_chan **pchan, unsigned long irqflags)
{
	struct nvkm_chan *chan = *pchan;

	if (!chan)
		return;

	*pchan = NULL;
	spin_unlock_irqrestore(&chan->cgrp->lock, irqflags);
}

struct nvkm_chan *
nvkm_chan_get_inst(struct nvkm_engine *engine, u64 inst, unsigned long *pirqflags)
{
	struct nvkm_fifo *fifo = engine->subdev.device->fifo;
	struct nvkm_runl *runl;
	struct nvkm_engn *engn;
	struct nvkm_chan *chan;

	nvkm_runl_foreach(runl, fifo) {
		nvkm_runl_foreach_engn(engn, runl) {
			if (engine == &fifo->engine || engn->engine == engine) {
				chan = nvkm_runl_chan_get_inst(runl, inst, pirqflags);
				if (chan || engn->engine == engine)
					return chan;
			}
		}
	}

	return NULL;
}

struct nvkm_chan *
nvkm_chan_get_chid(struct nvkm_engine *engine, int id, unsigned long *pirqflags)
{
	struct nvkm_fifo *fifo = engine->subdev.device->fifo;
	struct nvkm_runl *runl;
	struct nvkm_engn *engn;

	nvkm_runl_foreach(runl, fifo) {
		nvkm_runl_foreach_engn(engn, runl) {
			if (fifo->chid || engn->engine == engine)
				return nvkm_runl_chan_get_chid(runl, id, pirqflags);
		}
	}

	return NULL;
}

int
nvkm_chan_new_(const struct nvkm_chan_func *func, struct nvkm_runl *runl, int runq,
	       struct nvkm_cgrp *cgrp, const char *name, bool priv, u32 devm, struct nvkm_vmm *vmm,
	       struct nvkm_dmaobj *dmaobj, u64 offset, u64 length,
	       struct nvkm_memory *userd, u64 ouserd, struct nvkm_chan **pchan)
{
	struct nvkm_fifo *fifo = runl->fifo;
	struct nvkm_device *device = fifo->engine.subdev.device;
	struct nvkm_chan *chan;
	int ret;

	/* Validate arguments against class requirements. */
	if ((runq && runq >= runl->func->runqs) ||
	    (!func->inst->vmm != !vmm) ||
	    ((func->userd->bar < 0) == !userd) ||
	    (!func->ramfc->ctxdma != !dmaobj) ||
	    ((func->ramfc->devm < devm) && devm != BIT(0)) ||
	    (!func->ramfc->priv && priv)) {
		RUNL_DEBUG(runl, "args runq:%d:%d vmm:%d:%p userd:%d:%p "
				 "push:%d:%p devm:%08x:%08x priv:%d:%d",
			   runl->func->runqs, runq, func->inst->vmm, vmm,
			   func->userd->bar < 0, userd, func->ramfc->ctxdma, dmaobj,
			   func->ramfc->devm, devm, func->ramfc->priv, priv);
		return -EINVAL;
	}

	if (!(chan = *pchan = kzalloc(sizeof(*chan), GFP_KERNEL)))
		return -ENOMEM;

	chan->func = func;
	strscpy(chan->name, name, sizeof(chan->name));
	chan->runq = runq;
	chan->id = -1;
	spin_lock_init(&chan->lock);
	atomic_set(&chan->blocked, 1);
	atomic_set(&chan->errored, 0);
	INIT_LIST_HEAD(&chan->cctxs);
	INIT_LIST_HEAD(&chan->head);

	/* Join channel group.
	 *
	 * GK110 and newer support channel groups (aka TSGs), where individual channels
	 * share a timeslice, and, engine context(s).
	 *
	 * As such, engine contexts are tracked in nvkm_cgrp and we need them even when
	 * channels aren't in an API channel group, and on HW that doesn't support TSGs.
	 */
	if (!cgrp) {
		ret = nvkm_cgrp_new(runl, chan->name, vmm, fifo->func->cgrp.force, &chan->cgrp);
		if (ret) {
			RUNL_DEBUG(runl, "cgrp %d", ret);
			return ret;
		}

		cgrp = chan->cgrp;
	} else {
		if (cgrp->runl != runl || cgrp->vmm != vmm) {
			RUNL_DEBUG(runl, "cgrp %d %d", cgrp->runl != runl, cgrp->vmm != vmm);
			return -EINVAL;
		}

		chan->cgrp = nvkm_cgrp_ref(cgrp);
	}

	/* Allocate instance block. */
	ret = nvkm_gpuobj_new(device, func->inst->size, 0x1000, func->inst->zero, NULL,
			      &chan->inst);
	if (ret) {
		RUNL_DEBUG(runl, "inst %d", ret);
		return ret;
	}

	/* Initialise virtual address-space. */
	if (func->inst->vmm) {
		if (WARN_ON(vmm->mmu != device->mmu))
			return -EINVAL;

		ret = nvkm_vmm_join(vmm, chan->inst->memory);
		if (ret) {
			RUNL_DEBUG(runl, "vmm %d", ret);
			return ret;
		}

		chan->vmm = nvkm_vmm_ref(vmm);
	}

	/* Allocate HW ctxdma for push buffer. */
	if (func->ramfc->ctxdma) {
		ret = nvkm_object_bind(&dmaobj->object, chan->inst, -16, &chan->push);
		if (ret) {
			RUNL_DEBUG(runl, "bind %d", ret);
			return ret;
		}
	}

	/* Allocate channel ID. */
	chan->id = nvkm_chid_get(runl->chid, chan);
	if (chan->id < 0) {
		RUNL_ERROR(runl, "!chids");
		return -ENOSPC;
	}

<<<<<<< HEAD
	/* determine address of this channel's user registers */
	chan->addr = device->func->resource_addr(device, bar) +
		     base + user * chan->chid;
	chan->size = user;
#ifdef __NetBSD__
	if (bar == 0) {
		/*
		 * We already map BAR 0 in the engine device base, so
		 * grab a subregion of that.
		 */
		bus_space_tag_t mmiot = device->mmiot;
		bus_space_handle_t mmioh = device->mmioh;
		bus_size_t mmiosz = device->mmiosz;
		__diagused bus_addr_t mmioaddr =
		    device->func->resource_addr(device, bar);

		/* Check whether it lies inside the region.  */
		if (mmiosz < base ||
		    mmiosz - base < user * chan->chid ||
		    mmiosz - base - user * chan->chid < user) {
			nvif_error(&chan->object, "fifo channel out of range:"
			    " base 0x%jx chid 0x%jx user 0x%jx mmiosz 0x%jx\n",
			    (uintmax_t)base,
			    (uintmax_t)chan->chid, (uintmax_t)user,
			    (uintmax_t)mmiosz);
			return -EIO;
		}
		KASSERT(mmioaddr <= chan->addr);
		KASSERT(base + user * chan->chid <= mmiosz - user);
		KASSERT(chan->addr <= mmioaddr + (mmiosz - user));
		KASSERT(chan->addr - mmioaddr == base + user * chan->chid);
		/* XXX errno NetBSD->Linux */
		ret = -bus_space_subregion(mmiot, mmioh,
		    base + user * chan->chid, user, &chan->bsh);
		if (ret) {
			nvif_error(&chan->object, "bus_space_subregion failed:"
			    " %d\n", ret);
			return ret;
		}
		chan->bst = mmiot;
		chan->mapped = true;
		chan->subregion = true;
	} else {
		/* XXX Why does nouveau map this lazily?  */
		chan->bst = device->func->resource_tag(device, bar);
		chan->mapped = false;
		chan->subregion = false;
	}
#endif
=======
	if (cgrp->id < 0)
		cgrp->id = chan->id;

	/* Initialise USERD. */
	if (func->userd->bar < 0) {
		if (ouserd + chan->func->userd->size >= nvkm_memory_size(userd)) {
			RUNL_DEBUG(runl, "ouserd %llx", ouserd);
			return -EINVAL;
		}

		ret = nvkm_memory_kmap(userd, &chan->userd.mem);
		if (ret) {
			RUNL_DEBUG(runl, "userd %d", ret);
			return ret;
		}

		chan->userd.base = ouserd;
	} else {
		chan->userd.mem = nvkm_memory_ref(fifo->userd.mem);
		chan->userd.base = chan->id * chan->func->userd->size;
	}

	if (chan->func->userd->clear)
		chan->func->userd->clear(chan);

	/* Initialise RAMFC. */
	ret = chan->func->ramfc->write(chan, offset, length, devm, priv);
	if (ret) {
		RUNL_DEBUG(runl, "ramfc %d", ret);
		return ret;
	}
>>>>>>> vendor/linux-drm-v6.6.35

	return 0;
}
