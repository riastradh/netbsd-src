/*	$NetBSD: nouveau_nvkm_engine_fifo_gk104.c,v 1.6 2021/12/19 10:51:57 riastradh Exp $	*/

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
__KERNEL_RCSID(0, "$NetBSD: nouveau_nvkm_engine_fifo_gk104.c,v 1.6 2021/12/19 10:51:57 riastradh Exp $");

#include "priv.h"
#include "cgrp.h"
#include "chan.h"
#include "chid.h"
#include "runl.h"
#include "runq.h"

#include <core/gpuobj.h>
#include <subdev/mc.h>
#include <subdev/mmu.h>
#include <subdev/top.h>

#include <nvif/class.h>
#include <nvif/if900d.h>

void
gk104_chan_stop(struct nvkm_chan *chan)
{
	struct nvkm_device *device = chan->cgrp->runl->fifo->engine.subdev.device;

	nvkm_mask(device, 0x800004 + (chan->id * 8), 0x00000800, 0x00000800);
}

void
gk104_chan_start(struct nvkm_chan *chan)
{
	struct nvkm_device *device = chan->cgrp->runl->fifo->engine.subdev.device;

	nvkm_mask(device, 0x800004 + (chan->id * 8), 0x00000400, 0x00000400);
}

void
gk104_chan_unbind(struct nvkm_chan *chan)
{
	struct nvkm_device *device = chan->cgrp->runl->fifo->engine.subdev.device;

	nvkm_wr32(device, 0x800000 + (chan->id * 8), 0x00000000);
}

void
gk104_chan_bind_inst(struct nvkm_chan *chan)
{
	struct nvkm_device *device = chan->cgrp->runl->fifo->engine.subdev.device;

	nvkm_wr32(device, 0x800000 + (chan->id * 8), 0x80000000 | chan->inst->addr >> 12);
}

void
gk104_chan_bind(struct nvkm_chan *chan)
{
	struct nvkm_runl *runl = chan->cgrp->runl;
	struct nvkm_device *device = runl->fifo->engine.subdev.device;

	nvkm_mask(device, 0x800004 + (chan->id * 8), 0x000f0000, runl->id << 16);
	gk104_chan_bind_inst(chan);
}

static int
gk104_chan_ramfc_write(struct nvkm_chan *chan, u64 offset, u64 length, u32 devm, bool priv)
{
	const u64 userd = nvkm_memory_addr(chan->userd.mem) + chan->userd.base;
	const u32 limit2 = ilog2(length / 8);

	nvkm_kmap(chan->inst);
	nvkm_wo32(chan->inst, 0x08, lower_32_bits(userd));
	nvkm_wo32(chan->inst, 0x0c, upper_32_bits(userd));
	nvkm_wo32(chan->inst, 0x10, 0x0000face);
	nvkm_wo32(chan->inst, 0x30, 0xfffff902);
	nvkm_wo32(chan->inst, 0x48, lower_32_bits(offset));
	nvkm_wo32(chan->inst, 0x4c, upper_32_bits(offset) | (limit2 << 16));
	nvkm_wo32(chan->inst, 0x84, 0x20400000);
	nvkm_wo32(chan->inst, 0x94, 0x30000000 | devm);
	nvkm_wo32(chan->inst, 0x9c, 0x00000100);
	nvkm_wo32(chan->inst, 0xac, 0x0000001f);
	nvkm_wo32(chan->inst, 0xe4, priv ? 0x00000020 : 0x00000000);
	nvkm_wo32(chan->inst, 0xe8, chan->id);
	nvkm_wo32(chan->inst, 0xb8, 0xf8000000);
	nvkm_wo32(chan->inst, 0xf8, 0x10003080); /* 0x002310 */
	nvkm_wo32(chan->inst, 0xfc, 0x10000010); /* 0x002350 */
	nvkm_done(chan->inst);
	return 0;
}

const struct nvkm_chan_func_ramfc
gk104_chan_ramfc = {
	.write = gk104_chan_ramfc_write,
	.devm = 0xfff,
	.priv = true,
};

const struct nvkm_chan_func_userd
gk104_chan_userd = {
	.bar = 1,
	.size = 0x200,
	.clear = gf100_chan_userd_clear,
};

static const struct nvkm_chan_func
gk104_chan = {
	.inst = &gf100_chan_inst,
	.userd = &gk104_chan_userd,
	.ramfc = &gk104_chan_ramfc,
	.bind = gk104_chan_bind,
	.unbind = gk104_chan_unbind,
	.start = gk104_chan_start,
	.stop = gk104_chan_stop,
	.preempt = gf100_chan_preempt,
};

static void
gk104_ectx_bind(struct nvkm_engn *engn, struct nvkm_cctx *cctx, struct nvkm_chan *chan)
{
	u32 ptr0, ptr1 = 0;
	u64 addr = 0ULL;

	switch (engn->engine->subdev.type) {
	case NVKM_ENGINE_SW    : return;
	case NVKM_ENGINE_GR    : ptr0 = 0x0210; break;
	case NVKM_ENGINE_SEC   : ptr0 = 0x0220; break;
	case NVKM_ENGINE_MSPDEC: ptr0 = 0x0250; break;
	case NVKM_ENGINE_MSPPP : ptr0 = 0x0260; break;
	case NVKM_ENGINE_MSVLD : ptr0 = 0x0270; break;
	case NVKM_ENGINE_VIC   : ptr0 = 0x0280; break;
	case NVKM_ENGINE_MSENC : ptr0 = 0x0290; break;
	case NVKM_ENGINE_NVDEC :
		ptr1 = 0x0270;
		ptr0 = 0x0210;
		break;
	case NVKM_ENGINE_NVENC :
		if (!engn->engine->subdev.inst)
			ptr1 = 0x0290;
		ptr0 = 0x0210;
		break;
	default:
		WARN_ON(1);
		return;
	}

	if (cctx) {
		addr  = cctx->vctx->vma->addr;
		addr |= 4ULL;
	}

	nvkm_kmap(chan->inst);
	nvkm_wo32(chan->inst, ptr0 + 0, lower_32_bits(addr));
	nvkm_wo32(chan->inst, ptr0 + 4, upper_32_bits(addr));
	if (ptr1) {
		nvkm_wo32(chan->inst, ptr1 + 0, lower_32_bits(addr));
		nvkm_wo32(chan->inst, ptr1 + 4, upper_32_bits(addr));
	}
	nvkm_done(chan->inst);
}

int
gk104_ectx_ctor(struct nvkm_engn *engn, struct nvkm_vctx *vctx)
{
	struct gf100_vmm_map_v0 args = { .priv = 1 };
	int ret;

	ret = nvkm_vmm_get(vctx->vmm, 12, vctx->inst->size, &vctx->vma);
	if (ret)
		return ret;

	return nvkm_memory_map(vctx->inst, 0, vctx->vmm, vctx->vma, &args, sizeof(args));
}

/*TODO: clean this up */
struct gk104_engn_status {
	bool busy;
	bool faulted;
	bool chsw;
	bool save;
	bool load;
	struct {
		bool tsg;
		u32 id;
	} prev, next, *chan;
};

static void
gk104_engn_status(struct nvkm_engn *engn, struct gk104_engn_status *status)
{
	u32 stat = nvkm_rd32(engn->runl->fifo->engine.subdev.device, 0x002640 + (engn->id * 0x08));

	status->busy     = !!(stat & 0x80000000);
	status->faulted  = !!(stat & 0x40000000);
	status->next.tsg = !!(stat & 0x10000000);
	status->next.id  =   (stat & 0x0fff0000) >> 16;
	status->chsw     = !!(stat & 0x00008000);
	status->save     = !!(stat & 0x00004000);
	status->load     = !!(stat & 0x00002000);
	status->prev.tsg = !!(stat & 0x00001000);
	status->prev.id  =   (stat & 0x00000fff);
	status->chan     = NULL;

	if (status->busy && status->chsw) {
		if (status->load && status->save) {
			if (nvkm_engine_chsw_load(engn->engine))
				status->chan = &status->next;
			else
				status->chan = &status->prev;
		} else
		if (status->load) {
			status->chan = &status->next;
		} else {
			status->chan = &status->prev;
		}
	} else
	if (status->load) {
		status->chan = &status->prev;
	}

	ENGN_DEBUG(engn, "%08x: busy %d faulted %d chsw %d save %d load %d %sid %d%s-> %sid %d%s",
		   stat, status->busy, status->faulted, status->chsw, status->save, status->load,
		   status->prev.tsg ? "tsg" : "ch", status->prev.id,
		   status->chan == &status->prev ? "*" : " ",
		   status->next.tsg ? "tsg" : "ch", status->next.id,
		   status->chan == &status->next ? "*" : " ");
}

int
gk104_engn_cxid(struct nvkm_engn *engn, bool *cgid)
{
	struct gk104_engn_status status;

<<<<<<< HEAD
const struct gk104_fifo_pbdma_func
gk104_fifo_pbdma = {
	.nr = gk104_fifo_pbdma_nr,
	.init = gk104_fifo_pbdma_init,
};

static void
gk104_fifo_recover_work(struct work_struct *w)
{
	struct gk104_fifo *fifo = container_of(w, typeof(*fifo), recover.work);
	struct nvkm_device *device = fifo->base.engine.subdev.device;
	struct nvkm_engine *engine;
	unsigned long flags;
	u32 engm, runm, todo;
	int engn, runl;

	spin_lock_irqsave(&fifo->base.lock, flags);
	runm = fifo->recover.runm;
	engm = fifo->recover.engm;
	fifo->recover.engm = 0;
	fifo->recover.runm = 0;
	spin_unlock_irqrestore(&fifo->base.lock, flags);

	nvkm_mask(device, 0x002630, runm, runm);

	for (todo = engm; todo && (engn = __ffs64(todo), 1); todo &= ~BIT(engn)) {
		if ((engine = fifo->engine[engn].engine)) {
			nvkm_subdev_fini(&engine->subdev, false);
			WARN_ON(nvkm_subdev_init(&engine->subdev));
		}
	}

	for (todo = runm; todo && (runl = __ffs(todo), 1); todo &= ~BIT(runl))
		gk104_fifo_runlist_update(fifo, runl);

	nvkm_wr32(device, 0x00262c, runm);
	nvkm_mask(device, 0x002630, runm, 0x00000000);
}

static void gk104_fifo_recover_engn(struct gk104_fifo *fifo, int engn);

static void
gk104_fifo_recover_runl(struct gk104_fifo *fifo, int runl)
{
	struct nvkm_subdev *subdev = &fifo->base.engine.subdev;
	struct nvkm_device *device = subdev->device;
	const u32 runm = BIT(runl);

	assert_spin_locked(&fifo->base.lock);
	if (fifo->recover.runm & runm)
		return;
	fifo->recover.runm |= runm;

	/* Block runlist to prevent channel assignment(s) from changing. */
	nvkm_mask(device, 0x002630, runm, runm);

	/* Schedule recovery. */
	nvkm_warn(subdev, "runlist %d: scheduled for recovery\n", runl);
	schedule_work(&fifo->recover.work);
}

static struct gk104_fifo_chan *
gk104_fifo_recover_chid(struct gk104_fifo *fifo, int runl, int chid)
{
	struct gk104_fifo_chan *chan;
	struct nvkm_fifo_cgrp *cgrp;

	list_for_each_entry(chan, &fifo->runlist[runl].chan, head) {
		if (chan->base.chid == chid) {
			list_del_init(&chan->head);
			return chan;
		}
	}

	list_for_each_entry(cgrp, &fifo->runlist[runl].cgrp, head) {
		if (cgrp->id == chid) {
			chan = list_first_entry(&cgrp->chan, typeof(*chan), head);
			list_del_init(&chan->head);
			if (!--cgrp->chan_nr)
				list_del_init(&cgrp->head);
			return chan;
		}
	}

	return NULL;
}

static void
gk104_fifo_recover_chan(struct nvkm_fifo *base, int chid)
{
	struct gk104_fifo *fifo = gk104_fifo(base);
	struct nvkm_subdev *subdev = &fifo->base.engine.subdev;
	struct nvkm_device *device = subdev->device;
	const u32  stat = nvkm_rd32(device, 0x800004 + (chid * 0x08));
	const u32  runl = (stat & 0x000f0000) >> 16;
	const bool used = (stat & 0x00000001);
	unsigned long engn, engm = fifo->runlist[runl].engm;
	struct gk104_fifo_chan *chan;

	assert_spin_locked(&fifo->base.lock);
	if (!used)
		return;

	/* Lookup SW state for channel, and mark it as dead. */
	chan = gk104_fifo_recover_chid(fifo, runl, chid);
	if (chan) {
		chan->killed = true;
		nvkm_fifo_kevent(&fifo->base, chid);
	}

	/* Disable channel. */
	nvkm_wr32(device, 0x800004 + (chid * 0x08), stat | 0x00000800);
	nvkm_warn(subdev, "channel %d: killed\n", chid);

	/* Block channel assignments from changing during recovery. */
	gk104_fifo_recover_runl(fifo, runl);

	/* Schedule recovery for any engines the channel is on. */
	for_each_set_bit(engn, &engm, fifo->engine_nr) {
		struct gk104_fifo_engine_status status;
		gk104_fifo_engine_status(fifo, engn, &status);
		if (!status.chan || status.chan->id != chid)
			continue;
		gk104_fifo_recover_engn(fifo, engn);
	}
}

static void
gk104_fifo_recover_engn(struct gk104_fifo *fifo, int engn)
{
	struct nvkm_engine *engine = fifo->engine[engn].engine;
	struct nvkm_subdev *subdev = &fifo->base.engine.subdev;
	struct nvkm_device *device = subdev->device;
	const u32 runl = fifo->engine[engn].runl;
	const u32 engm = BIT(engn);
	struct gk104_fifo_engine_status status;
	int mmui = -1;

	assert_spin_locked(&fifo->base.lock);
	if (fifo->recover.engm & engm)
		return;
	fifo->recover.engm |= engm;

	/* Block channel assignments from changing during recovery. */
	gk104_fifo_recover_runl(fifo, runl);

	/* Determine which channel (if any) is currently on the engine. */
	gk104_fifo_engine_status(fifo, engn, &status);
=======
	gk104_engn_status(engn, &status);
>>>>>>> vendor/linux-drm-v6.6.35
	if (status.chan) {
		*cgid = status.chan->tsg;
		return status.chan->id;
	}

	return -ENODEV;
}

bool
gk104_engn_chsw(struct nvkm_engn *engn)
{
	struct gk104_engn_status status;

	gk104_engn_status(engn, &status);
	if (status.busy && status.chsw)
		return true;

<<<<<<< HEAD
	if (ee && ee->data2) {
		switch (ee->data2) {
		case NVKM_SUBDEV_BAR:
			nvkm_bar_bar1_reset(device);
			break;
		case NVKM_SUBDEV_INSTMEM:
			nvkm_bar_bar2_reset(device);
			break;
		case NVKM_ENGINE_IFB:
			nvkm_mask(device, 0x001718, 0x00000000, 0x00000000);
			break;
		default:
			engine = nvkm_device_engine(device, ee->data2);
			break;
		}
	}

	if (ee == NULL) {
		enum nvkm_devidx engidx = nvkm_top_fault(device, info->engine);
		if (engidx < NVKM_SUBDEV_NR) {
			const char *src = nvkm_subdev_name[engidx];
			char *dst = en;
			do {
				*dst++ = toupper(*src++);
			} while(*src);
			engine = nvkm_device_engine(device, engidx);
		}
	} else {
		snprintf(en, sizeof(en), "%s", ee->name);
	}

	spin_lock_irqsave(&fifo->base.lock, flags);
	chan = nvkm_fifo_chan_inst_locked(&fifo->base, info->inst);

	nvkm_error(subdev,
		   "fault %02x [%s] at %016"PRIx64" engine %02x [%s] client %02x "
		   "[%s%s] reason %02x [%s] on channel %d [%010"PRIx64" %s]\n",
		   info->access, ea ? ea->name : "", info->addr,
		   info->engine, ee ? ee->name : en,
		   info->client, ct, ec ? ec->name : "",
		   info->reason, er ? er->name : "", chan ? chan->chid : -1,
		   info->inst, chan ? chan->object.client->name : "unknown");

	/* Kill the channel that caused the fault. */
	if (chan)
		gk104_fifo_recover_chan(&fifo->base, chan->chid);

	/* Channel recovery will probably have already done this for the
	 * correct engine(s), but just in case we can't find the channel
	 * information...
	 */
	for (engn = 0; engn < fifo->engine_nr && engine; engn++) {
		if (fifo->engine[engn].engine == engine) {
			gk104_fifo_recover_engn(fifo, engn);
			break;
		}
	}

	spin_unlock_irqrestore(&fifo->base.lock, flags);
=======
	return false;
>>>>>>> vendor/linux-drm-v6.6.35
}

const struct nvkm_engn_func
gk104_engn = {
	.chsw = gk104_engn_chsw,
	.cxid = gk104_engn_cxid,
	.mmu_fault_trigger = gf100_engn_mmu_fault_trigger,
	.mmu_fault_triggered = gf100_engn_mmu_fault_triggered,
	.ctor = gk104_ectx_ctor,
	.bind = gk104_ectx_bind,
};

const struct nvkm_engn_func
gk104_engn_ce = {
	.chsw = gk104_engn_chsw,
	.cxid = gk104_engn_cxid,
	.mmu_fault_trigger = gf100_engn_mmu_fault_trigger,
	.mmu_fault_triggered = gf100_engn_mmu_fault_triggered,
};

bool
gk104_runq_idle(struct nvkm_runq *runq)
{
	struct nvkm_device *device = runq->fifo->engine.subdev.device;

	return !(nvkm_rd32(device, 0x003080 + (runq->id * 4)) & 0x0000e000);
}

static const struct nvkm_bitfield
gk104_runq_intr_1_names[] = {
	{ 0x00000001, "HCE_RE_ILLEGAL_OP" },
	{ 0x00000002, "HCE_RE_ALIGNB" },
	{ 0x00000004, "HCE_PRIV" },
	{ 0x00000008, "HCE_ILLEGAL_MTHD" },
	{ 0x00000010, "HCE_ILLEGAL_CLASS" },
	{}
};

static bool
gk104_runq_intr_1(struct nvkm_runq *runq)
{
	struct nvkm_subdev *subdev = &runq->fifo->engine.subdev;
	struct nvkm_device *device = subdev->device;
	u32 mask = nvkm_rd32(device, 0x04014c + (runq->id * 0x2000));
	u32 stat = nvkm_rd32(device, 0x040148 + (runq->id * 0x2000)) & mask;
	u32 chid = nvkm_rd32(device, 0x040120 + (runq->id * 0x2000)) & 0xfff;
	char msg[128];

	if (stat & 0x80000000) {
		if (runq->func->intr_1_ctxnotvalid &&
		    runq->func->intr_1_ctxnotvalid(runq, chid))
			stat &= ~0x80000000;
	}

	if (stat) {
		nvkm_snprintbf(msg, sizeof(msg), gk104_runq_intr_1_names, stat);
		nvkm_error(subdev, "PBDMA%d: %08x [%s] ch %d %08x %08x\n",
			   runq->id, stat, msg, chid,
			   nvkm_rd32(device, 0x040150 + (runq->id * 0x2000)),
			   nvkm_rd32(device, 0x040154 + (runq->id * 0x2000)));
	}

	nvkm_wr32(device, 0x040148 + (runq->id * 0x2000), stat);
	return true;
}

const struct nvkm_bitfield
gk104_runq_intr_0_names[] = {
	{ 0x00000001, "MEMREQ" },
	{ 0x00000002, "MEMACK_TIMEOUT" },
	{ 0x00000004, "MEMACK_EXTRA" },
	{ 0x00000008, "MEMDAT_TIMEOUT" },
	{ 0x00000010, "MEMDAT_EXTRA" },
	{ 0x00000020, "MEMFLUSH" },
	{ 0x00000040, "MEMOP" },
	{ 0x00000080, "LBCONNECT" },
	{ 0x00000100, "LBREQ" },
	{ 0x00000200, "LBACK_TIMEOUT" },
	{ 0x00000400, "LBACK_EXTRA" },
	{ 0x00000800, "LBDAT_TIMEOUT" },
	{ 0x00001000, "LBDAT_EXTRA" },
	{ 0x00002000, "GPFIFO" },
	{ 0x00004000, "GPPTR" },
	{ 0x00008000, "GPENTRY" },
	{ 0x00010000, "GPCRC" },
	{ 0x00020000, "PBPTR" },
	{ 0x00040000, "PBENTRY" },
	{ 0x00080000, "PBCRC" },
	{ 0x00100000, "XBARCONNECT" },
	{ 0x00200000, "METHOD" },
	{ 0x00400000, "METHODCRC" },
	{ 0x00800000, "DEVICE" },
	{ 0x02000000, "SEMAPHORE" },
	{ 0x04000000, "ACQUIRE" },
	{ 0x08000000, "PRI" },
	{ 0x20000000, "NO_CTXSW_SEG" },
	{ 0x40000000, "PBSEG" },
	{ 0x80000000, "SIGNATURE" },
	{}
};

bool
gk104_runq_intr(struct nvkm_runq *runq, struct nvkm_runl *null)
{
	bool intr0 = gf100_runq_intr(runq, NULL);
	bool intr1 = gk104_runq_intr_1(runq);

<<<<<<< HEAD
	if (stat & 0x00800000) {
		if (device->sw) {
			if (nvkm_sw_mthd(device->sw, chid, subc, mthd, data))
				show &= ~0x00800000;
		}
	}

	nvkm_wr32(device, 0x0400c0 + (unit * 0x2000), 0x80600008);

	if (show) {
		nvkm_snprintbf(msg, sizeof(msg), gk104_fifo_pbdma_intr_0, show);
		chan = nvkm_fifo_chan_chid(&fifo->base, chid, &flags);
		nvkm_error(subdev, "PBDMA%d: %08x [%s] ch %d [%010"PRIx64" %s] "
				   "subc %d mthd %04x data %08x\n",
			   unit, show, msg, chid, chan ? chan->inst->addr : 0,
			   chan ? chan->object.client->name : "unknown",
			   subc, mthd, data);
		nvkm_fifo_chan_put(&fifo->base, flags, &chan);
	}

	nvkm_wr32(device, 0x040108 + (unit * 0x2000), stat);
=======
	return intr0 || intr1;
>>>>>>> vendor/linux-drm-v6.6.35
}

void
gk104_runq_init(struct nvkm_runq *runq)
{
	struct nvkm_device *device = runq->fifo->engine.subdev.device;

	gf100_runq_init(runq);

	nvkm_wr32(device, 0x040148 + (runq->id * 0x2000), 0xffffffff); /* HCE.INTR */
	nvkm_wr32(device, 0x04014c + (runq->id * 0x2000), 0xffffffff); /* HCE.INTREN */
}

static u32
gk104_runq_runm(struct nvkm_runq *runq)
{
	return nvkm_rd32(runq->fifo->engine.subdev.device, 0x002390 + (runq->id * 0x04));
}

const struct nvkm_runq_func
gk104_runq = {
	.init = gk104_runq_init,
	.intr = gk104_runq_intr,
	.intr_0_names = gk104_runq_intr_0_names,
	.idle = gk104_runq_idle,
};

void
gk104_runl_fault_clear(struct nvkm_runl *runl)
{
	nvkm_wr32(runl->fifo->engine.subdev.device, 0x00262c, BIT(runl->id));
}

void
gk104_runl_allow(struct nvkm_runl *runl, u32 engm)
{
<<<<<<< HEAD
	struct nvkm_device *device = fifo->base.engine.subdev.device;
	u32 mask = nvkm_rd32(device, 0x002a00);
	while (mask) {
		int runl = __ffs(mask);
#ifdef __NetBSD__
		spin_lock(&fifo->runlist[runl].lock);
		DRM_SPIN_WAKEUP_ONE(&fifo->runlist[runl].wait,
		    &fifo->runlist[runl].lock);
		spin_unlock(&fifo->runlist[runl].lock);
#else
		wake_up(&fifo->runlist[runl].wait);
#endif
		nvkm_wr32(device, 0x002a00, 1 << runl);
		mask &= ~(1 << runl);
	}
=======
	nvkm_mask(runl->fifo->engine.subdev.device, 0x002630, BIT(runl->id), 0x00000000);
>>>>>>> vendor/linux-drm-v6.6.35
}

void
gk104_runl_block(struct nvkm_runl *runl, u32 engm)
{
	nvkm_mask(runl->fifo->engine.subdev.device, 0x002630, BIT(runl->id), BIT(runl->id));
}

bool
gk104_runl_pending(struct nvkm_runl *runl)
{
	struct nvkm_device *device = runl->fifo->engine.subdev.device;

	return nvkm_rd32(device, 0x002284 + (runl->id * 0x08)) & 0x00100000;
}

void
gk104_runl_commit(struct nvkm_runl *runl, struct nvkm_memory *memory, u32 start, int count)
{
	struct nvkm_fifo *fifo = runl->fifo;
	struct nvkm_device *device = fifo->engine.subdev.device;
	u64 addr = nvkm_memory_addr(memory) + start;
	int target;

	switch (nvkm_memory_target(memory)) {
	case NVKM_MEM_TARGET_VRAM: target = 0; break;
	case NVKM_MEM_TARGET_NCOH: target = 3; break;
	default:
		WARN_ON(1);
		return;
	}

	spin_lock_irq(&fifo->lock);
	nvkm_wr32(device, 0x002270, (target << 28) | (addr >> 12));
	nvkm_wr32(device, 0x002274, (runl->id << 20) | count);
	spin_unlock_irq(&fifo->lock);
}

void
gk104_runl_insert_chan(struct nvkm_chan *chan, struct nvkm_memory *memory, u64 offset)
{
<<<<<<< HEAD
	struct gk104_fifo *fifo = gk104_fifo(base);
	struct nvkm_subdev *subdev = &fifo->base.engine.subdev;
	struct nvkm_device *device = subdev->device;
	struct nvkm_vmm *bar = nvkm_bar_bar1_vmm(device);
	int engn, runl, pbid, ret, i, j;
	enum nvkm_devidx engidx;
	u32 *map;

	fifo->pbdma_nr = fifo->func->pbdma->nr(fifo);
	nvkm_debug(subdev, "%d PBDMA(s)\n", fifo->pbdma_nr);

	/* Read PBDMA->runlist(s) mapping from HW. */
	if (!(map = kcalloc(fifo->pbdma_nr, sizeof(*map), GFP_KERNEL)))
		return -ENOMEM;

	for (i = 0; i < fifo->pbdma_nr; i++)
		map[i] = nvkm_rd32(device, 0x002390 + (i * 0x04));

	/* Determine runlist configuration from topology device info. */
	i = 0;
	while ((int)(engidx = nvkm_top_engine(device, i++, &runl, &engn)) >= 0) {
		/* Determine which PBDMA handles requests for this engine. */
		for (j = 0, pbid = -1; j < fifo->pbdma_nr; j++) {
			if (map[j] & (1 << runl)) {
				pbid = j;
				break;
			}
		}

		nvkm_debug(subdev, "engine %2d: runlist %2d pbdma %2d (%s)\n",
			   engn, runl, pbid, nvkm_subdev_name[engidx]);

		fifo->engine[engn].engine = nvkm_device_engine(device, engidx);
		fifo->engine[engn].runl = runl;
		fifo->engine[engn].pbid = pbid;
		fifo->engine_nr = max(fifo->engine_nr, engn + 1);
		fifo->runlist[runl].engm |= 1 << engn;
		fifo->runlist_nr = max(fifo->runlist_nr, runl + 1);
	}

	kfree(map);

	for (i = 0; i < fifo->runlist_nr; i++) {
		for (j = 0; j < ARRAY_SIZE(fifo->runlist[i].mem); j++) {
			ret = nvkm_memory_new(device, NVKM_MEM_TARGET_INST,
					      fifo->base.nr * 2/* TSG+chan */ *
					      fifo->func->runlist->size,
					      0x1000, false,
					      &fifo->runlist[i].mem[j]);
			if (ret)
				return ret;
		}

#ifdef __NetBSD__
		spin_lock_init(&fifo->runlist[i].lock);
		DRM_INIT_WAITQUEUE(&fifo->runlist[i].wait, "gk104fifo");
#else
		init_waitqueue_head(&fifo->runlist[i].wait);
#endif
		INIT_LIST_HEAD(&fifo->runlist[i].cgrp);
		INIT_LIST_HEAD(&fifo->runlist[i].chan);
	}

	ret = nvkm_memory_new(device, NVKM_MEM_TARGET_INST,
			      fifo->base.nr * 0x200, 0x1000, true,
			      &fifo->user.mem);
	if (ret)
		return ret;

	ret = nvkm_vmm_get(bar, 12, nvkm_memory_size(fifo->user.mem),
			   &fifo->user.bar);
	if (ret)
		return ret;

	return nvkm_memory_map(fifo->user.mem, 0, bar, fifo->user.bar, NULL, 0);
}

static void
gk104_fifo_init(struct nvkm_fifo *base)
{
	struct gk104_fifo *fifo = gk104_fifo(base);
	struct nvkm_device *device = fifo->base.engine.subdev.device;
	int i;

	/* Enable PBDMAs. */
	fifo->func->pbdma->init(fifo);

	/* PBDMA[n] */
	for (i = 0; i < fifo->pbdma_nr; i++) {
		nvkm_mask(device, 0x04013c + (i * 0x2000), 0x10000100, 0x00000000);
		nvkm_wr32(device, 0x040108 + (i * 0x2000), 0xffffffff); /* INTR */
		nvkm_wr32(device, 0x04010c + (i * 0x2000), 0xfffffeff); /* INTREN */
	}

	/* PBDMA[n].HCE */
	for (i = 0; i < fifo->pbdma_nr; i++) {
		nvkm_wr32(device, 0x040148 + (i * 0x2000), 0xffffffff); /* INTR */
		nvkm_wr32(device, 0x04014c + (i * 0x2000), 0xffffffff); /* INTREN */
	}

	nvkm_wr32(device, 0x002254, 0x10000000 | fifo->user.bar->addr >> 12);

	if (fifo->func->pbdma->init_timeout)
		fifo->func->pbdma->init_timeout(fifo);

	nvkm_wr32(device, 0x002100, 0xffffffff);
	nvkm_wr32(device, 0x002140, 0x7fffffff);
}

static void *
gk104_fifo_dtor(struct nvkm_fifo *base)
{
	struct gk104_fifo *fifo = gk104_fifo(base);
	struct nvkm_device *device = fifo->base.engine.subdev.device;
	int i;

	nvkm_vmm_put(nvkm_bar_bar1_vmm(device), &fifo->user.bar);
	nvkm_memory_unref(&fifo->user.mem);

	for (i = 0; i < fifo->runlist_nr; i++) {
#ifdef __NetBSD__
		DRM_DESTROY_WAITQUEUE(&fifo->runlist[i].wait);
		spin_lock_destroy(&fifo->runlist[i].lock);
#endif
		nvkm_memory_unref(&fifo->runlist[i].mem[1]);
		nvkm_memory_unref(&fifo->runlist[i].mem[0]);
	}

	return fifo;
}

static const struct nvkm_fifo_func
gk104_fifo_ = {
	.dtor = gk104_fifo_dtor,
	.oneinit = gk104_fifo_oneinit,
	.info = gk104_fifo_info,
	.init = gk104_fifo_init,
	.fini = gk104_fifo_fini,
	.intr = gk104_fifo_intr,
	.fault = gk104_fifo_fault,
	.uevent_init = gk104_fifo_uevent_init,
	.uevent_fini = gk104_fifo_uevent_fini,
	.recover_chan = gk104_fifo_recover_chan,
	.class_get = gk104_fifo_class_get,
	.class_new = gk104_fifo_class_new,
=======
	nvkm_wo32(memory, offset + 0, chan->id);
	nvkm_wo32(memory, offset + 4, 0x00000000);
}

static const struct nvkm_runl_func
gk104_runl = {
	.size = 8,
	.update = nv50_runl_update,
	.insert_chan = gk104_runl_insert_chan,
	.commit = gk104_runl_commit,
	.wait = nv50_runl_wait,
	.pending = gk104_runl_pending,
	.block = gk104_runl_block,
	.allow = gk104_runl_allow,
	.fault_clear = gk104_runl_fault_clear,
	.preempt_pending = gf100_runl_preempt_pending,
>>>>>>> vendor/linux-drm-v6.6.35
};

static const struct nvkm_enum
gk104_fifo_mmu_fault_engine[] = {
	{ 0x00, "GR", NULL, NVKM_ENGINE_GR },
	{ 0x01, "DISPLAY" },
	{ 0x02, "CAPTURE" },
	{ 0x03, "IFB", NULL, NVKM_ENGINE_IFB },
	{ 0x04, "BAR1", NULL, NVKM_SUBDEV_BAR },
	{ 0x05, "BAR2", NULL, NVKM_SUBDEV_INSTMEM },
	{ 0x06, "SCHED" },
	{ 0x07, "HOST0" },
	{ 0x08, "HOST1" },
	{ 0x09, "HOST2" },
	{ 0x0a, "HOST3" },
	{ 0x0b, "HOST4" },
	{ 0x0c, "HOST5" },
	{ 0x0d, "HOST6" },
	{ 0x0e, "HOST7" },
	{ 0x0f, "HOSTSR" },
	{ 0x10, "MSVLD", NULL, NVKM_ENGINE_MSVLD },
	{ 0x11, "MSPPP", NULL, NVKM_ENGINE_MSPPP },
	{ 0x13, "PERF" },
	{ 0x14, "MSPDEC", NULL, NVKM_ENGINE_MSPDEC },
	{ 0x15, "CE0", NULL, NVKM_ENGINE_CE, 0 },
	{ 0x16, "CE1", NULL, NVKM_ENGINE_CE, 1 },
	{ 0x17, "PMU" },
	{ 0x18, "PTP" },
	{ 0x19, "MSENC", NULL, NVKM_ENGINE_MSENC },
	{ 0x1b, "CE2", NULL, NVKM_ENGINE_CE, 2 },
	{}
};

const struct nvkm_enum
gk104_fifo_mmu_fault_reason[] = {
	{ 0x00, "PDE" },
	{ 0x01, "PDE_SIZE" },
	{ 0x02, "PTE" },
	{ 0x03, "VA_LIMIT_VIOLATION" },
	{ 0x04, "UNBOUND_INST_BLOCK" },
	{ 0x05, "PRIV_VIOLATION" },
	{ 0x06, "RO_VIOLATION" },
	{ 0x07, "WO_VIOLATION" },
	{ 0x08, "PITCH_MASK_VIOLATION" },
	{ 0x09, "WORK_CREATION" },
	{ 0x0a, "UNSUPPORTED_APERTURE" },
	{ 0x0b, "COMPRESSION_FAILURE" },
	{ 0x0c, "UNSUPPORTED_KIND" },
	{ 0x0d, "REGION_VIOLATION" },
	{ 0x0e, "BOTH_PTES_VALID" },
	{ 0x0f, "INFO_TYPE_POISONED" },
	{}
};

const struct nvkm_enum
gk104_fifo_mmu_fault_hubclient[] = {
	{ 0x00, "VIP" },
	{ 0x01, "CE0" },
	{ 0x02, "CE1" },
	{ 0x03, "DNISO" },
	{ 0x04, "FE" },
	{ 0x05, "FECS" },
	{ 0x06, "HOST" },
	{ 0x07, "HOST_CPU" },
	{ 0x08, "HOST_CPU_NB" },
	{ 0x09, "ISO" },
	{ 0x0a, "MMU" },
	{ 0x0b, "MSPDEC" },
	{ 0x0c, "MSPPP" },
	{ 0x0d, "MSVLD" },
	{ 0x0e, "NISO" },
	{ 0x0f, "P2P" },
	{ 0x10, "PD" },
	{ 0x11, "PERF" },
	{ 0x12, "PMU" },
	{ 0x13, "RASTERTWOD" },
	{ 0x14, "SCC" },
	{ 0x15, "SCC_NB" },
	{ 0x16, "SEC" },
	{ 0x17, "SSYNC" },
	{ 0x18, "GR_CE" },
	{ 0x19, "CE2" },
	{ 0x1a, "XV" },
	{ 0x1b, "MMU_NB" },
	{ 0x1c, "MSENC" },
	{ 0x1d, "DFALCON" },
	{ 0x1e, "SKED" },
	{ 0x1f, "AFALCON" },
	{}
};

const struct nvkm_enum
gk104_fifo_mmu_fault_gpcclient[] = {
	{ 0x00, "L1_0" }, { 0x01, "T1_0" }, { 0x02, "PE_0" },
	{ 0x03, "L1_1" }, { 0x04, "T1_1" }, { 0x05, "PE_1" },
	{ 0x06, "L1_2" }, { 0x07, "T1_2" }, { 0x08, "PE_2" },
	{ 0x09, "L1_3" }, { 0x0a, "T1_3" }, { 0x0b, "PE_3" },
	{ 0x0c, "RAST" },
	{ 0x0d, "GCC" },
	{ 0x0e, "GPCCS" },
	{ 0x0f, "PROP_0" },
	{ 0x10, "PROP_1" },
	{ 0x11, "PROP_2" },
	{ 0x12, "PROP_3" },
	{ 0x13, "L1_4" }, { 0x14, "T1_4" }, { 0x15, "PE_4" },
	{ 0x16, "L1_5" }, { 0x17, "T1_5" }, { 0x18, "PE_5" },
	{ 0x19, "L1_6" }, { 0x1a, "T1_6" }, { 0x1b, "PE_6" },
	{ 0x1c, "L1_7" }, { 0x1d, "T1_7" }, { 0x1e, "PE_7" },
	{ 0x1f, "GPM" },
	{ 0x20, "LTP_UTLB_0" },
	{ 0x21, "LTP_UTLB_1" },
	{ 0x22, "LTP_UTLB_2" },
	{ 0x23, "LTP_UTLB_3" },
	{ 0x24, "GPC_RGG_UTLB" },
	{}
};

const struct nvkm_fifo_func_mmu_fault
gk104_fifo_mmu_fault = {
	.recover = gf100_fifo_mmu_fault_recover,
	.access = gf100_fifo_mmu_fault_access,
	.engine = gk104_fifo_mmu_fault_engine,
	.reason = gk104_fifo_mmu_fault_reason,
	.hubclient = gk104_fifo_mmu_fault_hubclient,
	.gpcclient = gk104_fifo_mmu_fault_gpcclient,
};

static const struct nvkm_enum
gk104_fifo_intr_bind_reason[] = {
	{ 0x01, "BIND_NOT_UNBOUND" },
	{ 0x02, "SNOOP_WITHOUT_BAR1" },
	{ 0x03, "UNBIND_WHILE_RUNNING" },
	{ 0x05, "INVALID_RUNLIST" },
	{ 0x06, "INVALID_CTX_TGT" },
	{ 0x0b, "UNBIND_WHILE_PARKED" },
	{}
};

void
gk104_fifo_intr_bind(struct nvkm_fifo *fifo)
{
	struct nvkm_subdev *subdev = &fifo->engine.subdev;
	u32 intr = nvkm_rd32(subdev->device, 0x00252c);
	u32 code = intr & 0x000000ff;
	const struct nvkm_enum *en = nvkm_enum_find(gk104_fifo_intr_bind_reason, code);

	nvkm_error(subdev, "BIND_ERROR %02x [%s]\n", code, en ? en->name : "");
}

void
gk104_fifo_intr_chsw(struct nvkm_fifo *fifo)
{
	struct nvkm_subdev *subdev = &fifo->engine.subdev;
	struct nvkm_device *device = subdev->device;
	u32 stat = nvkm_rd32(device, 0x00256c);

	nvkm_error(subdev, "CHSW_ERROR %08x\n", stat);
	nvkm_wr32(device, 0x00256c, stat);
}

static void
gk104_fifo_intr_dropped_fault(struct nvkm_fifo *fifo)
{
	struct nvkm_subdev *subdev = &fifo->engine.subdev;
	u32 stat = nvkm_rd32(subdev->device, 0x00259c);

	nvkm_error(subdev, "DROPPED_MMU_FAULT %08x\n", stat);
}

void
gk104_fifo_intr_runlist(struct nvkm_fifo *fifo)
{
	struct nvkm_device *device = fifo->engine.subdev.device;
	struct nvkm_runl *runl;
	u32 mask = nvkm_rd32(device, 0x002a00);

	nvkm_runl_foreach_cond(runl, fifo, mask & BIT(runl->id)) {
		nvkm_wr32(device, 0x002a00, BIT(runl->id));
	}
}

irqreturn_t
gk104_fifo_intr(struct nvkm_inth *inth)
{
	struct nvkm_fifo *fifo = container_of(inth, typeof(*fifo), engine.subdev.inth);
	struct nvkm_subdev *subdev = &fifo->engine.subdev;
	struct nvkm_device *device = subdev->device;
	u32 mask = nvkm_rd32(device, 0x002140);
	u32 stat = nvkm_rd32(device, 0x002100) & mask;

	if (stat & 0x00000001) {
		gk104_fifo_intr_bind(fifo);
		nvkm_wr32(device, 0x002100, 0x00000001);
		stat &= ~0x00000001;
	}

	if (stat & 0x00000010) {
		nvkm_error(subdev, "PIO_ERROR\n");
		nvkm_wr32(device, 0x002100, 0x00000010);
		stat &= ~0x00000010;
	}

	if (stat & 0x00000100) {
		gf100_fifo_intr_sched(fifo);
		nvkm_wr32(device, 0x002100, 0x00000100);
		stat &= ~0x00000100;
	}

	if (stat & 0x00010000) {
		gk104_fifo_intr_chsw(fifo);
		nvkm_wr32(device, 0x002100, 0x00010000);
		stat &= ~0x00010000;
	}

	if (stat & 0x00800000) {
		nvkm_error(subdev, "FB_FLUSH_TIMEOUT\n");
		nvkm_wr32(device, 0x002100, 0x00800000);
		stat &= ~0x00800000;
	}

	if (stat & 0x01000000) {
		nvkm_error(subdev, "LB_ERROR\n");
		nvkm_wr32(device, 0x002100, 0x01000000);
		stat &= ~0x01000000;
	}

	if (stat & 0x08000000) {
		gk104_fifo_intr_dropped_fault(fifo);
		nvkm_wr32(device, 0x002100, 0x08000000);
		stat &= ~0x08000000;
	}

	if (stat & 0x10000000) {
		gf100_fifo_intr_mmu_fault(fifo);
		stat &= ~0x10000000;
	}

	if (stat & 0x20000000) {
		if (gf100_fifo_intr_pbdma(fifo))
			stat &= ~0x20000000;
	}

	if (stat & 0x40000000) {
		gk104_fifo_intr_runlist(fifo);
		stat &= ~0x40000000;
	}

	if (stat & 0x80000000) {
		nvkm_wr32(device, 0x002100, 0x80000000);
		nvkm_event_ntfy(&fifo->nonstall.event, 0, NVKM_FIFO_NONSTALL_EVENT);
		stat &= ~0x80000000;
	}

	if (stat) {
		nvkm_error(subdev, "INTR %08x\n", stat);
		spin_lock(&fifo->lock);
		nvkm_mask(device, 0x002140, stat, 0x00000000);
		spin_unlock(&fifo->lock);
		nvkm_wr32(device, 0x002100, stat);
	}

	return IRQ_HANDLED;
}

void
gk104_fifo_init_pbdmas(struct nvkm_fifo *fifo, u32 mask)
{
	struct nvkm_device *device = fifo->engine.subdev.device;

	nvkm_wr32(device, 0x000204, mask);
	nvkm_mask(device, 0x002a04, 0xbfffffff, 0xbfffffff);
}

void
gk104_fifo_init(struct nvkm_fifo *fifo)
{
	struct nvkm_device *device = fifo->engine.subdev.device;

	if (fifo->func->chan.func->userd->bar == 1)
		nvkm_wr32(device, 0x002254, 0x10000000 | fifo->userd.bar1->addr >> 12);

	nvkm_wr32(device, 0x002100, 0xffffffff);
	nvkm_wr32(device, 0x002140, 0x7fffffff);
}

int
gk104_fifo_runl_ctor(struct nvkm_fifo *fifo)
{
	struct nvkm_device *device = fifo->engine.subdev.device;
	struct nvkm_top_device *tdev;
	struct nvkm_runl *runl;
	struct nvkm_runq *runq;
	const struct nvkm_engn_func *func;

	nvkm_list_foreach(tdev, &device->top->device, head, tdev->runlist >= 0) {
		runl = nvkm_runl_get(fifo, tdev->runlist, tdev->runlist);
		if (!runl) {
			runl = nvkm_runl_new(fifo, tdev->runlist, tdev->runlist, 0);
			if (IS_ERR(runl))
				return PTR_ERR(runl);

			nvkm_runq_foreach_cond(runq, fifo, gk104_runq_runm(runq) & BIT(runl->id)) {
				if (WARN_ON(runl->runq_nr == ARRAY_SIZE(runl->runq)))
					return -ENOMEM;

				runl->runq[runl->runq_nr++] = runq;
			}

		}

		if (tdev->engine < 0)
			continue;

		switch (tdev->type) {
		case NVKM_ENGINE_CE:
			func = fifo->func->engn_ce;
			break;
		case NVKM_ENGINE_GR:
			nvkm_runl_add(runl, 15, &gf100_engn_sw, NVKM_ENGINE_SW, 0);
			fallthrough;
		default:
			func = fifo->func->engn;
			break;
		}

		nvkm_runl_add(runl, tdev->engine, func, tdev->type, tdev->inst);
	}

	return 0;
}

int
gk104_fifo_chid_nr(struct nvkm_fifo *fifo)
{
	return 4096;
}

static const struct nvkm_fifo_func
gk104_fifo = {
	.chid_nr = gk104_fifo_chid_nr,
	.chid_ctor = gf100_fifo_chid_ctor,
	.runq_nr = gf100_fifo_runq_nr,
	.runl_ctor = gk104_fifo_runl_ctor,
	.init = gk104_fifo_init,
	.init_pbdmas = gk104_fifo_init_pbdmas,
	.intr = gk104_fifo_intr,
	.intr_mmu_fault_unit = gf100_fifo_intr_mmu_fault_unit,
	.intr_ctxsw_timeout = gf100_fifo_intr_ctxsw_timeout,
	.mmu_fault = &gk104_fifo_mmu_fault,
	.nonstall = &gf100_fifo_nonstall,
	.runl = &gk104_runl,
	.runq = &gk104_runq,
	.engn = &gk104_engn,
	.engn_ce = &gk104_engn_ce,
	.cgrp = {{                               }, &nv04_cgrp },
	.chan = {{ 0, 0, KEPLER_CHANNEL_GPFIFO_A }, &gk104_chan },
};

int
gk104_fifo_new(struct nvkm_device *device, enum nvkm_subdev_type type, int inst,
	       struct nvkm_fifo **pfifo)
{
	return nvkm_fifo_new_(&gk104_fifo, device, type, inst, pfifo);
}
