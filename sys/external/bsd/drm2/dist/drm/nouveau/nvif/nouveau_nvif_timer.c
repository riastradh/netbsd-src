/*	$NetBSD: nouveau_nvkm_engine_fifo_usergv100.c,v 1.3 2021/12/19 10:51:57 riastradh Exp $	*/

/*
 * Copyright 2020 Red Hat Inc.
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
 */
#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: nouveau_nvkm_engine_fifo_usergv100.c,v 1.3 2021/12/19 10:51:57 riastradh Exp $");

#include <nvif/timer.h>
#include <nvif/device.h>

<<<<<<< HEAD:sys/external/bsd/drm2/dist/drm/nouveau/nvkm/engine/fifo/nouveau_nvkm_engine_fifo_usergv100.c
static int
#ifdef __NetBSD__
gv100_fifo_user_map(struct nvkm_object *object, void *argv, u32 argc,
		    enum nvkm_object_map *type,
		    bus_space_tag_t *tag, u64 *addr, u64 *size)
#else
gv100_fifo_user_map(struct nvkm_object *object, void *argv, u32 argc,
		    enum nvkm_object_map *type, u64 *addr, u64 *size)
#endif
{
	struct nvkm_device *device = object->engine->subdev.device;
#ifdef __NetBSD__
	*tag = device->func->resource_tag(device, 0);
#endif
	*addr = 0x810000 + device->func->resource_addr(device, 0);
	*size = 0x010000;
	*type = NVKM_OBJECT_MAP_IO;
	return 0;
=======
s64
nvif_timer_wait_test(struct nvif_timer_wait *wait)
{
	u64 time = nvif_device_time(wait->device);

	if (wait->reads == 0) {
		wait->time0 = time;
		wait->time1 = time;
	}

	if (wait->time1 == time) {
		if (WARN_ON(wait->reads++ == 16))
			return -ETIMEDOUT;
	} else {
		wait->time1 = time;
		wait->reads = 1;
	}

	if (wait->time1 - wait->time0 > wait->limit)
		return -ETIMEDOUT;

	return wait->time1 - wait->time0;
>>>>>>> vendor/linux-drm-v6.6.35:sys/external/bsd/drm2/dist/drm/nouveau/nvif/nouveau_nvif_timer.c
}

void
nvif_timer_wait_init(struct nvif_device *device, u64 nsec,
		     struct nvif_timer_wait *wait)
{
	wait->device = device;
	wait->limit = nsec;
	wait->reads = 0;
}
