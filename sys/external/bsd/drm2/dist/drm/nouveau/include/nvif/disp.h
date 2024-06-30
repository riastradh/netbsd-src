/*	$NetBSD: disp.h,v 1.2 2021/12/18 23:45:33 riastradh Exp $	*/

#ifndef __NVIF_DISP_H__
#define __NVIF_DISP_H__
#include <nvif/object.h>
struct nvif_device;

struct nvif_disp {
	struct nvif_object object;
	unsigned long conn_mask;
	unsigned long outp_mask;
	unsigned long head_mask;
};

int nvif_disp_ctor(struct nvif_device *, const char *name, s32 oclass,
		   struct nvif_disp *);
void nvif_disp_dtor(struct nvif_disp *);
#endif
