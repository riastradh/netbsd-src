/*	$NetBSD: linux_rcu.c,v 1.5 2021/07/21 06:34:52 skrll Exp $	*/

/*-
 * Copyright (c) 2018 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Taylor R. Campbell.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: linux_rcu.c,v 1.5 2021/07/21 06:34:52 skrll Exp $");

#include <sys/param.h>
#include <sys/types.h>

#include <sys/condvar.h>
#include <sys/cpu.h>
#include <sys/kthread.h>
#include <sys/lockdebug.h>
#include <sys/mutex.h>
#include <sys/percpu.h>
#include <sys/sdt.h>
#include <sys/workqueue.h>
#include <sys/xcall.h>

#include <linux/rcupdate.h>
#include <linux/slab.h>

SDT_PROBE_DEFINE0(sdt, linux, rcu, synchronize__start);
SDT_PROBE_DEFINE1(sdt, linux, rcu, synchronize__cpu, "unsigned"/*cpu*/);
SDT_PROBE_DEFINE0(sdt, linux, rcu, synchronize__done);
SDT_PROBE_DEFINE0(sdt, linux, rcu, barrier__start);
SDT_PROBE_DEFINE0(sdt, linux, rcu, barrier__done);
SDT_PROBE_DEFINE0(sdt, linux, rcu, reclaim__start);
SDT_PROBE_DEFINE0(sdt, linux, rcu, reclaim__done);
SDT_PROBE_DEFINE2(sdt, linux, rcu, call__queue,
    "struct rcu_head *"/*head*/, "void (*)(struct rcu_head *)"/*callback*/);
SDT_PROBE_DEFINE2(sdt, linux, rcu, call__run,
    "struct rcu_head *"/*head*/, "void (*)(struct rcu_head *)"/*callback*/);
SDT_PROBE_DEFINE2(sdt, linux, rcu, call__done,
    "struct rcu_head *"/*head*/, "void (*)(struct rcu_head *)"/*callback*/);
SDT_PROBE_DEFINE2(sdt, linux, rcu, kfree__queue,
    "struct rcu_head *"/*head*/, "void *"/*obj*/);
SDT_PROBE_DEFINE2(sdt, linux, rcu, kfree__free,
    "struct rcu_head *"/*head*/, "void *"/*obj*/);
SDT_PROBE_DEFINE2(sdt, linux, rcu, kfree__done,
    "struct rcu_head *"/*head*/, "void *"/*obj*/);

struct rcu_queue {
	struct rcu_head	*callback;
	struct rcu_head	*kfree;
};

struct rcu_cpu {
	/*
	 * Q[0] -- where new objects get queued until first xcall
	 * Q[1] -- objects in limbo awaiting a complete xcall
	 * Q[2] -- objects ready for reclamation
	 */
	struct rcu_queue	Q[3];
	uint32_t		gen;
	struct work		work;
};

static struct {
	kmutex_t		lock;
	kcondvar_t		cv;
	struct lwp		*lwp;
	struct workqueue	*wq;
	struct percpu		*percpu;	/* struct rcu_cpu */
	volatile uint32_t	nextgen;
	uint32_t		gen;
	uint32_t		epoch;
	volatile bool		active;
	bool			dying;
} gc __cacheline_aligned;

static void
invariants(void)
{

	KASSERT(mutex_owned(&gc.lock));
	KASSERTMSG(gc.nextgen == gc.gen || gc.nextgen == gc.gen + 1,
	    "gen=%"PRIu32" nextgen=%"PRIu32, gc.gen, gc.nextgen);
	KASSERTMSG(gc.active ? gc.nextgen == gc.gen + 1 : 1,
	    "active=%d gen=%"PRIu32" nextgen=%"PRIu32,
	    (int)gc.active, gc.gen, gc.nextgen);
}

static struct rcu_head *
append_reverse(struct rcu_head *head, struct rcu_head *tail)
{
	struct rcu_head *next;

	for (; head != NULL; head = next) {
		next = head->rcuh_next;
		head->rcuh_next = tail;
		tail = head;
	}

	return tail;
}

static void
rcu_xc(void *a, void *b)
{
	struct rcu_cpu **Rp, *R;
	int s;
	bool already = false;

	SDT_PROBE1(sdt, linux, rcu, synchronize__cpu,  cpu_index(curcpu()));

	/*
	 * Get the per-CPU state and block out the workqueue thread by
	 * disabling kpreemption.
	 */
	Rp = percpu_getref(gc.percpu);
	R = *Rp;

	KASSERT(kpreempt_disabled());

	/*
	 * Determine whether the worker thread has already been
	 * scheduled but hasn't yet run; then append everything in
	 * limbo to the ready queue and, if necessary, schedule the
	 * worker to process it.
	 */
	already = R->Q[2].callback || R->Q[2].kfree;
	R->Q[2].callback = append_reverse(R->Q[1].callback, R->Q[2].callback);
	R->Q[2].kfree = append_reverse(R->Q[1].kfree, R->Q[2].kfree);
	if (!already && (R->Q[2].callback || R->Q[2].kfree))
		workqueue_enqueue(gc.wq, &R->work, curcpu());

	/*
	 * Move everything from the pending queue to the limbo queue,
	 * and update our local view of the global generation.  Do this
	 * at IPL_VM so that interrupt handlers running at IPL_VM can
	 * safely add things to the pending queue.
	 */
	s = splvm();
	R->Q[1] = R->Q[0];
	memset(&R->Q[0], 0, sizeof(R->Q[0]));
	R->gen = gc.nextgen;	/* stable at this point */
	splx(s);

	/*
	 * If there's anything in limbo, we need the GC to do another
	 * xcall to reap it, so reactivate the GC.
	 */
	if (R->Q[1].callback || R->Q[1].kfree)
		atomic_store_relaxed(&gc.active, true);

	/* Release the per-CPU state.  */
	percpu_putref(gc.percpu);
}

static void
rcu_work(struct work *work, void *cookie)
{
	struct rcu_cpu *R = container_of(work, struct rcu_cpu, work);
	struct rcu_queue Q;
	struct rcu_head *head, *next;

	/* We had better be bound to the CPU.  */
	KASSERT(curlwp->l_pflag & LP_BOUND);

	/*
	 * Block out the xcall by disabling kpreemption, grab the ready
	 * queue, and acknowledge the work by emptying the ready queue.
	 */
	kpreempt_disable();
	Q = R->Q[2];
	memset(&R->Q[2], 0, sizeof(R->Q[2]));
	kpreempt_enable();

	SDT_PROBE0(sdt, linux, rcu, reclaim__start);

	/*
	 * We have exclusive access to the queue of objects that are
	 * definitely ready to reclaim now.  Reclaim them.
	 */
	for (head = Q.callback; head != NULL; head = next) {
		void (*callback)(struct rcu_head *);

		callback = head->rcuh_u.callback;
		next = head->rcuh_next;

		SDT_PROBE2(sdt, linux, rcu, call__run,  head, callback);
		(*callback)(head);
		/* Can't dereference head or callback after this point.  */
		SDT_PROBE2(sdt, linux, rcu, call__done,  head, callback);
	}
	for (head = Q.kfree; head != NULL; head = next) {
		void *obj;

		obj = head->rcuh_u.obj;
		next = head->rcuh_next;

		SDT_PROBE2(sdt, linux, rcu, kfree__free,  head, obj);
		kfree(obj);
		/* Can't dereference head or obj after this point.  */
		SDT_PROBE2(sdt, linux, rcu, kfree__done,  head, obj);
	}

	SDT_PROBE0(sdt, linux, rcu, reclaim__done);
}

/*
 * activate_gc()
 *
 *	Advance the global generation number and activate the GC
 *	thread.  Caller must hold the global GC lock, and the GC must
 *	be currently inactive.
 */
static void
activate_gc(void)
{

	KASSERT(mutex_owned(&gc.lock));
	KASSERT(!gc.active);
	KASSERTMSG(gc.nextgen == gc.gen || gc.nextgen == gc.gen + 1,
	    "gen=%"PRIu32" nextgen=%"PRIu32, gc.gen, gc.nextgen);

	/* Request the GC be activated.  */
	atomic_store_relaxed(&gc.active, true);

	/*
	 * If we're not in the middle of advancing a generation, begin
	 * to advance it.  But if we are in the middle of advancing a
	 * generation, leave it be -- the GC thread will notice and
	 * advance it again as soon as this one has finished.
	 */
	if (gc.nextgen == gc.gen)
		atomic_store_relaxed(&gc.nextgen, gc.gen + 1);
	else
		KASSERT(gc.nextgen == gc.gen + 1);

	/* Wake the GC thread.  */
	cv_broadcast(&gc.cv);
}

/*
 * maybe_activate_gc(R)
 *
 *	Activate the GC thread, but only if necessary.  Caller must
 *	have kpreemption disabled.
 */
static void
maybe_activate_gc(struct rcu_cpu *R)
{

	/*
	 * This CPU's xcall thread must be blocked to prevent it from
	 * concurrently running R->gen++.
	 */
	KASSERT(kpreempt_disabled());

	/*
	 * If the global generation is advancing, the GC is active and
	 * will eventually get what we have queued on this CPU before
	 * advancing the local generation.
	 */
	if (atomic_load_relaxed(&gc.nextgen) != R->gen)
		return;

	/*
	 * If the GC has already been activated, or is being
	 * reactivated, then it will eventually issue an xcall.  This
	 * is a safe test because the GC thread does not set active to
	 * false except when it is about to issue an xcall.
	 */
	if (atomic_load_relaxed(&gc.active))
		return;

	/* Check again under the lock.  */
	mutex_enter(&gc.lock);
	invariants();
	if (!gc.active)
		activate_gc();
	invariants();
	mutex_exit(&gc.lock);
}

/*
 * synchronize_rcu()
 *
 *	Wait for any pending RCU read section on every CPU to complete
 *	by triggering on every CPU activity that is blocked by an RCU
 *	read section.
 *
 *	May sleep.  (Practically guaranteed to sleep!)
 */
void
synchronize_rcu(void)
{

	/*
	 * XXX figure out how to coordinate with the GC thread to
	 * reduce the number of xcalls
	 */
	synchronize_rcu_expedited();
}

static void
synchronize_rcu_expedited_xc(void *a, void *b)
{

	SDT_PROBE1(sdt, linux, rcu, synchronize__cpu,  cpu_index(curcpu()));
}

/*
 * synchronize_rcu_expedited()
 *
 *	Wait for any pending RCU read section on every CPU to complete
 *	by triggering on every CPU activity that is blocked by an RCU
 *	read section.  Try to get an answer faster than
 *	synchronize_rcu, at the cost of more activity triggered on
 *	other CPUs.
 *
 *	May sleep.  (Practically guaranteed to sleep!)
 */
void
synchronize_rcu_expedited(void)
{

	SDT_PROBE0(sdt, linux, rcu, synchronize__start);
	xc_wait(xc_broadcast(0, &synchronize_rcu_expedited_xc, NULL, NULL));
	SDT_PROBE0(sdt, linux, rcu, synchronize__done);
}

/*
 * cookie = get_state_synchronize_rcu(), cond_synchronize_rcu(cookie)
 *
 *	Optimization for synchronize_rcu -- skip if it has already
 *	happened between get_state_synchronize_rcu and
 *	cond_synchronize_rcu.  Memory ordering:
 *
 *	- get_state_synchronize_rcu implies a full SMP memory barrier
 *	  (membar_sync) (XXX why?).
 *
 *	- cond_synchronize_rcu happens-after all callbacks in the
 *	  intervening grace period.
 */
unsigned long
get_state_synchronize_rcu(void)
{

	membar_sync();
	return atomic_load_acquire(&gc.nextgen);
}

void
cond_synchronize_rcu(unsigned long cookie)
{
	uint32_t nextgen = cookie;

	/*
	 * If we haven't yet passed what was scheduled to be the next
	 * generation at the time of get_state_synchronize_rcu, just
	 * synchronize -- we can't prove that an xcall has been issued
	 * in the intervening time.
	 *
	 * (The event of rollover is not a problem: if the counter has
	 * rolled over, then we definitely did pass what was scheduled
	 * to be the next generation, so we can safely skip issuing
	 * another xcall.)
	 */
	if ((int)(atomic_load_acquire(&gc.gen) - nextgen) <= 0)
		synchronize_rcu();
}

/*
 * rcu_barrier()
 *
 *	Wait for all pending RCU callbacks to complete.
 *
 *	Does not imply, and is not implied by, synchronize_rcu.
 */
void
rcu_barrier(void)
{
	uint32_t nextgen;

	/*
	 * Cases:
	 *
	 * 1. gc.active is clear, gc.nextgen == gc.gen
	 *	=> No callbacks are queued anywhere, nothing to do.
	 *
	 * 2. gc.active is set, gc.nextgen == gc.gen + 1
	 *	=> New callbacks are pending OR callbacks are in limbo.
	 *	=> Need to wait until past gc.nextgen, or inactive.
	 *
	 * 3. gc.active is clear, gc.nextgen == gc.gen + 1
	 *	=> Waiting for xcall.  No new callbacks are pending,
	 *	   but callbacks may still be in limbo.
	 *	=> Need to wait until past gc.nextgen, or inactive.
	 *
	 * 4. gc.active is set, gc.nextgen == gc.gen
	 *	=> Should not happen.
	 */

	SDT_PROBE0(sdt, linux, rcu, barrier__start);
	mutex_enter(&gc.lock);
	invariants();
	if (gc.nextgen == gc.gen) {
		KASSERT(!gc.active);
	} else {
		KASSERT(gc.nextgen == gc.gen + 1);
		nextgen = gc.nextgen;
		for (;;) {
			invariants();
			cv_wait(&gc.cv, &gc.lock);
			invariants();

			/* If we passed the next generation, stop.  */
			if ((int)(gc.gen - nextgen) > 0)
				break;

			/* If the GC has no more work to do, stop.  */
			if (!gc.active && gc.nextgen == gc.gen)
				break;
		}
	}
	invariants();
	mutex_exit(&gc.lock);
	SDT_PROBE0(sdt, linux, rcu, barrier__done);
}

/*
 * call_rcu(head, callback)
 *
 *	Arrange to call callback(head) after any pending RCU read
 *	sections on every CPU is complete.  Return immediately.
 */
void
call_rcu(struct rcu_head *head, void (*callback)(struct rcu_head *))
{
	struct rcu_cpu *R;
	int s;

	head->rcuh_u.callback = callback;

	R = percpu_getref(gc.percpu);

	s = splvm();
	head->rcuh_next = R->Q[0].callback;
	R->Q[0].callback = head;
	splx(s);

	SDT_PROBE2(sdt, linux, rcu, call__queue,  head, callback);

	maybe_activate_gc(R);
	percpu_putref(gc.percpu);
}

/*
 * _kfree_rcu(head, obj)
 *
 *	kfree_rcu helper: schedule kfree(obj) using head for storage.
 */
void
_kfree_rcu(struct rcu_head *head, void *obj)
{
	struct rcu_cpu *R;
	int s;

	LOCKDEBUG_MEM_CHECK(obj, ((struct linux_malloc *)obj - 1)->lm_size);

	head->rcuh_u.obj = obj;

	R = percpu_getref(gc.percpu);

	s = splvm();
	head->rcuh_next = R->Q[0].kfree;
	R->Q[0].kfree = head;
	splx(s);

	SDT_PROBE2(sdt, linux, rcu, kfree__queue,  head, obj);

	maybe_activate_gc(R);
	percpu_putref(gc.percpu);
}

static void
gc_thread(void *cookie)
{

	mutex_enter(&gc.lock);
	invariants();
	for (;;) {
		/* Wait to be activated or exit.  */
		while (!gc.active) {
			if (gc.dying)
				goto out;
			invariants();
			cv_wait(&gc.cv, &gc.lock);
			invariants();
		}

		/*
		 * Whoever activated us should have also begun to
		 * advance the generation.
		 */
		KASSERT(gc.gen != gc.nextgen);

		/* Acknowledge the activation request.  */
		atomic_store_relaxed(&gc.active, false);

		/*
		 * Release the GC lock to issue an xcall to:
		 * - Reap objects in limbo on all CPUs.
		 * - Move pending objects to limbo on all CPUs.
		 * - Reactivate if any objects are now in limbo.
		 */
		invariants();
		mutex_exit(&gc.lock);
		SDT_PROBE0(sdt, linux, rcu, synchronize__start);
		xc_wait(xc_broadcast(0, &rcu_xc, NULL, NULL));
		SDT_PROBE0(sdt, linux, rcu, synchronize__done);
		mutex_enter(&gc.lock);
		invariants();

		/* Notify waiters that this generation is done.  */
		KASSERT(gc.nextgen == gc.gen + 1);
		atomic_store_release(&gc.gen, gc.nextgen);
		if (gc.gen == 0)
			gc.epoch++;
		cv_broadcast(&gc.cv);

		/*
		 * If there were objects now in limbo, start the next
		 * generation.
		 */
		if (gc.active)
			atomic_store_relaxed(&gc.nextgen, gc.gen + 1);

		/* Rate-limit the cross-calls.  */
		(void)kpause("lxrcubat", /*intr*/false, /*timo*/1, &gc.lock);
	}
out:	invariants();
	mutex_exit(&gc.lock);
	kthread_exit(0);
}

void
init_rcu_head(struct rcu_head *head)
{
}

void
destroy_rcu_head(struct rcu_head *head)
{
}

static void
rcu_cpu_init(void *ptr, void *cookie, struct cpu_info *ci)
{
	struct rcu_cpu **Rp = ptr;

	*Rp = kmem_zalloc(sizeof(**Rp), KM_SLEEP);
}

static void
rcu_cpu_fini(void *ptr, void *cookie, struct cpu_info *ci)
{
	struct rcu_cpu **Rp = ptr, *R = *Rp;
	unsigned i;

	for (i = 0; i < __arraycount(R->Q); i++) {
		KASSERT(R->Q[i].callback == NULL);
		KASSERT(R->Q[i].kfree == NULL);
	}

	*Rp = NULL;		/* paranoia */
	kmem_free(R, sizeof(*R));
}

int
linux_rcu_gc_init(void)
{
	int error;

	mutex_init(&gc.lock, MUTEX_DEFAULT, IPL_VM);
	cv_init(&gc.cv, "lnxrcugc");
	gc.percpu = percpu_create(sizeof(struct rcu_cpu),
	    rcu_cpu_init, rcu_cpu_fini, NULL);
	gc.nextgen = 0;
	gc.gen = 0;
	gc.epoch = 0;
	gc.active = false;
	gc.dying = false;

	error = kthread_create(PRI_NONE,
	    KTHREAD_MPSAFE|KTHREAD_TS|KTHREAD_MUSTJOIN, NULL, &gc_thread, NULL,
	    &gc.lwp, "lnxrcugc");
	if (error) {
		gc.lwp = NULL;
		goto fail;
	}

	error = workqueue_create(&gc.wq, "lxrcgcwq", &rcu_work, NULL, PRI_NONE,
	    IPL_NONE, WQ_MPSAFE|WQ_PERCPU);
	if (error) {
		gc.wq = NULL;
		goto fail;
	}

	/* Success!  */
	return 0;

fail:	linux_rcu_gc_fini();
	return error;
}

void
linux_rcu_gc_fini(void)
{

	mutex_enter(&gc.lock);
	invariants();
	gc.dying = true;
	cv_broadcast(&gc.cv);
	invariants();
	mutex_exit(&gc.lock);

	if (gc.wq) {
		workqueue_destroy(gc.wq);
		gc.wq = NULL;
	}

	if (gc.lwp) {
		kthread_join(gc.lwp);
		gc.lwp = NULL;
	}

	percpu_free(gc.percpu, sizeof(struct rcu_cpu));
	cv_destroy(&gc.cv);
	mutex_destroy(&gc.lock);
}
