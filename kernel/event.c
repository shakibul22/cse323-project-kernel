//
// Kernel Event Recorder ("black box").
//
// A fixed-size circular buffer of struct kernel_event living in kernel BSS.
// Kernel code calls eventrecord() from a handful of instrumentation points
// (fork, exit, exec, the syscall dispatcher, the page allocator and the
// scheduler).  User space reads the buffer with the getevents() system call.
//
// Design rules obeyed by every line of this file:
//   1. Never allocate memory.        (would recurse into the ALLOC hook)
//   2. Never print.                  (console lock + slow)
//   3. Never sleep, never take any other lock while holding elog.lock,
//      i.e. elog.lock is a LEAF lock -- this is what makes the recorder
//      deadlock-free no matter which kernel path calls it.
//   4. Never copyout() while holding elog.lock (copyout can fault, and
//      handling that fault calls kalloc(), which calls back into us).
//
#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "riscv.h"
#include "spinlock.h"
#include "proc.h"
#include "defs.h"
#include "event.h"

// The whole recorder.  Statically allocated: 256 * 40 = 10240 bytes.
// It never grows.
struct {
  struct spinlock lock;
  struct kernel_event buf[EVENT_BUFFER_SIZE];
  uint   head;  // index of the slot the NEXT event goes into
  uint   count; // number of valid events, 0 .. EVENT_BUFFER_SIZE
  uint64 nrec;  // total events ever recorded (source of e->seq)
  uint64 nlost; // events overwritten (i.e. lost) because the ring was full
  int    mask;  // bit i set => record events of type i
  int    ready; // 0 until eventinit() has run
} elog;

// Called once from main() on hart 0, AFTER kinit().  Recording is off until
// this returns, which conveniently drops the ~32000 kfree() calls that
// kinit()'s freerange() performs while building the free list.
void
eventinit(void)
{
  initlock(&elog.lock, "eventlog");
  elog.head = 0;
  elog.count = 0;
  elog.nrec = 0;
  elog.nlost = 0;
  elog.mask = EVMASK_DEFAULT;
  __sync_synchronize();
  elog.ready = 1;
}

// Record one event.  'pid' is passed in by the caller rather than read from
// myproc() because some call sites (the scheduler) know the pid they care
// about while myproc() would return 0 there.
//
// This is the hot path: no loops, no strings, no allocation, and the
// critical section is a dozen stores long.
void
eventrecord(int type, int pid, uint64 arg1, uint64 arg2)
{
  struct kernel_event *e;

  if (!elog.ready)
    return;
  if (type <= EV_NONE || type >= EV_NTYPES)
    return;
  // Racy read of elog.mask: worst case we record one event that was just
  // disabled, or miss one that was just enabled.  Taking the lock for the
  // common "type is disabled" case would cost far more than it is worth.
  if ((elog.mask & EVMASK(type)) == 0)
    return;

  acquire(&elog.lock); // also disables interrupts on this hart

  e = &elog.buf[elog.head];
  e->seq = elog.nrec++;
  // 'ticks' is read without tickslock on purpose.  Taking tickslock here
  // would nest the recorder inside the timer path; a 4-byte aligned load of
  // a monotonically increasing counter is a benign race whose worst outcome
  // is a timestamp that is one tick stale.
  e->ticks = ticks;
  e->pid = pid;
  e->cpu = cpuid(); // legal: acquire() turned interrupts off
  e->type = type;
  e->arg1 = arg1;
  e->arg2 = arg2;

  // Circular buffer advance.  EVENT_BUFFER_SIZE is a power of two, so the
  // wrap is a mask.  When the ring is full, head simply runs over the oldest
  // record: the count stays pinned at EVENT_BUFFER_SIZE and we bump nlost.
  elog.head = (elog.head + 1) & EVENT_BUFFER_MASK;
  if (elog.count < EVENT_BUFFER_SIZE)
    elog.count++;
  else
    elog.nlost++;

  release(&elog.lock);
}

// Convenience wrapper for call sites that just want "the current process".
// myproc() returns 0 in scheduler/boot context, which we report as pid 0.
void
eventcur(int type, uint64 arg1, uint64 arg2)
{
  struct proc *p = myproc();
  eventrecord(type, p ? p->pid : 0, arg1, arg2);
}

// Number of events staged on the kernel stack per copyout().
// 8 * 40 = 320 bytes; the kernel stack is only one page, so keep this small.
#define EVCHUNK 8

// Implementation of the getevents() system call.
// Copies at most 'max' events, oldest first, to the user buffer at dstva.
// The caller must supply room for 'max' events -- that is the interface
// contract, and it is what lets us validate the buffer before we touch it.
// Returns the number of events copied, or -1 on a bad argument/address.
int
eventread(uint64 dstva, int max)
{
  struct proc *p = myproc();
  struct kernel_event stage[EVCHUNK];
  uint64 need;
  int n, k, i, copied = 0;
  uint start;

  if (max < 0)
    return -1;

  // Validate the user buffer up front, before deciding how much to copy.
  // Doing it here (rather than relying on copyout to fail later) means a
  // bogus pointer is rejected even when the log happens to be empty, and it
  // means we never copy a partial result into a buffer that was too small.
  // The first test also catches an address+length overflow.
  need = (uint64)max * sizeof(struct kernel_event);
  if (dstva + need < dstva || dstva + need > p->sz)
    return -1;

  if (max == 0)
    return 0;
  if (!elog.ready)
    return 0;

  // Snapshot how many events we are going to return, and where the oldest
  // one lives.  start = head - n, modulo the ring size.
  acquire(&elog.lock);
  n = elog.count;
  if (n > max)
    n = max;
  start = (elog.head + EVENT_BUFFER_SIZE - n) & EVENT_BUFFER_MASK;
  release(&elog.lock);

  // Copy in small chunks.  We must NOT hold elog.lock across copyout():
  // copyout() may hit an unmapped lazily-allocated user page, which calls
  // vmfault() -> kalloc() -> eventrecord() -> acquire(&elog.lock) on this
  // same hart, and xv6 spinlocks are not recursive: that would panic with
  // "acquire".  So we stage a few records under the lock, drop the lock,
  // and only then touch user memory.
  while (copied < n) {
    k = n - copied;
    if (k > EVCHUNK)
      k = EVCHUNK;

    acquire(&elog.lock);
    for (i = 0; i < k; i++)
      stage[i] = elog.buf[(start + copied + i) & EVENT_BUFFER_MASK];
    release(&elog.lock);

    if (copyout(p->pagetable, p->sz,
                dstva + (uint64)copied * sizeof(struct kernel_event),
                (char *)stage, (uint64)k * sizeof(struct kernel_event)) < 0)
      return copied > 0 ? copied : -1;

    copied += k;
  }

  return copied;
}

// Implementation of the eventctl() system call.
//   newmask <  0 : query only, nothing changes.
//   newmask >= 0 : install the new type mask AND empty the ring.
// Returns the mask that was in effect before the call.
int
eventctl(int newmask)
{
  int old;

  if (!elog.ready)
    return -1;

  acquire(&elog.lock);
  old = elog.mask;
  if (newmask >= 0) {
    elog.mask = newmask & EVMASK_ALL;
    elog.head = 0;
    elog.count = 0;
    elog.nlost = 0;
  }
  release(&elog.lock);

  return old;
}
