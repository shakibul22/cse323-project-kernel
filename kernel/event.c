//
// Kernel event recorder ("black box").
//
// A fixed-size circular buffer of struct kernel_event in kernel BSS,
// written by instrumentation points in fork, exit, exec, the syscall
// dispatcher, the page allocator and the scheduler, and read by user
// space through getevents().
//
// elog.lock is a LEAF lock: this file never allocates, never prints,
// never sleeps and never takes another lock while holding it, so the
// recorder cannot deadlock whatever kernel path calls it.
//
#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "riscv.h"
#include "spinlock.h"
#include "proc.h"
#include "defs.h"
#include "event.h"

// The whole recorder: 256 * 40 = 10240 bytes, statically allocated.
struct {
  struct spinlock lock;
  struct kernel_event buf[EVENT_BUFFER_SIZE];
  uint   head;  // slot the next event goes into
  uint   count; // valid events, 0 .. EVENT_BUFFER_SIZE
  uint64 nrec;  // total ever recorded; source of e->seq
  uint64 nlost; // events overwritten because the ring was full
  int    mask;  // bit i set => record type i
  int    ready;
} elog;

// Called from main() on hart 0 after kinit(), so the ~32000 kfree() calls
// that freerange() makes while building the free list are not recorded.
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

// Record one event.  The caller passes 'pid' rather than letting us read
// myproc(), because the scheduler knows the pid it cares about while
// myproc() would return 0 there.
void
eventrecord(int type, int pid, uint64 arg1, uint64 arg2)
{
  struct kernel_event *e;

  if (!elog.ready)
    return;
  if (type <= EV_NONE || type >= EV_NTYPES)
    return;
  // Racy read of elog.mask: worst case we record one event that was just
  // disabled, or miss one that was just enabled.  Locking for the common
  // "type is disabled" case would cost more than it is worth.
  if ((elog.mask & EVMASK(type)) == 0)
    return;

  acquire(&elog.lock); // also disables interrupts on this hart

  e = &elog.buf[elog.head];
  e->seq = elog.nrec++;
  // 'ticks' is read without tickslock on purpose: taking it here would nest
  // the recorder inside the timer path, and the worst outcome of the race
  // is a timestamp one tick stale.
  e->ticks = ticks;
  e->pid = pid;
  e->cpu = cpuid(); // legal: acquire() turned interrupts off
  e->type = type;
  e->arg1 = arg1;
  e->arg2 = arg2;

  // When the ring is full head runs over the oldest record: count stays
  // pinned and nlost goes up.
  elog.head = (elog.head + 1) & EVENT_BUFFER_MASK;
  if (elog.count < EVENT_BUFFER_SIZE)
    elog.count++;
  else
    elog.nlost++;

  release(&elog.lock);
}

// For call sites that just mean "the current process".  myproc() is 0 in
// scheduler/boot context, which we report as pid 0.
void
eventcur(int type, uint64 arg1, uint64 arg2)
{
  struct proc *p = myproc();
  eventrecord(type, p ? p->pid : 0, arg1, arg2);
}

// Events staged on the kernel stack per copyout().  8 * 40 = 320 bytes;
// the kernel stack is one page, so keep this small.
#define EVCHUNK 8

// getevents(): copy at most 'max' events, oldest first, to dstva.  The
// caller must supply room for 'max' events.  Returns the number copied,
// or -1 on a bad argument or address.
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

  // Validate the buffer before deciding how much to copy, so a bogus
  // pointer is rejected even when the log is empty.  The first test also
  // catches an address+length overflow.
  need = (uint64)max * sizeof(struct kernel_event);
  if (dstva + need < dstva || dstva + need > p->sz)
    return -1;

  if (max == 0)
    return 0;
  if (!elog.ready)
    return 0;

  // Snapshot how much we return and where the oldest event lives.
  acquire(&elog.lock);
  n = elog.count;
  if (n > max)
    n = max;
  start = (elog.head + EVENT_BUFFER_SIZE - n) & EVENT_BUFFER_MASK;
  release(&elog.lock);

  // We must not hold elog.lock across copyout(): copyout() can fault on a
  // lazily-allocated user page, and that fault runs kalloc() ->
  // eventrecord() -> acquire(&elog.lock) on this same hart.  xv6 spinlocks
  // are not recursive, so that would panic.  Stage a few records under the
  // lock, drop it, then touch user memory.
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

// eventctl(): newmask < 0 queries; newmask >= 0 installs the new type mask
// and empties the ring.  Returns the previous mask.
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
