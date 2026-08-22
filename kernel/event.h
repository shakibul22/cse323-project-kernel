//
// Kernel event recorder ("black box") -- definitions shared by the kernel
// and by user/blackbox.c, so both sides agree on the record layout.
// Depends on nothing but kernel/types.h, so it is safe to include from
// user space.
//
#ifndef XV6_EVENT_H
#define XV6_EVENT_H

// Must be a power of two: the ring wraps with a mask, not a division.
#define EVENT_BUFFER_SIZE 256
#define EVENT_BUFFER_MASK (EVENT_BUFFER_SIZE - 1)

// Event types.  0 is reserved so a zeroed record is never a real event.
#define EV_NONE     0
#define EV_FORK     1
#define EV_EXIT     2
#define EV_EXEC     3
#define EV_SYSCALL  4
#define EV_ALLOC    5
#define EV_FREE     6
#define EV_SCHED    7
#define EV_NTYPES   8

// Bit i means "record type i" (see eventctl()).
#define EVMASK(t)   (1 << (t))
#define EVMASK_ALL  (EVMASK(EV_FORK) | EVMASK(EV_EXIT) | EVMASK(EV_EXEC) | \
                     EVMASK(EV_SYSCALL) | EVMASK(EV_ALLOC) |               \
                     EVMASK(EV_FREE) | EVMASK(EV_SCHED))

// Everything except the page allocator: kalloc()/kfree() run tens of
// thousands of times per second and would overwrite the ring before a
// human could read it.
#define EVMASK_DEFAULT (EVMASK_ALL & ~(EVMASK(EV_ALLOC) | EVMASK(EV_FREE)))

// One recorded event.  Fields are ordered largest-first so the layout is
// the same 40 bytes no matter which half is being compiled.
struct kernel_event {
  uint64 seq;   // global sequence number, +1 per recorded event
  uint64 arg1;
  uint64 arg2;
  uint   ticks;
  int    pid;   // process the event is about (0 = none)
  int    cpu;
  int    type;  // EV_*
};

// Per-type meaning of arg1/arg2:
//   EV_FORK    pid=parent  arg1=child pid
//   EV_EXIT    pid=exiting arg1=exit status
//   EV_EXEC    pid=caller  arg1,arg2 = first 16 bytes of the program name
//   EV_SYSCALL pid=caller  arg1=syscall number
//   EV_ALLOC   pid=caller  arg1=size (PGSIZE)  arg2=physical address
//   EV_FREE    pid=caller  arg1=size (PGSIZE)  arg2=physical address
//   EV_SCHED   pid=chosen  arg1=0

#endif // XV6_EVENT_H
