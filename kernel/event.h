//
// Kernel Event Recorder ("black box") -- shared definitions.
//
// This single header is included BOTH by the kernel and by the user-space
// utility (user/blackbox.c includes it as "kernel/event.h"; the Makefile
// passes -I. so that path resolves).  Sharing one header instead of
// duplicating the struct guarantees that both sides agree on the exact
// layout of struct kernel_event and on the numeric event codes.  If the two
// sides ever disagreed, getevents() would hand user space a byte blob that
// the user program would decode incorrectly -- a silent, very confusing bug.
//
// The header deliberately depends on nothing but kernel/types.h, so it is
// safe to include from user space.
//
#ifndef XV6_EVENT_H
#define XV6_EVENT_H

// Number of events kept in the kernel ring.  MUST be a power of two so the
// wrap-around can be done with a cheap mask instead of a division.
#define EVENT_BUFFER_SIZE 256
#define EVENT_BUFFER_MASK (EVENT_BUFFER_SIZE - 1)

// Event types.  0 is reserved so that a zeroed record is never mistaken for
// a real event.
#define EV_NONE     0
#define EV_FORK     1
#define EV_EXIT     2
#define EV_EXEC     3
#define EV_SYSCALL  4
#define EV_ALLOC    5
#define EV_FREE     6
#define EV_SCHED    7
#define EV_NTYPES   8   // one past the last valid type

// Bit masks used to enable/disable recording of individual event types at
// run time (see eventctl()).  Bit i means "record type i".
#define EVMASK(t)   (1 << (t))
#define EVMASK_ALL  (EVMASK(EV_FORK) | EVMASK(EV_EXIT) | EVMASK(EV_EXEC) | \
                     EVMASK(EV_SYSCALL) | EVMASK(EV_ALLOC) |               \
                     EVMASK(EV_FREE) | EVMASK(EV_SCHED))

// Default: everything except the page allocator.  kalloc()/kfree() run tens
// of thousands of times per second, so leaving them on would overwrite the
// 256-entry ring before a human could read it.  "blackbox -m alloc,free"
// turns them on for the memory test.
#define EVMASK_DEFAULT (EVMASK_ALL & ~(EVMASK(EV_ALLOC) | EVMASK(EV_FREE)))

// One recorded event.  Exactly 40 bytes on RV64 with no implicit padding:
//   0..7   seq     8..15  arg1   16..23 arg2
//   24..27 ticks   28..31 pid    32..35 cpu    36..39 type
// The fields are ordered largest-first on purpose so the layout is identical
// no matter which compiler builds the two halves.
struct kernel_event {
  uint64 seq;   // global sequence number; increases by 1 per recorded event
  uint64 arg1;  // meaning depends on type (see below)
  uint64 arg2;
  uint   ticks; // timer ticks since boot (kernel/trap.c: 'ticks')
  int    pid;   // pid of the process the event is about (0 = no process)
  int    cpu;   // hart that recorded the event
  int    type;  // EV_*
};

// Per-type meaning of arg1/arg2:
//   EV_FORK    pid=parent  arg1=child pid
//   EV_EXIT    pid=exiting arg1=exit status
//   EV_EXEC    pid=caller  arg1,arg2 = first 16 bytes of the program name
//   EV_SYSCALL pid=caller  arg1=syscall number
//   EV_ALLOC   pid=caller  arg1=size in bytes (PGSIZE)  arg2=physical address
//   EV_FREE    pid=caller  arg1=size in bytes (PGSIZE)  arg2=physical address
//   EV_SCHED   pid=chosen  arg1=0

#endif // XV6_EVENT_H
