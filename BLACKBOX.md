# Kernel Event Recorder ("Black Box") for xv6-riscv

**Base tree:** `mit-pdos/xv6-riscv`, commit `2534832` (20 Aug 2026)
**Target:** RISC-V 64, QEMU `virt`, 3 harts (default `CPUS=3`)

---

## 1. Abstract

The kernel is the one component of the system that cannot easily print a stack
trace after it misbehaves: by the time a user notices, the interesting moment
has passed. This project adds a *flight recorder* to xv6. The kernel writes a
small, fixed-size, structured record for each interesting event (process
creation, termination, `exec`, system calls, page allocation and scheduling)
into a 256-entry circular buffer in kernel memory. The buffer never grows and
never blocks; when it fills, the oldest record is overwritten, so the most
recent 256 events are always available. A new system call, `getevents()`,
copies the records out to user space with `copyout()`, and the `blackbox`
user program formats them for a human.

## 2. Introduction and problem statement

xv6 has `printk()`, but printing from inside the kernel is a poor observation
tool:

* it takes the console lock, so it changes the very locking behaviour being
  studied;
* it is slow (UART, one character at a time), so it changes timing;
* the output is unstructured text that scrolls away;
* it cannot be used at all from paths where taking the console lock would
  deadlock.

The problem this project solves: **record what the kernel did, cheaply, and
let user space decide how to display it.** That separation — the kernel stores
fixed-size binary records, user space does the string formatting — is the same
design used by real tracing infrastructure (Linux `ftrace`, DTrace, ETW).

## 3. Objectives

1. Define a compact, fixed-size kernel event record.
2. Implement a bounded circular buffer in kernel memory.
3. Make it safe on a multi-hart kernel using an xv6 spinlock.
4. Instrument fork, exit, exec, the syscall dispatcher, the page allocator and
   the scheduler.
5. Export the records to user space through a new system call.
6. Provide a user program that prints them, and a test suite that proves the
   above.
7. Do all of this without breaking xv6: `usertests` must still pass.

---

## 4. System design

```
        user space                    |            kernel space
                                      |
  blackbox ──getevents(buf,max)──────►│ sys_getevents()
                                      │      │
                                      │      ▼
                                      │  eventread()  ── chunk of 8 ──►┐
                                      │      ▲                          │
                                      │      │ copyout()  ◄─────────────┘
                                      │      │
  ┌───────────────────────────────────┼──────┴────────────────────────────┐
  │                                   │   struct elog                     │
  │  kfork()   ──┐                    │   ┌────────────────────────────┐  │
  │  kexit()   ──┤                    │   │ spinlock  lock             │  │
  │  kexec()   ──┼──► eventrecord() ──┼──►│ kernel_event buf[256]      │  │
  │  syscall() ──┤                    │   │ head, count, nrec, nlost   │  │
  │  scheduler()─┤                    │   │ mask, ready                │  │
  │  kalloc()  ──┘  (via eventcur())  │   └────────────────────────────┘  │
  └───────────────────────────────────┴───────────────────────────────────┘
```

### 4.1 The event record

```c
struct kernel_event {
  uint64 seq;   // global sequence number
  uint64 arg1;  // type-specific
  uint64 arg2;  // type-specific
  uint   ticks; // timer ticks since boot
  int    pid;
  int    cpu;
  int    type;  // EV_*
};
```

**Why this layout.**

* **Fixed size, no pointers.** 40 bytes, no implicit padding on RV64 (three
  8-byte fields, then four 4-byte fields). Recording is a handful of stores;
  copying to user space is one `memmove`. A record that contained a `char*`
  would be useless the moment the pointed-to memory was freed, and dangerous
  the moment it crossed into user space.
* **Fields ordered largest-first** so the kernel-side and user-side views of
  the struct are identical regardless of compiler alignment choices.
* **`seq` was added to the required fields.** It is the global count of events
  ever recorded. It lets the user program *detect* that events were lost:
  if two adjacent records differ by more than one, the ring wrapped. Without
  it, an overwritten history is indistinguishable from a quiet kernel.
* **`arg1`/`arg2` instead of a union of per-event structs.** A union would be
  larger and would require the user program to know a different layout per
  type. Two generic words keep the record small and the decoding trivial.

| Type | `pid` | `arg1` | `arg2` |
|---|---|---|---|
| `EV_FORK` | parent | child pid | — |
| `EV_EXIT` | exiting process | exit status | — |
| `EV_EXEC` | caller | first 8 bytes of program name | next 8 bytes |
| `EV_SYSCALL` | caller | syscall number | — |
| `EV_ALLOC` | caller (0 if none) | size (4096) | physical address |
| `EV_FREE` | caller (0 if none) | size (4096) | physical address |
| `EV_SCHED` | process chosen | — | — |

For `EV_EXEC` the program name is *packed into the two argument words* rather
than stored as a string: `p->name` is a fixed 16-byte array, so this is two
aligned 8-byte loads, and the user program unpacks it. No string handling
happens on a kernel path.

### 4.2 Timestamp

`ticks` is xv6's global timer-tick counter (`kernel/trap.c`), incremented by
the supervisor timer interrupt. It is the natural xv6 time source, it is
already maintained, and it costs one load to read. It is **not** a
scheduler-tick counter and not wall-clock time: at xv6's default timer rate,
many events share the same tick value. That is why `seq` — not `ticks` — is
what establishes the exact ordering of events.

The recorder reads `ticks` **without** `tickslock`, deliberately. Taking
`tickslock` inside the recorder would nest the recorder into the timer
interrupt path and create a lock-ordering obligation for no benefit. A 4-byte
aligned read of a monotonically increasing counter is a benign race whose
worst case is a timestamp that is one tick stale.

---

## 5. Circular buffer design

```
   buf[]  ┌────┬────┬────┬────┬────┬────┬────┬────┐
          │ e4 │ e5 │ e6 │    │    │ e1 │ e2 │ e3 │
          └────┴────┴────┴────┴────┴────┴────┴────┘
                       ▲                ▲
                       │                └── oldest = (head - count) & MASK
                       └── head = next slot to write
```

State (`kernel/event.c`):

* `head` — index of the slot the **next** event will use;
* `count` — number of valid records, saturating at `EVENT_BUFFER_SIZE`;
* `nrec` — total events ever recorded, the source of `seq`;
* `nlost` — how many records have been overwritten.

Insertion is:

```c
buf[head] = event;
head = (head + 1) & EVENT_BUFFER_MASK;
if (count < EVENT_BUFFER_SIZE) count++;   // still filling
else                           nlost++;   // overwrote the oldest
```

`EVENT_BUFFER_SIZE` is 256 — a **power of two**, so the wrap-around is a
single `AND` instead of a division. When the ring is full, nothing special
happens: `head` simply runs over the oldest record, `count` stays pinned at
256, and the reader derives the oldest index as `(head - count) & MASK`.

The buffer is a static struct in kernel BSS: 256 × 40 = 10 240 bytes. It is
never allocated, never resized and never freed. That is the whole point of a
black box — a recorder that could fail to allocate memory during a crisis
would be worthless.

---

## 6. Synchronization

### 6.1 Why a lock is required

Every hart executes kernel code concurrently. `eventrecord()` performs a
read-modify-write on shared state (`head`, `count`, `nrec`) and writes into
the slot those variables select. Without mutual exclusion, two harts can read
the same `head`, both write slot *i*, and both advance to *i+1* — one event is
lost, and worse, a *torn* record can appear (`pid` from one event, `arg1` from
another), which is exactly the kind of corruption a debugging tool must never
produce. `EV_SCHED` is recorded from every hart's scheduler loop, so this is
not a theoretical race: it happens within milliseconds of boot.

xv6's `spinlock` is the correct primitive: the critical section is a dozen
stores long, it must work in contexts where sleeping is forbidden (inside the
scheduler, with `p->lock` held), and `acquire()` disables interrupts on the
local hart, which is what makes the recorder safe against being re-entered by
an interrupt handler on the same core.

### 6.2 Why it cannot deadlock

`elog.lock` is a **leaf lock**: `eventrecord()` acquires it, does only plain
memory stores, and releases it. It never calls `kalloc`, `printk`, `sleep`,
`wakeup`, or any function that takes another lock. Since no lock is ever
acquired *while holding* `elog.lock`, `elog.lock` can never be part of a lock
cycle, no matter which kernel path calls the recorder.

The orders that actually occur are all harmless:

| Call site | Locks held on entry | Order created |
|---|---|---|
| `scheduler()` | `p->lock` | `p->lock → elog.lock` |
| `kfork()` | none | — |
| `kexit()` | none | — |
| `kexec()` | none | — |
| `syscall()` | none | — |
| `kalloc()`/`kfree()` | none (recorded **outside** `kmem.lock`) | — |

Interrupt safety follows the standard xv6 rule: a hart holding `elog.lock` has
interrupts disabled, so a timer interrupt on that hart cannot arrive and try to
re-acquire the same lock (xv6 spinlocks are not recursive; that would panic).

### 6.3 The scheduler instrumentation, specifically

```c
p->state = RUNNING;
c->proc = p;
eventrecord(EV_SCHED, p->pid, 0, 0);   // <-- here
swtch(&c->context, &p->context);
```

This is *after* the scheduler has committed to `p` (so the record is truthful)
and *before* `swtch()` (so the record is written while this hart still owns
`p->lock` and `p` cannot disappear). `acquire()`/`release()` are balanced, so
`mycpu()->noff` is back to 1 when `swtch()` runs — which matters, because
`sched()` panics with `"sched locks"` if the switched-to thread finds
`noff != 1`. Recording *after* `swtch()` would be wrong: control does not come
back until the process yields, possibly on a different hart.

### 6.4 Recursive logging — the real hazard

Two recursion paths exist, and both are handled:

**(a) The observer erasing what it observes.** `getevents()` is itself a
system call. If the syscall hook recorded it, every attempt to read the log
would first append to the log. Worse, reading 256 events would push out the
oldest ones. `SYSCALL_TRACE_MASK` in `kernel/syscall.c` therefore excludes
`SYS_getevents` and `SYS_eventctl` (and the high-frequency pollers `getpid`
and `uptime`).

**(b) `copyout()` → `vmfault()` → `kalloc()` → `eventrecord()`.** This tree
supports lazily allocated user memory, so `copyout()` can fault on the user
buffer and allocate a page. If `eventread()` held `elog.lock` across
`copyout()`, that allocation would call `eventrecord()` on the same hart,
which would try to acquire `elog.lock` again — and xv6 spinlocks are not
recursive, so the kernel would panic with `"acquire"`.

The fix is structural, not a flag: `eventread()` **stages 8 records on the
kernel stack under the lock, releases the lock, and only then calls
`copyout()`**, repeating until done. The lock is never held across any call
that can fault, allocate, or sleep.

```c
while (copied < n) {
  acquire(&elog.lock);
  for (i = 0; i < k; i++) stage[i] = elog.buf[(start + copied + i) & MASK];
  release(&elog.lock);
  copyout(...stage...);      // may fault, allocate, even sleep — safe now
  copied += k;
}
```

The cost is that a very long read is not one atomic snapshot: events can be
overwritten between chunks. That is why `seq` exists — `blackbox` reports
`"N sequence gap(s)"` when it detects it. A correct, non-atomic read is a much
better trade than an atomic read that can panic the kernel.

**(c) The allocator flood.** `kalloc()`/`kfree()` run tens of thousands of
times per second. Recording them is *safe* (see above) but drowns a 256-entry
ring instantly, so `EVMASK_DEFAULT` leaves `EV_ALLOC`/`EV_FREE` off; the
`blackbox -m alloc,free` command turns them on for the memory test. Also,
recording only starts after `eventinit()`, which `main()` calls **after**
`kinit()` — this drops the ~32 000 `kfree()` calls that `freerange()` makes
while building the free list at boot.

---

## 7. System call design

```c
int getevents(struct kernel_event *buf, int max);   // SYS_getevents = 23
int eventctl(int mask);                             // SYS_eventctl  = 24
```

`getevents()` copies up to `max` records, **oldest first**, and returns the
number copied (0 if the log is empty, −1 on a bad argument). The caller must
provide room for `max` records — that contract is what allows the kernel to
validate the destination *before* touching it:

```c
need = (uint64)max * sizeof(struct kernel_event);
if (dstva + need < dstva || dstva + need > p->sz)   // overflow, or past the
  return -1;                                        // end of user memory
```

(That check was added because a test caught a real hole: with an empty ring,
no `copyout()` happened, so a garbage pointer was silently accepted and 0
returned. Validating up front fixes it.)

The kernel never returns a raw kernel pointer, and never lets user space see
the ring itself. Everything crosses the boundary through `copyout()`, which
walks the *caller's* page table and so cannot be tricked into writing kernel
memory.

`eventctl(mask)` sets which event types are recorded and clears the ring;
`eventctl(-1)` queries without changing anything. It exists for two practical
reasons: it makes each test able to isolate one event type, and it makes the
allocator events usable at all on a 256-entry ring.

---

## 8. File-by-file changes

### Added

| File | Purpose |
|---|---|
| `kernel/event.h` | Shared record layout, event codes, masks, buffer size |
| `kernel/event.c` | The recorder: buffer, lock, `eventrecord/eventread/eventctl` |
| `user/blackbox.c` | User utility that prints the log |
| `user/eventtest.c` | 8-test suite |

### Modified

| File | Change | Why |
|---|---|---|
| `Makefile` | `$K/event.o` in `OBJS`; `_blackbox`, `_eventtest` in `UPROGS` | build and install |
| `kernel/defs.h` | prototypes for the five event functions | kernel-wide visibility |
| `kernel/main.c` | `eventinit()` after `kinit()` | init lock; skip boot `freerange` flood |
| `kernel/proc.c` | record in `kfork()`, `kexit()`, `scheduler()` | process + scheduler events |
| `kernel/exec.c` | record in `kexec()` after `p->name` is set | exec events with program name |
| `kernel/kalloc.c` | record in `kalloc()`/`kfree()`, outside `kmem.lock` | memory events |
| `kernel/syscall.c` | one hook in `syscall()`, plus dispatch-table entries | all syscalls from one place |
| `kernel/syscall.h` | `SYS_getevents 23`, `SYS_eventctl 24` | syscall numbers |
| `kernel/sysproc.c` | `sys_getevents()`, `sys_eventctl()` | argument fetch + validation |
| `user/user.h` | user prototypes, `struct kernel_event` forward decl | user API |
| `user/usys.pl` | `entry("getevents")`, `entry("eventctl")` | generated `ecall` stubs |

### Sharing the struct between kernel and user

`user/blackbox.c` includes `kernel/event.h` directly (the Makefile passes
`-I.`), exactly as other user programs include `kernel/stat.h`. The header
depends on nothing but `kernel/types.h`, so it is safe on both sides.

The alternative — copying the struct into a user header — is what makes
tracing tools rot: the day someone adds a field to the kernel struct, the two
definitions disagree, `getevents()` fills the user's buffer with correctly
copied bytes that are then decoded at the wrong offsets, and the tool prints
plausible nonsense with no error anywhere. One definition, one truth.

---

## 9. Build and run

```bash
make clean
make            # builds kernel/kernel
make qemu       # builds fs.img (user programs) and boots
```

Note: plain `make` builds only the kernel. The user programs and `fs.img` are
built by the `fs.img` target, which `make qemu` depends on — so use
`make qemu`, or `make fs.img` explicitly.

Inside xv6:

```
$ blackbox                 # print the log
$ blackbox -n 20           # print the 20 most recent events
$ blackbox -c              # clear the log
$ blackbox -s              # show which event types are being recorded
$ blackbox -m all          # record everything (including alloc/free)
$ blackbox -m fork,exit    # record only these, and clear
$ blackbox -m default      # back to the default set
$ eventtest                # run all 8 tests
$ eventtest 6              # run only test 6
```

Multi-CPU / single-CPU:

```bash
make qemu CPUS=1
make qemu CPUS=3     # default
```

Quit QEMU with `Ctrl-A` then `x`.

---

## 10. Demonstration

```
$ blackbox -c
blackbox: event log cleared
$ echo hi
hi
$ blackbox -n 22

===== XV6 KERNEL EVENT RECORDER =====

   SEQ    TIME  CPU    PID  EVENT      DETAILS
----------------------------------------------------------
   146       1    0      2  SYSCALL    read
   ...                                          (15 × read)
   160       1    0      2  SYSCALL    read
   161       1    0      2  SYSCALL    fork
   162       1    0      2  FORK       child=5
   163       1    0      2  SYSCALL    wait
   164       1    0      5  SCHEDULE
   165       1    0      5  SYSCALL    sbrk
   166       1    0      5  SYSCALL    exec
   167       1    0      5  EXEC       blackbox
----------------------------------------------------------
Total events shown: 22 (seq 146..167)
=====================================
```

Read it as a story:

1. **`SYSCALL read` × 15** — pid 2 is the shell, reading the typed command one
   character at a time from the console.
2. **`SYSCALL fork` then `FORK child=5`** — the shell asks for a child; the
   recorder captures both the *entry* to the call (from the dispatcher) and
   its *effect* (from inside `kfork()`, where the child pid is known).
3. **`SYSCALL wait`** — the shell blocks waiting for the child.
4. **`SCHEDULE` pid=5** — a hart's scheduler picked the new child to run.
5. **`SYSCALL sbrk`** — the child's C library grows the heap.
6. **`SYSCALL exec` then `EXEC blackbox`** — the child replaces its image;
   the second record carries the program name unpacked from `arg1`/`arg2`.

The `CPU` column shows which hart recorded each event; on a 3-hart boot the
same process visibly migrates between harts.

---

## 11. Testing

`user/eventtest.c`. Each test calls `eventctl()` first, which clears the ring
*and* narrows recording to the types under test — otherwise the test's own
`printf()` calls (which are `write()` syscalls) would flood the evidence.

| # | Test | What it asserts |
|---|---|---|
| 1 | Basic recording | events exist; `ticks` non-decreasing; `seq` contiguous |
| 2 | Fork | exactly 3 `FORK` records; parent pid correct; child pid non-zero |
| 3 | Exit | exactly 3 `EXIT` records; exit status 7 preserved |
| 4 | System calls | `write`, `open`, `close`, `pause`, `fork` all appear; `getevents` never appears |
| 5 | Scheduler | `SCHEDULE` records exist for more than one pid |
| 6 | Circular buffer | 512 events into a 256 ring ⇒ exactly 256 returned, contiguous, `seq ≥ 256` (oldest overwritten), kernel alive |
| 7 | Multiple CPUs | all `cpu` ids plausible; no record has a corrupt type |
| 8 | Empty buffer / bad args | empty ⇒ 0; `max=0` ⇒ 0; `max<0` ⇒ −1; bad pointer ⇒ −1 |

### Results (measured, not predicted)

```
$ eventtest
=== kernel event recorder tests ===
test 1: basic recording            PASS ×3
test 2: fork events                PASS ×3
test 3: exit events                PASS ×2
test 4: syscall events             PASS ×6
test 5: scheduler events           PASS ×3
test 6: circular buffer overflow   PASS ×4
test 7: multiple CPUs              PASS ×3   (cpu ids seen: 0..2)
test 8: empty buffer / bad args    PASS ×4
=== ALL TESTS PASSED ===
```

* Passes with `CPUS=3` **and** with `CPUS=1`.
* `blackbox -m all` (allocator hooks on) followed by `forktest`: no panic, no
  deadlock, events recorded from all harts.
* `usertests` — xv6's own regression suite — still passes with the recorder
  compiled in, which is the real proof that the instrumentation did not break
  the kernel.

---

## 12. Debugging guide

**Compile errors.**

| Symptom | Cause |
|---|---|
| `implicit declaration of 'eventrecord'` | prototype missing from `kernel/defs.h`, or `event.h` not included |
| `undefined reference to 'sys_getevents'` | `$K/event.o` missing from `OBJS`, or `sysproc.c` not rebuilt |
| `undefined reference to 'getevents'` (user link) | `entry("getevents")` missing from `user/usys.pl` — delete `user/usys.S` and rebuild |
| `unknown sys call 23` at runtime | number in `syscall.h` does not match the dispatch table entry |
| Fields print as garbage | kernel and user disagree on the struct — check both include `kernel/event.h` |

Note `make` alone rebuilds only the kernel; if the user program seems not to
change, you forgot `make fs.img` (or just use `make qemu`).

**Kernel panic.** `panic: acquire` means a lock was taken twice on one hart —
for this project that almost certainly means something was recorded while
`elog.lock` was held (see §6.4). `panic: sched locks` means an instrumentation
point left `noff` unbalanced. To debug:

```bash
make qemu-gdb            # terminal 1
riscv64-unknown-elf-gdb  # terminal 2, picks up ./.gdbinit
(gdb) b eventrecord
(gdb) b panic
(gdb) c
(gdb) p elog.head
(gdb) p elog.count
(gdb) p elog.buf[elog.head-1]
```

`kernel/kernel.asm` (produced by every build) maps the panic PC to a source
line.

**Suspected deadlock** (xv6 hangs, no output): break in with GDB and inspect
`elog.lock.locked` and `elog.lock.cpu`. If `locked` is 1 and the owning hart is
inside `copyout`/`kalloc`, the lock is being held across a faulting call —
that is the bug §6.4 describes.

**Missing events.** In order: (1) `blackbox -s` — is the type in the mask?
(2) is `eventinit()` being called (`elog.ready`)? (3) is the instrumentation on
the path actually taken — e.g. `read`/`write` from the shell go through
`syscall()`, but a kernel-internal `readi()` does not? (4) did the ring wrap —
does `blackbox` report sequence gaps?

---

## 13. Limitations

1. **256 events is a few milliseconds of history** on a busy kernel. With
   allocator tracing on, the ring covers well under one second.
2. **A long read is not an atomic snapshot** (§6.4). `seq` gaps make this
   visible rather than silent.
3. **Tick resolution is coarse.** Many events share one tick; `seq`, not
   `ticks`, gives the true order.
4. **The buffer does not survive a reboot** — a real black box would persist
   to disk. It *does* survive a kernel panic, and can be read with GDB.
5. **Syscall arguments are not recorded**, only the number. Recording argument
   *values* would mean touching user memory on every syscall.
6. **Recording is not free.** Each event is a lock acquire/release plus ~40
   bytes of stores; with every type enabled, allocator-heavy workloads
   contend on one global lock. A per-hart ring merged at read time would scale
   better.
7. **Events from very early boot are dropped** by design (before
   `eventinit()`).

## 14. Future improvements

* **Per-CPU ring buffers**, merged and sorted by `seq` at read time —
  removes the last shared lock from the hot path.
* **Persist the ring across reboot** by placing it at a fixed physical address
  the kernel does not clear, which is what makes it a true black box.
* **Blocking read** (`sleep`/`wakeup` on the ring) so a monitor program can
  stream events instead of polling.
* **Record syscall return values** by adding a second hook after dispatch.
* **A `dmesg`-style panic dump**: have `panic()` walk the ring and print the
  last N events before halting.

## 15. Conclusion

The recorder demonstrates, in about 300 lines of kernel code, the core
mechanisms of an operating-systems course: bounded kernel data structures,
spinlocks and lock ordering on a multiprocessor, the kernel/user boundary and
why `copyout()` exists, system-call plumbing from `ecall` stub to dispatch
table, and — most instructively — the failure modes that instrumentation
itself creates: recursion through the allocator, an observer that erases its
own evidence, and locks held across faulting calls. Every one of those was a
real hazard in this tree, and each is solved structurally rather than papered
over.

---

## 16. Viva questions and answers

**1. Why a circular buffer and not a plain array?**
A plain array either stops recording when full (you lose the *newest* events,
which are the ones that explain a crash) or must grow (allocation in a crash
path — unacceptable). A ring keeps the most recent N events in constant memory.

**2. Why fixed size — why not grow it dynamically?**
Growing means calling `kalloc()` from inside the recorder, which is called
*from* `kalloc()`. Bounded memory also means the recorder can never be the
reason the kernel runs out of memory.

**3. Why must the size be a power of two?**
So `(head + 1) & 255` replaces `(head + 1) % 256`. A division on the hot path
for no reason would be poor engineering.

**4. Why is a spinlock required?**
The buffer is shared by all harts. `head`, `count` and `nrec` are updated
read-modify-write; concurrent updates would lose events and produce torn
records with fields from two different events.

**5. Why a spinlock rather than a sleeplock?**
The recorder runs in contexts where sleeping is forbidden — inside the
scheduler with `p->lock` held, and potentially in interrupt context. The
critical section is a few dozen instructions, so spinning is cheaper anyway.

**6. What happens when two CPUs record at the same instant?**
One wins the `acquire()`; the other spins for a few dozen cycles. Both events
are recorded, in the order the lock was acquired, with distinct `seq` values
and their own (possibly different) `cpu` fields.

**7. What happens when the buffer is full?**
`head` overwrites the oldest record, `count` stays at 256, `nlost` is
incremented, and the reader still sees the newest 256 events. No error, no
blocking.

**8. How does user space get the events?**
`getevents(buf, max)` → `ecall` → `syscall()` → `sys_getevents()` →
`eventread()`, which stages records under the lock and `copyout()`s them into
the caller's buffer, returning the count.

**9. Why is `copyout()` required — why not `memmove()`?**
The kernel runs on its own page table; the user pointer is a *user* virtual
address that means nothing in that context. `copyout()` walks the caller's
page table, validates each page, and handles pages that are lazily allocated.
It is also the security check: without it, user space could pass a kernel
address and have the kernel write to it.

**10. Why not just return a pointer to the ring?**
It would hand user space a kernel address (an information leak and a write
primitive if mapped), and the buffer would change under the reader.

**11. Where exactly is the scheduler instrumented, and why there?**
In `scheduler()`, after `p->state = RUNNING; c->proc = p;` and before
`swtch()`. Then the decision is final, `p->lock` is held so `p` cannot vanish,
and `noff` is balanced before the switch. After `swtch()` would be wrong —
control does not return there until the process yields.

**12. Is it safe to take a lock while holding `p->lock`?**
Yes, provided ordering is consistent. `elog.lock` is a *leaf*: nothing is ever
acquired while holding it, so it cannot close a cycle. `p->lock → elog.lock`
is the only order that ever occurs.

**13. Why is logging inside the allocator dangerous?**
Two reasons. Recursion: `copyout()` can fault → `vmfault()` → `kalloc()` →
recorder; if the recorder's lock were already held by that hart, xv6 would
panic (spinlocks are not recursive). And volume: `kalloc` runs tens of
thousands of times per second and would erase the ring instantly.

**14. So how did you handle it?**
Structurally: `eventread()` never holds the lock across `copyout()` — it
stages 8 records, drops the lock, then copies. And the allocator hooks record
*outside* `kmem.lock`, and are off by default in `EVMASK_DEFAULT`.

**15. Why isn't `getevents` itself traced?**
Because then observing the system would modify it: every read would append an
event, and reading 256 events would push out the oldest ones. It is masked out
in `SYSCALL_TRACE_MASK`.

**16. Should every system call be recorded?**
Not blindly. `getpid`/`uptime` are called in tight polling loops and carry
almost no information. The design instruments the single dispatch point in
`syscall()` and filters with a bitmask, so the policy is one editable constant
rather than 20 scattered edits.

**17. Why instrument the dispatcher instead of each `sys_*()`?**
One hook covers all present and future syscalls, cannot be forgotten when a
syscall is added, and keeps the change reviewable. Per-call hooks would be 20+
edits with 20+ chances to get it wrong.

**18. What does the timestamp mean?**
Timer ticks since boot — xv6's existing `ticks`, bumped by the supervisor
timer interrupt. Not wall-clock, not scheduler ticks. Coarse, so `seq`
establishes exact ordering.

**19. Why read `ticks` without `tickslock`?**
Taking it would nest the recorder into the timer path and impose a lock order
for no gain. An aligned read of a monotonic counter is benign; the worst case
is a one-tick-stale timestamp.

**20. What is `seq` for?**
It is the global event counter. Contiguous `seq` values prove no events were
lost between two records; a jump proves the ring wrapped. It also gives an
unambiguous total order that `ticks` cannot.

**21. What information does one event contain?**
`seq`, `ticks`, `pid`, `cpu`, `type`, and two type-specific words — 40 bytes,
fixed.

**22. Why not store strings (e.g. the syscall name) in the kernel?**
String formatting on a kernel hot path costs time and space and would need a
larger record. The kernel stores a number; `blackbox` maps it to a name in
user space. Storing data, not presentation, is the general principle.

**23. What happens during a kernel panic?**
The ring is static kernel memory, so it survives. Under
`make qemu-gdb` you can inspect `elog.buf` after the panic and see exactly
what the kernel was doing beforehand — which is the entire motivation.

**24. What is the difference between kernel logging and event recording?**
Logging formats text and prints it immediately: it takes the console lock, is
slow enough to perturb timing, and scrolls away. Event recording stores
fixed-size binary records in bounded memory, retains only recent history, and
defers formatting to user space.

**25. Why is it called a black box?**
By analogy with an aircraft flight recorder: a small, bounded, always-on
device that keeps only the most recent history and is read *after* something
goes wrong.

**26. How do you know the instrumentation did not break xv6?**
`usertests`, xv6's own regression suite, still passes with the recorder
compiled in, on 1 and on 3 harts.

**27. What would you change to make it scale?**
Per-hart ring buffers merged by `seq` at read time. The single global lock is
the only real contention point in the design.
