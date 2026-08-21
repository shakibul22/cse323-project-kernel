#include "types.h"
#include "riscv.h"
#include "defs.h"
#include "param.h"
#include "memlayout.h"
#include "spinlock.h"
#include "proc.h"
#include "vm.h"

uint64
sys_exit(void)
{
  int n;
  argint(0, &n);
  kexit(n);
  return 0; // not reached
}

uint64
sys_getpid(void)
{
  return myproc()->pid;
}

uint64
sys_fork(void)
{
  return kfork();
}

uint64
sys_wait(void)
{
  uint64 p;
  argaddr(0, &p);
  return kwait(p);
}

uint64
sys_sbrk(void)
{
  uint64 addr;
  int t;
  int n;

  argint(0, &n);
  argint(1, &t);
  addr = myproc()->sz;

  if (t == SBRK_EAGER || n < 0) {
    if (growproc(n) < 0) {
      return -1;
    }
  } else {
    // Lazily allocate memory for this process: increase its memory
    // size but don't allocate memory. If the processes uses the
    // memory, vmfault() will allocate it.
    if (addr + n < addr)
      return -1;
    if (addr + n > TRAPFRAME)
      return -1;
    myproc()->sz += n;
  }
  return addr;
}

uint64
sys_pause(void)
{
  int n;
  uint ticks0;

  argint(0, &n);
  if (n < 0)
    n = 0;
  acquire(&tickslock);
  ticks0 = ticks;
  while (ticks - ticks0 < n) {
    if (killed(myproc())) {
      release(&tickslock);
      return -1;
    }
    sleep_prepare(&ticks);
    release(&tickslock);
    sleep();
    acquire(&tickslock);
  }
  release(&tickslock);
  return 0;
}

uint64
sys_kill(void)
{
  int pid;

  argint(0, &pid);
  return kkill(pid);
}

// return how many clock tick interrupts have occurred
// since start.
uint64
sys_uptime(void)
{
  uint xticks;

  acquire(&tickslock);
  xticks = ticks;
  release(&tickslock);
  return xticks;
}

// int getevents(struct kernel_event *buf, int max)
//
// Copy up to 'max' recorded kernel events, oldest first, into the user
// buffer.  Returns the number of events copied (0 if the log is empty),
// or -1 on a bad argument or an unwritable user buffer.
//
// Note what we do NOT do: we never hand a kernel pointer to user space and
// we never let user space see the ring itself.  Everything crosses the
// boundary through copyout(), which validates the destination against the
// caller's page table.
uint64
sys_getevents(void)
{
  uint64 dst;
  int max;

  argaddr(0, &dst);
  argint(1, &max);

  if (max < 0)
    return -1;

  return eventread(dst, max);
}

// int eventctl(int mask)
//
// mask <  0 : return the current event-type mask, change nothing.
// mask >= 0 : install 'mask' as the set of event types to record and empty
//             the ring.  Returns the previous mask.
uint64
sys_eventctl(void)
{
  int mask;

  argint(0, &mask);
  return eventctl(mask);
}
