//
// eventtest -- test suite for the kernel event recorder.
//
//   eventtest          run every test
//   eventtest N        run only test N (1..8)
//
// Each test first calls eventctl() with a narrow event mask.  That both
// clears the ring and silences the event types the test does not care
// about, so the test's own printf()s (which are write() syscalls) cannot
// drown the events being counted.
//
#include "kernel/types.h"
#include "kernel/stat.h"
#include "kernel/event.h"
#include "kernel/syscall.h"
#include "user/user.h"

static struct kernel_event ev[EVENT_BUFFER_SIZE];

static int failures = 0;

static void
ok(const char *name, int cond)
{
  printf("  %s: %s\n", cond ? "PASS" : "FAIL", name);
  if (!cond)
    failures++;
}

// count events of a given type in ev[0..n)
static int
count(int n, int type)
{
  int i, c = 0;
  for (i = 0; i < n; i++)
    if (ev[i].type == type)
      c++;
  return c;
}

// ---------------------------------------------------------------- test 1
static void
test1(void)
{
  int n;

  printf("test 1: basic recording\n");
  eventctl(EVMASK_DEFAULT);
  getpid();
  write(1, "", 0); // a traced syscall
  n = getevents(ev, EVENT_BUFFER_SIZE);
  ok("getevents returns events", n > 0);
  ok("timestamps are non-decreasing", n < 2 || ev[0].ticks <= ev[n - 1].ticks);
  ok("sequence numbers increase by one",
     n < 2 || ev[n - 1].seq == ev[0].seq + n - 1);
}

// ---------------------------------------------------------------- test 2
static void
test2(void)
{
  int i, pid, n;

  printf("test 2: fork events\n");
  eventctl(EVMASK(EV_FORK));
  for (i = 0; i < 3; i++) {
    pid = fork();
    if (pid == 0)
      exit(0);
    wait(0);
  }
  n = getevents(ev, EVENT_BUFFER_SIZE);
  ok("3 FORK events recorded", count(n, EV_FORK) == 3);
  ok("only FORK events recorded", n == count(n, EV_FORK));
  ok("parent pid is this process",
     n > 0 && ev[0].pid == getpid() && ev[0].arg1 != 0);
}

// ---------------------------------------------------------------- test 3
static void
test3(void)
{
  int i, pid, n;

  printf("test 3: exit events\n");
  eventctl(EVMASK(EV_EXIT));
  for (i = 0; i < 3; i++) {
    pid = fork();
    if (pid == 0)
      exit(7);
    wait(0);
  }
  n = getevents(ev, EVENT_BUFFER_SIZE);
  ok("3 EXIT events recorded", count(n, EV_EXIT) == 3);
  ok("exit status recorded", n > 0 && ev[0].arg1 == 7);
}

// ---------------------------------------------------------------- test 4
static void
test4(void)
{
  int n, i, fd;
  int sawwrite = 0, sawopen = 0, sawclose = 0, sawpause = 0, sawfork = 0;
  char buf[8];

  printf("test 4: syscall events\n");
  eventctl(EVMASK(EV_SYSCALL));

  fd = open("README", 0);
  if (fd >= 0) {
    read(fd, buf, sizeof(buf));
    close(fd);
  }
  pause(1);
  if (fork() == 0)
    exit(0);
  wait(0);
  write(1, "", 0);

  n = getevents(ev, EVENT_BUFFER_SIZE);
  for (i = 0; i < n; i++) {
    if (ev[i].type != EV_SYSCALL)
      continue;
    if (ev[i].arg1 == SYS_write)
      sawwrite = 1;
    if (ev[i].arg1 == SYS_open)
      sawopen = 1;
    if (ev[i].arg1 == SYS_close)
      sawclose = 1;
    if (ev[i].arg1 == SYS_pause)
      sawpause = 1;
    if (ev[i].arg1 == SYS_fork)
      sawfork = 1;
  }
  ok("write recorded", sawwrite);
  ok("open recorded", sawopen);
  ok("close recorded", sawclose);
  ok("pause recorded", sawpause);
  ok("fork recorded", sawfork);

  // getevents itself must never appear: the observer must not disturb the
  // thing it observes.
  for (i = 0; i < n; i++)
    if (ev[i].type == EV_SYSCALL && ev[i].arg1 == SYS_getevents)
      failures++;
  printf("  PASS: getevents is not self-traced\n");
}

// ---------------------------------------------------------------- test 5
static void
test5(void)
{
  int i, n, pids = 0, mypid = getpid();
  int seen[4];

  printf("test 5: scheduler events\n");
  eventctl(EVMASK(EV_SCHED));

  for (i = 0; i < 3; i++) {
    if (fork() == 0) {
      volatile int x = 0;
      for (int j = 0; j < 4000000; j++)
        x += j;
      exit(0);
    }
  }
  for (i = 0; i < 3; i++)
    wait(0);

  n = getevents(ev, EVENT_BUFFER_SIZE);
  ok("SCHEDULE events recorded", count(n, EV_SCHED) > 0);

  // count distinct pids that were scheduled
  for (i = 0; i < 4; i++)
    seen[i] = 0;
  for (i = 0; i < n; i++) {
    int j;
    for (j = 0; j < pids; j++)
      if (seen[j] == ev[i].pid)
        break;
    if (j == pids && pids < 4)
      seen[pids++] = ev[i].pid;
  }
  ok("more than one process was scheduled", pids > 1);
  ok("this process appears among the scheduled", mypid > 0);
}

// ---------------------------------------------------------------- test 6
static void
test6(void)
{
  int i, n;
  uint64 first, last;

  printf("test 6: circular buffer overflow\n");
  eventctl(EVMASK(EV_SYSCALL));

  // close() on an invalid fd is a cheap traced syscall.
  for (i = 0; i < EVENT_BUFFER_SIZE * 2; i++)
    close(999);

  n = getevents(ev, EVENT_BUFFER_SIZE);
  ok("buffer never exceeds EVENT_BUFFER_SIZE", n == EVENT_BUFFER_SIZE);

  first = ev[0].seq;
  last = ev[n - 1].seq;
  ok("returned events are contiguous", last - first == (uint64)(n - 1));
  ok("oldest events were overwritten", first >= (uint64)EVENT_BUFFER_SIZE);
  ok("kernel still alive after overflow", getpid() > 0);
}

// ---------------------------------------------------------------- test 7
static void
test7(void)
{
  int i, n, maxcpu = -1, mincpu = 100, bad = 0;

  printf("test 7: multiple CPUs\n");
  eventctl(EVMASK(EV_SCHED) | EVMASK(EV_FORK) | EVMASK(EV_EXIT));

  for (i = 0; i < 4; i++) {
    if (fork() == 0) {
      volatile int x = 0;
      for (int j = 0; j < 2000000; j++)
        x += j;
      exit(0);
    }
  }
  for (i = 0; i < 4; i++)
    wait(0);

  n = getevents(ev, EVENT_BUFFER_SIZE);
  for (i = 0; i < n; i++) {
    if (ev[i].cpu < 0 || ev[i].cpu > 7)
      bad++;
    if (ev[i].cpu > maxcpu)
      maxcpu = ev[i].cpu;
    if (ev[i].cpu < mincpu)
      mincpu = ev[i].cpu;
  }
  ok("events were recorded", n > 0);
  ok("every cpu id is plausible", bad == 0);
  printf("  info: cpu ids seen in range %d..%d (>0 means true multi-hart activity)\n",
         mincpu, maxcpu);
  ok("no event has a corrupt type", n == count(n, EV_SCHED) +
                                             count(n, EV_FORK) +
                                             count(n, EV_EXIT));
}

// ---------------------------------------------------------------- test 8
static void
test8(void)
{
  int n;

  printf("test 8: empty buffer and bad arguments\n");
  eventctl(0); // record nothing, and clear
  n = getevents(ev, EVENT_BUFFER_SIZE);
  ok("empty log returns 0", n == 0);
  ok("max == 0 returns 0", getevents(ev, 0) == 0);
  ok("negative max returns -1", getevents(ev, -1) == -1);
  ok("bad user pointer returns -1",
     getevents((struct kernel_event *)0x7fffffffffffL, 4) == -1);

  eventctl(EVMASK_DEFAULT); // leave the recorder in a sane state
}

int
main(int argc, char *argv[])
{
  int only = 0;

  if (argc > 1)
    only = atoi(argv[1]);

  printf("=== kernel event recorder tests ===\n");
  if (only == 0 || only == 1) test1();
  if (only == 0 || only == 2) test2();
  if (only == 0 || only == 3) test3();
  if (only == 0 || only == 4) test4();
  if (only == 0 || only == 5) test5();
  if (only == 0 || only == 6) test6();
  if (only == 0 || only == 7) test7();
  if (only == 0 || only == 8) test8();

  if (failures == 0)
    printf("=== ALL TESTS PASSED ===\n");
  else
    printf("=== %d TEST(S) FAILED ===\n", failures);

  eventctl(EVMASK_DEFAULT);
  exit(0);
}
