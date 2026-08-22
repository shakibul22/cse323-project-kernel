//
// blackbox -- read and pretty-print the kernel event recorder.
//
// Usage:
//   blackbox                 dump the events currently in the kernel ring
//   blackbox -n N            dump at most the N most recent events
//   blackbox -c              clear the ring (keeps the current type mask)
//   blackbox -m LIST         set which event types are recorded, then clear.
//                            LIST is a comma-separated list of type names,
//                            or "all" / "none" / "default".
//   blackbox -s              show the current recording mask and exit
//
// All formatting happens here.  The kernel stores fixed-size binary records
// only; turning a number into "SYSCALL write" must not happen on a kernel
// fast path.
//
#include "kernel/types.h"
#include "kernel/stat.h"
#include "kernel/event.h"
#include "kernel/syscall.h"
#include "user/user.h"

// Global, so the whole ring fits in BSS rather than the one-page stack.
static struct kernel_event evbuf[EVENT_BUFFER_SIZE];

static const char *typename[EV_NTYPES] = {
  [EV_NONE]    "NONE",    [EV_FORK]  "FORK",  [EV_EXIT] "EXIT",
  [EV_EXEC]    "EXEC",    [EV_SYSCALL] "SYSCALL", [EV_ALLOC] "ALLOC",
  [EV_FREE]    "FREE",    [EV_SCHED] "SCHEDULE",
};

// Syscall number -> name.
static const char *syscallname[] = {
  [SYS_fork] "fork",     [SYS_exit] "exit",     [SYS_wait] "wait",
  [SYS_pipe] "pipe",     [SYS_read] "read",     [SYS_kill] "kill",
  [SYS_exec] "exec",     [SYS_fstat] "fstat",   [SYS_chdir] "chdir",
  [SYS_dup] "dup",       [SYS_getpid] "getpid", [SYS_sbrk] "sbrk",
  [SYS_pause] "pause",   [SYS_uptime] "uptime", [SYS_open] "open",
  [SYS_write] "write",   [SYS_mknod] "mknod",   [SYS_unlink] "unlink",
  [SYS_link] "link",     [SYS_mkdir] "mkdir",   [SYS_close] "close",
  [SYS_sync] "sync",     [SYS_getevents] "getevents",
  [SYS_eventctl] "eventctl",
};
#define NSYSCALLNAME (sizeof(syscallname) / sizeof(syscallname[0]))

// xv6's printf has no width or alignment flags ("%-8s" prints literally),
// so the columns are padded by hand.
static void
padstr(const char *s, int w)
{
  int i = 0;
  while (s[i]) {
    printf("%c", s[i]);
    i++;
  }
  while (i < w) {
    printf(" ");
    i++;
  }
}

static void
padnum(uint64 v, int w)
{
  uint64 t = v;
  int digits = 1;

  while (t >= 10) {
    t /= 10;
    digits++;
  }
  while (digits < w) {
    printf(" ");
    digits++;
  }
  printf("%ld", v);
}

// Unpack the 16 name bytes the EXEC hook packed into arg1/arg2.
static void
printname(uint64 a1, uint64 a2)
{
  char nm[17];
  int i;

  memcpy(nm, &a1, 8);
  memcpy(nm + 8, &a2, 8);
  nm[16] = 0;
  for (i = 0; i < 16 && nm[i]; i++)
    printf("%c", nm[i]);
}

static int
hasdetails(struct kernel_event *e)
{
  return e->type != EV_SCHED;
}

static void
describe(struct kernel_event *e)
{
  switch (e->type) {
  case EV_FORK:
    printf("child=%ld", e->arg1);
    break;
  case EV_EXIT:
    printf("status=%ld", e->arg1);
    break;
  case EV_EXEC:
    printname(e->arg1, e->arg2);
    break;
  case EV_SYSCALL:
    if (e->arg1 < NSYSCALLNAME && syscallname[e->arg1])
      printf("%s", syscallname[e->arg1]);
    else
      printf("#%ld", e->arg1);
    break;
  case EV_ALLOC:
  case EV_FREE:
    printf("size=%ld pa=0x%lx", e->arg1, e->arg2);
    break;
  case EV_SCHED:
    break;
  }
}

// Parse "fork,exit,alloc" / "all" / "none" / "default".  -1 if unknown.
static int
parsemask(char *s)
{
  char word[16];
  int mask = 0, i, t, found;

  if (strcmp(s, "all") == 0)
    return EVMASK_ALL;
  if (strcmp(s, "none") == 0)
    return 0;
  if (strcmp(s, "default") == 0)
    return EVMASK_DEFAULT;

  while (*s) {
    i = 0;
    while (*s && *s != ',' && i < (int)sizeof(word) - 1)
      word[i++] = *s++;
    word[i] = 0;
    if (*s == ',')
      s++;

    found = 0;
    for (t = 1; t < EV_NTYPES; t++) {
      // accept both "sched" and "schedule"
      if (strcmp(word, typename[t]) == 0 ||
          (t == EV_SCHED && strcmp(word, "SCHED") == 0)) {
        mask |= EVMASK(t);
        found = 1;
        break;
      }
      {
        char up[16];
        int k;
        for (k = 0; word[k] && k < 15; k++)
          up[k] = (word[k] >= 'a' && word[k] <= 'z') ? word[k] - 32 : word[k];
        up[k] = 0;
        if (strcmp(up, typename[t]) == 0 ||
            (t == EV_SCHED && strcmp(up, "SCHED") == 0)) {
          mask |= EVMASK(t);
          found = 1;
          break;
        }
      }
    }
    if (!found)
      return -1;
  }
  return mask;
}

static void
showmask(int mask)
{
  int t, first = 1;

  printf("recording mask 0x%x: ", mask);
  if (mask == 0)
    printf("(nothing)");
  for (t = 1; t < EV_NTYPES; t++) {
    if (mask & EVMASK(t)) {
      if (!first)
        printf(",");
      printf("%s", typename[t]);
      first = 0;
    }
  }
  printf("\n");
}

static void
usage(void)
{
  printf("usage: blackbox [-n count] [-c] [-s] [-m all|none|default|"
         "type,type,...]\n");
  exit(1);
}

int
main(int argc, char *argv[])
{
  int i, n, max = EVENT_BUFFER_SIZE;
  uint64 lastseq = 0;
  int gaps = 0;

  for (i = 1; i < argc; i++) {
    if (strcmp(argv[i], "-c") == 0) {
      int old = eventctl(-1); // query
      eventctl(old);          // reinstall the same mask => clears the ring
      printf("blackbox: event log cleared\n");
      exit(0);
    } else if (strcmp(argv[i], "-s") == 0) {
      showmask(eventctl(-1));
      exit(0);
    } else if (strcmp(argv[i], "-m") == 0) {
      int mask;
      if (i + 1 >= argc)
        usage();
      mask = parsemask(argv[++i]);
      if (mask < 0) {
        printf("blackbox: unknown event type in \"%s\"\n", argv[i]);
        exit(1);
      }
      eventctl(mask);
      showmask(mask);
      printf("blackbox: event log cleared\n");
      exit(0);
    } else if (strcmp(argv[i], "-n") == 0) {
      if (i + 1 >= argc)
        usage();
      max = atoi(argv[++i]);
      if (max <= 0 || max > EVENT_BUFFER_SIZE)
        max = EVENT_BUFFER_SIZE;
    } else {
      usage();
    }
  }

  n = getevents(evbuf, max);
  if (n < 0) {
    printf("blackbox: getevents failed\n");
    exit(1);
  }

  printf("\n===== XV6 KERNEL EVENT RECORDER =====\n\n");

  if (n == 0) {
    printf("(no events recorded)\n\n");
    printf("=====================================\n");
    exit(0);
  }

  printf("   SEQ    TIME  CPU    PID  EVENT      DETAILS\n");
  printf("----------------------------------------------------------\n");

  for (i = 0; i < n; i++) {
    struct kernel_event *e = &evbuf[i];

    // A jump in seq means the ring wrapped before we read it.
    if (i > 0 && e->seq != lastseq + 1)
      gaps++;
    lastseq = e->seq;

    padnum(e->seq, 6);
    printf("  ");
    padnum(e->ticks, 6);
    printf("  ");
    padnum(e->cpu, 3);
    printf("  ");
    padnum(e->pid, 5);
    printf("  ");
    if (hasdetails(e)) {
      padstr(e->type > 0 && e->type < EV_NTYPES ? typename[e->type] : "?", 9);
      printf("  ");
      describe(e);
    } else {
      printf("%s", e->type > 0 && e->type < EV_NTYPES ? typename[e->type] : "?");
    }
    printf("\n");
  }

  printf("----------------------------------------------------------\n");
  printf("Total events shown: %d (seq %ld..%ld)\n", n, evbuf[0].seq,
         evbuf[n - 1].seq);
  if (gaps)
    printf("Note: %d sequence gap(s) -- older events were overwritten.\n",
           gaps);
  printf("=====================================\n");

  exit(0);
}
