#ifndef _PROJECTS_PROJECT1_AW_THREAD_H__
#define _PROJECTS_PROJECT1_AW_THREAD_H__

#include "threads/interrupt.h"
#include "threads/synch.h"
#include "threads/thread.h"

extern struct list blocked_threads;

void aw_thread_init(void);

void block_thread(void);

void unblock_threads(void);

int check_blocked_thread_full(int n);

#endif
