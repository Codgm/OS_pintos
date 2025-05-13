#include "projects/automated_warehouse/aw_thread.h"
#include "threads/interrupt.h"
#include "threads/thread.h"
#include <list.h>

//
// You need to read carefully thread/synch.h and thread/synch.c
// 
// In the code, a fucntion named "sema_down" implements blocking thread and 
// makes list of blocking thread
// 
// And a function named "sema_up" implements unblocing thread using blocking list
//
// You must implement blocking list using "blocking_threads" in this code.
// Then you can also implement unblocking thread.
//


struct list blocked_threads;

int check_blocked_thread_full(int n) {
    int size = list_size(&blocked_threads);
    return size == n;
}

void aw_thread_init(void) {
    list_init(&blocked_threads);
}

/**
 * A function unblocking all blocked threads in "blocked_threads" 
 * It must be called by robot threads
 */

void block_thread(void){
    enum intr_level old_level;
    old_level = intr_disable ();

    list_push_back(&blocked_threads, &thread_current()->elem);
    thread_block ();

    intr_set_level (old_level);
}

/**
 * A function unblocking all blocked threads in "blocked_threads" 
 * It must be called by central control thread
 */

void unblock_threads(void){
    enum intr_level old_level;
    old_level = intr_disable ();

    /* 리스트가 빌 때까지 순회하며 언블록 */
    while (!list_empty(&blocked_threads)){
        struct thread *t = list_entry(list_pop_front(&blocked_threads), struct thread, elem);
        thread_unblock(t);
    }

    intr_set_level(old_level);
}
