/*
 * Copyright (C) 2017 Unwired Devices LLC
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     test
 * @{
 *
 * @file
 * @brief       minimal RIOT application to test RTC subsecond timers
 *
 * @author      Oleg Artamonov
 *
 * @}
 */

#include <stdio.h>

#include "periph/pm.h"
#include "thread.h"
#include "lptimer.h"
#include "xtimer.h"
#include "board.h"

static msg_t timer1_msg;
static kernel_pid_t timer1_pid;
static lptimer_t timer1;
static volatile uint32_t timer1_prev;
char stack1[2048];
#define TIMER1_PERIOD 1000000

static void *timer1_thread(void *arg) {
    (void)arg;
    msg_t msg;
    msg_t msg_queue[4];
    msg_init_queue(msg_queue, 4);

    puts("Timer1 thread started");

    while (1) {
        msg_receive(&msg);
        
        volatile uint32_t now = xtimer_now_usec()/1000;
        printf("Timer1: %" PRIu32 " ms\n", now - timer1_prev);
        timer1_prev = now;
        
        lptimer_set_msg(&timer1, TIMER1_PERIOD, &timer1_msg, timer1_pid);
    }
    return NULL;
}
/*
static msg_t timer2_msg;
static kernel_pid_t timer2_pid;
static lptimer_t timer2;
static volatile uint32_t timer2_prev;
char stack2[2048];
#define TIMER2_PERIOD 800000

static void *timer2_thread(void *arg) {
    msg_t msg;
    msg_t msg_queue[4];
    msg_init_queue(msg_queue, 4);

    puts("Timer2 thread started");

    while (1) {
        msg_receive(&msg);
        
        volatile uint32_t now = xtimer_now_usec()/1000;
        printf("Timer2: %" PRIu32 " ms\n", now - timer2_prev);
        timer2_prev = now;
        
        lptimer_set_msg(&timer2, TIMER2_PERIOD, &timer2_msg, timer2_pid);
    }
    return NULL;
}
*/
int main(void)
{
    puts("rtctimers-millis test");
    
    pm_block(PM_SLEEP);
    
	lptimer_init();
	xtimer_init();
    
    uint32_t now = xtimer_now_usec()/1000;
    
    timer1_prev = now;
    puts("Creating timer1 thread");
    timer1_pid = thread_create(stack1, 2048, THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST, timer1_thread, NULL, "Timer 1");
    lptimer_set_msg(&timer1, TIMER1_PERIOD, &timer1_msg, timer1_pid);

/*    
    timer2_prev = now;
    puts("Creating timer2 thread");
    timer2_pid = thread_create(stack2, 2048, THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST, timer2_thread, NULL, "Timer 2");
    lptimer_set_msg(&timer2, TIMER2_PERIOD, &timer2_msg, timer2_pid);
*/
    /* lptimer_sleep(300000); */
    
    puts("Done.");
    
    while (1) {};

    return 0;
}
