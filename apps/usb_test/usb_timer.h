#ifndef __TIMER_H__
#define __TIMER_H__
#include "timer.h"
//-----------------------------------------------------------------
// Types
//-----------------------------------------------------------------
typedef unsigned long   t_time;

#define F_CPU 25000000 // 25MHz
#define ONE_MS (F_CPU*0.001)

// TODO: Implementation specific millisecond timer...
static t_time  timer_now(void) {  return 0; }

static void    timer_sleep(int timeMs)
{
  reset_timer();
  start_timer();
  while (get_time() < timeMs*ONE_MS);
  stop_timer();
}

#endif
