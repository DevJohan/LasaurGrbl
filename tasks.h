/*
  tasks.h - manage non-ISR tasks
  Part of LasaurGrbl

  Copyright (c) 2013 Richard Taylor

  LasaurGrbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  LasaurGrbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
*/


#ifndef tasks_h
#define tasks_h

#include <stdint.h>
#include "config.h"

// Available tasks.
// Uses 1 bit per task, so adjust task_t if you need > 16 tasks.
typedef enum {
	TASK_READY_WAIT = 0,
	TASK_SERIAL_RX,
	TASK_MANUAL_MOVE,
	TASK_SET_OFFSET,
	TASK_MOTOR_DELAY,
#ifdef ENABLE_LCD
	TASK_UPDATE_LCD,
#endif
	TASK_END,
} TASK;

typedef uint16_t	task_t;

// non-trivial data structures.
struct task_manual_move_data {
	double x_offset;
	double y_offset;
	double z_offset;
	double rate;
};


void tasks_init(void);
void tasks_loop(void);
void task_enable(TASK task, void* data);
void task_disable(TASK task);
uint8_t task_running(TASK task);

#endif
