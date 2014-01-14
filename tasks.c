/*
  tasks.c - manage non-ISR tasks
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
#include <inc/hw_ints.h>
#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <inc/hw_timer.h>
#include <inc/hw_gpio.h>

#include <driverlib/gpio.h>
#include <driverlib/sysctl.h>
#include <driverlib/timer.h>
#include <driverlib/interrupt.h>

#include "tasks.h"

#include "gcode.h"
#include "planner.h"
#include "serial.h"
#include "stepper.h"
#include "sense_control.h"
#include "lcd.h"
#include "joystick.h"


static volatile task_t task_status = 0;
static void *task_data[TASK_END];

static uint64_t timer_load;
uint32_t system_time_ms = 0;
extern uint8_t current_jog_z;

void gp_timer_isr(void) {
	TimerLoadSet64(GP_TIMER, timer_load);
	TimerIntClear(GP_TIMER, TIMER_TIMA_TIMEOUT);
	system_time_ms++;
}

void tasks_init(void) {
	task_status = 0;

	// Configure GP timer
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
	TimerConfigure(GP_TIMER, TIMER_CFG_PERIODIC);

	// Create a 1ms timer
	timer_load = SysCtlClockGet() / 1000;
	TimerLoadSet64(GP_TIMER, timer_load);
	TimerIntRegister(GP_TIMER, TIMER_A, gp_timer_isr);
	TimerIntEnable(GP_TIMER, TIMER_TIMA_TIMEOUT);
	IntPrioritySet(INT_TIMER4A, CONFIG_GPTIMER_PRIORITY);

	TimerEnable(GP_TIMER, TIMER_A);
}

void task_enable(TASK task, void* data) {
	task_status |= (1 << task);
	task_data[task] = data;
}

void task_disable(TASK task) {
	task_status &= ~(1 << task);
	task_data[task] = 0;
}

uint8_t task_running(TASK task) {
	if (task_status & (1 << task))
		return 1;
	return 0;
}

void tasks_loop(void) {
	uint8_t serial_active = 0;
	double last_x = 0;
	double last_y = 0;
	double last_z = 0;

	// Main task loop, does not return
	// None of the tasks should block, other than
	// when performing work.
	while (1) {

		// Wait for the machine to be ready (available blocks)
    	if (task_running(TASK_READY_WAIT)) {
    		if (planner_blocks_available() >= PLANNER_FIFO_READY_THRESHOLD)
			{
				char tmp[2] = {0x12, 0};
				printString(tmp);
				task_disable(TASK_READY_WAIT);
			}
    	}

		// Process any serial data available
    	if (task_running(TASK_SERIAL_RX)) {
			// Disable Joystick control whilst under Serial control
			joystick_disable();
			serial_active = 1;

    		if (gcode_process_data(task_data[TASK_SERIAL_RX]) == 0) {
    			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
        		task_disable(TASK_SERIAL_RX);

        		serial_active = 0;
    		}
    	}

		// Process manual moves
    	if (task_running(TASK_MANUAL_MOVE)) {
    		struct task_manual_move_data *move = task_data[TASK_MANUAL_MOVE];
    		if (planner_blocks_available() >= PLANNER_FIFO_READY_THRESHOLD) {
    			gcode_manual_move(move->x_offset, move->y_offset, move->z_offset, move->rate);
    			task_disable(TASK_MANUAL_MOVE);
    		}
    	}

		// Process offset set
    	if (task_running(TASK_SET_OFFSET)) {
			gcode_set_offset_to_current_position();
			task_disable(TASK_SET_OFFSET);
    	}

		// Z Motor Run
    	if (task_running(TASK_MOTOR_DELAY)) {
    		if (system_time_ms > (uint32_t)task_data[TASK_MOTOR_DELAY])
    		{
    			GPIOPinWrite(STEP_DIR_PORT, GPIO_PIN_5 | GPIO_PIN_7, 0);
    			task_disable(TASK_MOTOR_DELAY);
    		}
    	}

#ifdef ENABLE_LCD
    	// LCD Update
    	if (task_running(TASK_UPDATE_LCD)) {
    		if (system_time_ms % 500 == 0)
    		{
    			double x = stepper_get_position_x();
    			double y = stepper_get_position_y();
    			double z = stepper_get_position_z();

    			if (x != last_x || y != last_y || z != last_z) {
    				uint32_t power = control_get_intensity();
    				block_t *block = planner_get_current_block();
    				uint32_t ppi = 0;
    				last_x = x;
    				last_y = y;
    				last_z = z;

    				if (block) {
    					power = block->laser_pwm * 100 / 255;
    					ppi = block->laser_ppi;
    				}

    				lcd_clear();
        			lcd_setCursor(0, 0);
        	    	lcd_drawstring("LaserGRBL");
        	    	lcd_drawstring( current_jog_z & (1<<JOG_Z_UP_BIT) ? "1":"0");
        	    	lcd_drawstring( current_jog_z & (1<<JOG_Z_DOWN_BIT) ? "1":"0");
        	    	lcd_drawstring("\n");
        	    	lcd_drawstring("Power: ");
        	    	lcd_drawint(power);
        	    	lcd_drawstring("%\n");
        	    	lcd_drawstring("PPI: ");
        	    	lcd_drawint(ppi);
        	    	lcd_drawstring("\n");
        	    	lcd_drawstring("X: ");
        	    	lcd_drawfloat(x);
        	    	lcd_drawstring("\n");
        	    	lcd_drawstring("Y: ");
        	    	lcd_drawfloat(y);
        	    	lcd_drawstring("\n");
        	    	lcd_drawstring("Z: ");
        	    	lcd_drawfloat(z);
        	    	lcd_drawstring("\n");
        	    	lcd_display();
    			}
    		}
    	}
#endif // ENABLE_LCD

    	if (serial_active == 0 && !stepper_active()) {
			// Allow Joystick control
			joystick_enable();
    	}
	}
}

