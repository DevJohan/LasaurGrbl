/*
  stepper.c - stepper motor pulse generation
  Processes block from the queue generated by the planer and pulses
  steppers accordingly via a dynamically adapted timer interrupt.
  Part of LasaurGrbl

  Copyright (c) 2011 Stefan Hechenberger
  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011 Sungeun K. Jeon
  
  Inspired by the 'RepRap cartesian firmware' by Zack Smith and 
  Philipp Tiefenbacher.

  LasaurGrbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  LasaurGrbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  ---
  
           __________________________
          /|                        |\     _________________         ^
         / |                        | \   /|               |\        |
        /  |                        |  \ / |               | \       s
       /   |                        |   |  |               |  \      p
      /    |                        |   |  |               |   \     e
     +-----+------------------------+---+--+---------------+----+    e
     |               BLOCK 1            |      BLOCK 2          |    d
  
                             time ----->

  The speed profile starts at block->initial_rate, accelerates by block->rate_delta
  during the first block->accelerate_until step_events_completed, then keeps going at constant speed until
  step_events_completed reaches block->decelerate_after after which it decelerates until final_rate is reached.
  The slope of acceleration is always +/- block->rate_delta and is applied at a constant rate following the midpoint rule.
  Speed adjustments are made ACCELERATION_TICKS_PER_SECOND times per second.  
*/

#define __DELAY_BACKWARD_COMPATIBLE__  // _delay_us() make backward compatible see delay.h

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <inc/hw_timer.h>
#include <inc/hw_ints.h>
#include <inc/hw_gpio.h>

#include <driverlib/gpio.h>
#include "driverlib/rom.h"
#include <driverlib/sysctl.h>
#include <driverlib/timer.h>

#include "config.h"
#include "stepper.h"
#include "gcode.h"
#include "planner.h"
#include "sense_control.h"
#include "temperature.h"


#define CYCLES_PER_MICROSECOND (SysCtlClockGet()/1000000)  // 80MHz = 80
#define CYCLES_PER_ACCELERATION_TICK (SysCtlClockGet()/ACCELERATION_TICKS_PER_SECOND)  // 80MHz/100 = 800000

typedef enum
{
	STEP_AXIS_X = 0,
	STEP_AXIS_Y = 1,
	STEP_AXIS_Z = 2,
	STEP_NUM_AXIS
} STEP_AXIS;

static int64_t stepper_position[STEP_NUM_AXIS];  // real-time position in absolute steps
static block_t *current_block;  // A pointer to the block currently being traced

// Variables used by The Stepper Driver Interrupt
static uint8_t out_dir_bits;      // The next direction-bits to be output
static uint8_t out_step_bits;     // The next stepping-bits to be output

static int32_t counter_x,       // Counter variables for the bresenham line tracer
               counter_y,
               counter_z;
static uint32_t step_events_completed; // The number of step events executed in the current block
static volatile uint8_t busy;  // true whe stepper ISR is in already running

// Variables used by the trapezoid generation
static uint32_t cycles_per_step_event;        // The number of machine cycles between each step event
static uint32_t acceleration_tick_counter;    // The cycles since last acceleration_tick.
                                              // Used to generate ticks at a steady pace without allocating a separate timer.
static uint32_t adjusted_rate;                // The current rate of step_events according to the speed profile
static bool processing_flag;                  // indicates if blocks are being processed
static volatile bool stop_requested;          // when set to true stepper interrupt will go idle on next entry
static volatile uint8_t stop_status;          // yields the reason for a stop request

static uint16_t timer_prescaler = 0;
static uint16_t timer_preload = 0xffff;
static uint8_t pulse_active = 0;


// prototypes for static functions (non-accesible from other files)
static bool acceleration_tick();
static void adjust_speed( uint32_t steps_per_minute );
static uint32_t config_step_timer(uint32_t cycles);

void pulse_isr(void);
void stepper_isr(void);


// Initialize and start the stepper motor subsystem
void stepper_init() {  
	// Configure directions of interface pins
	GPIOPinTypeGPIOOutput(STEP_EN_PORT, STEP_EN_MASK);
	GPIOPinTypeGPIOOutput(STEP_DIR_PORT, STEP_DIR_MASK);
	GPIOPinTypeGPIOOutput(STEP_PORT, STEP_MASK);
	GPIOPinTypeGPIOOutput(STEP_PORT, STEP_MASK);
	GPIOPinTypeGPIOOutput(STATUS_PORT, STATUS_MASK);

	GPIOPadConfigSet(STEP_PORT, STEP_MASK, GPIO_STRENGTH_8MA_SC, GPIO_PIN_TYPE_STD);
	GPIOPadConfigSet(STEP_DIR_PORT, STEP_DIR_MASK, GPIO_STRENGTH_8MA_SC, GPIO_PIN_TYPE_STD);

	GPIOPinWrite(STEP_PORT, STEP_MASK, 0);
	GPIOPinWrite(STEP_DIR_PORT, STEP_DIR_MASK, STEP_DIR_INVERT);
	GPIOPinWrite(STEP_EN_PORT, STEP_EN_MASK, STEP_EN_MASK ^ STEP_EN_INVERT);

	// Configure timer
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	TimerConfigure(STEPPING_TIMER, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC | TIMER_CFG_B_ONE_SHOT);

	TimerIntRegister(STEPPING_TIMER, TIMER_A, stepper_isr);
	TimerIntRegister(STEPPING_TIMER, TIMER_B, pulse_isr);

	ROM_IntEnable(INT_TIMER1A);
	ROM_IntEnable(INT_TIMER1B);
	TimerIntEnable(STEPPING_TIMER, TIMER_TIMA_TIMEOUT);
	TimerIntEnable(STEPPING_TIMER, TIMER_TIMB_TIMEOUT);

	adjust_speed(MINIMUM_STEPS_PER_MINUTE);
	clear_vector(stepper_position);
	stepper_set_position( CONFIG_X_ORIGIN_OFFSET,
                          CONFIG_Y_ORIGIN_OFFSET,
                          CONFIG_Z_ORIGIN_OFFSET );

	acceleration_tick_counter = 0;
	current_block = NULL;
	stop_requested = false;
	stop_status = GCODE_STATUS_OK;
	busy = false;
  
	// start in the idle state
	// The stepper interrupt gets started when blocks are being added.
	stepper_go_idle();

	// Go Home
	gcode_do_home();
}


// block until all command blocks are executed
void stepper_synchronize() {
  while(processing_flag) { 
    // sleep_mode();
  }
}


// start processing command blocks
void stepper_wake_up() {
  if (!processing_flag) {
    processing_flag = true;

    // Initialize stepper output bits
	GPIOPinWrite(STEP_PORT, STEP_MASK, 0);
	GPIOPinWrite(STEP_DIR_PORT, STEP_DIR_MASK, STEP_DIR_INVERT);
	GPIOPinWrite(STEP_EN_PORT, STEP_EN_MASK, STEP_EN_MASK ^ STEP_EN_INVERT);

    // Enable stepper driver interrupt
    TimerEnable(STEPPING_TIMER, TIMER_A);

    // Turn off Green LED
    GPIOPinWrite(STATUS_PORT, STATUS_MASK, STATUS_INVERT);
  }
}


// stop processing command blocks
void stepper_go_idle() {
  processing_flag = false;
  current_block = NULL;
  // Disable stepper driver interrupt
  TimerDisable(STEPPING_TIMER, TIMER_A);
  control_laser_intensity(0);

  // Turn on Green LED
  GPIOPinWrite(STATUS_PORT, STATUS_MASK, STATUS_MASK ^ STATUS_INVERT);
}

// stop event handling
void stepper_request_stop(uint8_t status) {
  stop_status = status;
  stop_requested = true;
  GPIOPinWrite(STEP_EN_PORT, STEP_EN_MASK, STEP_EN_INVERT);
}

uint8_t stepper_stop_status() {
  return stop_status;
}

bool stepper_stop_requested() {
  return stop_requested;
}

void stepper_stop_resume() {
  stop_status = 0;
  stop_requested = false;
  GPIOPinWrite(STEP_EN_PORT, STEP_EN_MASK, STEP_EN_MASK ^ STEP_EN_INVERT);
}




double stepper_get_position_x() {
  return stepper_position[X_AXIS]/CONFIG_X_STEPS_PER_MM;
}
double stepper_get_position_y() {
  return stepper_position[Y_AXIS]/CONFIG_Y_STEPS_PER_MM;
}
double stepper_get_position_z() {
  return stepper_position[Z_AXIS]/CONFIG_Z_STEPS_PER_MM;
}
void stepper_set_position(double x, double y, double z) {
  stepper_synchronize();  // wait until processing is done
  stepper_position[X_AXIS] = floor(x*CONFIG_X_STEPS_PER_MM + 0.5);
  stepper_position[Y_AXIS] = floor(y*CONFIG_Y_STEPS_PER_MM + 0.5);
  stepper_position[Z_AXIS] = floor(z*CONFIG_Z_STEPS_PER_MM + 0.5);  
}



// The Stepper Reset ISR
// It resets the motor port after a short period completing one step cycle.
// TODO: It is possible for the serial interrupts to delay this interrupt by a few microseconds, if
// they execute right before this interrupt. Not a big deal, but could use some TLC at some point.
void pulse_isr (void) {
	// reset step pins
	GPIOPinWrite(STEP_PORT, STEP_MASK, 0);

	// This is a one-shot timer, so just ACK the IRQ.
	TimerIntClear(TIMER1_BASE, TIMER_TIMB_TIMEOUT);

	pulse_active = 0;
}
  

// The Stepper ISR
// This is the workhorse of LasaurGrbl. It is executed at the rate set with
// config_step_timer. It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately.
// The bresenham line tracer algorithm controls all three stepper outputs simultaneously.
void stepper_isr (void) {
	uint32_t raster_index;
	uint8_t intensity;

	if (busy) { return; } // The busy-flag is used to avoid reentering this interrupt

	// Reset the timer
	TimerLoadSet(STEPPING_TIMER, TIMER_A, timer_preload);
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

	if (pulse_active) { return; }	// If the step pulse hasn't completed yet, try again later.

	busy = true;
  if (stop_requested) {
    // go idle and absorb any blocks
    stepper_go_idle(); 
    planner_reset_block_buffer();
    planner_request_position_update();
    gcode_request_position_update();
    busy = false;
    return;
  }

  #ifndef DEBUG_IGNORE_SENSORS
	// stop program when any limit is hit or the e-stop turned the power off
	if (SENSE_LIMITS) {
		stepper_request_stop(GCODE_STATUS_LIMIT_HIT);
		busy = false;
		return;
	}

    // Pause if we have a (transient) safety issue, will be abrupt and may skip steps...
    if (SENSE_SAFETY) {
    	// Turn off the laser
    	control_laser_intensity(0);
    	// Make sure that the rate (laser power) will be set when we resume
    	adjusted_rate = 0;
    	busy = false;
    	return;
    }
  #endif
  
	// pulse steppers
	GPIOPinWrite(STEP_DIR_PORT, STEP_DIR_MASK, out_dir_bits);
	GPIOPinWrite(STEP_PORT, STEP_MASK, out_step_bits);
	out_step_bits = 0;
	pulse_active = 1;
	// prime for reset pulse in CONFIG_PULSE_MICROSECONDS
	//set period

	TimerPrescaleSet(STEPPING_TIMER, TIMER_B, 0);
	TimerLoadSet(STEPPING_TIMER, TIMER_B, (CONFIG_PULSE_MICROSECONDS - 3) * CYCLES_PER_MICROSECOND);
	TimerEnable(STEPPING_TIMER, TIMER_B);


  // If there is no current block, attempt to pop one from the buffer
  if (current_block == NULL) {
    // Anything in the buffer?
    current_block = planner_get_current_block();
    // if still no block command, go idle, disable interrupt
    if (current_block == NULL) {
      stepper_go_idle();
      busy = false;
      return;       
    }      
    if (current_block->block_type == BLOCK_TYPE_LINE
    	|| current_block->block_type == BLOCK_TYPE_RASTER_LINE) {  // starting on new line block
      adjusted_rate = current_block->initial_rate;
      acceleration_tick_counter = CYCLES_PER_ACCELERATION_TICK/2; // start halfway, midpoint rule.
      adjust_speed( adjusted_rate ); // initialize cycles_per_step_event
      counter_x = -(current_block->step_event_count >> 1);
      counter_y = counter_x;
      counter_z = counter_x;
      step_events_completed = 0;
    }
  }

  // process current block, populate out_bits (or handle other commands)
  switch (current_block->block_type) {
  	case BLOCK_TYPE_RASTER_LINE:
 	  raster_index = (step_events_completed * current_block->raster_buffer_len) / current_block->steps_x;
 	  intensity = (current_block->raster_buffer[raster_index] == '1')?current_block->raster_intensity:0;
 	  if (intensity != current_block->nominal_laser_intensity) {
 		  current_block->nominal_laser_intensity = intensity;
 		  adjusted_rate = 0;
 	  }
    case BLOCK_TYPE_LINE:
      ////// Execute step displacement profile by bresenham line algorithm
      out_dir_bits = current_block->direction_bits;
      counter_x += current_block->steps_x;
      if (counter_x > 0) {
        out_step_bits |= (1<<STEP_X_BIT);
        counter_x -= current_block->step_event_count;
        // also keep track of absolute position
        if ((out_dir_bits >> STEP_X_DIR) & 1 ) {
          stepper_position[X_AXIS] -= 1;
        } else {
          stepper_position[X_AXIS] += 1;
        }        
      }
      counter_y += current_block->steps_y;
      if (counter_y > 0) {
        out_step_bits |= (1<<STEP_Y_BIT);
        counter_y -= current_block->step_event_count;
        // also keep track of absolute position
        if ((out_dir_bits >> STEP_Y_DIR) & 1 ) {
          stepper_position[Y_AXIS] -= 1;
        } else {
          stepper_position[Y_AXIS] += 1;
        }        
      }
      counter_z += current_block->steps_z;
      if (counter_z > 0) {
        out_step_bits |= (1<<STEP_Z_BIT);
        counter_z -= current_block->step_event_count;
        // also keep track of absolute position        
        if ((out_step_bits >> STEP_Z_DIR) & 1 ) {
          stepper_position[Z_AXIS] -= 1;
        } else {
          stepper_position[Z_AXIS] += 1;
        }        
      }
      //////
      
      step_events_completed++;  // increment step count
      
      // apply stepper invert mask
      out_dir_bits ^= STEP_DIR_INVERT;

      ////////// SPEED ADJUSTMENT
      if (step_events_completed < current_block->step_event_count) {  // block not finished
      
        // accelerating
        if (step_events_completed < current_block->accelerate_until) {
          if ( acceleration_tick() ) {  // scheduled speed change
            adjusted_rate += current_block->rate_delta;
            if (adjusted_rate > current_block->nominal_rate) {  // overshot
              adjusted_rate = current_block->nominal_rate;
            }
            adjust_speed( adjusted_rate );
          }
        
        // deceleration start
        } else if (step_events_completed == current_block->decelerate_after) {
            // reset counter, midpoint rule
            // makes sure deceleration is performed the same every time
            acceleration_tick_counter = CYCLES_PER_ACCELERATION_TICK/2;

        // decelerating
        } else if (step_events_completed >= current_block->decelerate_after) {
          if ( acceleration_tick() ) {  // scheduled speed change
        	  if (adjusted_rate > current_block->rate_delta)
        		  adjusted_rate -= current_block->rate_delta;
        	  else
        		  adjusted_rate = 0;
            if (adjusted_rate < current_block->final_rate) {  // overshot
              adjusted_rate = current_block->final_rate;
            }
            adjust_speed( adjusted_rate );
          }
        
        // cruising
        } else {
          // No accelerations. Make sure we cruise exactly at the nominal rate.
          if (adjusted_rate != current_block->nominal_rate) {
            adjusted_rate = current_block->nominal_rate;
            adjust_speed( adjusted_rate );
          }
        }
      } else {  // block finished
        current_block = NULL;
        planner_discard_current_block();
      }
      ////////// END OF SPEED ADJUSTMENT
    
      break; 

    case BLOCK_TYPE_AIR_ASSIST_ENABLE:
      control_air_assist(true);
      current_block = NULL;
      planner_discard_current_block();  
      break;

    case BLOCK_TYPE_AIR_ASSIST_DISABLE:
      control_air_assist(false);
      current_block = NULL;
      planner_discard_current_block();  
      break;

    case BLOCK_TYPE_AUX1_ASSIST_ENABLE:
      control_aux1_assist(true);
      current_block = NULL;
      planner_discard_current_block();  
      break;

    case BLOCK_TYPE_AUX1_ASSIST_DISABLE:
      control_aux1_assist(false);
      current_block = NULL;
      planner_discard_current_block();  
      break;    
  }

  //GPIOPinWrite(STEP_PORT, STEP_MASK, 0);
  busy = false;
}




// This function determines an acceleration velocity change every CYCLES_PER_ACCELERATION_TICK by
// keeping track of the number of elapsed cycles during a de/ac-celeration. The code assumes that
// step_events occur significantly more often than the acceleration velocity iterations.
static bool acceleration_tick() {
  acceleration_tick_counter += cycles_per_step_event;
  if(acceleration_tick_counter > CYCLES_PER_ACCELERATION_TICK) {
    acceleration_tick_counter -= CYCLES_PER_ACCELERATION_TICK;
    return true;
  } else {
    return false;
  }
}


// Configures the prescaler and ceiling of timer 1 to produce the given rate as accurately as possible.
// Returns the actual number of cycles per interrupt.
static uint32_t config_step_timer(uint32_t cycles) {
	uint32_t prescaled_cycles = cycles;

	// Temporarily disable the timer
	ROM_IntDisable(INT_TIMER1A);

	timer_prescaler = 0;

	while (prescaled_cycles > 65535)
	{
		timer_prescaler++;
		prescaled_cycles /= 2;
	}

	timer_preload = prescaled_cycles;

	//set period
	TimerPrescaleSet(STEPPING_TIMER, TIMER_A, timer_prescaler);
	TimerLoadSet(STEPPING_TIMER, TIMER_A, timer_preload);

	// Re-Enable the Timer
	ROM_IntEnable(INT_TIMER1A);

	return(timer_preload * (1 + timer_prescaler));
}


static void adjust_speed( uint32_t steps_per_minute ) {
  // steps_per_minute is typicaly just adjusted_rate
  if (steps_per_minute < MINIMUM_STEPS_PER_MINUTE) { steps_per_minute = MINIMUM_STEPS_PER_MINUTE; }
  cycles_per_step_event = config_step_timer( (CYCLES_PER_MICROSECOND*1000000/steps_per_minute) * 60 );

  if (cycles_per_step_event == 0)
  {
	  stepper_request_stop(GCODE_STATUS_LIMIT_HIT);
  }

  // This can be called from init, so make sure we don't dereference a NULL block.
  if (current_block != NULL)
  {
	  // beam dynamics
	  uint8_t adjusted_intensity = current_block->nominal_laser_intensity *
								   ((float)steps_per_minute/(float)current_block->nominal_rate);
	  uint8_t constrained_intensity = max(adjusted_intensity, 0);
	  control_laser_intensity(constrained_intensity);
  }
}





static void homing_cycle(bool x_axis, bool y_axis, bool z_axis, bool reverse_direction, uint32_t microseconds_per_pulse) {
  
  uint32_t step_delay = microseconds_per_pulse - CONFIG_PULSE_MICROSECONDS;
  uint8_t out_dir_bits = STEP_DIR_MASK;
  uint8_t limit_bits;
  uint8_t x_overshoot_count = 12;
  uint8_t y_overshoot_count = 12;
  uint8_t z_overshoot_count = 12;

  if (x_axis) { out_step_bits |= (1<<STEP_X_BIT); }
  if (y_axis) { out_step_bits |= (1<<STEP_Y_BIT); }
  if (z_axis) { out_step_bits |= (1<<STEP_Z_BIT); }
  
  // Invert direction bits if this is a reverse homing_cycle
  if (reverse_direction) {
    out_dir_bits ^= STEP_DIR_MASK;
  }
  
  // Apply the global invert mask
  out_dir_bits ^= STEP_DIR_INVERT;
  
  // Set direction pins
	GPIOPinWrite(STEP_DIR_PORT, STEP_DIR_MASK, out_dir_bits);
  
  for(;;) {
    limit_bits = GPIOPinRead(LIMIT_PORT, LIMIT_MASK);
    if (reverse_direction) {         
      // Invert limit_bits if this is a reverse homing_cycle
      limit_bits ^= LIMIT_MASK;
    }
    if (x_axis && !(limit_bits & (1<<X_LIMIT_BIT))) {
      if(x_overshoot_count == 0) {
        x_axis = false;
        out_step_bits ^= (1<<STEP_X_BIT);
      } else {
        x_overshoot_count--;
      }     
    } 
    if (y_axis && !(limit_bits & (1<<Y_LIMIT_BIT))) {
      if(y_overshoot_count == 0) {
        y_axis = false;
        out_step_bits ^= (1<<STEP_Y_BIT);
      } else {
        y_overshoot_count--;
      }        
    }
    if (z_axis && !(limit_bits & (1<<Z_LIMIT_BIT))) {
      if(z_overshoot_count == 0) {
        z_axis = false;
        out_step_bits ^= (1<<STEP_Z_BIT);
      } else {
        z_overshoot_count--;
      }
    }
    if(x_axis || y_axis || z_axis) {
        // step all axes still in out_bits
    	GPIOPinWrite(STEP_PORT, STEP_MASK, out_step_bits);
        __delay_us(CONFIG_PULSE_MICROSECONDS);
    	GPIOPinWrite(STEP_PORT, STEP_MASK, 0);
        __delay_us(step_delay);
    } else { 
        break;
    }
  }
  clear_vector(stepper_position);
  return;
}

static void approach_limit_switch(bool x, bool y, bool z) {
  homing_cycle(x, y, z, false, 50);
}

static void leave_limit_switch(bool x, bool y, bool z) {
  homing_cycle(x, y, z, true, 50);
}

void stepper_homing_cycle() {
  stepper_synchronize();  
  // home the x and y axis
  approach_limit_switch(true, true, false);
  leave_limit_switch(true, true, false);
}


uint8_t stepper_active(void) {
	return processing_flag;
}
