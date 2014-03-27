/*
  config.h - compile time configuration
  Part of LasaurGrbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011 Sungeun K. Jeon
  Copyright (c) 2011 Stefan Hechenberger

  LasaurGrbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  LasaurGrbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
*/

#ifndef config_h
#define config_h

#include <stdint.h>


// Version number
// (must not contain capital letters)
#define LASAURGRBL_VERSION "13.04"
//#define DEBUG_IGNORE_SENSORS  // set for debugging

// Whether or not to drive an LCD.
#define ENABLE_LCD


//??Whether or not to use a DC-motor for Z-axis??
//#define MOTOR_Z

// This defines the maximum number of dots in a raster.
#define RASTER_BUFFER_SIZE	1024

#define CONFIG_X_STEPS_PER_MM 157.48 //microsteps/mm
#define CONFIG_Y_STEPS_PER_MM 157.48 //microsteps/mm
#define CONFIG_Z_STEPS_PER_MM 1500.0 //milliseconds/mm
#define CONFIG_PULSE_MICROSECONDS 10
#define CONFIG_DEFAULT_RATE 8000.0
#define CONFIG_MAX_FEEDRATE 35000.0 // in millimeters per minute
#define CONFIG_MAX_SEEKRATE 35000.0
#define CONFIG_DEFAULT_ACCELERATION 8000000.0 // mm/min^2, typically 1000000-8000000, divide by (60*60) to get mm/sec^2
#define CONFIG_JUNCTION_DEVIATION 0.006 // mm
#define CONFIG_X_ORIGIN_OFFSET 0.0  // mm, x-offset of table origin from physical home
#define CONFIG_Y_ORIGIN_OFFSET 0.0  // mm, y-offset of table origin from physical home
#define CONFIG_Z_ORIGIN_OFFSET 0.0   // mm, z-offset of table origin from physical home
#define CONFIG_X_MIN 0.0
#define CONFIG_X_MAX 340.0
#define CONFIG_Y_MIN 0.0
#define CONFIG_Y_MAX 225.0
#define CONFIG_Z_MIN 0.0
#define CONFIG_Z_MAX 50.0

#define CONFIG_INVERT_X_AXIS 1  // 0 is regular, 1 inverts the x direction
#define CONFIG_INVERT_Y_AXIS 1  // 0 is regular, 1 inverts the y direction
#define CONFIG_INVERT_Z_AXIS 0  // 0 is regular, 1 inverts the z direction

#define CONFIG_LASER_PWM_FREQ			40000
#define CONFIG_LASER_PPI_PULSE_MS		4

// This will use a timer to guarantee a step pulse length.
// However the maximum seek rate will be limited (approx. 15000mm/min)
// Disabling this define will pulse for the duration of the stepper ISR (< 1us)
// This works with Pololu drivers, and achieves >=30000mm/min.
//#define CONFIG_STEPPER_USE_PULSE_TIMER

// Interrupt Priorities (0 highest)
#define	CONFIG_STEPPER_PRIORITY		(0 << 5)
#define	CONFIG_LASER_PRIORITY		(1 << 5)
#define	CONFIG_USB_PRIORITY			(2 << 5)
#define	CONFIG_SENSE_PRIORITY		(3 << 5)
#define	CONFIG_JOY_PRIORITY			(4 << 5)
#define CONFIG_I2C_PRIORITY			(5 << 5)
#define	CONFIG_GPTIMER_PRIORITY		(7 << 5)

#define JOY_PORT              	GPIO_PORTF_BASE
#define JOY_BIT                	0
#define JOY_MASK 				(1<<JOY_BIT)
#define JOY_TIMER				TIMER3_BASE

#define JOY_INVERT_Y
//#define JOY_INVERT_X

//#define JOG_Z_PORT              GPIO_PORTA_BASE
//#define JOG_Z_UP_BIT            6
//#define JOG_Z_DOWN_BIT 			7
//#define JOG_Z_MASK 				(1<<JOG_Z_UP_BIT) | (1<<JOG_Z_DOWN_BIT)


#define SENSE_PORT              GPIO_PORTE_BASE
#define DOOR_BIT                1
#define SENSE_MASK 				(1<<DOOR_BIT)

#define SENSE_TIMER				TIMER2_BASE

#define GP_TIMER				TIMER4_BASE

#define LASER_SAFETY_PORT		GPIO_PORTA_BASE
#define LASER_SAFETY_I2C		1
#define LASER_SAFETY_SCL  		6
#define LASER_SAFETY_SDA  		7
#define LASER_SAFETY_MASK  		( 1<<LASER_SAFETY_SCL ) | ( 1<<LASER_SAFETY_SDA )

#define ASSIST_PORT           	GPIO_PORTD_BASE
#define AIR_ASSIST_BIT        	2
#define AUX1_ASSIST_BIT       	6
#define ASSIST_MASK				(1 << AIR_ASSIST_BIT) | (1<< AUX1_ASSIST_BIT)
  
#define LIMIT_PORT              GPIO_PORTC_BASE
#define X_LIMIT_BIT             4
#define Y_LIMIT_BIT             5
#define Z_LIMIT_BIT             6
#define E_LIMIT_BIT             7
#define LIMIT_MASK 				((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)|(1<<E_LIMIT_BIT))


#define STEPPING_TIMER			TIMER1_BASE

#define STEP_EN_PORT         	GPIO_PORTB_BASE
#define STEP_EN          		0
#define STEP_EN_MASK			(1 << STEP_EN)
#define STEP_EN_INVERT			(1 << STEP_EN)

#define STEP_DIR_PORT         	GPIO_PORTB_BASE
#define STEP_X_DIR           	3
#define STEP_Y_DIR           	4

#ifndef MOTOR_Z
#define STEP_Z_DIR           	5
#define STEP_DIR_MASK			((1 << STEP_X_DIR) | (1 << STEP_Y_DIR) | (1 << STEP_Z_DIR))
#define STEP_DIR_INVERT			((CONFIG_INVERT_X_AXIS<<STEP_X_DIR)|(CONFIG_INVERT_Y_AXIS<<STEP_Y_DIR)|(CONFIG_INVERT_Z_AXIS<<STEP_Z_DIR))
#else
#define STEP_DIR_MASK			((1 << STEP_X_DIR) | (1 << STEP_Y_DIR) /*| (1 << STEP_Z_DIR)*/)
#define STEP_DIR_INVERT			((CONFIG_INVERT_X_AXIS<<STEP_X_DIR)|(CONFIG_INVERT_Y_AXIS<<STEP_Y_DIR)/*|(CONFIG_INVERT_Z_AXIS<<STEP_Z_DIR)*/)
#define	STEP_Z_MASK				((1 << 5) | (1 << 7))
#define	STEP_Z_UP				(1 << 7)
#define	STEP_Z_DOWN				(1 << 5)
#endif

#define STEP_PORT           	GPIO_PORTE_BASE
#define STEP_X_BIT              2
#define STEP_Y_BIT              3
#define STEP_Z_BIT              4
#define STEP_MASK				((1 << STEP_X_BIT) | (1 << STEP_Y_BIT) | (1 << STEP_Z_BIT))

#define LASER_PORT           	GPIO_PORTB_BASE
#define LASER_BIT        		6
#define LASER_TIMER				TIMER0_BASE

#define LASER_EN_PORT         	GPIO_PORTB_BASE
#define LASER_EN_BIT       		1
#define LASER_EN_MASK			(1 << LASER_EN_BIT)
#define LASER_EN_INVERT			0

// The temporal resolution of the acceleration management subsystem. Higher number give smoother
// acceleration but may impact performance.
// NOTE: Increasing this parameter will help any resolution related issues, especially with machines 
// requiring very high accelerations and/or very fast feedrates. In general, this will reduce the 
// error between how the planner plans the motions and how the stepper program actually performs them.
// However, at some point, the resolution can be high enough, where the errors related to numerical 
// round-off can be great enough to cause problems and/or it's too fast for the Arduino. The correct
// value for this parameter is machine dependent, so it's advised to set this only as high as needed.
// Approximate successful values can range from 30L to 100L or more.
#define ACCELERATION_TICKS_PER_SECOND 400L

// Minimum planner junction speed. Sets the default minimum speed the planner plans for at the end
// of the buffer and all stops. This should not be much greater than zero and should only be changed
// if unwanted behavior is observed on a user's machine when running at very slow speeds.
#define ZERO_SPEED 0.0 // (mm/min)

// Minimum stepper rate. Sets the absolute minimum stepper rate in the stepper program and never runs
// slower than this value, except when sleeping. This parameter overrides the minimum planner speed.
// This is primarily used to guarantee that the end of a movement is always reached and not stop to
// never reach its target. This parameter should always be greater than zero.
#define MINIMUM_STEPS_PER_MINUTE 1600U // (steps/min) - Integer value only
// 1600 @ 32step_per_mm = 50mm/min
  

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2


#define clear_vector(a) memset(a, 0, sizeof(a))
#define clear_vector_double(a) memset(a, 0.0, sizeof(a))
#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))

#define MM_PER_INCH (25.4)

#define M_PI 3.14159

// Arc interpretation settings:
#define MM_PER_ARC_SEGMENT 1
#define N_ARC_CORRECTION 25

#endif

// bit math
// see: http://www.arduino.cc/playground/Code/BitMath
// see: http://graphics.stanford.edu/~seander/bithacks.html
//
// y = (x >> n) & 1; // n=0..15. stores nth bit of x in y. y becomes 0 or 1.
//
// x &= ~(1 << n); // forces nth bit of x to be 0. all other bits left alone.
//
// x &= (1<<(n+1))-1; // leaves alone the lowest n bits of x; all higher bits set to 0.
//
// x |= (1 << n); // forces nth bit of x to be 1. all other bits left alone.
//
// x ^= (1 << n); // toggles nth bit of x. all other bits left alone.
//
// x = ~x; // toggles ALL the bits in x.

extern void __delay_us(uint32_t delay);
extern uint32_t system_time_ms;
