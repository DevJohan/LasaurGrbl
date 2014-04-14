/*
  sense_control.h - sensing and controlling inputs and outputs
  Part of LasaurGrbl

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

#include <math.h>
#include <stdlib.h>

#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <inc/hw_timer.h>
#include <inc/hw_gpio.h>
#include <inc/hw_ints.h>

#include <driverlib/gpio.h>
#include <driverlib/sysctl.h>
#include <driverlib/timer.h>
#include <driverlib/rom.h>
#include <driverlib/interrupt.h>

#include "config.h"

#include "sense_control.h"
#include "stepper.h"
#include "planner.h"

static uint32_t laser_cycles;
static uint32_t laser_divider;

static uint32_t ppi_cycles;
static uint32_t ppi_divider;

static uint8_t laser_intensity = 0;

// Laser pulse one-shot timer.
static void laser_isr(void) {
	TimerIntClear(LASER_TIMER, TIMER_TIMB_TIMEOUT);

	// Turn off the Laser
	GPIOPinWrite(LASER_EN_PORT, LASER_EN_MASK, LASER_EN_INVERT);
}


void sense_init() {
	//// chiller, door, (power)
	GPIOPinTypeGPIOInput(SENSE_PORT, SENSE_MASK);
	GPIOPadConfigSet(SENSE_PORT, SENSE_MASK, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);

	//// x1_lmit, x2_limit, y1_limit, y2_limit, z1_limit, z2_limit
	GPIOPinTypeGPIOInput(LIMIT_PORT, LIMIT_MASK);
	GPIOPadConfigSet(LIMIT_PORT, LIMIT_MASK, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);
}

void control_init() {
  //// laser control
  // Setup Timer0 for a 488.28125Hz "phase correct PWM" wave (assuming a 16Mhz clock)
  // Timer0 can pwm either PD5 (OC0B) or PD6 (OC0A), we use PD6
  // TCCR0A and TCCR0B are the registers to setup Timer0
  // see chapter "8-bit Timer/Counter0 with PWM" in Atmga328 specs
  // OCR0A sets the duty cycle 0-255 corresponding to 0-100%
  // also see: http://arduino.cc/en/Tutorial/SecretsOfArduinoPWM

	GPIOPinTypeGPIOOutput(LASER_EN_PORT, LASER_EN_MASK);
	GPIOPinWrite(LASER_EN_PORT, LASER_EN_MASK, LASER_EN_INVERT);

	// Configure timer
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerConfigure(LASER_TIMER, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PWM|TIMER_CFG_B_ONE_SHOT);
	TimerControlLevel(LASER_TIMER, TIMER_A, 1);

	// PPI = PWMfreq/(feedrate/MM_PER_INCH/60)

	// Set PPI Pulse timer
	ppi_cycles = SysCtlClockGet() / 1000 * CONFIG_LASER_PPI_PULSE_MS;
	ppi_divider = ppi_cycles >> 16;
	ppi_cycles /= (ppi_divider + 1);
	TimerPrescaleSet(LASER_TIMER, TIMER_B, ppi_divider);
	TimerLoadSet(LASER_TIMER, TIMER_B, ppi_cycles);

	// Setup ISR
	TimerIntRegister(LASER_TIMER, TIMER_B, laser_isr);
	TimerIntEnable(LASER_TIMER, TIMER_TIMB_TIMEOUT);
    IntPrioritySet(INT_TIMER0B, CONFIG_LASER_PRIORITY);

	// Set PWM refresh rate
	laser_cycles = SysCtlClockGet() / CONFIG_LASER_PWM_FREQ; /*Hz*/
	laser_divider = laser_cycles >> 16;
	laser_cycles /= (laser_divider + 1);

	// Setup Laser PWM Timer
	TimerPrescaleSet(LASER_TIMER, TIMER_A, laser_divider);
	TimerLoadSet(LASER_TIMER, TIMER_A, laser_cycles);
	TimerPrescaleMatchSet(LASER_TIMER, TIMER_A, laser_divider);
	laser_intensity = 0;

	// Set default value
	control_laser_intensity(0);
	control_laser(0, 0);

	TimerEnable(LASER_TIMER, TIMER_A);

	// ToDo: Map the timer ccp pin sensibly
	GPIOPinConfigure(GPIO_PB6_T0CCP0);
	GPIOPinTypeTimer(LASER_PORT, (1 << LASER_BIT));

	//// air and aux assist control
	GPIOPinTypeGPIOOutput(ASSIST_PORT, ASSIST_MASK);
	control_air_assist(false);
	control_aux1_assist(false);
}

void control_laser_intensity(uint8_t intensity) {
	laser_intensity = intensity;
	if (intensity == 0) intensity = 1;

	// Set the PWM (Intensity).
	TimerMatchSet(LASER_TIMER, TIMER_A, laser_cycles - (laser_cycles * intensity / 255));
}

uint8_t control_get_intensity(void) {
	return laser_intensity;
}

void control_laser(uint8_t on_off, uint8_t pulse_length) {

	// If required, (re)set the PPI timer.
	if (pulse_length > 0) {
		// Schedule a timer to turn off the laser
		TimerLoadSet(LASER_TIMER, TIMER_B, ppi_cycles);
		// Clear any interrupts to avoid a race.
		// This function is called from a higher priority ISR.
		TimerIntClear(LASER_TIMER, TIMER_TIMB_TIMEOUT);
	}

	// Control the beam enable.
	if (on_off == 0)
		GPIOPinWrite(LASER_EN_PORT, LASER_EN_MASK, LASER_EN_INVERT);
	else
		GPIOPinWrite(LASER_EN_PORT, LASER_EN_MASK, LASER_EN_MASK ^ LASER_EN_INVERT);
}



void control_air_assist(bool enable) {
  if (enable) {
    GPIOPinWrite(ASSIST_PORT, (1 << AIR_ASSIST_BIT), (1 << AIR_ASSIST_BIT));
  } else {
    GPIOPinWrite(ASSIST_PORT, (1 << AIR_ASSIST_BIT), 0);
  }
}

void control_aux1_assist(bool enable) {
  if (enable) {
#if defined(CROSSHAIR_ENABLE_BIT) && CROSSHAIR_ENABLE_BIT >= 0
    GPIOPinWrite(CROSSHAIR_PORT, (1 << CROSSHAIR_ENABLE_BIT), (1 << CROSSHAIR_ENABLE_BIT));
#endif
  } else {
#if defined(CROSSHAIR_ENABLE_BIT) && CROSSHAIR_ENABLE_BIT >= 0
    GPIOPinWrite(CROSSHAIR_PORT, (1 << CROSSHAIR_ENABLE_BIT), 0);
#endif
  }  
}
