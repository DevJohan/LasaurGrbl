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

#ifndef sense_control_h
#define sense_control_h

#include <stdbool.h>
#include "config.h"


void sense_init();
#define SENSE_X_LIMIT (GPIOPinRead(LIMIT_PORT, (1 << X_LIMIT_BIT)) != 0)
#define SENSE_Y_LIMIT (GPIOPinRead(LIMIT_PORT, (1 << Y_LIMIT_BIT)) != 0)
#define SENSE_Z_LIMIT (GPIOPinRead(LIMIT_PORT, (1 << Z_LIMIT_BIT)) == 0)
#define SENSE_E_LIMIT (GPIOPinRead(LIMIT_PORT, (1 << E_LIMIT_BIT)) == 0)
#define SENSE_DOOR_OPEN (GPIOPinRead(SENSE_PORT, (1 << DOOR_BIT)) != 0)
#define SENSE_CHILLER_OFF (temperature_read(0) > (20 * 16))
// invert door, remove power, add z_limits
#define SENSE_LIMITS (SENSE_X_LIMIT || SENSE_Y_LIMIT || SENSE_Z_LIMIT || SENSE_E_LIMIT || SENSE_DOOR_OPEN)
//#define SENSE_LIMITS (SENSE_X_LIMIT || SENSE_Y_LIMIT || SENSE_Z_LIMIT || SENSE_DOOR_OPEN)
//#define SENSE_LIMITS (SENSE_X_LIMIT || SENSE_Y_LIMIT)
#define SENSE_SAFETY (/*SENSE_CHILLER_OFF ||*/ SENSE_DOOR_OPEN)

void control_init();

void control_laser_intensity(uint8_t intensity);  //0-255 is 0-100%
void control_laser(uint8_t on_off, uint8_t pulse_length);
uint8_t control_get_intensity(void);

void control_air_assist(bool enable);
void control_aux1_assist(bool enable);

#endif
