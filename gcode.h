/*
  gcode.c - rs274/ngc parser.
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef gcode_h
#define gcode_h

#include <inc/hw_types.h>
#include <usblib/usblib.h>

typedef enum {
	GCODE_STATUS_OK = 0,
	GCODE_STATUS_RX_BUFFER_OVERFLOW,
	GCODE_STATUS_LINE_BUFFER_OVERFLOW,
	GCODE_STATUS_TRANSMISSION_ERROR,
	GCODE_STATUS_BAD_NUMBER_FORMAT,
	GCODE_STATUS_EXPECTED_COMMAND_LETTER,
	GCODE_STATUS_UNSUPPORTED_STATEMENT,
	GCODE_STATUS_SERIAL_STOP_REQUEST,
	GCODE_STATUS_LIMIT_HIT,
	GCODE_STATUS_POWER_OFF,
	GCODE_STATUS_DOOR_OPEN,
	GCODE_STATUS_CHILLER_OFF,
	GCODE_STATUS_ARC_RADIUS_ERROR,
} GCODE_STATUS;

// Initialize the parser
void gcode_init();

// Process a USB buffer and execute line(s) when complete.
uint8_t gcode_process_data(const tUSBBuffer *psBuffer);

// process a line of gcode
void gcode_process_line(char *buffer, int length);

// Execute one line of rs275/ngc/g-code
// blocks when 
uint8_t gcode_execute_line(char *line);

// update to stepper position when steppers have been stopped
// called from the stepper code that executes the stop
void gcode_request_position_update();

// Manually moves the machine by these offsets
void gcode_manual_move(double x, double y, double z, double rate);

// Set the offsets to the current location
void gcode_set_offset_to_current_position(void);

void gcode_do_home(void);

#endif
