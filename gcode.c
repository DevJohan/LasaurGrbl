/*
 gcode.c - rs274/ngc parser.
 Part of LasaurGrbl

 Copyright (c) 2009-2011 Simen Svale Skogsrud
 Copyright (c) 2011 Stefan Hechenberger
 Copyright (c) 2011 Sungeun K. Jeon

 Inspired by the Arduino GCode Interpreter by Mike Ellery and the
 NIST RS274/NGC Interpreter by Kramer, Proctor and Messina.

 LasaurGrbl is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 LasaurGrbl is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 */

#include <string.h>
#include <math.h>
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>

#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>
#include <inc/hw_gpio.h>

#include <driverlib/gpio.h>

#include "config.h"

#include "gcode.h"
#include "serial.h"
#include "sense_control.h"
#include "motion_control.h"
#include "planner.h"
#include "stepper.h"
#include "laser_safety.h"
#include "tasks.h"

enum {
	NEXT_ACTION_NONE = 0,
	NEXT_ACTION_SEEK,
	NEXT_ACTION_FEED,
	NEXT_ACTION_CW_ARC,
	NEXT_ACTION_CCW_ARC,
	NEXT_ACTION_RASTER,
	NEXT_ACTION_DWELL,
	NEXT_ACTION_STOP,
	NEXT_ACTION_HOMING_CYCLE,
	NEXT_ACTION_SET_COORDINATE_OFFSET,
	NEXT_ACTION_AIR_ASSIST_ENABLE,
	NEXT_ACTION_AIR_ASSIST_DISABLE,
	NEXT_ACTION_AUX1_ASSIST_ENABLE,
	NEXT_ACTION_AUX1_ASSIST_DISABLE,
	NEXT_ACTION_SET_ACCELERATION,
	NEXT_ACTION_SET_PPI,
};

#define OFFSET_G54 0
#define OFFSET_G55 1

#define BUFFER_LINE_SIZE 80

static uint8_t raster_buffer[RASTER_BUFFER_SIZE];

static char rx_line[BUFFER_LINE_SIZE] = {0};
static int rx_chars = 0;
static char *rx_line_cursor;

static uint8_t line_checksum_ok_already;

#define FAIL(status) gc.status_code = status;

typedef struct {
	GCODE_STATUS status_code;     		// return codes
	uint8_t motion_mode;             	// {G0, G1}
	bool inches_mode;         			// 0 = millimeter mode, 1 = inches mode {G20, G21}
	bool absolute_mode;   				// 0 = relative motion, 1 = absolute motion {G90, G91}
	double feed_rate;                	// mm/min {F}
	double seek_rate;                	// mm/min {F}
	double position[3]; 				// projected position once all scheduled motions will have been executed
	double offsets[6]; 					// coord system offsets {G54_X,G54_Y,G54_Z,G55_X,G55_Y,G55_Z}
	uint8_t offselect;            		// currently active offset, 0 -> G54, 1 -> G55
	uint8_t laser_pwm;					// 0-255 percentage
	uint16_t laser_ppi;					// Laser PPI (Pulses Per Inch)
	double acceleration;			   	// mm/min/min
	raster_t raster;					// Raster State
} parser_state_t;
static parser_state_t gc;

static volatile bool position_update_requested; // make sure to update to stepper position on next occasion

// prototypes for static functions (non-accesible from other files)
static int next_statement(char *letter, double *double_ptr, char *line,
		uint8_t *char_counter);
static int read_double(char *line, uint8_t *char_counter, double *double_ptr);

void gcode_init() {
	memset(&gc, 0, sizeof(gc));
	gc.feed_rate = CONFIG_DEFAULT_RATE;
	gc.seek_rate = CONFIG_DEFAULT_RATE;
	gc.acceleration = CONFIG_DEFAULT_ACCELERATION;
	gc.absolute_mode = true;
	gc.laser_pwm = 0U;
	gc.laser_ppi = 0U;
	gc.offselect = OFFSET_G54;
	// prime G54 cs
	// refine with "G10 L2 P0 X_ Y_ Z_"
	gc.offsets[X_AXIS] = CONFIG_X_ORIGIN_OFFSET;
	gc.offsets[Y_AXIS] = CONFIG_Y_ORIGIN_OFFSET;
	gc.offsets[Z_AXIS] = CONFIG_Z_ORIGIN_OFFSET;
	// prime G55 cs
	// refine with "G10 L2 P1 X_ Y_ Z_"
	// or set to any current location with "G10 L20 P1"
	gc.offsets[3 + X_AXIS] = CONFIG_X_ORIGIN_OFFSET;
	gc.offsets[3 + Y_AXIS] = CONFIG_Y_ORIGIN_OFFSET;
	gc.offsets[3 + Z_AXIS] = CONFIG_Z_ORIGIN_OFFSET;
	position_update_requested = false;
	line_checksum_ok_already = false;

	gc.raster.buffer = raster_buffer;
}

static void check_ppi_feedrate(void) {
	  // Check that the configured PPI and Feedrate are compatible
	  // Prefer PPI (and slow down) if not.
	  uint32_t pulses_per_min = gc.laser_ppi * gc.feed_rate / MM_PER_INCH;
	  uint32_t max_pulses_per_min = 60000 / CONFIG_LASER_PPI_PULSE_MS;

	  // Set the Feedrate to the maximum it can be for this PPI.
	  if (pulses_per_min > max_pulses_per_min) {
		  gc.feed_rate = max_pulses_per_min * MM_PER_INCH / gc.laser_ppi;
	  }
}

static float to_millimeters(float value)
{
	return(gc.inches_mode ? (value * MM_PER_INCH) : value);
}

uint8_t gcode_process_data(const tUSBBuffer *psBuffer) {
	uint8_t chr = '\0';

	if (planner_blocks_available() < PLANNER_FIFO_READY_THRESHOLD) {
		return 1;
	}

	// Read all data available...
	while ((planner_blocks_available() >= PLANNER_FIFO_READY_THRESHOLD) &&
		   (USBBufferRead(psBuffer, &chr, 1) == 1) ) {
		if ((chr == 0x0A) || (chr == 0x0D)) {
			//// process line
			if (rx_chars > 0) {          // Line is complete. Then execute!
				rx_line[rx_chars] = '\0';  // terminate string
				gcode_process_line(rx_line, rx_chars);
				rx_chars = 0;
			}
		} else if (rx_chars + 1 >= BUFFER_LINE_SIZE) {
			// reached line size, other side sent too long lines
			stepper_request_stop(GCODE_STATUS_LINE_BUFFER_OVERFLOW);
			break;
		} else if (chr == 0x14) {
			// Respond to Lasersaur's ready request
			if (planner_blocks_available() >= PLANNER_FIFO_READY_THRESHOLD) {
				char tmp[2] = { 0x12, 0 };
				printString(tmp);
			} else {
				// We can't wait here (ISR context) set a flag so that the main thread
				// sends a response when planner blocks become free.
				task_enable(TASK_READY_WAIT, 0);
			}
		} else if (chr <= 0x20) {
			// ignore control characters and space
		} else {
			// add to line, as char which is signed
			rx_line[rx_chars++] = (char) chr;
		}
	}

	if (planner_blocks_available() < PLANNER_FIFO_READY_THRESHOLD) {
		return 1;
	}

	return 0;
}

void gcode_process_line(char *buffer, int length) {
	GCODE_STATUS status_code = GCODE_STATUS_OK;
	uint8_t skip_line = 0;
	uint8_t print_extended_status = 0;

	// handle position update after a stop
	if (position_update_requested) {
		gc.position[X_AXIS] = stepper_get_position_x();
		gc.position[Y_AXIS] = stepper_get_position_y();
		gc.position[Z_AXIS] = stepper_get_position_z();
		position_update_requested = false;
		//printString("gcode pos update\n");  // debug
	}

	// Stop Request
	if (buffer[0] == '!') {
		// Tell the machine to stop.
		stepper_request_stop(GCODE_STATUS_SERIAL_STOP_REQUEST);
		stepper_synchronize();

	} else if (buffer[0] == '~') {
		// Resume the machine (we probably need a home cycle).
		stepper_stop_resume();

	} else if (stepper_stop_requested()) {
		printString("!");  // report harware is in stop mode
		status_code = stepper_stop_status();
		// report stop conditions
		switch (status_code) {
			case GCODE_STATUS_POWER_OFF:
				printString("P");  // Stop: Power Off
			break;

			case GCODE_STATUS_LIMIT_HIT:
				printString("L");  // Stop: Limit Hit
			break;

			case GCODE_STATUS_SERIAL_STOP_REQUEST:
				printString("R");  // Stop: Serial Request
			break;

			case GCODE_STATUS_RX_BUFFER_OVERFLOW:
				printString("B");  // Stop: Rx Buffer Overflow
			break;

			case GCODE_STATUS_LINE_BUFFER_OVERFLOW:
				printString("I");  // Stop: Line Buffer Overflow
			break;

			case GCODE_STATUS_TRANSMISSION_ERROR:
				printString("T");  // Stop: Serial Transmission Error
			break;

			default:
				printString("O");  // Stop: Other error
				printInteger(status_code);
			break;
		}

		skip_line = 1;
	} else {
		if (buffer[0] == '*' || buffer[0] == '^') {
			// receiving a line with checksum
			// expecting 0-n redundant lines starting with '^'
			// followed by a final line prepended by '*'
			if (!line_checksum_ok_already) {
				rx_line_cursor = buffer + 2;  // set line offset
				uint8_t rx_checksum = (uint8_t) buffer[1];
				if (rx_checksum < 128) {
					printString(buffer);
					printString(" -> checksum outside [128,255]");
					stepper_request_stop(GCODE_STATUS_TRANSMISSION_ERROR);
				}
				char *itr = rx_line_cursor;
				uint16_t checksum = 0;
				while (*itr) {  // all chars without 0-termination
					checksum += (uint8_t) *itr++;
					if (checksum >= 128) {
						checksum -= 128;
					}
				}
				checksum = (checksum >> 1) + 128; //  /2, +128
				// printString("(");
				// printInteger(rx_checksum);
				// printString(",");
				// printInteger(checksum);
				// printString(")");
				if (checksum != rx_checksum) {
					if (buffer[0] == '^') {
						skip_line = 1;
						printString("^");
					} else {  // '*'
						printString(buffer);
						stepper_request_stop(GCODE_STATUS_TRANSMISSION_ERROR);
						// line_checksum_ok_already = false;
					}
				} else {  // we got a good line
					// printString("$");
					if (buffer[0] == '^') {
						line_checksum_ok_already = true;
					}
				}
			} else {  // we already got a correct line
				// printString("&");
				skip_line = 1;
				if (buffer[0] == '*') {  // last redundant line
					line_checksum_ok_already = false;
				}
			}
		} else {
			rx_line_cursor = buffer;
		}

		if (!skip_line) {
			if (rx_line_cursor[0] != '?') {
				// process the next line of G-code
				status_code = gcode_execute_line(rx_line_cursor);
				switch (status_code) {
				case GCODE_STATUS_OK:
					break;

				case GCODE_STATUS_BAD_NUMBER_FORMAT:
					printString("N");  // Warning: Bad number format
					break;

				case GCODE_STATUS_EXPECTED_COMMAND_LETTER:
					printString("E");  // Warning: Expected command letter
					break;

				case GCODE_STATUS_UNSUPPORTED_STATEMENT:
					printString("U");  // Warning: Unsupported statement
					break;

				default:
					printString("W");  // Warning: Other error
					printInteger(status_code);
					break;
				}
			} else {
				print_extended_status = true;
			}
		}
	}

#ifndef DEBUG_IGNORE_SENSORS
	//// door and chiller status
	if (SENSE_DOOR_OPEN) {
		printString("D");  // Warning: Door is open
	}
	if (SENSE_E_LIMIT) {
		printString("G");  // Emergency stop
	}
	if (SENSE_TEMP_WARNING) {
		printString("C");  // Warning: Chiller water temperature is too high
	}
	if (SENSE_FLOW_WARNING) {
		printString("F");  // Warning: Chiller water flow is too low
	}
	// limit
	if (SENSE_LIMITS) {
		if (SENSE_X_LIMIT) {
			printString("L1");  // Limit X Hit
		}
		if (SENSE_Y_LIMIT) {
			printString("L2");  // Limit Y Hit
		}
		if (SENSE_Z_LIMIT) {
			printString("L3");  // Limit Z Hit
		}
		if (SENSE_E_LIMIT) {
			printString("L4");  // E Stop Hit
		}
	}
#endif

	//
	if (print_extended_status) {
		// position
		printString("X");
		printFloat(stepper_get_position_x());
		printString("Y");
		printFloat(stepper_get_position_y());
		// version
		printPgmString("V" LASAURGRBL_VERSION);
	}
	printString("\n");
}

// Executes one line of 0-terminated G-Code. The line is assumed to contain only uppercase
// characters and signed floating point values (no whitespace). Comments and block delete
// characters have been removed.
uint8_t gcode_execute_line(char *line) {
	uint8_t char_counter = 0;
	char letter;
	double value;
	int int_value;
	uint8_t next_action = NEXT_ACTION_NONE;
	double target[3];
	double offset[3];
	double vector[3] = {0.0};
	int l = 0;
	double n = -1.0;
	double p = 0.0;
	double r = 0.0;
	int cs = 0;
	bool got_actual_line_command = false;  // as opposed to just e.g. G1 F1200

	clear_vector(target); // XYZ(ABC) axes parameters.
	clear_vector(offset); // IJK Arc offsets are incremental. Value of zero indicates no change.

	gc.status_code = GCODE_STATUS_OK;

	//// Pass 1: Commands
	while (next_statement(&letter, &value, line, &char_counter)) {
		int_value = trunc(value);
		switch (letter) {
		case 'G':
			switch (int_value) {
			case 0:
				gc.motion_mode = next_action = NEXT_ACTION_SEEK;
				break;
			case 1:
				gc.motion_mode = next_action = NEXT_ACTION_FEED;
				break;
			case 2:
				gc.motion_mode = next_action = NEXT_ACTION_CW_ARC;
				break;
			case 3:
				gc.motion_mode = next_action = NEXT_ACTION_CCW_ARC;
				break;
			case 4:
				next_action = NEXT_ACTION_DWELL;
				break;
			case 8:
				// Special case to append raster data
				if (line[char_counter] == 'D') {
					uint32_t len;
					char_counter++;

					len = strlen(line) - char_counter;

					if (gc.raster.length + len >= RASTER_BUFFER_SIZE || len > 70)
					{
						gc.status_code = GCODE_STATUS_RX_BUFFER_OVERFLOW;
						stepper_request_stop(gc.status_code);
					}

					memcpy(&gc.raster.buffer[gc.raster.length], &line[char_counter], len);
					gc.raster.length += len;
					return gc.status_code;
				} else {
					next_action = NEXT_ACTION_RASTER;
				}
				break;
			case 10:
				next_action = NEXT_ACTION_SET_COORDINATE_OFFSET;
				break;
			case 20:
				gc.inches_mode = true;
				break;
			case 21:
				gc.inches_mode = false;
				break;
			case 28:
				next_action = NEXT_ACTION_HOMING_CYCLE;
				break;
			case 30:
				next_action = NEXT_ACTION_HOMING_CYCLE;
				break;
			case 54:
				gc.offselect = OFFSET_G54;
				break;
			case 55:
				gc.offselect = OFFSET_G55;
				break;
			case 90:
				gc.absolute_mode = true;
				break;
			case 91:
				gc.absolute_mode = false;
				break;
			default:
				FAIL(GCODE_STATUS_UNSUPPORTED_STATEMENT);
				break;
			}
			break;
		case 'M':
			switch (int_value) {
			case 3:
			case 4:
				next_action = NEXT_ACTION_SET_PPI;
				gc.laser_ppi = 0;
				break;
			case 5:
				gc.laser_ppi = 0;
				break;
			case 17:
				stepper_wake_up();
				break;
			case 18:
				stepper_request_stop(GCODE_STATUS_SERIAL_STOP_REQUEST);
				break;
			case 80:
				next_action = NEXT_ACTION_AIR_ASSIST_ENABLE;
				break;
			case 81:
				next_action = NEXT_ACTION_AIR_ASSIST_DISABLE;
				break;
			case 82:
				next_action = NEXT_ACTION_AUX1_ASSIST_ENABLE;
				break;
			case 83:
				next_action = NEXT_ACTION_AUX1_ASSIST_DISABLE;
				break;
			case 105:
				printString("ok T:");
				printFloat(temperature_read(0) / 16.0);
				printString(" B:");
				printFloat(temperature_read(1) / 16.0);
				printString("\n");
				break;
			case 106:
				next_action = NEXT_ACTION_AIR_ASSIST_ENABLE;
				break;
			case 107:
				next_action = NEXT_ACTION_AIR_ASSIST_DISABLE;
				break;
			case 114:
				printString("ok C: X:");
				printFloat(stepper_get_position_x());
				printString(" Y:");
				printFloat(stepper_get_position_y());
				printString(" Z:");
				printFloat(stepper_get_position_z());
				printString("\n");
				break;
			case 204:
				next_action = NEXT_ACTION_SET_ACCELERATION;
				break;
			default:
				FAIL(GCODE_STATUS_UNSUPPORTED_STATEMENT);
				break;
			}
			break;
		}
		if (gc.status_code) {
			break;
		}
	}

	// bail when errors
	if (gc.status_code) {
		return gc.status_code;
	}

	char_counter = 0;
	memcpy(target, gc.position, sizeof(target)); // i.e. target = gc.position

	//// Pass 2: Parameters
	while (next_statement(&letter, &value, line, &char_counter)) {
		switch (letter) {
			case 'F':
				if (to_millimeters(value) <= 0) {
					FAIL(GCODE_STATUS_BAD_NUMBER_FORMAT);
				}
				if (gc.motion_mode == NEXT_ACTION_SEEK) {
					gc.seek_rate = min(CONFIG_MAX_SEEKRATE, to_millimeters(value));
				} else {
					gc.feed_rate = min(CONFIG_MAX_FEEDRATE, to_millimeters(value));
					check_ppi_feedrate();
				}
				break;
			case 'I': case 'J': case 'K': offset[letter-'I'] = to_millimeters(value); break;
			case 'L': l = trunc(value); break;
			case 'N': n = value; break;
			case 'P':  // dwelling seconds or CS selector
				if (next_action == NEXT_ACTION_SET_COORDINATE_OFFSET) {
					cs = trunc(value);
				} else {
					p = value;
				}
				break;
			case 'R': r = to_millimeters(value); break;
			case 'S':
				if (next_action == NEXT_ACTION_SET_ACCELERATION) {
					gc.acceleration = value * 3600;
				}
				else if (next_action == NEXT_ACTION_SET_PPI) {
					gc.laser_ppi = value;
					check_ppi_feedrate();
				}
				else {
					gc.laser_pwm = value;
					control_laser_intensity(gc.laser_pwm);
				}
				break;
			case 'X':
			case 'Y':
			case 'Z':
				// We don't want to update the target when setting the raster offset.
				if (next_action != NEXT_ACTION_RASTER) {
					if (gc.absolute_mode) {
						target[letter - 'X'] = to_millimeters(value);
					} else {
						target[letter - 'X'] += to_millimeters(value);
					}
				}
				vector[letter - 'X'] = to_millimeters(value);
				got_actual_line_command = true;
				break;
		}
	}

	// bail when error
	if (gc.status_code) {
		return (gc.status_code);
	}

	//// Perform any physical actions
	switch (next_action) {
	case NEXT_ACTION_SEEK:
		if (got_actual_line_command) {
			planner_line(target[X_AXIS] + gc.offsets[3 * gc.offselect + X_AXIS],
					target[Y_AXIS] + gc.offsets[3 * gc.offselect + Y_AXIS],
					target[Z_AXIS] + gc.offsets[3 * gc.offselect + Z_AXIS],
					gc.seek_rate, gc.acceleration, 0, 0);
		}
		break;
	case NEXT_ACTION_FEED:
		if (got_actual_line_command) {
			planner_line(target[X_AXIS] + gc.offsets[3 * gc.offselect + X_AXIS],
					target[Y_AXIS] + gc.offsets[3 * gc.offselect + Y_AXIS],
					target[Z_AXIS] + gc.offsets[3 * gc.offselect + Z_AXIS],
					gc.feed_rate, gc.acceleration, gc.laser_pwm, gc.laser_ppi);
		}
		break;
	case NEXT_ACTION_CW_ARC:
	case NEXT_ACTION_CCW_ARC:
		if (got_actual_line_command) {
			double arc_target[3];
			double arc_position[3];
	          if (r != 0) { // Arc Radius Mode
	            /*
	              We need to calculate the center of the circle that has the designated radius and passes
	              through both the current position and the target position. This method calculates the following
	              set of equations where [x,y] is the vector from current to target position, d == magnitude of
	              that vector, h == hypotenuse of the triangle formed by the radius of the circle, the distance to
	              the center of the travel vector. A vector perpendicular to the travel vector [-y,x] is scaled to the
	              length of h [-y/d*h, x/d*h] and added to the center of the travel vector [x/2,y/2] to form the new point
	              [i,j] at [x/2-y/d*h, y/2+x/d*h] which will be the center of our arc.

	              d^2 == x^2 + y^2
	              h^2 == r^2 - (d/2)^2
	              i == x/2 - y/d*h
	              j == y/2 + x/d*h

	                                                                   O <- [i,j]
	                                                                -  |
	                                                      r      -     |
	                                                          -        |
	                                                       -           | h
	                                                    -              |
	                                      [0,0] ->  C -----------------+--------------- T  <- [x,y]
	                                                | <------ d/2 ---->|

	              C - Current position
	              T - Target position
	              O - center of circle that pass through both C and T
	              d - distance from C to T
	              r - designated radius
	              h - distance from center of CT to O

	              Expanding the equations:

	              d -> sqrt(x^2 + y^2)
	              h -> sqrt(4 * r^2 - x^2 - y^2)/2
	              i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2
	              j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2

	              Which can be written:

	              i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2
	              j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2

	              Which we for size and speed reasons optimize to:

	              h_x2_div_d = sqrt(4 * r^2 - x^2 - y^2)/sqrt(x^2 + y^2)
	              i = (x - (y * h_x2_div_d))/2
	              j = (y + (x * h_x2_div_d))/2

	            */

	            // Calculate the change in position along each selected axis
	            double x = target[X_AXIS]-gc.position[X_AXIS];
	            double y = target[Y_AXIS]-gc.position[Y_AXIS];

	            clear_vector(offset);
	            // First, use h_x2_div_d to compute 4*h^2 to check if it is negative or r is smaller
	            // than d. If so, the sqrt of a negative number is complex and error out.
	            double h_x2_div_d = 4 * r*r - x*x - y*y;
	            if (h_x2_div_d < 0) { FAIL(GCODE_STATUS_ARC_RADIUS_ERROR); return(gc.status_code); }
	            // Finish computing h_x2_div_d.
	            h_x2_div_d = -sqrt(h_x2_div_d)/hypot(x,y); // == -(h * 2 / d)
	            // Invert the sign of h_x2_div_d if the circle is counter clockwise (see sketch below)
	            if (gc.motion_mode == NEXT_ACTION_CCW_ARC) { h_x2_div_d = -h_x2_div_d; }

	            /* The counter clockwise circle lies to the left of the target direction. When offset is positive,
	               the left hand circle will be generated - when it is negative the right hand circle is generated.


	                                                             T  <-- Target position

	                                                             ^
	                  Clockwise circles with this center         |          Clockwise circles with this center will have
	                  will have > 180 deg of angular travel      |          < 180 deg of angular travel, which is a good thing!
	                                                   \         |          /
	      center of arc when h_x2_div_d is positive ->  x <----- | -----> x <- center of arc when h_x2_div_d is negative
	                                                             |
	                                                             |

	                                                             C  <-- Current position                                 */


	            // Negative R is g-code-alese for "I want a circle with more than 180 degrees of travel" (go figure!),
	            // even though it is advised against ever generating such circles in a single line of g-code. By
	            // inverting the sign of h_x2_div_d the center of the circles is placed on the opposite side of the line of
	            // travel and thus we get the unadvisably long arcs as prescribed.
	            if (r < 0) {
	                h_x2_div_d = -h_x2_div_d;
	                r = -r; // Finished with r. Set to positive for mc_arc
	            }
	            // Complete the operation by calculating the actual center of the arc
	            offset[X_AXIS] = 0.5*(x-(y*h_x2_div_d));
	            offset[Y_AXIS] = 0.5*(y+(x*h_x2_div_d));

	          } else { // Arc Center Format Offset Mode
	            r = hypot(offset[X_AXIS], offset[Y_AXIS]); // Compute arc radius for mc_arc
	          }

			arc_target[X_AXIS] = target[X_AXIS] + gc.offsets[3 * gc.offselect + X_AXIS];
			arc_target[Y_AXIS] = target[Y_AXIS] + gc.offsets[3 * gc.offselect + Y_AXIS];
			arc_target[Z_AXIS] = target[Z_AXIS] + gc.offsets[3 * gc.offselect + Z_AXIS];

			arc_position[X_AXIS] = gc.position[X_AXIS] + gc.offsets[3 * gc.offselect + X_AXIS];
			arc_position[Y_AXIS] = gc.position[Y_AXIS] + gc.offsets[3 * gc.offselect + Y_AXIS];
			arc_position[Z_AXIS] = gc.position[Z_AXIS] + gc.offsets[3 * gc.offselect + Z_AXIS];

			mc_arc(arc_position, arc_target, offset, X_AXIS, Y_AXIS, Z_AXIS, gc.feed_rate, r, (next_action==NEXT_ACTION_CW_ARC)?true:false, gc.acceleration, gc.laser_pwm, gc.laser_ppi);
		}
		break;
	case NEXT_ACTION_RASTER:
		if (got_actual_line_command) {
			gc.raster.x_off = vector[X_AXIS];
			gc.raster.y_off = vector[Y_AXIS];
			if (vector[Z_AXIS] < 0) {
				gc.raster.invert = 1;
			} else {
				gc.raster.invert = 0;
			}

		}
		if (p > 0.0) {
			gc.raster.dot_size = p;
		}
		if (n >= 0.0) {
			// Here we go...
			if (gc.raster.length > 0) {
				planner_raster(target[X_AXIS] + gc.offsets[3 * gc.offselect + X_AXIS],
						target[Y_AXIS] + gc.offsets[3 * gc.offselect + Y_AXIS],
						target[Z_AXIS] + gc.offsets[3 * gc.offselect + Z_AXIS],
						gc.feed_rate, gc.acceleration, gc.laser_pwm, &gc.raster);

				if (gc.raster.x_off != 0.0)
					target[Y_AXIS] += gc.raster.dot_size;
				else if (gc.raster.y_off != 0.0)
					target[X_AXIS] -= gc.raster.dot_size;
			}

			// Reset the buffer.
			gc.raster.length = 0;
			gc.raster.buffer = raster_buffer;
		}
		break;
	case NEXT_ACTION_DWELL:
		planner_dwell(p, gc.laser_pwm);
		break;
	case NEXT_ACTION_HOMING_CYCLE:
		clear_vector(target);
		gcode_do_home();
		break;
	case NEXT_ACTION_SET_COORDINATE_OFFSET:
		if (cs == OFFSET_G54 || cs == OFFSET_G55) {
			if (l == 2) {
				//set offset to target, eg: G10 L2 P1 X15 Y15 Z0
				gc.offsets[3 * cs + X_AXIS] = target[X_AXIS];
				gc.offsets[3 * cs + Y_AXIS] = target[Y_AXIS];
				gc.offsets[3 * cs + Z_AXIS] = target[Z_AXIS];
				// Set target in ref to new coord system so subsequent moves are calculated correctly.
				target[X_AXIS] = (gc.position[X_AXIS]
						+ gc.offsets[3 * gc.offselect + X_AXIS])
						- gc.offsets[3 * cs + X_AXIS];
				target[Y_AXIS] = (gc.position[Y_AXIS]
						+ gc.offsets[3 * gc.offselect + Y_AXIS])
						- gc.offsets[3 * cs + Y_AXIS];
				target[Z_AXIS] = (gc.position[Z_AXIS]
						+ gc.offsets[3 * gc.offselect + Z_AXIS])
						- gc.offsets[3 * cs + Z_AXIS];

			} else if (l == 20) {
				// set offset to current pos, eg: G10 L20 P2
				gc.offsets[3 * cs + X_AXIS] = gc.position[X_AXIS]
						+ gc.offsets[3 * gc.offselect + X_AXIS];
				gc.offsets[3 * cs + Y_AXIS] = gc.position[Y_AXIS]
						+ gc.offsets[3 * gc.offselect + Y_AXIS];
				gc.offsets[3 * cs + Z_AXIS] = gc.position[Z_AXIS]
						+ gc.offsets[3 * gc.offselect + Z_AXIS];
				target[X_AXIS] = 0;
				target[Y_AXIS] = 0;
				target[Z_AXIS] = 0;
			}
		}
		break;
	case NEXT_ACTION_AIR_ASSIST_ENABLE:
		planner_control_air_assist_enable();
		break;
	case NEXT_ACTION_AIR_ASSIST_DISABLE:
		planner_control_air_assist_disable();
		break;
	case NEXT_ACTION_AUX1_ASSIST_ENABLE:
		planner_control_aux1_assist_enable();
		break;
	case NEXT_ACTION_AUX1_ASSIST_DISABLE:
		planner_control_aux1_assist_disable();
		break;
	}

	// As far as the parser is concerned, the position is now == target. In reality the
	// motion control system might still be processing the action and the real tool position
	// in any intermediate location.
	memcpy(gc.position, target, sizeof(double) * 3); // gc.position[] = target[];
	return gc.status_code;
}

void gcode_request_position_update() {
	position_update_requested = true;
}

// Move by the supplied offset(s).
// Used by the joystick to move the head manually.
void gcode_manual_move(double x, double y, double z, double rate) {
	double target[3];

	memcpy(target, gc.position, sizeof(target));
	target[X_AXIS] += x;
	target[Y_AXIS] += y;
	target[Z_AXIS] += z;

	planner_line(target[X_AXIS] + gc.offsets[3 * gc.offselect + X_AXIS],
				 target[Y_AXIS] + gc.offsets[3 * gc.offselect + Y_AXIS],
				 target[Z_AXIS] + gc.offsets[3 * gc.offselect + Z_AXIS],
				 rate, gc.acceleration, 0, 0);

	//target[X_AXIS] = max(target[X_AXIS] + gc.offsets[3 * gc.offselect + X_AXIS], CONFIG_X_MIN);
	//target[X_AXIS] = min(target[X_AXIS] + gc.offsets[3 * gc.offselect + X_AXIS], CONFIG_X_MAX);
	//target[Y_AXIS] = max(target[Y_AXIS] + gc.offsets[3 * gc.offselect + Y_AXIS], CONFIG_Y_MIN);
	//target[Y_AXIS] = min(target[Y_AXIS] + gc.offsets[3 * gc.offselect + Y_AXIS], CONFIG_Y_MAX);

	memcpy(gc.position, target, sizeof(target));
}

// Set the offset to the current position.
// Used by the joystick to set 0,0 to to the current location.
void gcode_set_offset_to_current_position(void) {
	// set offset to current pos, eg: G10 L20 P2
	gc.offsets[3 * gc.offselect + X_AXIS] = gc.position[X_AXIS]
			+ gc.offsets[3 * gc.offselect + X_AXIS];
	gc.offsets[3 * gc.offselect + Y_AXIS] = gc.position[Y_AXIS]
			+ gc.offsets[3 * gc.offselect + Y_AXIS];
	gc.offsets[3 * gc.offselect + Z_AXIS] = gc.position[Z_AXIS]
			+ gc.offsets[3 * gc.offselect + Z_AXIS];

	clear_vector(gc.position);
}

// Utility function to home the machine
void gcode_do_home(void) {
	stepper_homing_cycle();
	// now that we are at the physical home
	// zero all the position vectors
	clear_vector(gc.position);
	planner_set_position(0.0, 0.0, 0.0);

	// move head to g54 offset
	gc.offselect = OFFSET_G54;
	planner_line(gc.offsets[3 * gc.offselect + X_AXIS],
			gc.offsets[3 * gc.offselect + Y_AXIS],
			gc.offsets[3 * gc.offselect + Z_AXIS], gc.seek_rate,
			gc.acceleration, 0, 0);
}

// Parses the next statement and leaves the counter on the first character following
// the statement. Returns 1 if there was a statements, 0 if end of string was reached
// or there was an error (check state.status_code).
static int next_statement(char *letter, double *double_ptr, char *line,
		uint8_t *char_counter) {
	if (line[*char_counter] == 0) {
		return (0); // No more statements
	}

	*letter = line[*char_counter];
	if ((*letter < 'A') || (*letter > 'Z')) {
		FAIL(GCODE_STATUS_EXPECTED_COMMAND_LETTER);
		return (0);
	}
	(*char_counter)++;

	if (!read_double(line, char_counter, double_ptr)) {
		FAIL(GCODE_STATUS_BAD_NUMBER_FORMAT);
		return (0);
	};
	return (1);
}

// Read a floating point value from a string. Line points to the input buffer, char_counter 
// is the indexer pointing to the current character of the line, while double_ptr is 
// a pointer to the result variable. Returns true when it succeeds
static int read_double(char *line, uint8_t *char_counter, double *double_ptr) {
	char *start = line + *char_counter;
	char *end;
	char *search;
	char mod_char = 0;

	// Quick search for any X's (don't want G0X0Y0 interpreting as a hex value!).
	// The alternative was sscanf, but that adds 15K of code.
	for (search = line + *char_counter; *search != 0x00; search++)
	{
		if (*search == 'X' || *search == 'Y' || *search == 'Z' || *search == 'E') {
			// Temporarily replace this with string terminator
			mod_char = *search;
			*search = 0;
			break;
		}
	}

	*double_ptr = strtod(start, &end);
	// Revert the string (if needed)
	if (mod_char != 0)
		*search = mod_char;

	// Nothing found
	if (end == start)
		return (false);

	// Update our char counter
	*char_counter = end - line;

	return (true);
}

/* 
 Intentionally not supported:

 - arcs {G2, G3}
 - Canned cycles
 - Tool radius compensation
 - A,B,C-axes
 - Evaluation of expressions
 - Variables
 - Multiple home locations
 - Probing
 - Override control

 */
