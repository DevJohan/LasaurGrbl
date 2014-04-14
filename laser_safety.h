/*
  laser_safety.h - Distributed laser tube safety function.
  Configures and periodically reads laser tube safety status from a
  external sensor board

  Based on temperature.h
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
#ifndef _laser_safety_flow_temperature_h
#define _laser_safety_flow_temperature_h

void flow_temperature_init(void);
uint32_t time_since_read_ms();
uint8_t temperature_num_sensors(void);
uint16_t temperature_read(uint8_t sensor);
uint16_t flow_read();

void I2CIssueWrite(
		unsigned char slave_adress,
		unsigned char *pucData,
		unsigned long ulCount,
		void(*callback)(void) );
void I2CIssueRead(
		unsigned char slave_adress,
		unsigned char *pucData,
		unsigned long ulCount,
		void(*callback)(void));

#endif /* laser_safety_flow_temperature_h */
