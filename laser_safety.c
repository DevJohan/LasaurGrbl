/*
  laser_safety.c - Distributed laser tube safety function.
  Configures and periodically reads laser tube safety status from a
  external sensor board

  Based on temperature.c
  Copyright (c) 2013 Richard Taylor

  LasaurGrbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  LasaurGrbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  ---
*/

#include <string.h>

#include <inc/hw_ints.h>
#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <inc/hw_timer.h>
#include <inc/hw_gpio.h>
#include <inc/hw_i2c.h>

#include <driverlib/gpio.h>
#include <driverlib/sysctl.h>
#include <driverlib/timer.h>
#include <driverlib/interrupt.h>
#include <driverlib/i2c.h>

#include "config.h"

#include "laser_safety.h"
#include "tasks.h"

#define LASER_SAFETY_SLAVE_ADRESS (2)

static uint16_t temperature[3] = {0};
static uint16_t flow_reading = 0;
static uint64_t timer_load;
static uint32_t last_read_time = 0;

static uint32_t cycles_per_us = 0;
static uint32_t timer_calibration = 0;


void I2C1_IntHandler_idle(void);
void I2C1_IntHandler_wait(void);
void I2C1_IntHandler_write(void);
void I2C1_IntHandler_read(void);

#define STATE_IDLE			0
#define STATE_WAIT			1
#define STATE_WRITE			2
#define STATE_READ			3

int StartI2CReadWriteEx( unsigned long new_state, unsigned char slave_adress, void(*callback)(void) );
int StartI2CReadWrite( unsigned long new_state, unsigned char slave_adress );

typedef struct{
	//*****************************************************************************
	//
	// The current state of the interrupt handler state machine.
	//
	//*****************************************************************************
	volatile unsigned long g_ulState;
	unsigned char slave_adress;
	unsigned char *data_buffer;
	unsigned long data_count;
	void(*complete_callback)(void);
} i2c_state;

static i2c_state i2c1_state = { STATE_IDLE, 0, 0, 0 };
static unsigned char i2c_read_data_buffer[16] = {0};
static unsigned char i2c_write_data_buffer[16] = {0};


int StartI2CReadWrite( unsigned long new_state, unsigned char slave_adress ){
	return StartI2CReadWriteEx( new_state, slave_adress,0 );
}
int StartI2CReadWriteEx( unsigned long new_state, unsigned char slave_adress, void(*callback)(void) ){
	//TODO: Add mutex to keep i2c state safe
	if( i2c1_state.g_ulState != STATE_IDLE ){
		return 0;
	}

	i2c1_state.slave_adress = slave_adress;
	i2c1_state.complete_callback = callback;

	switch( new_state ){
	case STATE_WRITE:
		I2CIntRegister( I2C1_MASTER_BASE, I2C1_IntHandler_write );
		return 1;
		break;
	case STATE_READ:
		I2CIntRegister( I2C1_MASTER_BASE, I2C1_IntHandler_read );
		return 1;
		break;
	default:
		i2c1_state.slave_adress = 0;
		i2c1_state.complete_callback = 0;
		return 0;
	}
}

void _InternalChangeI2CState( unsigned long new_state ){
	switch( new_state ){
	case STATE_IDLE:
		i2c1_state.complete_callback = 0;
		I2CIntRegister( I2C1_MASTER_BASE, I2C1_IntHandler_idle );
		break;
	case STATE_WAIT:
		I2CIntRegister( I2C1_MASTER_BASE, I2C1_IntHandler_wait );
		break;
	case STATE_WRITE:
		I2CIntRegister( I2C1_MASTER_BASE, I2C1_IntHandler_write );
		break;
	case STATE_READ:
		I2CIntRegister( I2C1_MASTER_BASE, I2C1_IntHandler_read );
		break;
	}
}

static void setupI2C(){
	SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOA );
	SysCtlPeripheralEnable( SYSCTL_PERIPH_I2C1 );
	SysCtlPeripheralReset( SYSCTL_PERIPH_I2C1 );

	/***
	 * I2C Setting
	 */

	//  special I2CSCL treatment for M4F devices
	GPIOPinTypeI2CSCL(LASER_SAFETY_PORT, LASER_SAFETY_SCL);
	GPIOPinTypeI2C(LASER_SAFETY_PORT, LASER_SAFETY_SDA);

	GPIOPinConfigure( GPIO_PA6_I2C1SCL );
	GPIOPinConfigure( GPIO_PA7_I2C1SDA );

	I2CMasterInitExpClk( I2C1_MASTER_BASE, SysCtlClockGet(), false);

    I2CMasterEnable( I2C1_MASTER_BASE );

    // Enable the I2C interrupt.
    I2CIntRegister( I2C1_MASTER_BASE, I2C1_IntHandler_idle );

    // Enable the I2C master interrupt.
	I2CMasterIntEnableEx( I2C1_MASTER_BASE, 0x3 );

	IntPrioritySet( INT_I2C1, CONFIG_I2C_PRIORITY );

	SysCtlDelay(10000);
}

void timer_cal_isr(void) {
	TimerLoadSet64( SENSE_TIMER, timer_load );
	TimerIntClear( SENSE_TIMER, TIMER_TIMA_TIMEOUT );
	timer_calibration++;
}

void __delay_us(uint32_t delay) {
	if (delay == 0) return;
    SysCtlDelay(cycles_per_us * delay);
}

void save_flow_temperature_reading(void) {
	temperature[0] = i2c_read_data_buffer[0];
	flow_reading = i2c_read_data_buffer[1];
	last_read_time = get_system_time_ms();
}


void temperature_update_isr(void) {
    I2CIssueRead( LASER_SAFETY_SLAVE_ADRESS, i2c_read_data_buffer, 2, save_flow_temperature_reading);

	// Schedule timer to go off
	TimerLoadSet64(SENSE_TIMER, timer_load);
	TimerIntClear(SENSE_TIMER, TIMER_TIMA_TIMEOUT);
}




void flow_temperature_init(void) {
	// Configure input pins
	setupI2C();

	// Configure timer
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
	TimerConfigure(SENSE_TIMER, TIMER_CFG_PERIODIC);

	// Create a 1us timer
	timer_load = SysCtlClockGet() / 1000;
	TimerLoadSet64(SENSE_TIMER, timer_load);

	TimerIntRegister(SENSE_TIMER, TIMER_A, timer_cal_isr);
	TimerIntEnable(SENSE_TIMER, TIMER_TIMA_TIMEOUT);
	IntPrioritySet(INT_TIMER2A, CONFIG_SENSE_PRIORITY);

	cycles_per_us = 1;
	timer_calibration = 0;
	TimerEnable(SENSE_TIMER, TIMER_A);
	__delay_us(SysCtlClockGet() / 1000);
	cycles_per_us = SysCtlClockGet() / timer_calibration / 1000000;
	TimerDisable(SENSE_TIMER, TIMER_A);

	// Create a timer ISR to fire the conversions read the temperature.
	// The minimum conversion time is approx. 750ms.
	timer_load = SysCtlClockGet() / 2;
	TimerLoadSet64(SENSE_TIMER, timer_load);

	TimerIntRegister(SENSE_TIMER, TIMER_A, temperature_update_isr);
	TimerEnable(SENSE_TIMER, TIMER_A);
}

// Returns a 12-bit right-justified 2's complement in 1/16ths of a Degree C.
uint16_t temperature_read(uint8_t sensor)
{
	return temperature[sensor];
}

uint16_t flow_read(){
	return flow_reading;
}

uint32_t time_since_read_ms(){
	return ( get_system_time_ms() - last_read_time );
}


//*****************************************************************************
//
// The I2C interrupt handler.
//
//*****************************************************************************
void I2C1_IntHandler_idle(void){
    I2CMasterIntClearEx( I2C1_MASTER_BASE, 0x3 );
	SysCtlDelay(10);
}

void I2C1_IntHandler_wait(void) {
    // Clear the I2C interrupt.
    I2CMasterIntClearEx( I2C1_MASTER_BASE, 0x3 );

    // Wait for an ACK on the read after a write.
	// See if there was an error on the previously issued read.
	if( I2CMasterErr(I2C1_MASTER_BASE) == I2C_MASTER_ERR_NONE )
	{
		// Read the byte received.
		I2CMasterDataGet(I2C1_MASTER_BASE);

		// There was no error, so the state machine is now idle.
		_InternalChangeI2CState( STATE_IDLE );
	}else{
		// Put the I2C master into receive mode.
		I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, i2c1_state.slave_adress, true);

		// Perform a single byte read.
		I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
	}
}

void I2C1_IntHandler_write(void) {
    // Clear the I2C interrupt.
    I2CMasterIntClearEx( I2C1_MASTER_BASE, 0x3 );

    // Write the next byte to the data register.
    I2CMasterDataPut( I2C1_MASTER_BASE, *i2c1_state.data_buffer++ );
    i2c1_state.data_count--;

    // Determine what to do based on the current state.
    switch( i2c1_state.data_count )
    {
    default:
    {
    	// Continue the burst write.
    	I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);

    	break;
    }
    // The state for the middle of a burst write.
    case 1:
    {
    	// Finish the burst write.
    	I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);

    	// This state is done.
    	break;
    }

    // The state for the final write of a burst sequence.
    case 0:
    {
		if(i2c1_state.complete_callback)
			i2c1_state.complete_callback();

    	_InternalChangeI2CState( STATE_WAIT );

    	// Put the I2C master into receive mode.
    	I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, i2c1_state.slave_adress, true);
    	// Perform a single byte read.
    	I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

    	break;
    }
    }
}


void I2C1_IntHandler_read(void) {
	// Clear the I2C interrupt.
	I2CMasterIntClearEx( I2C1_MASTER_BASE, 0x3 );

	// Read the received character.
	*i2c1_state.data_buffer++ = I2CMasterDataGet(I2C1_MASTER_BASE);
	i2c1_state.data_count--;

	// Determine what to do based on the current state.
	switch( i2c1_state.data_count ) {
	default:
	{
		// Continue the burst read.
		I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
		break;
	}
	case 1:
	{
		// Finish the burst read.
		I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
		break;
	}
	case 0:
	{
		// This state is for the final read of a single or burst read.
		if(i2c1_state.complete_callback)
			i2c1_state.complete_callback();
		_InternalChangeI2CState( STATE_IDLE );
		break;
	}

	}
}


//*****************************************************************************
//
// Write to the Arduino device.
//
//*****************************************************************************
void I2CIssueWrite(
		unsigned char slave_adress,
		unsigned char *pucData,
        unsigned long ulCount,
        void(*callback)(void)
){
    // Save the data buffer to be written.
	i2c1_state.data_buffer = pucData;
	i2c1_state.data_count = ulCount;

	StartI2CReadWrite( STATE_WRITE, slave_adress );

	// Put the I2C master into sending mode.
	I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, slave_adress, false);

    I2CMasterDataPut(I2C1_MASTER_BASE, *i2c1_state.data_buffer++);
    i2c1_state.data_count--;

	if( ulCount == 0 ){
		// Perform a single byte read.
		I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_SINGLE_SEND);
	}else{
		// Start the burst receive.
		I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	}
}

//*****************************************************************************
//
// Read from the Arduino device.
//
//*****************************************************************************
void I2CIssueRead(
		unsigned char slave_adress,
		unsigned char *pucData,
		unsigned long ulCount,
		void(*callback)(void)
){
    // Save the data buffer to be read.
	i2c1_state.data_buffer = pucData;
	i2c1_state.data_count = ulCount;

	StartI2CReadWriteEx( STATE_READ, slave_adress, callback );

    // Put the I2C master into receive mode.
	I2CMasterSlaveAddrSet( I2C1_MASTER_BASE, LASER_SAFETY_SLAVE_ADRESS, true );

	if( ulCount == 1 ){
		// Perform a single byte read.
		I2CMasterControl( I2C1_MASTER_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE );
	}else{
		// Start the burst receive.
		I2CMasterControl( I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START );
	}
}


