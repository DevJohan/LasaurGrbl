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

#define LASER_SAFETY_SLAVE_ADRESS (2)

static uint16_t temperature[3] = {0};
static uint64_t timer_load;

static uint32_t cycles_per_us = 0;
static uint32_t timer_calibration = 0;

void I2C1_IntHandler_idle(void);
void I2C1_IntHandler_write(void);
void I2C1_IntHandler_read(void);
void I2CIssueWrite(
		unsigned char *pucData,
		unsigned long ulCount
);
void I2CIssueRead(
		unsigned char *pucData,
		unsigned long ulCount
);

#define STATE_IDLE         0
#define STATE_WRITE_NEXT   1
#define STATE_WRITE_FINAL  2
#define STATE_WAIT_ACK     3
#define STATE_SEND_ACK     4
#define STATE_READ_ONE     5
#define STATE_READ_FIRST   6
#define STATE_READ_NEXT    7
#define STATE_READ_FINAL   8
#define STATE_READ_WAIT    9


typedef struct{
	//*****************************************************************************
	//
	// The current state of the interrupt handler state machine.
	//
	//*****************************************************************************
	volatile unsigned long g_ulState;
	unsigned char *g_pucData;
	unsigned long g_ulCount;
} i2c_state;

static i2c_state i2c1_state = {STATE_IDLE,0,0};
static unsigned char pucData[16] = {0,0};

static void setupI2C(){
	SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOA );
	SysCtlPeripheralEnable( SYSCTL_PERIPH_I2C1 );
	SysCtlPeripheralReset( SYSCTL_PERIPH_I2C1 );

	/***
	 * I2C Setting
	 */

	//  special I2CSCL treatment for M4F devices
	GPIOPinTypeI2CSCL(LASER_SAFETY_PORT, 1<<LASER_SAFETY_SCL);
	GPIOPinTypeI2C(LASER_SAFETY_PORT, 1<<LASER_SAFETY_SDA);

	GPIOPinConfigure( GPIO_PA6_I2C1SCL );
	GPIOPinConfigure( GPIO_PA7_I2C1SDA );

	I2CMasterInitExpClk( I2C1_MASTER_BASE, SysCtlClockGet(), false);

    I2CMasterEnable(I2C1_MASTER_BASE);

    // Enable the I2C interrupt.
    I2CIntRegister( I2C1_MASTER_BASE, I2C1_IntHandler_write );

    // Enable the I2C master interrupt.
	I2CMasterIntEnableEx(I2C1_MASTER_BASE,0x3);

	IntPrioritySet( INT_I2C1, CONFIG_I2C_PRIORITY );

	SysCtlDelay(10000);
}

void timer_cal_isr(void) {
	TimerLoadSet64(SENSE_TIMER, timer_load);
	TimerIntClear(SENSE_TIMER, TIMER_TIMA_TIMEOUT);
	timer_calibration++;
}

void __delay_us(uint32_t delay) {
	if (delay == 0) return;
    SysCtlDelay(cycles_per_us * delay);
}


void temperature_update_isr(void) {
	temperature[0] = pucData[0];
	temperature[1] = pucData[1];
    I2CIssueRead( pucData, 2 );

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




//*****************************************************************************
//
// The I2C interrupt handler.
//
//*****************************************************************************
void I2C1_IntHandler_idle(void){
    I2CMasterIntClear( I2C1_MASTER_BASE );
	SysCtlDelay(10);
}
void I2C1_IntHandler_write(void) {
//	temperature[1]++;
    // Clear the I2C interrupt.
    I2CMasterIntClear( I2C1_MASTER_BASE );
    HWREG(I2C1_MASTER_BASE + I2C_O_MCS) = 0x4;

    // Determine what to do based on the current state.
    switch( i2c1_state.g_ulState )
    {
        // The idle state.
        case STATE_IDLE:
        {
            // There is nothing to be done.
//        	I2CIntRegister( I2C1_MASTER_BASE, I2C1_IntHandler_idle );
            break;
        }

        // The state for the middle of a burst write.
        case STATE_WRITE_NEXT:
        {
            // Write the next byte to the data register.
            I2CMasterDataPut(I2C1_MASTER_BASE, *(i2c1_state.g_pucData++));
            i2c1_state.g_ulCount--;

            // Continue the burst write.
            I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);

            // If there is one byte left, set the next state to the final write
            // state.
            if(i2c1_state.g_ulCount == 1)
            {
            	i2c1_state.g_ulState = STATE_WRITE_FINAL;
            }

            // This state is done.
            break;
        }

        // The state for the final write of a burst sequence.
        case STATE_WRITE_FINAL:
        {
            // Write the final byte to the data register.
            I2CMasterDataPut(I2C1_MASTER_BASE, *i2c1_state.g_pucData++);
            i2c1_state.g_ulCount--;

            // Finish the burst write.
            I2CMasterControl(I2C1_MASTER_BASE,
                             I2C_MASTER_CMD_BURST_SEND_FINISH);

            // The next state is to wait for the burst write to complete.
            i2c1_state.g_ulState = STATE_SEND_ACK;

            // This state is done.
            break;
        }

        // Wait for an ACK on the read after a write.
        case STATE_WAIT_ACK:
        {
            // See if there was an error on the previously issued read.
            if(I2CMasterErr(I2C1_MASTER_BASE) == I2C_MASTER_ERR_NONE)
            {
                // Read the byte received.
                I2CMasterDataGet(I2C1_MASTER_BASE);

                // There was no error, so the state machine is now idle.
                i2c1_state.g_ulState = STATE_IDLE;

                // This state is done.
                break;
            }

            // Fall through to STATE_SEND_ACK. //removed
            // Put the I2C master into receive mode.
            I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, LASER_SAFETY_SLAVE_ADRESS, true);

            // Perform a single byte read.
            I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

            // The next state is the wait for the ack.
            i2c1_state.g_ulState = STATE_WAIT_ACK;

            // This state is done.
            break;
        }

        // Send a read request, looking for the ACK to indicate that the write
        // is done.
        case STATE_SEND_ACK:
        {
            // Put the I2C master into receive mode.
            I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, LASER_SAFETY_SLAVE_ADRESS, true);

            // Perform a single byte read.
            I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

            // The next state is the wait for the ack.
            i2c1_state.g_ulState = STATE_WAIT_ACK;

            // This state is done.
            break;
        }

    }
}


void I2C1_IntHandler_read(void) {
//	temperature[1]++;
    // Clear the I2C interrupt.
    I2CMasterIntClearEx(I2C1_MASTER_BASE,0x3);

    // Determine what to do based on the current state.
    switch( i2c1_state.g_ulState )
    {
        // The idle state.
        case STATE_IDLE:
        {
//        	I2CIntRegister( I2C1_MASTER_BASE, I2C1_IntHandler_idle );
            break;
        }


        // The state for the middle of a burst read.
        case STATE_READ_NEXT:
        {
            // Read the received character.
            *i2c1_state.g_pucData++ = I2CMasterDataGet(I2C1_MASTER_BASE);
            i2c1_state.g_ulCount--;

            // Continue the burst read.
            I2CMasterControl(I2C1_MASTER_BASE,
                             I2C_MASTER_CMD_BURST_RECEIVE_CONT);

            // If there are two characters left to be read, make the next
            // state be the end of burst read state.
            if(i2c1_state.g_ulCount == 1)
            {
            	i2c1_state.g_ulState = STATE_READ_FINAL;
            }

            // This state is done.
            break;
        }

        // The state for the end of a burst read.
        case STATE_READ_FINAL:
        {
            // Read the received character.
            *i2c1_state.g_pucData++ = I2CMasterDataGet(I2C1_MASTER_BASE);
            i2c1_state.g_ulCount--;

            // Finish the burst read.
            I2CMasterControl(I2C1_MASTER_BASE,
                             I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

            // The next state is the wait for final read state.
            i2c1_state.g_ulState = STATE_READ_WAIT;

            // This state is done.
            break;
        }

        // This state is for the final read of a single or burst read.
        case STATE_READ_WAIT:
        {
            // Read the received character.
            *i2c1_state.g_pucData++  = I2CMasterDataGet(I2C1_MASTER_BASE);
            i2c1_state.g_ulCount--;

            // The state machine is now idle.
            i2c1_state.g_ulState = STATE_IDLE;

            // This state is done.
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
		unsigned char *pucData,
        unsigned long ulCount)
{
    // Save the data buffer to be written.
	i2c1_state.g_pucData = pucData;
	i2c1_state.g_ulCount = ulCount;

    // Set the next state of the interrupt state machine based on the number of
    // bytes to write.
	i2c1_state.g_ulState =
			(ulCount != 1) ?
					STATE_WRITE_NEXT:
					STATE_WRITE_FINAL;


	I2CIntRegister(I2C1_MASTER_BASE, I2C1_IntHandler_write );
	I2CMasterIntEnableEx(I2C1_MASTER_BASE,0x3);
	IntEnable(INT_I2C1);

	// Put the I2C master into sending mode.
	I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, LASER_SAFETY_SLAVE_ADRESS, false);

    I2CMasterDataPut(I2C1_MASTER_BASE, *i2c1_state.g_pucData++);
    i2c1_state.g_ulCount--;

	if( ulCount == 0 ){
		// Perform a single byte read.
		I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_SINGLE_SEND);

		// The next state is the wait for final read state.
		i2c1_state.g_ulState = STATE_WRITE_FINAL;
	}else{
		// Start the burst receive.
		I2CMasterControl(I2C1_MASTER_BASE,
				I2C_MASTER_CMD_BURST_SEND_START);

		// The next state is the middle of the burst read.
		i2c1_state.g_ulState = STATE_WRITE_NEXT;
	}
}

//*****************************************************************************
//
// Read from the Arduino device.
//
//*****************************************************************************
void I2CIssueRead(
		unsigned char *pucData,
		unsigned long ulCount
){
    // Save the data buffer to be read.
	i2c1_state.g_pucData = pucData;
	i2c1_state.g_ulCount = ulCount;

    // Set the next state of the interrupt state machine based on the number of
    // bytes to read.

	I2CIntRegister(I2C1_MASTER_BASE, I2C1_IntHandler_read );
//    IntEnable(INT_I2C1);
//	I2CMasterIntEnableEx(I2C1_MASTER_BASE,0x3);

    // Put the I2C master into receive mode.
	I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, LASER_SAFETY_SLAVE_ADRESS, true);

	if( ulCount == 1 ){

		// Perform a single byte read.
		I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

		// The next state is the wait for final read state.
		i2c1_state.g_ulState = STATE_READ_WAIT;
	}else{

		// Start the burst receive.
		I2CMasterControl(I2C1_MASTER_BASE,
				I2C_MASTER_CMD_BURST_RECEIVE_START);

		// The next state is the middle of the burst read.
		i2c1_state.g_ulState = STATE_READ_NEXT;
	}
}


