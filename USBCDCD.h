/*
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== USBCDCD.h ========
 */

#ifndef USBCDCD_H_
#define USBCDCD_H_

#ifdef __cplusplus
extern "C" {
#endif

/*!
 *  ======== USBCDCD_init ========
 *  Function to initialize the USB serial reference module.
 *
 *  Note: This function is not reentrant safe.
 */
extern uint8_t USBCDCD_init(void);

/*!
 *  ======== USBCDCD_sendData ========
 *  A blocking function that sends a specified amount of data.
 *
 *  Function sends size number of bytes from the buffer pointed by pStr.
 *
 *  @param(pStr)    Pointer to a buffer of data
 *
 *  @param(size)    Number of bytes to be sent
 *
 *  @return         Number of bytes added to the USB Buffer for transmission 
 */
extern uint32_t USBCDCD_sendData(const uint8_t *pStr, uint32_t length);

/*!
 *  ======== USBCDCD_waitForConnect ========
 *  This function blocks while the USB is not connected
 */
extern void USBCDCD_waitForConnect(void);

#ifdef __cplusplus
}
#endif

#endif /* USBCDCD_H_ */
