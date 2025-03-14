/*
* Brian R Taylor
* brian.taylor@bolderflight.com
*
* Copyright (c) 2022 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

/* Software has been adapted and modified from the original to work with
 * Infineon XMC4700 series by Michael Schmidt. (26.07.2023)
 */

#ifndef SRC_SBUS_H_
#define SRC_SBUS_H_

#include "cybsp.h"
#include "cy_utils.h"

#include "xmc_uart.h"

/* Baudrate & TX setup */
#define SBUS_BAUDRATE				100000
#define SBUS_NUM_CH					16

/* Message len */
#define	SBUS_PAYLOAD_LEN			23
#define SBUS_HEADER_LEN				1
#define SBUS_FOOTER_LEN 			1

/* SBUS message defs */
#define SBUS_HEADER 				0x0F
#define SBUS_FOOTER					0x00
#define SBUS_FOOTER2  				0x04
#define SBUS_CH17_MASK  			0x01
#define SBUS_CH18_MASK  			0x02
#define SBUS_LOST_FRAME_MASK  		0x04
#define SBUS_FAILSAFE_MASK  		0x08

typedef struct {
  bool lost_frame;
  bool failsafe;
  bool ch17, ch18;
  int16_t ch[SBUS_NUM_CH];
} SbusData_t;

/* Parsing state tracking */
extern int8_t sbus_state;
extern uint8_t sbus_prev_byte;
extern uint8_t sbus_cur_byte;

/* Buffer for storing messages */
extern uint8_t sbus_recv_buf[SBUS_HEADER_LEN + SBUS_PAYLOAD_LEN + SBUS_FOOTER_LEN];

/* Data */
extern SbusData_t sbus_data;

bool SBUSRX_Parse(void);

#endif  // SRC_SBUS_H_
