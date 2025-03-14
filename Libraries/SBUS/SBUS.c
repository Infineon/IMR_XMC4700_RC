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

#include "SBUS.h"

/* Parsing state tracking */
int8_t 	sbus_state 		= 0;
uint8_t sbus_prev_byte 	= SBUS_FOOTER;
uint8_t sbus_cur_byte = 0;

/* Buffer for storing messages */
uint8_t sbus_recv_buf[SBUS_HEADER_LEN + SBUS_PAYLOAD_LEN + SBUS_FOOTER_LEN] = { 0 };

/* Data */
SbusData_t sbus_data = { 0 };

bool SBUSRX_Parse(void) {
	/* Parse messages */
	if(!XMC_USIC_CH_RXFIFO_IsEmpty(SBUS_UART_HW)) {
		sbus_cur_byte = XMC_UART_CH_GetReceivedData(SBUS_UART_HW);

		if (sbus_state == 0) {
            if ((sbus_cur_byte == SBUS_HEADER) && ((sbus_prev_byte == SBUS_FOOTER) || ((sbus_prev_byte & 0x0F) == SBUS_FOOTER2))) {
				sbus_recv_buf[sbus_state++] = sbus_cur_byte;
			} else {
				sbus_state = 0;
			}
		} else if (sbus_state < SBUS_PAYLOAD_LEN + SBUS_HEADER_LEN) {
			sbus_recv_buf[sbus_state++] = sbus_cur_byte;
		} else if (sbus_state < SBUS_PAYLOAD_LEN + SBUS_HEADER_LEN + SBUS_FOOTER_LEN) {
			sbus_state = 0;
			sbus_prev_byte = sbus_cur_byte;

			if (sbus_cur_byte == SBUS_FOOTER || ((sbus_cur_byte & 0x0F) == SBUS_FOOTER2)) {
				/* Grab the channel data */
				sbus_data.ch[0]  = (int16_t)(sbus_recv_buf[1] | ((sbus_recv_buf[2] << 8) & 0x07FF));
				sbus_data.ch[1]  = (int16_t)((sbus_recv_buf[2] >> 3) | ((sbus_recv_buf[3] << 5) & 0x07FF));
				sbus_data.ch[2]  = (int16_t)((sbus_recv_buf[3] >> 6) | (sbus_recv_buf[4] << 2) | ((sbus_recv_buf[5] << 10) & 0x07FF));

				sbus_data.ch[3]  = (int16_t)((sbus_recv_buf[5] >> 1) | ((sbus_recv_buf[6] << 7) & 0x07FF));
				sbus_data.ch[4]  = (int16_t)((sbus_recv_buf[6] >> 4) | ((sbus_recv_buf[7] << 4) & 0x07FF));
				sbus_data.ch[5]  = (int16_t)((sbus_recv_buf[7] >> 7) | (sbus_recv_buf[8] << 1) | ((sbus_recv_buf[9] << 9) & 0x07FF));

				sbus_data.ch[6]  = (int16_t)((sbus_recv_buf[9] >> 2) | ((sbus_recv_buf[10] << 6) & 0x07FF));
				sbus_data.ch[7]  = (int16_t)((sbus_recv_buf[10] >> 5) | ((sbus_recv_buf[11] << 3) & 0x07FF));
				sbus_data.ch[8]  = (int16_t)(sbus_recv_buf[12] | ((sbus_recv_buf[13] << 8) & 0x07FF));

				sbus_data.ch[9]  = (int16_t)((sbus_recv_buf[13] >> 3) | ((sbus_recv_buf[14] << 5) & 0x07FF));
				sbus_data.ch[10] = (int16_t)((sbus_recv_buf[14] >> 6) | (sbus_recv_buf[15] << 2) | ((sbus_recv_buf[16] << 10) & 0x07FF));

				sbus_data.ch[11] = (int16_t)((sbus_recv_buf[16] >> 1) | ((sbus_recv_buf[17] << 7) & 0x07FF));
				sbus_data.ch[12] = (int16_t)((sbus_recv_buf[17] >> 4) | ((sbus_recv_buf[18] << 4) & 0x07FF));
				sbus_data.ch[13] = (int16_t)((sbus_recv_buf[18] >> 7) | (sbus_recv_buf[19] << 1) | ((sbus_recv_buf[20] << 9) & 0x07FF));

				sbus_data.ch[14] = (int16_t)((sbus_recv_buf[20] >> 2) | ((sbus_recv_buf[21] << 6) & 0x07FF));
				sbus_data.ch[15] = (int16_t)((sbus_recv_buf[21] >> 5) | ((sbus_recv_buf[22] << 3) & 0x07FF));

				sbus_data.ch17 = sbus_recv_buf[23] & SBUS_CH17_MASK;	        /* CH 17 */
				sbus_data.ch18 = sbus_recv_buf[23] & SBUS_CH18_MASK;	        /* CH 18 */

				sbus_data.lost_frame = sbus_recv_buf[23] & SBUS_LOST_FRAME_MASK;	    /* Grab the lost frame */
				sbus_data.failsafe = sbus_recv_buf[23] & SBUS_FAILSAFE_MASK;			/* Grab the failsafe */

				if (sbus_data.ch[0] && sbus_data.ch[1] && sbus_data.ch[2] && sbus_data.ch[3])
					return true;
				else
					return false;
			} else {
				return false;
			}
		} else {
			sbus_state = 0;
		}
		sbus_prev_byte = sbus_cur_byte;
	}
	return false;
}

