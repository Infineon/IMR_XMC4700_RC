
/******************************************************************************
 * (c) (2024), Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product").
 * By including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing so
 * agrees to indemnify Cypress against all liability.
******************************************************************************/

#include "IMRLedPattern.h"

/**
 * Converts the robot LED pattern to the board LED patterns.
 * 
 * This function maps the robot LED pattern to the board LED patterns.
 * The robot LED pattern is a 3x56 array, and the LED patterns for the 12 boards is a 12x23 array.
 * 
 * robotLED: A 3x56 array representing the robot LED pattern.
 * boardLEDs: A 12x23 array representing the board LED patterns.
 */
void robotLED2boardLED(bool robotLED[3][56], bool boardLEDs[12][23])
{
	for (int i = 0; i < 3; i++)  // layers
	{
		uint8_t i_led = 0; // index of LED in layer (robotled indexing)
		for (int j = 0; j < 4; j++) // boards
		{	
			uint8_t n_leds = j%2 == 0 ?	23 : 5;  // number of leds on board
			for (int k = 0; k < n_leds; k++) // LEDs
			{
				boardLEDs[i*4+j][k] = robotLED[i][k + i_led];
			}
			i_led += n_leds;
		}
	}
}

/**
 * Converts the board LED pattern to a CAN mask.
 * 
 * This function converts the board LED pattern to a CAN mask, which is used for communication.
 * The board LED pattern is a 23-element array, and the CAN mask is a 3-element array.
 * 
 * boardLED: A 23-element array representing the board LED pattern.
 * preserve_old: A boolean flag indicating whether to preserve the state of the masked leds or turn them off.
 * CANmask: A 3-element array representing the CAN mask.
 */
void boardLED2CANmask(bool boardLED[23], bool preserve_old, uint8_t CANmask[3])
{
	for (int i = 0; i < 23; i++) {
		int byte_index = 2 - (i / 8);
		int bit_index = i % 8;
		if (boardLED[i]) {
			CANmask[byte_index] |= (1 << bit_index);
		} else {
			CANmask[byte_index] &= ~(1 << bit_index);
		}
	}

	// Invert CANmask
	for (int i = 0; i < 3; i++) {
		CANmask[i] = ~CANmask[i];
	}

	if (preserve_old) {
		CANmask[0] |= 0x80;  // set bit 7 to 1 to keep masked LEDs in old status
	}
}

/**
 * Controls the LED bar graph.
 * 
 * This function controls the LED bar graph based on the given parameters.
 * It expects values in the range [-27, 27] for the number of LEDs on the left and right sides.
 * The function serves as a sample for implementing custom LED patterns on the IMR.
 * 
 * CAN_ID: The CAN ID for the bar graph.
 * n_leds_left: The number of LEDs on the left side of the bar graph.
 * n_leds_right: The number of LEDs on the right side of the bar graph.
 */
void barGraph(uint32_t CAN_ID, int8_t n_leds_left, int8_t n_leds_right)
{
	uint8_t led_command[8];
	uint8_t color_left_r = 0x70;
	uint8_t color_left_g = 0x15;
	uint8_t color_left_b = 0x45;

	if (n_leds_left < 0) 
	{
		n_leds_left = -n_leds_left;
		color_left_r = 0x15;	// change color for forward/backward
		color_left_g = 0x70;
		color_left_b = 0x45;
	}

	uint8_t color_right_r = 0x15;
	uint8_t color_right_g = 0x70;
	uint8_t color_right_b = 0x45;

	if (n_leds_right < 0) 
	{
		n_leds_right = -n_leds_right;
		color_right_r = 0x70;
		color_right_g = 0x15;
		color_right_b = 0x45;
	}

	led_command[0] = LED_MODE_STEADY;
	led_command[4] = 30;  // not used

	bool robotLEDlayer[56] = {0};		// fill layer with left and right bar
	bool robotLEDlayer_off[56] = {0};	// fill layer with the positions where the LEDs have to be turned off
	
	bool robotLED[3][56] = {{0}};  // all 3 layers identical
	bool robotLED_off[3][56] = {{0}};

	bool boardLEDs[12][23] = {{0}};
	bool boardLEDs_off[12][23] = {{0}};

	// bitwise representation of all leds (horizontally) on left & right side;
	// shift whole bar one bit to the right to prevent LED 0 lighting up at speed 0
	uint32_t bar_left = (n_leds_left == 0 ? 0 :((1 << (n_leds_left))-1));
	uint32_t bar_right = (n_leds_right == 0 ? 0 : ((1 << (n_leds_right))-1));

	//left side
	led_command[1] = color_left_r;
	led_command[2] = color_left_g;
	led_command[3] = color_left_b;

	if (CAN_ID == BAR_GRAPH)
	{
#if (BARGRAPH_FRONT_ONLY)
		for (int i = 0; i < 27; i++)
		{
			robotLEDlayer[i + 12] = (bar_left >> (i)) & 1;
			robotLEDlayer_off[i + 12] = !((bar_left >> (i)) & 1);
		}
#else
		for (int i = 0; i < 13; i++)
		{
			robotLEDlayer[i + 12] = (bar_left >> (i)) & 1;
			robotLEDlayer_off[i + 12] = !((bar_left >> (i)) & 1);
		}
#endif
	}
#if (!BARGRAPH_FRONT_ONLY)
	else if (CAN_ID == BAR_GRAPH_BACK)
	{
		for (int i = 0; i < 13; i++)
		{
			robotLEDlayer[38 - i] = (bar_left >> (i)) & 1;
			robotLEDlayer_off[38 - i] = !((bar_left >> (i)) & 1);
		}
	}
#endif
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 56; j++) {
			robotLED[i][j] = robotLEDlayer[j];
			robotLED_off[i][j] = robotLEDlayer_off[j];
		}
	}

	robotLED2boardLED(robotLED, boardLEDs);
	robotLED2boardLED(robotLED_off, boardLEDs_off);

	for (int i = 0; i < 12; i++)  // loop over boards
	{
		uint8_t CANmask[3] = {0};
		uint8_t	CANmask_off[3] = {0};
		boardLED2CANmask(boardLEDs[i], true, CANmask);
		boardLED2CANmask(boardLEDs_off[i], true, CANmask_off);
		for (int j=0; j<3; j++)
		{
			led_command[5+j] = CANmask[j];
		}
		CAN_TX_Request(LED_LAYER_1_FRONT + i, led_command, 8);
		XMC_Delay(1);
	}

	//right side
	led_command[1] = color_right_r;
	led_command[2] = color_right_g;
	led_command[3] = color_right_b;

	// reset robotLEDlayer
	memset(robotLEDlayer, 0, sizeof(robotLEDlayer));
	// robotLEDlayer_off does not need to be reset, contains info from left and right bar

	if (CAN_ID == BAR_GRAPH)
	{
#if (BARGRAPH_FRONT_ONLY)
		for (int i = 0; i < 27; i++)
		{
			robotLEDlayer[(10 - i + 56) % 56] = (bar_right >> i) & 1;
			robotLEDlayer_off[(10 - i + 56) % 56] = !((bar_right >> i) & 1);
		}
#else
		for (int i = 0; i < 13; i++)
		{
			robotLEDlayer[(10 - i + 56) % 56] = (bar_right >> i) & 1;
			robotLEDlayer_off[(10 - i + 56) % 56] = !((bar_right >> i) & 1);
		}
#endif
	}
#if (!BARGRAPH_FRONT_ONLY)
	else if (CAN_ID == BAR_GRAPH_BACK)
	{
		for (int i = 40; i < 53; i++)
		{
			robotLEDlayer[i%56] = (bar_right >> (i-40)) & 1;
			robotLEDlayer_off[i%56] = !((bar_right >> (i-40)) & 1);
		}
	}
#endif
	// reset both robotLED and robotLED_off
	memset(robotLED, 0, sizeof(robotLED));
	memset(robotLED_off, 0, sizeof(robotLED_off));

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 56; j++) {
			robotLED[i][j] = robotLEDlayer[j];
			robotLED_off[i][j] = robotLEDlayer_off[j];
		}
	}

	// reset boardLEDs
	memset(boardLEDs, 0, sizeof(boardLEDs));

	robotLED2boardLED(robotLED, boardLEDs);
	robotLED2boardLED(robotLED_off, boardLEDs_off);

	for (int i = 0; i < 12; i++)  // loop over boards
	{
		uint8_t CANmask[3] = {0};
		boardLED2CANmask(boardLEDs[i], true, CANmask);
		for (int j=0; j<3; j++)
		{
			led_command[5+j] = CANmask[j];
		}
		CAN_TX_Request(LED_LAYER_1_FRONT + i, led_command, 8);
		XMC_Delay(1);
	}

	// send set LEDs within bargraph that are supposed to be turned off to RGB=[000] manually
	led_command[1] = 0; // using LED_MODE_OFF would not work as it does not respect the bitmask
	led_command[2] = 0;
	led_command[3] = 0;

	for (int i = 0; i < 12; i++)
	{
		uint8_t	CANmask_off[3] = {0};
		boardLED2CANmask(boardLEDs_off[i], true, CANmask_off);
		for (int j=0; j<3; j++)
		{
			led_command[5+j] = CANmask_off[j];
		}
		CAN_TX_Request(LED_LAYER_1_FRONT + i, led_command, 8);
		XMC_Delay(1);
	}
}
