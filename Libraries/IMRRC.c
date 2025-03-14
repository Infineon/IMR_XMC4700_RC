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

#include "IMRRC.h"
#include "IMR_CAN_GLOBAL.h"
#include <stdint.h>

RCinput_t Process_RC_Inputs(SbusData_t RC_Input) {
	RCinput_t processed_input = { 0 };
	/* CH0: sbus_data.ch[0] ... Right Stick Left & Right
	 * CH1: sbus_data.ch[1] ... Right Stick Up & Down
	 * CH2: sbus_data.ch[2] ... Left Stick Up & Down
	 * CH3: sbus_data.ch[3] ... Left Stick Left & Right
	 * CH4: sbus_data.ch[4] ... Switch SD
	 * CH5: sbus_data.ch[5] ... Switch SA
	 * CH6: sbus_data.ch[6] ... Switch SB */

	/* Process Channels from Remote Control */
	processed_input.LeftStick_UpDown = sbus_data.ch[0] - RC_STICK_CENTER_VALUE;
	processed_input.LeftStick_UpDown /= (RC_STICK_MAX_DEVIATION / RC_STICK_MAX_TARGET);

	if ((processed_input.LeftStick_UpDown < RC_STICK_DEADZONE_POS) && (processed_input.LeftStick_UpDown > RC_STICK_DEADZONE_NEG))
		processed_input.LeftStick_UpDown = 0;

	processed_input.LeftStick_LeftRight = sbus_data.ch[3] - RC_STICK_CENTER_VALUE;
	processed_input.LeftStick_LeftRight /= (RC_STICK_MAX_DEVIATION / RC_STICK_MAX_TARGET);

	if ((processed_input.LeftStick_LeftRight < RC_STICK_DEADZONE_POS) && (processed_input.LeftStick_LeftRight > RC_STICK_DEADZONE_NEG))
		processed_input.LeftStick_LeftRight = 0;

	processed_input.RightStick_UpDown = sbus_data.ch[2] - RC_STICK_CENTER_VALUE;
	processed_input.RightStick_UpDown /= (RC_STICK_MAX_DEVIATION / RC_STICK_MAX_TARGET);

	if ((processed_input.RightStick_UpDown < RC_STICK_DEADZONE_POS) && (processed_input.RightStick_UpDown > RC_STICK_DEADZONE_NEG))
		processed_input.RightStick_UpDown = 0;

	processed_input.RightStick_LeftRight = sbus_data.ch[1] - RC_STICK_CENTER_VALUE;
	processed_input.RightStick_LeftRight /= (RC_STICK_MAX_DEVIATION / RC_STICK_MAX_TARGET);

	if ((processed_input.RightStick_LeftRight < RC_STICK_DEADZONE_POS) && (processed_input.RightStick_LeftRight > RC_STICK_DEADZONE_NEG))
		processed_input.RightStick_LeftRight = 0;

	/* Process Buttons from Remote Control */
	if (RC_Input.ch[4] < 750)
		processed_input.Switch_SD = 1;		// Switch High Position
	else if ((RC_Input.ch[4] > 750) && (RC_Input.ch[4] < 1250))
		processed_input.Switch_SD = 2;		// Switch Middle Position
	else if (RC_Input.ch[4] > 1250)
		processed_input.Switch_SD = 3;		// Switch Low Position

	if (RC_Input.ch[5] < 750)
		processed_input.Switch_SA = 1;		// Switch High Position
	else if ((RC_Input.ch[5] > 750) && (RC_Input.ch[5] < 1250))
		processed_input.Switch_SA = 2;		// Switch Middle Position
	else if (RC_Input.ch[5] > 1250)
		processed_input.Switch_SA = 3;		// Switch Low Position

	if (RC_Input.ch[6] < 750)
		processed_input.Switch_SB = 1;		// Switch High Position
	else if ((RC_Input.ch[6] > 750) && (RC_Input.ch[6] < 1250))
		processed_input.Switch_SB = 2;		// Switch Middle Position
	else if (RC_Input.ch[6] > 1250)
		processed_input.Switch_SB = 3;		// Switch Low Position

	processed_input.RC_TimeOut = (RC_Input.failsafe | RC_Input.lost_frame);

	return processed_input;
}

uint8_t LEDSW_current_state = { 2U };               // Setting initial Switch A State
uint8_t ColorSW_current_state = { 2U };             // Setting initial Switch B State
uint8_t LED_Color_Data[] = { 0, 0, 0 };             // Default (0,0,0) is Color OCEAN (See Infineon Color Palette)

void readLEDinput(RCinput_t RCinput) {
	if (LEDSW_current_state != RCinput.Switch_SA) {
		LEDSW_current_state = RCinput.Switch_SA;
		switch (LEDSW_current_state) {
		case 1:
			LEDSW_current_state = LED_MODE_PULSE;
			break;
		case 2:
			LEDSW_current_state = LED_MODE_STEADY;
			break;
		case 3:
			LEDSW_current_state = LED_MODE_CHASER;
			break;
		default:
			LEDSW_current_state = LED_MODE_OFF;  // not reachable with current Remote implementation
			break;
		}
		sendLedCmds (LEDSW_current_state, LED_Color_Data);
	}

	if (ColorSW_current_state != RCinput.Switch_SB) {
        ColorSW_current_state = RCinput.Switch_SB;
        switch (ColorSW_current_state) {
        case 1:                                 // Setting Color to OCEAN
            LED_Color_Data[0] = 0x06;
            LED_Color_Data[1] = 0x53;
            LED_Color_Data[2] = 0x2D;
            break;
		case 2:									// Setting Color to WHITE
			LED_Color_Data[0] = 0xB9;
            LED_Color_Data[1] = 0xB9;
            LED_Color_Data[2] = 0xB9;
			break;
		case 3:                                 // Setting Color to LAWN
            LED_Color_Data[0] = 0x9B;
            LED_Color_Data[1] = 0xBA;
            LED_Color_Data[2] = 0x15;
            break;
        default:                                // Setting Color to BERRY
            LED_Color_Data[0] = 0xBE;
            LED_Color_Data[1] = 0x32;
            LED_Color_Data[2] = 0x83;
            break;
        }
		sendLedCmds (LEDSW_current_state, LED_Color_Data);
    }

}

void sendLedCmds(uint8_t LEDSW_current_state, uint8_t RGB[3])
{
	uint8_t n;
  	uint32_t board_IDs[16];
  	if (LEDSW_current_state == LED_MODE_CHASER){ // send chaser only to first column
    	sendLedCmds(LED_MODE_OFF, LED_Color_Data);  // send "off" command to turn of previous chaser
    	board_IDs[0] = LED_LAYER_1_FRONT;
    	board_IDs[1] = LED_LAYER_2_FRONT;
    	board_IDs[2] = LED_LAYER_3_FRONT;
    	n = 3;
  	}else // send other commands to all LED boards (will not be propagated by boards)
	{ 
    	n = 12;
    	for (uint16_t i = 0; i < n; i++)
    	{
      	board_IDs[i] = LED_LAYER_1_FRONT+i;
    	}
  	}
	for (int i=0; i<n; i++)
	{
		uint8_t target_data[5] = {LEDSW_current_state, LED_Color_Data[0], LED_Color_Data[1], LED_Color_Data[2], 100};
		CAN_TX_Request(board_IDs[i], target_data, 5);
	}
}
