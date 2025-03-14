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

#ifndef LIBRARIES_IMRRC_H_
#define LIBRARIES_IMRRC_H_

#include "IMRRC.h"
#include "cybsp.h"
#include "cy_utils.h"
#include "SBUS/SBUS.h"

#define RC_STICK_CENTER_VALUE		1000	// approximate stick center value received from remote control
#define RC_STICK_MAX_DEVIATION		800		// max. deviation from RC_STICK_CENTER_VALUE
#define RC_STICK_MAX_TARGET			100		// max. desired value (+ & -) for stick values
#define RC_STICK_DEADZONE_POS		5		// positive deadzone value for remote control
#define RC_STICK_DEADZONE_NEG		-5		// negative deadzone value for remote control

#define RC_TIME_TO_LIVE				150		// max num of iterations a RC command will be used until it is declared as disconnected

typedef struct {
	bool		RC_TimeOut;
	int16_t		LeftStick_UpDown;
	int16_t		LeftStick_LeftRight;
	int16_t		RightStick_UpDown;
	int16_t		RightStick_LeftRight;

	int16_t		Switch_SD;		// 3 ... Low Position; 2 ... Middle Position; 1 ... Top Position
	int16_t		Switch_SA;		// 3 ... Low Position; 2 ... Middle Position; 1 ... Top Position
	int16_t		Switch_SB;		// 3 ... Low Position; 2 ... Middle Position; 1 ... Top Position
} RCinput_t;

RCinput_t Process_RC_Inputs(SbusData_t RC_Input);

void readLEDinput(RCinput_t RCinput);
void sendLedCmds(uint8_t LEDSW_current_state, uint8_t RGB[3]);

#endif  // LIBRARIES_IMRRC_H_