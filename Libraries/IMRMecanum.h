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

#ifndef IMRMECANUM_H_
#define IMRMECANUM_H_

#include "cybsp.h"
#include "cy_utils.h"

#define PI						3.1415926
#define MAX_WHEELSPEED_RPM		(300*0.6)						// [RPM] limit of the motor, change when motor changes
#define VMAX_ROBOT 				3.0								// [m/s]
#define TURNMAX_ROBOT			(2*PI)							// [rad/s]
#define R_WHEEL 				0.05							// [m]	Radius of the wheels

#define SMAX_WHEEL				(VMAX_ROBOT/R_WHEEL) 			// [rad/s] theoretical maximum value for the wheel speed
#define _2_15					32768
#define MPS2_15BIT				(_2_15/VMAX_ROBOT)				// m/s of the robot mapped to 15 Bit
#define RTPS2_BIT				(_2_15/TURNMAX_ROBOT)			// rad/s of the robot turn rate mapped to 15 Bit
#define RADPS2_15BIT			(_2_15/SMAX_WHEEL)				// rad/s of the wheels mapped to 15 Bit
#define IK_IMR_TURN				4.78							// forward kinematics constant for IMR (see User Guid Main Control)
#define MAX_WHEELSPEED_RADPS 	((MAX_WHEELSPEED_RPM*2*PI) / 60)// [rad/s]

#define WHEEL_DIST_X			0.115                           // [m] distance between the wheels long side
#define WHEEL_DIST_Y			0.124                           // [m] distance between the wheels short side

extern uint16_t encoderVals[4];
extern float enc_wheelspeeds[4];	// [rad/s]

void can2Vel(uint8_t CAN_data[6], int16_t* wS);

void inverseKinematics(int16_t v_pro[3], int16_t* wS);

void applyFIRFilter(int16_t wS_in[4], int16_t* wS_out);

void estimateOdometry(float* enc_wheelspeeds, int16_t* v_odo);

#endif /* IMRMECANUM_H_ */
