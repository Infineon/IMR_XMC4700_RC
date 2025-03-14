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

#include "IMRMecanum.h"
#include <stdint.h>

// mat_T contains the parameters for the inverse kinematics
float jacobian_inv[4][3] = {{-1/R_WHEEL/MPS2_15BIT, 1/R_WHEEL/MPS2_15BIT, IK_IMR_TURN/RTPS2_BIT}, 
    {1/R_WHEEL/MPS2_15BIT, 1/R_WHEEL/MPS2_15BIT, IK_IMR_TURN/RTPS2_BIT}, 
    {-1/R_WHEEL/MPS2_15BIT, -1/R_WHEEL/MPS2_15BIT, IK_IMR_TURN/RTPS2_BIT}, 
    {1/R_WHEEL/MPS2_15BIT, -1/R_WHEEL/MPS2_15BIT, IK_IMR_TURN/RTPS2_BIT}};
float FIR_params[5] = {0.0f, 0.0f, 0.0f, 0.0f, 1.0f};  // Limit rate of motor speed change
int16_t pastMotorVals[5][4] = { 0 };

/**
 * Converts CAN data to velocity values.
 *
 * This function takes an array of CAN data and converts it into velocity values.
 *
 * CAN_data: An array of 6 bytes containing the CAN data.
 * vel: A pointer to an array of int16_t where the resulting velocity values will be stored.
 */
void can2Vel(uint8_t CAN_data[6], int16_t* vel)
{
    vel[0] = ((int16_t)CAN_data[0] << 8) + CAN_data[1];
    vel[1] = ((int16_t)CAN_data[2] << 8) + CAN_data[3];
    vel[2] = ((int16_t)CAN_data[4] << 8) + CAN_data[5];
}

/**
 * Calculates the inverse kinematics for a mecanum wheeled robot.
 *
 * This function computes the wheel speeds required to achieve the desired
 * robot velocities in the x, y, and rotational directions.
 *
 * v_pro: Array of 3 elements representing the desired velocities:
 *              - v_pro[0]: Velocity in the x direction (forward/backward).
 *              - v_pro[1]: Velocity in the y direction (left/right).
 *              - v_pro[2]: Rotational velocity (yaw rate).
 * wS: Pointer to an array where the calculated wheel speeds will be stored.
 *           The array should have space for 4 elements, corresponding to the
 *           speeds of the 4 mecanum wheels.
 */
void inverseKinematics(int16_t v_pro[3], int16_t* wS) {
    float wheelspeeds[4] = {0,0,0,0};
    float ovbuffer = 0;
    memset(wS,0,8);

    // Matrix Multiplication
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            wheelspeeds[i] += jacobian_inv[i][j] * v_pro[j];
        }
        if(wheelspeeds[i] > MAX_WHEELSPEED_RADPS && wheelspeeds[i] > ovbuffer) {
        	ovbuffer = wheelspeeds[i];
        }
        if(wheelspeeds[i] < -MAX_WHEELSPEED_RADPS && wheelspeeds[i] < -ovbuffer) {
        	ovbuffer = -wheelspeeds[i];
        }
    }

    if(ovbuffer > 0){
        float span = ovbuffer / MAX_WHEELSPEED_RADPS;
        for (int i = 0; i < 4; i++) {wheelspeeds[i]/=span;}
    }

    for (int i = 0; i < 4; i++) wS[i] = ((int16_t) (wheelspeeds[i] * RADPS2_15BIT));
}


/**
 * Applies a Finite Impulse Response (FIR) filter to the input wheel speeds.
 *
 * This function takes an array of input wheel speeds and applies an FIR filter to produce
 * the filtered output wheel speeds.
 *
 * wheelspeed_IN: An array of 4 int16_t values representing the input wheel speeds.
 * wheelspeed_OUT: A pointer to an int16_t where the filtered output wheel speed will be stored.
 */
void applyFIRFilter(int16_t wheelspeed_IN[4], int16_t* wheelspeed_OUT){

    float temp_wS_out;
    for (int i = 0; i<4; i++) {
        for (int j = 0; j < 4; j++) {
            pastMotorVals[i][j] = pastMotorVals[i+1][j];
        }
    }
    for (int j = 0; j < 4; j++) {
        pastMotorVals[4][j] = wheelspeed_IN[j];
        temp_wS_out = 0;
        for (int i = 0; i < 5; i++) {
            temp_wS_out +=  (((float)pastMotorVals[i][j]) * FIR_params[i]);
        }
        wheelspeed_OUT[j] = (int16_t) temp_wS_out;
    }
}

/**
 * Estimates the odometry based on encoder wheel speeds.
 *
 * This function calculates the odometry values using the provided encoder wheel speeds.
 *
 * enc_Ws: Pointer to an array of floats representing the encoder wheel speeds (in m/s or rad/s).
 * v_odo: Pointer to an array of int16_t where the calculated odometry values will be stored (scaled to 16 bit).
 */
void estimateOdometry(float* enc_Ws, int16_t* v_odo) {
	float odo_ms[3] = { 0 };	// Odometry wheelspeeds in [m/s] or [rad/s]
    odo_ms[0] = ((- enc_Ws[0] * R_WHEEL
                + enc_Ws[1] * R_WHEEL
                - enc_Ws[2] * R_WHEEL
                + enc_Ws[3] * R_WHEEL) / 4.0);
    odo_ms[1] = ((+ enc_Ws[0] * R_WHEEL
                + enc_Ws[1] * R_WHEEL
                - enc_Ws[2] * R_WHEEL
                - enc_Ws[3] * R_WHEEL) / 4.0);
    odo_ms[2] = ((+ enc_Ws[0]
                + enc_Ws[1]
                + enc_Ws[2]
                + enc_Ws[3]) 
                / 4 / IK_IMR_TURN);
    v_odo[0] = (int16_t) (odo_ms[0] * MPS2_15BIT);
    v_odo[1] = (int16_t) (odo_ms[1] * MPS2_15BIT);
    v_odo[2] = (int16_t) (odo_ms[2] * RTPS2_BIT);
}
