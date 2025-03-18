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

#include "IMR_CAN.h"
#include "xmc_common.h"

bool BMS_PowerReduced = false;
uint8_t Trajectory_ttl = TRAJECTORY_FREQUENCY;
uint8_t Trajectory_data[6] = {0};
uint16_t encoderPos[4] = { 0 };
float encoderSpeed[4] = { 0 };


/*****************************************************************************
 * Handler for Reduce Power message
 *****************************************************************************/
void CAN_IRQ_RX_REDUCE_PWR_MSG_HANDLER(void) {
	// Receive data from CAN Node and transfer into CAN data structure
	XMC_CAN_MO_Receive(&CAN_NODE_RX_REDUCE_PWR_LMO_NAME);
	uint32_t id = XMC_CAN_MO_GetIdentifier(&CAN_NODE_RX_REDUCE_PWR_LMO_NAME);

	// Set motor speed targets to zero and disable further motor messages
	if ((id == BMS_REDUCE_POWER_FOR_HS) && (!BMS_PowerReduced)) {
		// Disable CCU4 timer period match interrupt
		NVIC_DisableIRQ(TIMER_TIMEOUT_PERIOD_MATCH_EVENT_IRQN);
		XMC_GPIO_SetOutputHigh(LED_Blue_PORT, LED_Blue_PIN);
		// Set flag for main-loop to reduce IMR Power
		// i.e. Motors, Optional: Sensor LEDs
		BMS_PowerReduced = true;

		// Disable all Motors for the Duration of the Hotswap
		CAN_TX_Request(MOT_FL_SPEED_COMMAND, (uint8_t[]) {0x00, 0x00}, 2);
		CAN_TX_Request(MOT_FR_SPEED_COMMAND, (uint8_t[]) {0x00, 0x00}, 2);
		CAN_TX_Request(MOT_BL_SPEED_COMMAND, (uint8_t[]) {0x00, 0x00}, 2);
		CAN_TX_Request(MOT_BR_SPEED_COMMAND, (uint8_t[]) {0x00, 0x00}, 2);
	}
}

/*****************************************************************************
 * Handler for Enable Power message
 *****************************************************************************/
void CAN_IRQ_RX_ENABLE_PWR_MSG_HANDLER(void) {
	// Receive data from CAN Node and transfer into CAN data structure
	XMC_CAN_MO_Receive(&CAN_NODE_RX_ENABLE_PWR_LMO_NAME);
	uint32_t id = XMC_CAN_MO_GetIdentifier(&CAN_NODE_RX_ENABLE_PWR_LMO_NAME);

	// Enable further motor messages
	if ((id == BMS_ENABLE_POWER_FOR_HS) && (BMS_PowerReduced)) {
		// Enable CCU4 timer period match interrupt
		NVIC_EnableIRQ(TIMER_TIMEOUT_PERIOD_MATCH_EVENT_IRQN);
		// Reset flag for main-loop to resume normal power operation
		BMS_PowerReduced = false;
	}
}

/*****************************************************************************
 * Handler for Trajectory Command message
 *****************************************************************************/
void CAN_IRQ_RX_VELOCITY_MSG_HANDLER(void) {
	// Receive data from CAN Node and transfer into CAN data structure
	XMC_CAN_MO_Receive(&CAN_NODE_RX_VELOCITY_LMO_NAME);

	uint32_t id = XMC_CAN_MO_GetIdentifier(&CAN_NODE_RX_VELOCITY_LMO_NAME);
	uint8_t *data = CAN_NODE_RX_VELOCITY_LMO_NAME.can_data_byte;

	if (((id == ROBOT_VELOCITY_COMMAND) && (!BMS_PowerReduced))) {
		// By setting the Trajectory_ttl to a value > 0.
		// The Trajectory commands will be processed in main.c
		Trajectory_ttl = TRAJECTORY_FREQUENCY;
		for (uint8_t i = 0; i < 6; i++){
			Trajectory_data[i] = data[i];
		}
	}
}

/*****************************************************************************
 * Handler for Front Left Encoder message
 *****************************************************************************/
void CAN_IRQ_RX_ENCODER_FL_MSG_HANDLER(void) {
	XMC_CAN_MO_Receive(&CAN_NODE_RX_ENCODER_FL_LMO_NAME);

	uint32_t id = XMC_CAN_MO_GetIdentifier(&CAN_NODE_RX_ENCODER_FL_LMO_NAME);
	uint8_t *data = CAN_NODE_RX_ENCODER_FL_LMO_NAME.can_data_byte;

	if (((id == MOT_FL_ENCODER_DATA) && (!BMS_PowerReduced))) {
		encoderPos[0] = (data[2] << 8) + data[3];
		// Getting [rad/s] from CAN [per-unit]
		encoderSpeed[0] = ((float)((int16_t)((data[0] << 8) + data[1]))) /
							RADPS2_15BIT;
	}
}

/*****************************************************************************
 * Handler for Front Right Encoder message
 *****************************************************************************/
void CAN_IRQ_RX_ENCODER_FR_MSG_HANDLER(void) {
	XMC_CAN_MO_Receive(&CAN_NODE_RX_ENCODER_FR_LMO_NAME);

	uint32_t id = XMC_CAN_MO_GetIdentifier(&CAN_NODE_RX_ENCODER_FR_LMO_NAME);
	uint8_t *data = CAN_NODE_RX_ENCODER_FR_LMO_NAME.can_data_byte;

	if (((id == MOT_FR_ENCODER_DATA) && (!BMS_PowerReduced))) {
		encoderPos[1] = (data[2] << 8) + data[3];
		// Getting [rad/s] from CAN [per-unit]
		encoderSpeed[1] = ((float)((int16_t)((data[0] << 8) + data[1]))) /
							RADPS2_15BIT;
	}
}

/*****************************************************************************
 * Handler for Back Left Encoder message
 *****************************************************************************/
void CAN_IRQ_RX_ENCODER_BL_MSG_HANDLER(void) {
	XMC_CAN_MO_Receive(&CAN_NODE_RX_ENCODER_BL_LMO_NAME);

	uint32_t id = XMC_CAN_MO_GetIdentifier(&CAN_NODE_RX_ENCODER_BL_LMO_NAME);
	uint8_t *data = CAN_NODE_RX_ENCODER_BL_LMO_NAME.can_data_byte;

	if (((id == MOT_BL_ENCODER_DATA) && (!BMS_PowerReduced))) {
		encoderPos[2] = (data[2] << 8) + data[3];
		// Getting [rad/s] from CAN [per-unit]
		encoderSpeed[2] = ((float)((int16_t)((data[0] << 8) + data[1]))) /
							RADPS2_15BIT;
	}
}

/*****************************************************************************
 * Handler for Back Right Encoder message
 * the Logical Message Object (LMO) is also used for bar-graph function
 * to set the LEDs in IMR as speed indicator
 * Bar-graph function is currently meant for dual-motor control
 * instead of single-motor control.
 *****************************************************************************/
void CAN_IRQ_RX_ENCODER_BR_MSG_HANDLER(void) {
	XMC_CAN_MO_Receive(&CAN_NODE_RX_ENCODER_BR_LMO_NAME);

	uint32_t id = XMC_CAN_MO_GetIdentifier(&CAN_NODE_RX_ENCODER_BR_LMO_NAME);
	uint8_t *data = CAN_NODE_RX_ENCODER_BR_LMO_NAME.can_data_byte;

	if (((id == MOT_BR_ENCODER_DATA) && (!BMS_PowerReduced))) {
		encoderPos[3] = (data[2] << 8) + data[3];
		// Getting [rad/s] from CAN [per-unit]
		encoderSpeed[3] = ((float)((int16_t)((data[0] << 8) + data[1]))) /
							RADPS2_15BIT;
	}
#if (BARGRAPH_ENABLED)
	//Bargraph Pattern for embedded world demo
	if (id == BAR_GRAPH) {
		barGraph(id, data[0], data[1]);
	}
#if (!BARGRAPH_FRONT_ONLY)
	if ((id == BAR_GRAPH_BACK)) {
		barGraph(id, data[1], data[0]);
	}
#endif
#endif
}

/*****************************************************************************
 * Initialize all CAN Logical Message Objects (LMOs)
 * and enable the related interrupts
 *****************************************************************************/
void CAN_Initialize(void) {
	XMC_CAN_MO_SetIdentifier(&CAN_NODE_RX_REDUCE_PWR_LMO_NAME, BMS_REDUCE_POWER_FOR_HS);
	XMC_CAN_MO_SetIdentifier(&CAN_NODE_RX_ENABLE_PWR_LMO_NAME, BMS_ENABLE_POWER_FOR_HS);

	XMC_CAN_MO_SetIdentifier(&CAN_NODE_RX_VELOCITY_LMO_NAME, ROBOT_VELOCITY_COMMAND);
	XMC_CAN_MO_SetIdentifier(&CAN_NODE_RX_ENCODER_FL_LMO_NAME, MOT_FL_ENCODER_DATA);
	XMC_CAN_MO_SetIdentifier(&CAN_NODE_RX_ENCODER_FR_LMO_NAME, MOT_FR_ENCODER_DATA);
	XMC_CAN_MO_SetIdentifier(&CAN_NODE_RX_ENCODER_BL_LMO_NAME, MOT_BL_ENCODER_DATA);
	XMC_CAN_MO_SetIdentifier(&CAN_NODE_RX_ENCODER_BR_LMO_NAME, MOT_BR_ENCODER_DATA);

	// Toggle CAN RX LED to indicate that a message has been received
	#if (CAN_TRANSCEIVER_STB_PIN_ENABLE)
	XMC_GPIO_SetOutputLow(CAN_STB_PIN_PORT_NAME, CAN_STB_PIN_PIN_NAME);
	#endif

	// Enable CCU4 timer period match interrupt
	NVIC_EnableIRQ(TIMER_TIMEOUT_PERIOD_MATCH_EVENT_IRQN);

	// Enable NVIC IRQ with correct IRQ number: see Reference Manual
	NVIC_EnableIRQ(CAN_IRQ_RX_REDUCE_PWR_NUMBER);
	NVIC_EnableIRQ(CAN_IRQ_RX_ENABLE_PWR_NUMBER);

	NVIC_EnableIRQ(CAN_IRQ_RX_VELOCITY_NUMBER);
	NVIC_EnableIRQ(CAN_IRQ_RX_ENCODER_FL_NUMBER);
	NVIC_EnableIRQ(CAN_IRQ_RX_ENCODER_FR_NUMBER);
	NVIC_EnableIRQ(CAN_IRQ_RX_ENCODER_BL_NUMBER);
	NVIC_EnableIRQ(CAN_IRQ_RX_ENCODER_BR_NUMBER);
}
