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

#ifndef LIBRARIES_IMR_CAN_H_
#define LIBRARIES_IMR_CAN_H_

#include "cybsp.h"
#include "cy_utils.h"
#include "IMR_CAN_GLOBAL.h"
#include "IMRMecanum.h"
#include "IMRLedPattern.h"
#include <stdint.h>

#define CAT(x, y) CAT_(x, y)
#define CAT_(x, y) x ## y
#define CAN_TX_TIMEOUT		0x400			// Timerout counter value ... 1024

void CAN_Initialize(void);

void CAN_IRQ_RX_VELOCITY_MSG_HANDLER(void);

void CAN_IRQ_RX_ENCODER_FL_MSG_HANDLER(void);
void CAN_IRQ_RX_ENCODER_FR_MSG_HANDLER(void);
void CAN_IRQ_RX_ENCODER_BL_MSG_HANDLER(void);
void CAN_IRQ_RX_ENCODER_BR_MSG_HANDLER(void);

void CAN_IRQ_RX_ENABLE_PWR_MSG_HANDLER(void);
void CAN_IRQ_RX_REDUCE_PWR_MSG_HANDLER(void);

uint32_t XMC_CAN_MO_Busy(XMC_CAN_MO_t* mo_ptr);

#define CAN_NODE_CONFIGURATOR_NAME				CAN_NODE	// Select the CAN_Node name chosen in the MTB Device Configurator;
#define CAN_NODE_CONFIGURATOR_CHANNEL			CAN_NODE2	// Select the CAN_Node number chosen in the MTB Device Configurator; 			CAN Node 0 ... CAN_NODE0		CAN Node 1 ... CAN_NODE1

/* IRQ Event Source Names for XMC4700 - see XMC4700 Reference Manual Table 5-1 */
#define CAN_IRQ_RX_VELOCITY_NUMBER       		CAN0_1_IRQn
#define CAN_IRQ_RX_VELOCITY_MSG_HANDLER      	IRQ_Hdlr_77

#define CAN_IRQ_RX_ENCODER_FL_NUMBER     		CAN0_2_IRQn
#define CAN_IRQ_RX_ENCODER_FL_MSG_HANDLER   	IRQ_Hdlr_78
#define CAN_IRQ_RX_ENCODER_FR_NUMBER     		CAN0_3_IRQn
#define CAN_IRQ_RX_ENCODER_FR_MSG_HANDLER   	IRQ_Hdlr_79
#define CAN_IRQ_RX_ENCODER_BL_NUMBER     		CAN0_4_IRQn
#define CAN_IRQ_RX_ENCODER_BL_MSG_HANDLER   	IRQ_Hdlr_80
#define CAN_IRQ_RX_ENCODER_BR_NUMBER     		CAN0_5_IRQn
#define CAN_IRQ_RX_ENCODER_BR_MSG_HANDLER   	IRQ_Hdlr_81

#define CAN_IRQ_RX_ENABLE_PWR_NUMBER     		CAN0_6_IRQn
#define CAN_IRQ_RX_ENABLE_PWR_MSG_HANDLER   	IRQ_Hdlr_82
#define CAN_IRQ_RX_REDUCE_PWR_NUMBER            CAN0_0_IRQn /* CAN Interrupt Number Setting:  XMC1404 = IRQ3_IRQn;    XMC4700 = CAN0_0_IRQn */
#define CAN_IRQ_RX_REDUCE_PWR_MSG_HANDLER       IRQ_Hdlr_76 /* CAN Interrupt Handler Setting: XMC1404 = IRQ3_Handler; XMC4700 = IRQ_Hdlr_76 */


#define CAN_STATUS_NODE_BUSY		( 2U )

#define CAN_NODE_GLOBAL_HW_NAME				CAT(CAN_NODE_CONFIGURATOR_NAME, _HW)
#define CAN_NODE_TRANSMIT_LMO_NAME		 	CAT(CAN_NODE_CONFIGURATOR_NAME, _LMO_1)

#define CAN_NODE_RX_VELOCITY_LMO_NAME		CAT(CAN_NODE_CONFIGURATOR_NAME, _LMO_2)
#define CAN_NODE_RX_ENCODER_FL_LMO_NAME		CAT(CAN_NODE_CONFIGURATOR_NAME, _LMO_3)
#define CAN_NODE_RX_ENCODER_FR_LMO_NAME		CAT(CAN_NODE_CONFIGURATOR_NAME, _LMO_4)
#define CAN_NODE_RX_ENCODER_BL_LMO_NAME		CAT(CAN_NODE_CONFIGURATOR_NAME, _LMO_5)
#define CAN_NODE_RX_ENCODER_BR_LMO_NAME		CAT(CAN_NODE_CONFIGURATOR_NAME, _LMO_6)

#define CAN_NODE_RX_ENABLE_PWR_LMO_NAME    	CAT(CAN_NODE_CONFIGURATOR_NAME, _LMO_7)
#define CAN_NODE_RX_REDUCE_PWR_LMO_NAME    	CAT(CAN_NODE_CONFIGURATOR_NAME, _LMO_0)


/* ------------------------------------------------------------------------------------------------------- */

/***************************************************************************************************************/
/*********************************** DO NOT CHANGE SETTINGS ABOVE THIS LINE ************************************/
/***************************************************************************************************************/

#define CAN_NODE_RECEIVE_ENABLE					(1U)			// Select if the board is supposed to receive CAN messages;						0U ... CAN Receive NOT used; 		1U ... CAN Receive used;
#define CAN_NODE_TRANSMIT_ENABLE				(1U)			// Select if the board is supposed to transmit CAN messages;					0U ... CAN Transmit NOT used; 		1U ... CAN Transmit used;
#define CAN_NODE_RECEIVE_LED_ENABLE				(0U)			// Select if a specific LED should indicate the receive of a CAN message; 		0U ... CAN RX LED NOT used; 		1U ... CAN RX LED used;
#define CAN_TRANSCEIVER_STB_PIN_ENABLE			(1U)			// Select if the Transceiver has a STB shutdown pin that is being used; 		0U ... STB Pin NOT used; 			1U ... STB Pin used;
#define TRAJECTORY_FREQUENCY					(2U)			// Frequency of incoming trajectory commands ( 1 / (f * 0.1s) -> 1Hz = 10 || 5Hz = 2)


#if (CAN_NODE_RECEIVE_LED_ENABLE)
	#define CAN_RX_LED_PIN_CONFIGURATOR_NAME	LED_RED			// Select the CAN RX LED Pin name chosen in the MTB Device Configurator;
#endif

#if (CAN_TRANSCEIVER_STB_PIN_ENABLE)
	#define CAN_STB_PIN_CONFIGURATOR_NAME		CAN_NODE_STB	// Select the STB Pin name chosen in the MTB Device Configurator;
#endif

extern bool BMS_PowerReduced;
extern uint8_t Trajectory_ttl;
extern uint8_t Trajectory_data[6];

/***************************************************************************************************************/
/*********************************** DO NOT CHANGE SETTINGS BELOW THIS LINE ************************************/
/***************************************************************************************************************/

#define CAN_NODE_TRANSMIT_LMO_NAME	 		CAT(CAN_NODE_CONFIGURATOR_NAME, _LMO_1)
#define CAN_NODE_RX_TRAJECTORY_LMO_NAME		CAT(CAN_NODE_CONFIGURATOR_NAME, _LMO_2)

#define CAN_NODE_RX_ENCODER_FL_LMO_NAME		CAT(CAN_NODE_CONFIGURATOR_NAME, _LMO_3)
#define CAN_NODE_RX_ENCODER_FR_LMO_NAME		CAT(CAN_NODE_CONFIGURATOR_NAME, _LMO_4)
#define CAN_NODE_RX_ENCODER_BL_LMO_NAME		CAT(CAN_NODE_CONFIGURATOR_NAME, _LMO_5)
#define CAN_NODE_RX_ENCODER_BR_LMO_NAME		CAT(CAN_NODE_CONFIGURATOR_NAME, _LMO_6)

#define CAN_NODE_RX_ENABLE_PWR_LMO_NAME		CAT(CAN_NODE_CONFIGURATOR_NAME, _LMO_7)
#define CAN_NODE_RX_REDUCE_PWR_LMO_NAME		CAT(CAN_NODE_CONFIGURATOR_NAME, _LMO_0)

#if (CAN_NODE_RECEIVE_LED_ENABLE)
	#define CAN_RX_LED_PIN_PORT_NAME	 CAT(CAN_RX_LED_PIN_CONFIGURATOR_NAME, _PORT)
	#define CAN_RX_LED_PIN_PIN_NAME		 CAT(CAN_RX_LED_PIN_CONFIGURATOR_NAME, _PIN)
#endif

#if (CAN_TRANSCEIVER_STB_PIN_ENABLE)
	#define CAN_STB_PIN_PORT_NAME		 CAT(CAN_STB_PIN_CONFIGURATOR_NAME, _PORT)
	#define CAN_STB_PIN_PIN_NAME		 CAT(CAN_STB_PIN_CONFIGURATOR_NAME, _PIN)
#endif

/* Interrupt event source names - see XMC4700 Reference Manual */
#define TIMER_TIMEOUT_PERIOD_MATCH_EVENT_IRQN 				CCU40_2_IRQn		/* Defines IRQ number of the period match event interrupt */
#define TIMER_TIMEOUT_PERIOD_MATCH_EVENT_HANDLER 			IRQ_Hdlr_46			/* Defines handler of the period match event interrupt */

#endif /* LIBRARIES_IMR_CAN_H_ */
