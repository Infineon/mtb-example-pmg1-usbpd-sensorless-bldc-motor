/******************************************************************************
* File Name: psink.h
*
* Description: This header file defines function prototypes for power consumer
*              path control and fault detection as part of the PMG1 MCU USB-PD
*              Sink Code Example for ModusToolBox.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2022-2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
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
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#ifndef _PSINK_H_
#define _PSINK_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/
#include "cy_pdstack_common.h"

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/

/**
 * @brief This function sets the expected VBus voltage when
 * PMG1 is functioning as a sink. The voltage level is used
 * to configure the Over-Voltage Protection on the device.
 *
 * @param port Port index the function is performed for.
 * @param volt_mV Expected VBus voltage level in mV units.
 *
 * @return None
 */
void psnk_set_voltage (cy_stc_pdstack_context_t * context, uint16_t volt_mV);

/**
 * @brief This function notifies the application code about
 * the amount of current the system is allowed to take from
 * the VBus power supply. The application logic should configure its
 * load and battery charging circuits based on this value so that
 * the power source does not see any overload condition.
 *
 * @param port Port index the function is performed for.
 * @param cur_10mA Maximum allowed current in 10mA units.
 *
 * @return None
 */
void psnk_set_current (cy_stc_pdstack_context_t * context, uint16_t cur_10mA);

/**
 * @brief Function to enable the power consumer path to that the system
 * can received power from the Type-C VBus. The expected voltage and maximum
 * allowed current would have been notified through the psnk_set_voltage()
 * and psnk_set_current() functions.
 *
 * @param port Port index the function is performed for.
 * @return None
 */
void psnk_enable (cy_stc_pdstack_context_t * context);

/**
 * @brief Disable the VBus power sink path and discharge VBus supply down
 * to a safe level. This function is called by the PD stack at times when
 * the system is not allowed to draw power from the VBus supply. The application
 * can use this call to initiate a VBus discharge operation so that a subsequent
 * Type-C connection is speeded up. The snk_discharge_off_handler callback
 * function should be called once VBus has been discharged down to vSafe0V.
 *
 * @param port Port index the function is performed for.
 * @param snk_discharge_off_handler Sink Discharge fet off callback pointer
 * @return None
 */
void psnk_disable (cy_stc_pdstack_context_t * context, cy_pdstack_sink_discharge_off_cbk_t snk_discharge_off_handler);

#endif /* _PSINK_H_ */

/* End of File */
