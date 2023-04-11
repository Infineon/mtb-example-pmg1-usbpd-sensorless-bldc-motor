/******************************************************************************
* File Name: pdo.h
*
* Description: This header file implements defines function prototypes for the
*              source capability (PDO) evaluation functions which are part of
*              the PMG1 MCU USB-PD Sink Code Example for ModusToolBox.
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

#ifndef _PDO_H_
#define _PDO_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/

#include "cy_pdstack_common.h"

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/

/**
 * @brief This function is called by the PD stack to allow the application
 * logic to evaluate the Source Capabilities received from the port partner
 * and generate the desired request. The request object is expected to be
 * passed back to the stack through the app_resp_handler() callback.
 *
 * The default implementation of this function matches each of the received
 * source PDOs against the active sink capabilities; and then selects the
 * source PDO that can deliver the maximum power to the system as a sink.
 *
 * @param port Port index the function is performed for.
 * @param srcCap Pointer to PD packet which contains source capabilities.
 * @param app_resp_handler Application handler callback function.
 *
 * @return None
 */
void eval_src_cap(cy_stc_pdstack_context_t* context, const cy_stc_pdstack_pd_packet_t* srcCap,
        cy_pdstack_app_resp_cbk_t app_resp_handler) ;

/**
 * @brief This function is called by the PD stack to allow the application
 * to evaluate a power request data object received from the port partner and
 * decide whether it should be satisfied. The response to the request should
 * be passed back to the stack through the app_resp_handler() callback.
 *
 * @param port Port index the function is performed for.
 * @param rdo The request data object received.
 * @param app_resp_handler Application handler callback function.
 *
 * @return None
 */
void eval_rdo(cy_stc_pdstack_context_t* context, cy_pd_pd_do_t rdo, cy_pdstack_app_resp_cbk_t app_resp_handler) ;

#endif /* _PDO_H_ */

/* End of File */

