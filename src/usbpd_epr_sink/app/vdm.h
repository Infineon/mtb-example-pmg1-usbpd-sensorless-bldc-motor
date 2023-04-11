/******************************************************************************
* File Name: vdm.h
*
* Description: This header file defines data structures and function prototypes
*              for the Vendor Defined Message (VDM) handler as part of the PMG1
*              MCU USB-PD Sink Code Example for ModusToolBox.
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

#ifndef _VDM_H_
#define _VDM_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/

#include "cy_pdstack_common.h"

/* Data object index of the ID Header field in the Discover Identity response. */
#define ID_HEADER_DO_INDEX                      (1u)

/* Data object index of the Product VDO field in the Discover Identity response. */
#define PRODUCT_VDO_DO_INDEX                    (3u)

/* Position of Vendor ID portion in ID Header VDO. */
#define ID_HEADER_VID_POS                       (0u)

/* Mask for Vendor ID portion in ID Header VDO. */
#define ID_HEADER_VID_MASK                      (0xFFFFUL)

/* Position of Product ID portion in Product VDO. */
#define PRODUCT_VDO_PID_POS                     (16u)

/* Mask for Product ID portion in Product VDO. */
#define PRODUCT_VDO_PID_MASK                    (0xFFFF0000UL)

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/

/**
 * @brief Store the VDM data from the configuration table.
 *
 * This function retrieves the VDM data (for CCG as UFP) that is stored in
 * the configuration table and stores it in the run-time data structures.
 *
 * @param port USB-PD port for which the data is to be stored.
 *
 * @return None.
 */
void vdm_data_init (cy_stc_pdstack_context_t * context);

/**
 * @brief This function is responsible for analysing and processing received VDM.
 * This function also makes a decision about necessity of response to the received
 * VDM.
 *
 * @param port Port index the function is performed for.
 * @param vdm Pointer to pd packet which contains received VDM.
 * @param vdm_resp_handler VDM handler callback function.
 *
 * @return None
 */
void eval_vdm(cy_stc_pdstack_context_t * context, const cy_stc_pdstack_pd_packet_t *vdm,
        cy_pdstack_vdm_resp_cbk_t vdm_resp_handler);

#endif /* _VDM_H_ */

/* End of File */

