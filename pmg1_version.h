/******************************************************************************
* @file pmg1_version.h
*
* @brief This file defines the version details of the PMG1 Code Example.
*
*******************************************************************************
* Copyright 2022, Cypress Semiconductor Corporation (an Infineon company) or
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

#ifndef _PMG1_VERSION_H_
#define _PMG1_VERSION_H_

/**
   @brief Major version of the PMG1 CE.
 */
#define PMG1_CE_MAJOR_VERSION                                (3)

/**
   @brief Minor version of the PMG1 CE.
 */
#define PMG1_CE_MINOR_VERSION                                (0)

/**
   @brief Patch version of the PMG1 CE.
 */
#define PMG1_CE_PATCH_VERSION                                (0)

/**
   @brief Build number of the PMG1 CE. Base Build number: 0000
          When the value reaches 9999 this shall be reset to 0.
 */
#define PMG1_CE_BUILD_NUMBER                                 (3)

/**
 *  @brief Composite PMG1 CE version value.
 *
 *  PMG1 CE version value. This is a 4 byte value with the following format:
 *  Bytes 1-0: Build number
 *  Byte    2: Patch version
 *  Byte 3 (Bits 0:3): Minor Version
 *  Byte 3 (Bits 4:7): Major Version
 */
#define PMG1_CE_BASE_VERSION                                             \
        ((PMG1_CE_MAJOR_VERSION << 28) | (PMG1_CE_MINOR_VERSION << 24) |      \
        (PMG1_CE_PATCH_VERSION << 16) | (PMG1_CE_BUILD_NUMBER))

#endif /* _PMG1_VERSION_H_ */

/* End of file */
