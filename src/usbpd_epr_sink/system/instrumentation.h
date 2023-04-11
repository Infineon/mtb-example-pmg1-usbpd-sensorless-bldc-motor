/******************************************************************************
* File Name: instrumentation.h
*
* Description: This header file defines data structures and function prototypes
*              to monitor CPU resource (execution time and stack) usage in the
*              PMG1 MCU USB-PD Sink Code Example for ModusToolBox.
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

#ifndef _INSTRUMENTATION_H_
#define _INSTRUMENTATION_H_

#include <stdint.h>

/**
 * @brief Enumeration of all instrumentation fault events.
 */
typedef enum instrumentation_events
{
    INST_EVT_WDT_RESET = 0,                 /**< 0x00: Instrumentation fault event for watchdog reset. */
    INST_EVT_HARD_FAULT = 1                 /**< 0x01: Instrumentation fault event for hard fault. */
} inst_evt_t;

/**
 * @brief Callback function to solution level handler for instrumentation faults.
 */
typedef void (*instrumentation_cb_t)(uint8_t port, uint8_t evt);

/**
 * @brief Initialize data structures associated with application instrumentation.
 * @return None
 */
void instrumentation_init(void);

/**
 * @brief Start any timers or tasks associated with application instrumentation.
 * @return None
 */
void instrumentation_start(void);

/**
 * @brief Perform tasks associated with application instrumentation. The specific
 * functionality implemented is user defined and can vary.
 * @return None
 */
void instrumentation_task(void);

/**
 * @brief Register solution level callback function to be executed when instrumentation fault occurs.
 * @return None
 */
void instrumentation_register_cb(instrumentation_cb_t cb);

#endif /* _INSTRUMENTATION_H_ */

