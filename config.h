/******************************************************************************
* File Name: config.h
*
* Description: This header file defines the application configuration for the PMG1
*              MCU USBPD EPR Sink Example for ModusToolBox.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2021-2022, Cypress Semiconductor Corporation (an Infineon company) or
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

#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "cybsp.h"

/* Restrict PD stack to sink operation. */
#ifndef CY_PD_SINK_ONLY
#define CY_PD_SINK_ONLY                         (1u)
#endif /* CY_PD_SINK_ONLY */

/* This application only supports devices with a single USB-C Port with Power Delivery Support. */
#ifndef NO_OF_TYPEC_PORTS
#if PMG1_PD_DUALPORT_ENABLE
#define NO_OF_TYPEC_PORTS                       (2u)
#else
#define NO_OF_TYPEC_PORTS                       (1u)
#endif /* PMG1_PD_DUALPORT_ENABLE */
#endif /* NO_OF_TYPEC_PORTS */

/* Enable PD spec Rev 3.x support */
#ifndef CY_PD_REV3_ENABLE
#define CY_PD_REV3_ENABLE                       (1u)
#endif /* CY_PD_REV3_ENABLE */

/* Use the default Source PDO selection algorithm. */
#define PD_PDO_SEL_ALGO                         (0u)

/* PMG1-S0 only: Gate driver which supports pull-up for faster turn off when a fault is detected. */
#define VBUS_FET_CTRL_0                         (1u)

/* PMG1-S0 only: Gate driver which does not support internal pull-up (requires external pull-up for turning off. */
#define VBUS_FET_CTRL_1                         (0u)

/* PMG1-S0: Choose the gate driver which should be used to turn the consumer power path ON. */
#define VBUS_FET_CTRL                           (VBUS_FET_CTRL_0)

/* The ADC which should be used to measure VBus voltage on the Type-C side. */
#define APP_VBUS_POLL_ADC_ID                    (CY_USBPD_ADC_ID_0)

/* Period in ms for turning on VBus FET. */
#define APP_VBUS_FET_ON_TIMER_PERIOD            (5u)

/* Period in ms for turning off VBus FET. */
#define APP_VBUS_FET_OFF_TIMER_PERIOD           (1u)

/*
 * Use AMUXBUS_B analog-MUX bus input to measure VBUS voltage.
 */
#define APP_VBUS_POLL_ADC_INPUT                 (CY_USBPD_ADC_INPUT_AMUX_B)

/*
 * Enable/Disable delay between fault retries for Type-C/PD faults.
 */
#define FAULT_RETRY_DELAY_EN                    (0u)

#if FAULT_RETRY_DELAY_EN

/*
 * Delay between fault retries in ms.
 */
#define FAULT_RETRY_DELAY_MS                    (500u)

#endif /* FAULT_RETRY_DELAY_EN */

/*
 * Enable/Disable delayed infinite fault recovery for Type-C/PD faults.
 * Fault recovery shall be tried with a fixed delay after configured
 * fault retry count is elapsed.
 */
#define FAULT_INFINITE_RECOVERY_EN              (0u)

#if FAULT_INFINITE_RECOVERY_EN

/*
 * Delayed fault recovery period in ms.
 */
#define FAULT_INFINITE_RECOVERY_DELAY_MS        (5000u)

#endif /* FAULT_INFINITE_RECOVERY_EN */

/*
 * Disable PMG1 device reset on error (watchdog expiry or hard fault).
 * NOTE: Enabling this feature can cause unexpected device reset during SWD debug sessions.
 */
#define RESET_ON_ERROR_ENABLE                   (0u)

/*
 * Watchdog reset period in ms. This should be set to a value greater than
 * 500 ms to avoid significant increase in power consumption.
 */
#define WATCHDOG_RESET_PERIOD_MS                (750u)

/* Disable tracking of maximum stack usage. Can be enabled for debug purposes. */
#define STACK_USAGE_CHECK_ENABLE                (0u)

/*
 * Set this to 1 to Shutdown the SNK FET in the application layer in states where power consumption needs to be
 * reduced to standby level.
 */
#define SNK_STANDBY_FET_SHUTDOWN_ENABLE         (1u)

#ifndef SYS_DEEPSLEEP_ENABLE
/* Enable PMG1 deep sleep to save power. */
#define SYS_DEEPSLEEP_ENABLE                    (0u)
#endif /* SYS_DEEPSLEEP_ENABLE */

/*
 * Enable watchdog hardware reset for CPU lock-up recovery. Note that watchdog reset can only be enabled if we have
 * any periodic timers running in the application.
 */
#if RESET_ON_ERROR_ENABLE
#define WATCHDOG_HARDWARE_RESET_ENABLE          (1u)
#else
#define WATCHDOG_HARDWARE_RESET_ENABLE          (0u)
#endif /* RESET_ON_ERROR_ENABLE */

#define EPR_SNK_ENTRY_TIMER_PERIOD              (100u)

#endif /* _CONFIG_H_ */

/* End of file */
