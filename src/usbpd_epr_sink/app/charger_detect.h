/******************************************************************************
* File Name: charger_detect.h
*
* Description: This header file defines the data structures and function
*              prototypes associated with the BC 1.2 (legacy) charger detect
*              module as part of the PMG1 MCU USBPD Sink Example for
*              ModusToolBox.
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

#ifndef __CHARGER_DETECT_H__
#define __CHARGER_DETECT_H__

/*******************************************************************************
 * Header files including
 ******************************************************************************/

#include "config.h"
#include "cy_usbpd_common.h"
#include "cy_usbpd_bch.h"
#include "cy_pdstack_common.h"

/*******************************************************************************
 * MACRO Definition
 ******************************************************************************/

#define BC_CMP_0_IDX                    (0u)    /**< Battery charger comparator #1. */
#define BC_CMP_1_IDX                    (1u)    /**< Battery charger comparator #2. */

/*******************************************************************************
 * Data Structure Definition
 ******************************************************************************/

/**
 * @typedef chgdet_fsm_evt_t
 * @brief List of charger detect events notified by the PDL.
 */
typedef enum
{
    CHGDET_FSM_EVT_ENTRY = 0,                   /**<  0: Charger Detect Event: State entry. */
    CHGDET_FSM_EVT_CMP1_FIRE,                   /**<  1: Charger Detect Event: CMP1 interrupt. */
    CHGDET_FSM_EVT_CMP2_FIRE,                   /**<  2: Charger Detect Event: CMP2 interrupt. */
    CHGDET_FSM_EVT_QC_CHANGE,                   /**<  3: Charger Detect Event: QC state change. */
    CHGDET_FSM_EVT_QC_CONT,                     /**<  4: Charger Detect Event: QC continuous mode entry. */
    CHGDET_FSM_EVT_AFC_RESET_RCVD,              /**<  5: Charger Detect Event: AFC reset received. */
    CHGDET_FSM_EVT_AFC_MSG_RCVD,                /**<  6: Charger Detect Event: AFC message received. */
    CHGDET_FSM_EVT_AFC_MSG_SENT,                /**<  7: Charger Detect Event: AFC message sent. */
    CHGDET_FSM_EVT_AFC_MSG_SEND_FAIL,           /**<  8: Charger Detect Event: AFC message sending failed. */
    CHGDET_FSM_EVT_TIMEOUT1,                    /**<  9: Charger Detect Event: Timer1 expiry interrupt. */
    CHGDET_FSM_EVT_TIMEOUT2,                    /**< 10: Charger Detect Event: Timer2 expiry interrupt. */
    CHGDET_FSM_EVT_DISCONNECT,                  /**< 11: Charger Detect Event: Device disconnect. */
    CHGDET_FSM_MAX_EVTS                         /**< 12: Number of events. */
} chgdet_fsm_evt_t;

/**
 * @brief Union to hold Dp/Dm status.
 */
typedef union
{
    uint16_t state;                                     /**< Combined status of Dp and Dm. */
    uint8_t  d[2];                                      /**< Individual status of Dp(d[0]) and Dm(d[1]). */
} chgdet_dp_dm_state_t;

/**
 * @typedef chgdet_state_t
 * @brief List of states in the legacy battery charging state machine.
 */
typedef enum{
    CHGDET_FSM_OFF = 0,                                 /**< BC state machine inactive. */
    CHGDET_FSM_SINK_START,                              /**< BC sink state machine start state. */
    CHGDET_FSM_SINK_PRIMARY_CHARGER_DETECT,             /**< BC 1.2 primary charger detect state. */
    CHGDET_FSM_SINK_TYPE_C_ONLY_SOURCE_CONNECTED,       /**< BC 1.2 src detection failed, connected as Type-C sink. */
    CHGDET_FSM_SINK_SECONDARY_CHARGER_DETECT,           /**< BC 1.2 secondary charger detect state. */
    CHGDET_FSM_SINK_DCP_CONNECTED,                      /**< Sink connected to a BC 1.2 DCP. */
    CHGDET_FSM_SINK_SDP_CONNECTED,                      /**< Sink connected to a Standard Downstream Port (SDP). */
    CHGDET_FSM_SINK_CDP_CONNECTED,                      /**< Sink connected to a BC 1.2 CDP. */
    CHGDET_FSM_MAX_STATES,                              /**< Invalid state ID. */
} chgdet_state_t;

/**
 * @typedef chgdet_timer_t
 * @brief List of soft timers used by the charger detect state machine.
 */
typedef enum
{
    CHGDET_TIMER_NONE = 0,                             /**< No timers running. */
    CHGDET_DCD_DEBOUNCE_TIMER = 1                      /**< DCD Debounce timer. */
} chgdet_timer_t;

/*******************************************************************************
 * Data Struct Definition
 ******************************************************************************/

/**
 * @brief Struct to define battery charger status.
 */
typedef struct {
    chgdet_state_t chgdet_fsm_state;            /**< Current state of the BC state machine. */
    uint32_t       chgdet_evt;                  /**< Bitmap representing event notifications to the state machine. */
    uint16_t       cur_volt;                    /**< Active VBus voltage in mV units. */
    uint16_t       cur_amp;                     /**< Active supply current in 10 mA units. */

    chgdet_dp_dm_state_t dp_dm_status;          /**< Debounced status of the DP/DM pins. */
    chgdet_dp_dm_state_t old_dp_dm_status;      /**< Previous status of the DP/DM pins. */
    chgdet_timer_t       cur_timer;             /**< Identifies the timer that is running. */

    bool comp_rising;                           /**< Whether comparator is looking for a rising edge or falling edge. */
    bool connected;                             /**< Whether charger connection is detected. */
    bool attach;                                /**< Whether charger attach has been detected. */
} chgdet_status_t;

/*******************************************************************************
 * Global Function Declaration
 ******************************************************************************/

/**
 * @brief This function initializes the Charger Detect block. This should be
 * called one time only at system startup for each port on which the charger detect
 * state machine needs to run.
 *
 * @param stack_ctx PD Stack context pointer.
 * @return cy_en_usbpd_status_t
 */
cy_en_usbpd_status_t chgdet_init(cy_stc_pdstack_context_t *stack_ctx);

/**
 * @brief This function starts the Charger Detect block operation after Type-C
 * connection has been detected.
 *
 * @param stack_ctx PD Stack context pointer.
 * @return cy_en_usbpd_status_t
 */
cy_en_usbpd_status_t chgdet_start(cy_stc_pdstack_context_t *stack_ctx);

/**
 * @brief This function stops the Charger Detect block operation once Type-C
 * disconnect has been detected.
 *
 * @param stack_ctx PD Stack context pointer.
 * @return cy_en_usbpd_status_t
 */
cy_en_usbpd_status_t chgdet_stop(cy_stc_pdstack_context_t *stack_ctx);

/**
 * @brief This function returns whether the Charger Detect module is active or not.
 *
 * @param stack_ctx PD Stack context pointer.
 * @return true if the charger detect module is running, false otherwise.
 */
bool chgdet_is_active(cy_stc_pdstack_context_t *stack_ctx);

/**
 * @brief This function performs actions associated with the Charger Detect state
 * machine, and is expected to be called from the application main.
 *
 * @param stack_ctx PD Stack context pointer.
 * @return cy_en_usbpd_status_t
 */
cy_en_usbpd_status_t chgdet_task(cy_stc_pdstack_context_t *stack_ctx);

/**
 * @brief This function prepares the charger detect block for device entry into deep sleep
 * state.
 *
 * @param stack_ctx PD Stack context pointer.
 * @return Returns true if the deep sleep mode can be entered, false otherwise.
 */
bool chgdet_prepare_deepsleep(cy_stc_pdstack_context_t *stack_ctx);

/**
 * @brief This function restores the charger detect block into functional state after the
 * PMG1 device wakes from deep sleep.
 *
 * @param stack_ctx PD Stack context pointer.
 * @return void
 */
void chgdet_resume(cy_stc_pdstack_context_t *stack_ctx);

/**
 * @brief This function retrieves the current status of the charger detect state machine.
 *
 * @param stack_ctx PD Stack context pointer.
 * @return Pointer to charger detect status. The structure must not be modified by caller.
 */
const chgdet_status_t* chgdet_get_status(cy_stc_pdstack_context_t *stack_ctx);

/**
 * @brief This function updates the charger detect state machine based on event notifications
 * from the USB-PD Stack. This event handler calls the chgdet_start and chgdet_stop functions
 * as required.
 *
 * @param stack_ctx PD Stack context pointer.
 * @param evt USB-PD event ID.
 * @param dat Optional data associated with the event.
 * @return void
 */
void chgdet_pd_event_handler(cy_stc_pdstack_context_t *stack_ctx, cy_en_pdstack_app_evt_t evt, const void* dat);

/**
 * @brief This function sets an event status for the charger detect state machine to process.
 *
 * @param stack_ctx PD Stack context pointer.
 * @param evt_mask Mask specifying events to be set.
 *
 * @return void
 */
void chgdet_fsm_set_evt(cy_stc_pdstack_context_t *stack_ctx, uint32_t evt_mask);

/**
 * @brief This function clears one or more events after the charger detect state machine has
 * dealt with them.
 *
 * @param stack_ctx PD Stack context pointer.
 * @param evt_mask Event Mask to be cleared.
 * @return void
 */
void chgdet_fsm_clear_evt(cy_stc_pdstack_context_t *stack_ctx, uint32_t evt_mask);

#endif /* __CHARGER_DETECT_H__ */

/* End of File */
