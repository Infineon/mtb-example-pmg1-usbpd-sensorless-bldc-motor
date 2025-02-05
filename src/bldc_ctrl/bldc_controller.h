/*******************************************************************************
* File Name: bldc_controller.h
* Version 1.0
*
* Description:
*  This file contains the structure declaration and function prototypes used in
*  the bldc_controller source code and other source files.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2025, Cypress Semiconductor Corporation (an Infineon company) or
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
#ifndef __BLDC_CONTROLLER_H__
#define __BLDC_CONTROLLER_H__

/******************************************************************************
 * Header file including
 ******************************************************************************/


/******************************************************************************
 * Macro declaration
 ******************************************************************************/
#define CY_ASSERT_FAILED                      (0u)

/******************************************************************************
 * Structure/Enum type declaration
 ******************************************************************************/
typedef struct
{
    volatile bool bldc_on_flag;
    volatile bool closedLoop;
    volatile uint8_t startup_cycle_cnt;
    volatile bool commutateFlag;
    volatile uint8_t opamp_intr_cnt;
    volatile uint8_t sectorCnt;
    volatile uint8_t directionReverse;
    volatile uint8_t closedloop_ctrl_delay_cnt;
    volatile uint8_t bemf_ch;
    volatile uint8_t sector;
    volatile uint8_t ctrl_interval;
    volatile uint8_t errorCode;
    uint8_t uv_counter;
    uint8_t ov_counter;
    uint8_t OC_Counter;
    bool sw_timer_flag;
    uint8_t errorFlag;
    uint8_t commutate_delay_cnt;
    uint8_t zerocross_delay_cnt;

    volatile uint16_t startup_timer_period;
    uint16_t pwmCompare;

    volatile uint32_t speed_capture_val;
    uint32_t ref_speed_capture_val;

    volatile int32_t speed_capture_old_val;
    int32_t ctrl_error; /* Unsigned: error can be +/- */
    int32_t diff_error;  /* Differential error */
}bldc_controller_status_t;

/******************************************************************************
 * Global variables declaration
 ******************************************************************************/


/******************************************************************************
 * Global function declaration
 ******************************************************************************/
void BLDC_Peripheral_Init(void);
void BLDC_Motor_Init(void);
void BLDC_Start(void);
void BLDC_Stop(void);
void Set_PeriClk_Divider_Val(uint32_t periDiv);
bldc_controller_status_t* bldc_controller_get_status(void);
cy_stc_tcpwm_counter_config_t* CYBSP_CONTROL_TIMER_get_config(void);
cy_stc_sysint_t* ctrl_timer_intr_get_config(void);

/******************************************************************************
 * End of declaration
 ******************************************************************************/
#endif  /* __BLDC_CONTROLLER_H__ */

/* [] END OF FILE */
