/*******************************************************************************
* File Name: control.c
* Version 1.20
*
* Description:
*  This file provides the source code to the PID Control implementation
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
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
#include "cybsp.h"
#include "cy_pdl.h"
#include "control.h"
#include "bldc_controller.h"
#include "parameters.h"


/******************************************************************************
 * Global variables definition
 ******************************************************************************/


/*******************************************************************************
* Function Name: RefSpeedCheck
********************************************************************************
* Summary:
* This function calculates the reference speed value based on signal read through ADC channel-0
*
* Parameters:
*  void:
*
* Return:
*  void
*
*******************************************************************************/
void RefSpeedCheck(void)
{
    bldc_controller_status_t *ptr_bldc_status = bldc_controller_get_status();
    uint16_t adc_result_ch0;

    /* Read the ADC channel 'SPEED_REF_CHAN' */
     adc_result_ch0 = Cy_SAR_GetResult16(SAR0, SPEED_REF_CHAN);

     /* Calculate the required reference capture value corresponding to the time interval between two consecutive zero-crossings */
     ptr_bldc_status->ref_speed_capture_val = (((SPEED_CAPTURE_MAX - SPEED_CAPTURE_MIN) * adc_result_ch0) / ADC_RSLT_RANGE) + SPEED_CAPTURE_MIN;
}


/*******************************************************************************
* Function Name: SpeedCounter_ISR
********************************************************************************
* Summary:
* This is the interrupt handler for SpeedCounter
* - It calculates the current running speed
* - Compares with the reference speed to generate error
* - Runs the PID control loop to update the PWM duty cycle
*
* Parameters:
*  void:
*
* Return:
*  void
*
*******************************************************************************/
void SpeedCounter_ISR(void)
{
    bldc_controller_status_t *ptr_bldc_status = bldc_controller_get_status();

    /* Used for setting the time delay for control actuation after the loop is closed
     * This time is required for the BEMF signal shape settle down between that of open-loop and closed-loop */
    if(ptr_bldc_status->closedloop_ctrl_delay_cnt < CONTROL_START_DELAY) /* Control starts at the fourth commutation (CRITICAL)*/
    {
        ptr_bldc_status->closedloop_ctrl_delay_cnt++;
    }

    /* Execute control only while in closed-loop */
    else if(ptr_bldc_status->closedLoop)
    {
        /* Variable to set the error measuring and control variable update frequency */
        ptr_bldc_status->ctrl_interval++;

        /* Read the captured value of the SpeedCounter, corresponding to the time interval between two consecutive zero-crossings */
        ptr_bldc_status->speed_capture_val = Cy_TCPWM_Counter_GetCapture(TCPWM, CYBSP_CONTROL_TIMER_NUM);

        /*----------------------------------------------------------------------------------*/
        /* Run-time ERROR detection */
        if((ptr_bldc_status->speed_capture_val > SPEED_CAPTURE_THRESHOLD_MAX) && ptr_bldc_status->bldc_on_flag)
        {
            BLDC_Stop(); /* Stop the BLDC motor */
            ptr_bldc_status->errorCode = OVERLOAD; /* Over load; Reached minimum threshold RPM */
            ptr_bldc_status->errorFlag = 1;
        }

        if((ptr_bldc_status->speed_capture_val < SPEED_CAPTURE_THRESHOLD_MIN) && ptr_bldc_status->bldc_on_flag)
        {
            BLDC_Stop(); /* Stop the BLDC motor */
            ptr_bldc_status->errorCode = COMMUTATION_ERROR; /* Detected irregularities in commutation */
            ptr_bldc_status->errorFlag = 1;
        }

        /* This threshold value may be tuned; value '1000' can sense small jerks as occurring here */
        /* Convert to signed integer for for signed calculations */
        if(((((int32_t)ptr_bldc_status->speed_capture_val - ptr_bldc_status->speed_capture_old_val) > SPEED_CAPTURE_STEP_THRESHOLD) || ((ptr_bldc_status->speed_capture_old_val - (int32_t)ptr_bldc_status->speed_capture_val) > SPEED_CAPTURE_STEP_THRESHOLD)) && ptr_bldc_status->bldc_on_flag)
        {
            BLDC_Stop(); /* Stop the BLDC motor */
            ptr_bldc_status->errorCode = COMMUTATION_ERROR; /* Detect unexpected motor failure, jerk, winding damage etc. */
            ptr_bldc_status->errorFlag = 1;
        }
        ptr_bldc_status->speed_capture_old_val = (int32_t)ptr_bldc_status->speed_capture_val; /* Store the previous capture value (RPM info) */
        /* Run-time ERROR detection */
        /*----------------------------------------------------------------------------------*/

        /* Control update interval */
        /* Measure error and update control variable once in 3 commutation cycles to reduce oscillations */
        if(ptr_bldc_status->ctrl_interval == CONTROL_UPDATE_INTERVAL)
        {
            if((ptr_bldc_status->speed_capture_val > SPEED_CAPTURE_MIN) && (ptr_bldc_status->speed_capture_val <= SPEED_CAPTURE_MID))
            {
                /* Adjust the commutation delay in closed-loop, based on the captured value of the motor speed (CRITICAL) */
                ptr_bldc_status->commutate_delay_cnt = (uint8_t)(((uint8_t)(ptr_bldc_status->speed_capture_val * CAPTURE_DELAY_CONVERT_FACTOR / SPEED_COUNTER_PERIOD_CNT)) - DELAY_OFFSET_TUNE);
            }

            else if((ptr_bldc_status->speed_capture_val > SPEED_CAPTURE_MID) && (ptr_bldc_status->speed_capture_val < SPEED_CAPTURE_MAX))
            {
                /* Limit the commutation delay adjustment at lower speed to account for the non-linearity in speed vs PWM duty at very low speed */
                ptr_bldc_status->commutate_delay_cnt = (uint8_t)(ptr_bldc_status->speed_capture_val * LS_CAPTURE_DELAY_CONVERT_FACTOR / SPEED_COUNTER_PERIOD_CNT);
            }

            /* Commutation delay to switch the high-side MOSFETs */
            ptr_bldc_status->zerocross_delay_cnt = ptr_bldc_status->commutate_delay_cnt/2 + 0;


        /*--------------------------------------------------------------------*/
        /* PID CONTROL LOOP */
            /* Measure the SpeedError */
            ptr_bldc_status->ctrl_error = ptr_bldc_status->speed_capture_val - ptr_bldc_status->ref_speed_capture_val;

            /* Measure the Differential SpeedError */
            ptr_bldc_status->diff_error = ptr_bldc_status->ctrl_error - ptr_bldc_status->diff_error;

            /* Update the integrated value of control signal */
            ptr_bldc_status->pwmCompare += (uint16_t)((KI_VALUE)*(KP_VALUE * ptr_bldc_status->ctrl_error + KD_VALUE * ptr_bldc_status->diff_error));

            if(ptr_bldc_status->pwmCompare > PWM_COMPARE_MAX)
            {
                /* Maximum drive capability to prevent commutation failures */
                ptr_bldc_status->pwmCompare = PWM_COMPARE_MAX;
            }

            if(ptr_bldc_status->pwmCompare < PWM_COMPARE_MIN)
            {
                /* Minimum drive capability to prevent commutation failures */
                ptr_bldc_status->pwmCompare = PWM_COMPARE_MIN;
            }

            /* Store the previous value of SpeedError */
            ptr_bldc_status->diff_error = ptr_bldc_status->ctrl_error;

            ptr_bldc_status->ctrl_interval = 0;
        }

        /* Update the PWM compare value for the new duty cycle */
        Cy_TCPWM_PWM_SetCompare0(CYBSP_PWM_AH_HW, CYBSP_PWM_AH_NUM, ptr_bldc_status->pwmCompare);
        Cy_TCPWM_PWM_SetCompare0(CYBSP_PWM_BH_HW, CYBSP_PWM_BH_NUM, ptr_bldc_status->pwmCompare);
        Cy_TCPWM_PWM_SetCompare0(CYBSP_PWM_CH_HW, CYBSP_PWM_CH_NUM, ptr_bldc_status->pwmCompare);
        /* PID CONTROL LOOP */
        /*--------------------------------------------------------------------*/

    }
    /* Reload the SpeedCounter with the initial value for the next capture event */
    Cy_TCPWM_TriggerReloadOrIndex(TCPWM, CYBSP_CONTROL_TIMER_MASK);

    /* Clear SpeedCounter interrupt */
    Cy_TCPWM_ClearInterrupt(CYBSP_CONTROL_TIMER_HW, CYBSP_CONTROL_TIMER_NUM, CY_TCPWM_INT_ON_CC);
}
/* [] END OF FILE */
