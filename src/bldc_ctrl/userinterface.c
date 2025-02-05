/*******************************************************************************
* File Name: userinterface.c
* Version 1.0
*
* Description:
*  This file provides the source code to the various user interfaces in this project
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
#include "cybsp.h"
#include "cy_pdl.h"
#include "userinterface.h"
#include "bldc_controller.h"
#include "parameters.h"


/*******************************************************************************
* Macros
*******************************************************************************/
#define SW_DEBOUNCE_DELAY           (25u)


/******************************************************************************
 * Global variables definition
 ******************************************************************************/
/* Flags to detect switch press event */
volatile bool gl_ctrl_switch_press_flag = 0;

/* Variables for LED timer */
volatile uint8_t gl_led_blink_cnt;
volatile uint8_t gl_timerCnt = 0;
volatile uint8_t gl_switch_timer_intr_cnt = 0;

/*******************************************************************************
* Function Name: ErrorHandler
********************************************************************************
* Summary:
* - This function is used for indicating various types of static as well as run-time errors
* - It is used for setting the blink-rate of the user LED by controlling the LED timer
*
* Parameters:
*  void:
*
* Return:
*  void
*
*******************************************************************************/
void ErrorHandler(void)
{
    cy_rslt_t result;
    bldc_controller_status_t *ptr_bldc_status = bldc_controller_get_status();
    cy_stc_tcpwm_counter_config_t* ptr_ctrl_timer_config = CYBSP_CONTROL_TIMER_get_config();
    cy_stc_sysint_t* ptr_ctrl_timer_intr_config = ctrl_timer_intr_get_config();

    switch(ptr_bldc_status->errorCode)
    {
        case UNDER_VOLTAGE: /* Under-voltage is detected */
            /* Blink the user LED in sets of 2 */
            gl_led_blink_cnt = 2 * UV_ERROR_BLNK_CNT - 1;
            break;

        case OVER_VOLTAGE: /* Over-voltage is detected */
            /* Blink the user LED in sets of 3 */
            gl_led_blink_cnt = 2 * OV_ERROR_BLNK_CNT - 1;
            break;

        case OVER_CURRENT: /* Over-current on winding is detected */
            /* Blink the user LED in sets of 4 */
            gl_led_blink_cnt = 2 * OC_ERROR_BLNK_CNT - 1;
            break;

        case OVERLOAD: /* Motor is overloaded at low RPM */
            /* Blink the user LED in sets of 6 */
            gl_led_blink_cnt = 2 * OL_ERROR_BLNK_CNT - 1;
            break;

        case COMMUTATION_ERROR: /* Unexpected commutation failures, jerks, winding damages etc. detected */
            /* Blink the user LED continuously at period specified by LED_BLNK_PERIOD */
            gl_led_blink_cnt = COMM_ERROR_BLNK_CNT;
            break;

        default: /* No errors detected */
            break;
    }
    /* Conditional loop to trigger LED timer in case an error is detected */
    if((ptr_bldc_status->errorFlag == 1) && (!ptr_bldc_status->sw_timer_flag))
    {
        /* Initialize the interrupt vector table with the LED Timer interrupt handler address */
        result = Cy_SysInt_Init(ptr_ctrl_timer_intr_config, LEDTimer_ISR);
        if(result != CY_SYSINT_SUCCESS)
        {
            CY_ASSERT(CY_ASSERT_FAILED);
        }

        /* Modify the LED Timer period */
        ptr_ctrl_timer_config->period = LED_BLNK_INTRVL;

        /* Reinitialize the counter after parameter modification */
        result = Cy_TCPWM_Counter_Init(CYBSP_CONTROL_TIMER_HW, CYBSP_CONTROL_TIMER_NUM, ptr_ctrl_timer_config);
        if(result != CY_TCPWM_SUCCESS)
        {
            CY_ASSERT(CY_ASSERT_FAILED);
        }

        /* Setting the error flag to acknowledge the occurrence of an error */
        ptr_bldc_status->sw_timer_flag = 1;

        /* Initialize the LED state */
        Cy_GPIO_Set(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);

        /* Trigger start the LED Timer */
        Cy_TCPWM_TriggerReloadOrIndex(TCPWM, CYBSP_CONTROL_TIMER_MASK);
    }
}

/*******************************************************************************
* Function Name: Ctrl_SwitchControl
********************************************************************************
*
* Summary:
* - This function handles the switch press event for CTRL_SW
* - Performs: Motor ON/OFF (press CTRL_SW once),
* - Reset the controller after the occurrence of an error/fault (press CTRL_SW twice),
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void Ctrl_SwitchControl(void)
{
    bldc_controller_status_t* ptr_bldc_status = bldc_controller_get_status();

    /* Check if Ctrl Switch is pressed */
    if(gl_ctrl_switch_press_flag)
    {
        /* Wait for 25 milliseconds for switch de-bounce */
        Cy_SysLib_Delay(SW_DEBOUNCE_DELAY);

        /* Check if switch input is held low */
        if(Cy_GPIO_Read(CTRL_SW_PORT, CTRL_SW_PIN))
        {
            /* Stop switch timer and continue normal switch operations (ON/OFF/Reset) */
            Cy_TCPWM_TriggerStopOrKill(TCPWM, CYBSP_CONTROL_TIMER_MASK);

            /* 5ms delay between successive timer triggers to ensure fail-safe operation */
            Cy_SysLib_Delay(5u);

            gl_switch_timer_intr_cnt = 0;

            /* Check if motor is OFF and start motor only if no error is detected */
            if(!ptr_bldc_status->bldc_on_flag && gl_ctrl_switch_press_flag)
            {
                if(!ptr_bldc_status->errorFlag)
                {
                    /* Start the motor */
                    BLDC_Start();
                }
                else if (ptr_bldc_status->errorFlag == SW_RESET_CNT)
                {
                    /* Clear the error flag */
                    ptr_bldc_status->errorFlag = 0;

                    /* Clear the error-code and switch-timer-flag */
                    ptr_bldc_status->sw_timer_flag = 0;
                    ptr_bldc_status->errorCode = NO_ERROR;

                    /* Reset the controller after an error event for a fresh start */
                    BLDC_Stop();
                }
                else
                {
                    ptr_bldc_status->errorFlag++;
                    /* Turn OFF the user LED when control is reset */
                    Cy_GPIO_Set(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);
                }
            }

            /* Check if motor is ON */
            else if(ptr_bldc_status->bldc_on_flag)
            {
                /* Stop the motor */
                BLDC_Stop();
            }
            /* Clear the switch press event */
            gl_ctrl_switch_press_flag = 0;
        }

    }
}


/*******************************************************************************
* Function Name: Ctrl_Switch_ISR
********************************************************************************
*
* Summary:
* This is the interrupt handler for CTRL_SW interrupt to turn the motor ON/OFF
* This switch is also used to reset the controller in case of unexpected errors/faults
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void Ctrl_Switch_ISR(void)
{
    bldc_controller_status_t* ptr_bldc_status = bldc_controller_get_status();

    /* Set Ctrl Switch press flag to 1 */
    gl_ctrl_switch_press_flag = 1;

    if(!ptr_bldc_status->bldc_on_flag)
        /* Trigger start switch timer */
        Cy_TCPWM_TriggerReloadOrIndex(TCPWM, CYBSP_CONTROL_TIMER_MASK); /* Start the switch timer while switching the motor ON */

    /* Clear the switch interrupt interrupt */
    Cy_GPIO_ClearInterrupt(CTRL_SW_PORT, CTRL_SW_NUM);
}


/*******************************************************************************
* Function Name: SwitchTimer_ISR
********************************************************************************
*
* Summary:
* This is the interrupt handler for CTRL_SW interrupt to handle the
* switch press & hold logic for motor direction reversal
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void SwitchTimer_ISR(void)
{
    bldc_controller_status_t* ptr_bldc_status = bldc_controller_get_status();

    /* Clear the terminal count/capture interrupt of the timer */
    Cy_TCPWM_ClearInterrupt(CYBSP_CONTROL_TIMER_HW, CYBSP_CONTROL_TIMER_NUM, CY_TCPWM_INT_ON_CC_OR_TC);

    if(gl_switch_timer_intr_cnt < SW_HOLD_CNT)
        gl_switch_timer_intr_cnt++;
    else
    {
        /* Stop switch timer and continue normal switch operations (ON/OFF/Reset) */
        Cy_TCPWM_TriggerStopOrKill(TCPWM, CYBSP_CONTROL_TIMER_MASK);

        gl_switch_timer_intr_cnt = 0;

        if(ptr_bldc_status->directionReverse)
            /* Enable default direction for rotation */
            ptr_bldc_status->directionReverse = 0;
        else
            /* Enable reverse direction for rotation */
            ptr_bldc_status->directionReverse = 1;

        BLDC_Motor_Init();

        for(uint8_t n = 0; n < (2 * SW_TIMER_BLNK_CNT); n++)
        {
            Cy_GPIO_Inv(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);
            Cy_SysLib_Delay(60u);
        }

        /* Clear the switch press event */
        gl_ctrl_switch_press_flag = 0;
    }
}


/*******************************************************************************
* Function Name: LEDTimer_ISR
********************************************************************************
* Summary:
* - This timer is used to control the user LED blink rate to indicate different types of errors
*
* Parameters:
*  void:
*
* Return:
*  void
*
*******************************************************************************/
void LEDTimer_ISR(void)
{
    cy_rslt_t result;
    cy_stc_tcpwm_counter_config_t* ptr_ctrl_timer_config = CYBSP_CONTROL_TIMER_get_config();

    /* Controls the blink-rate for LED */
    if(gl_timerCnt < gl_led_blink_cnt)
    {
        /* Modify the LED Timer period for LED blink */
        ptr_ctrl_timer_config->period = LED_BLNK_INTRVL;

        /* Reinitialize the counter after parameter modification */
        result = Cy_TCPWM_Counter_Init(CYBSP_CONTROL_TIMER_HW, CYBSP_CONTROL_TIMER_NUM, ptr_ctrl_timer_config);
        if(result != CY_TCPWM_SUCCESS)
        {
            CY_ASSERT(CY_ASSERT_FAILED);
        }

        /* Toggle the user LED */
        Cy_GPIO_Inv(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);

        gl_timerCnt++;
    }
    /* Controls the delay between sets of blinks */
    else
    {
        gl_timerCnt = 0;

        /* Toggle the user LED */
        Cy_GPIO_Inv(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);

        /* Modify the LED Timer period to set the delay between sets of blinks */
        ptr_ctrl_timer_config->period = LED_BLNK_PERIOD;

        /* Reinitialize the counter after parameter modification */
        result = Cy_TCPWM_Counter_Init(CYBSP_CONTROL_TIMER_HW, CYBSP_CONTROL_TIMER_NUM, ptr_ctrl_timer_config);
        if(result != CY_TCPWM_SUCCESS)
        {
            CY_ASSERT(CY_ASSERT_FAILED);
        }
    }

    /* Clear the terminal count/capture interrupt of the timer */
    Cy_TCPWM_ClearInterrupt(CYBSP_CONTROL_TIMER_HW, CYBSP_CONTROL_TIMER_NUM, CY_TCPWM_INT_ON_CC_OR_TC);
}

/* [] END OF FILE */
