/*******************************************************************************
* File Name: startup.c
* Version 1.0
*
* Description:
*  This file provides the source code to the Startup algorithm for sensorless BLDC motor
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
#include "startup.h"
#include "bldc_controller.h"
#include "control.h"
#include "parameters.h"


/*******************************************************************************
* Global Variables
*******************************************************************************/


/*******************************************************************************
* Function Name: StartupTimer_ISR
********************************************************************************
* Summary:
* - This timer is used to initially align the rotor during the pre-position phase
* - Drives the motor in open-loop during free-running phase
*
* Parameters:
*  void:
*
* Return:
*  void
*
*******************************************************************************/
void StartupTimer_ISR(void)
{
    cy_rslt_t result;
    bldc_controller_status_t* ptr_bldc_status = bldc_controller_get_status();
    cy_stc_tcpwm_counter_config_t* ptr_ctrl_timer_config = CYBSP_CONTROL_TIMER_get_config();
    cy_stc_sysint_t* ptr_ctrl_timer_intr_config = ctrl_timer_intr_get_config();

    /* Check for open-loop mode */
    if(!ptr_bldc_status->closedLoop)
    {
        if(ptr_bldc_status->directionReverse)
            ptr_bldc_status->sectorCnt--; /* variable to track the current sector number of motor operation */
        else
            ptr_bldc_status->sectorCnt++;

        switch(ptr_bldc_status->sectorCnt)
        {
            case SECTOR_1: /* Pre-position phase: Initial rotor alignment */
                Cy_TCPWM_TriggerStopOrKill(TCPWM, CYBSP_PWM_BH_MASK);    //BH = 0
                Cy_TCPWM_TriggerStopOrKill(TCPWM, CYBSP_PWM_CH_MASK);    //CH = 0
                Cy_TCPWM_TriggerReloadOrIndex(TCPWM, CYBSP_PWM_AH_MASK); //AH = 1

                Cy_GPIO_Clr(PWM_AL_PORT, PWM_AL_PIN); //AL = 0
                Cy_GPIO_Clr(PWM_BL_PORT, PWM_BL_PIN); //BL = 0
                Cy_GPIO_Set(PWM_CL_PORT, PWM_CL_PIN); //CL = 1

                if(ptr_bldc_status->directionReverse)
                {
                    /* Set the counter period with new value */
                    Cy_TCPWM_Counter_SetPeriod(CYBSP_CONTROL_TIMER_HW, CYBSP_CONTROL_TIMER_NUM, ptr_bldc_status->startup_timer_period);
                    ptr_bldc_status->sectorCnt = 6+1;
                }

                break;

            case SECTOR_2:
                if(!ptr_bldc_status->directionReverse)
                {
                    /* Default initial PWM compare value for the motor startup */
                    ptr_bldc_status->pwmCompare = FREERUN_PWM_COMPARE;

                    /* Update the PWM compare value for the startup */
                    Cy_TCPWM_PWM_SetCompare0(CYBSP_PWM_AH_HW, CYBSP_PWM_AH_NUM, ptr_bldc_status->pwmCompare);
                    Cy_TCPWM_PWM_SetCompare0(CYBSP_PWM_BH_HW, CYBSP_PWM_BH_NUM, ptr_bldc_status->pwmCompare);
                    Cy_TCPWM_PWM_SetCompare0(CYBSP_PWM_CH_HW, CYBSP_PWM_CH_NUM, ptr_bldc_status->pwmCompare);
                }

                Cy_TCPWM_TriggerStopOrKill(TCPWM, CYBSP_PWM_AH_MASK);    //AH = 0
                Cy_TCPWM_TriggerStopOrKill(TCPWM, CYBSP_PWM_CH_MASK);    //CH = 0
                Cy_TCPWM_TriggerReloadOrIndex(TCPWM, CYBSP_PWM_BH_MASK); //BH = 1

                Cy_GPIO_Clr(PWM_AL_PORT, PWM_AL_PIN); //AL = 0
                Cy_GPIO_Clr(PWM_BL_PORT, PWM_BL_PIN); //BL = 0
                Cy_GPIO_Set(PWM_CL_PORT, PWM_CL_PIN); //CL = 1

                /* Set the counter period for startup */
                Cy_TCPWM_Counter_SetPeriod(CYBSP_CONTROL_TIMER_HW, CYBSP_CONTROL_TIMER_NUM, ptr_bldc_status->startup_timer_period);

                break;

            case SECTOR_3:
                Cy_TCPWM_TriggerStopOrKill(TCPWM, CYBSP_PWM_AH_MASK);    //AH = 0
                Cy_TCPWM_TriggerStopOrKill(TCPWM, CYBSP_PWM_CH_MASK);    //CH = 0
                Cy_TCPWM_TriggerReloadOrIndex(TCPWM, CYBSP_PWM_BH_MASK); //BH = 1

                Cy_GPIO_Clr(PWM_BL_PORT, PWM_BL_PIN); //BL = 0
                Cy_GPIO_Clr(PWM_CL_PORT, PWM_CL_PIN); //CL = 0
                Cy_GPIO_Set(PWM_AL_PORT, PWM_AL_PIN); //AL = 1

                /* Set the counter period with new value */
                Cy_TCPWM_Counter_SetPeriod(CYBSP_CONTROL_TIMER_HW, CYBSP_CONTROL_TIMER_NUM, ptr_bldc_status->startup_timer_period);

                break;

            case SECTOR_4:
                Cy_TCPWM_TriggerStopOrKill(TCPWM, CYBSP_PWM_AH_MASK);    //AH = 0
                Cy_TCPWM_TriggerStopOrKill(TCPWM, CYBSP_PWM_BH_MASK);    //BH = 0
                Cy_TCPWM_TriggerReloadOrIndex(TCPWM, CYBSP_PWM_CH_MASK); //CH = 1

                Cy_GPIO_Clr(PWM_BL_PORT, PWM_BL_PIN); //BL = 0
                Cy_GPIO_Clr(PWM_CL_PORT, PWM_CL_PIN); //CL = 0
                Cy_GPIO_Set(PWM_AL_PORT, PWM_AL_PIN); //AL = 1

                /* Set the counter period with new value */
                Cy_TCPWM_Counter_SetPeriod(CYBSP_CONTROL_TIMER_HW, CYBSP_CONTROL_TIMER_NUM, ptr_bldc_status->startup_timer_period);

                if(ptr_bldc_status->directionReverse)
                    ptr_bldc_status->startup_cycle_cnt++;
                break;

            case SECTOR_5:
                if(ptr_bldc_status->directionReverse)
                {
                    /* Default initial PWM compare value for the motor startup */
                    ptr_bldc_status->pwmCompare = FREERUN_PWM_COMPARE;

                    /* Update the PWM compare value for the startup */
                    Cy_TCPWM_PWM_SetCompare0(CYBSP_PWM_AH_HW, CYBSP_PWM_AH_NUM, ptr_bldc_status->pwmCompare);
                    Cy_TCPWM_PWM_SetCompare0(CYBSP_PWM_BH_HW, CYBSP_PWM_BH_NUM, ptr_bldc_status->pwmCompare);
                    Cy_TCPWM_PWM_SetCompare0(CYBSP_PWM_CH_HW, CYBSP_PWM_CH_NUM, ptr_bldc_status->pwmCompare);
                }

                Cy_TCPWM_TriggerStopOrKill(TCPWM, CYBSP_PWM_AH_MASK);    //AH = 0
                Cy_TCPWM_TriggerStopOrKill(TCPWM, CYBSP_PWM_BH_MASK);    //BH = 0
                Cy_TCPWM_TriggerReloadOrIndex(TCPWM, CYBSP_PWM_CH_MASK); //CH = 1

                Cy_GPIO_Clr(PWM_AL_PORT, PWM_AL_PIN); //AL = 0
                Cy_GPIO_Clr(PWM_CL_PORT, PWM_CL_PIN); //CL = 0
                Cy_GPIO_Set(PWM_BL_PORT, PWM_BL_PIN); //BL = 1

                /* Set the counter period with new value */
                Cy_TCPWM_Counter_SetPeriod(CYBSP_CONTROL_TIMER_HW, CYBSP_CONTROL_TIMER_NUM, ptr_bldc_status->startup_timer_period);

                if(!ptr_bldc_status->directionReverse)
                    ptr_bldc_status->startup_cycle_cnt++; /* Comment this line to continuously operate in open loop */
                break;

            case SECTOR_6:
                Cy_TCPWM_TriggerStopOrKill(TCPWM, CYBSP_PWM_BH_MASK);    //BH = 0
                Cy_TCPWM_TriggerStopOrKill(TCPWM, CYBSP_PWM_CH_MASK);    //CH = 0
                Cy_TCPWM_TriggerReloadOrIndex(TCPWM, CYBSP_PWM_AH_MASK); //AH = 1

                Cy_GPIO_Clr(PWM_AL_PORT, PWM_AL_PIN); //AL = 0
                Cy_GPIO_Clr(PWM_CL_PORT, PWM_CL_PIN); //CL = 0
                Cy_GPIO_Set(PWM_BL_PORT, PWM_BL_PIN); //BL = 1

                if(!ptr_bldc_status->directionReverse)
                {
                    /* Set the counter period with new value */
                    Cy_TCPWM_Counter_SetPeriod(CYBSP_CONTROL_TIMER_HW, CYBSP_CONTROL_TIMER_NUM, ptr_bldc_status->startup_timer_period);
                    ptr_bldc_status->sectorCnt = 0;
                }
                break;
        }

        if(ptr_bldc_status->startup_timer_period > FREERUN_TIMER_PERIOD_CNT_MIN)
        {
            /* Decrement the timer period to accelerate the motor */
            ptr_bldc_status->startup_timer_period -= FREERUN_TIMER_PERIOD_STEP;
        }

        /* Check the no. of open-loop commutation cycles elapsed in startup */
        if(ptr_bldc_status->startup_cycle_cnt == (uint8_t)(STARTUP_CYCLE_CNT/SECTOR_T))
        {

            /* Close the loop after sufficient startup commutation cycles are completed */
            ptr_bldc_status->closedLoop = 1; /* Set this flag to '0' to continuously operate in open loop */

            /*-----------------------------------------------------------------------------*/
            /* INIT: CYBSP_CONTROL_TIMER in SpeedCounter mode */
            /* Update PeriClk divider value to 50 => 960kHz clock frequency */
            Set_PeriClk_Divider_Val(SPEED_COUNTER_CLKDIV);

            /* Initialize the interrupt vector table with the Speed Counter interrupt handler address and assign priority */
            result = Cy_SysInt_Init(ptr_ctrl_timer_intr_config, SpeedCounter_ISR);
            if(result != CY_SYSINT_SUCCESS)
            {
                CY_ASSERT(CY_ASSERT_FAILED);
            }

            /* Modify the  Counter parameters for Speed Counter */
            ptr_ctrl_timer_config->period = SPEED_COUNTER_PERIOD_CNT;
            ptr_ctrl_timer_config->compareOrCapture = CY_TCPWM_COUNTER_MODE_CAPTURE;
            ptr_ctrl_timer_config->interruptSources = CY_TCPWM_INT_ON_CC;

            /* Reinitialize the counter in Speed Counter mode */
            result = Cy_TCPWM_Counter_Init(CYBSP_CONTROL_TIMER_HW, CYBSP_CONTROL_TIMER_NUM, ptr_ctrl_timer_config);
            if(result != CY_TCPWM_SUCCESS)
            {
                CY_ASSERT(CY_ASSERT_FAILED);
            }
            /* INIT COMPLETE: CYBSP_CONTROL_TIMER in SpeedCounter mode */
            /*-----------------------------------------------------------------------------*/
        }
    }

    /* Clear the terminal count/capture interrupt */
    Cy_TCPWM_ClearInterrupt(CYBSP_CONTROL_TIMER_HW, CYBSP_CONTROL_TIMER_NUM, CY_TCPWM_INT_ON_CC_OR_TC);
}
/* [] END OF FILE */
