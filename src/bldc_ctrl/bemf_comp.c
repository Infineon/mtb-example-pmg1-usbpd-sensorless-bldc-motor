/*******************************************************************************
* File Name: bemf_comp.c
* Version 1.0
*
* Description:
*  This file provides the source code to the API for the Comparator component
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
#include "cy_pdl.h"
#include "cybsp.h"
#include "bemf_comp.h"
#include "bldc_controller.h"
#include "parameters.h"


/*******************************************************************************
* Global Variables
*******************************************************************************/
/* Initialize the interrupt vector table with the OPAMP1 interrupt handler address and assign priority. */
cy_stc_sysint_t intrCfg =
{
    /*.intrSrc =*/ CYBSP_CTB_OA1_IRQ,    /* Interrupt source is OpAmp1 interrupt */
    /*.intrPriority =*/ 3UL          /* Interrupt priority is 3 */
};


/*******************************************************************************
* Function Name: BEMF_Comp_Init
********************************************************************************
*
* Summary:
* - Configure and initialize interrupt on CTBM0 OpAmp1
* - Enable CTBM0
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void BEMF_Comp_Init(void)
{
    cy_en_sysint_status_t result = Cy_SysInt_Init(&intrCfg, OA1_ISR);
    if(result != CY_SYSINT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /* Initialize PASS 0 CTB 0 OpAmp 1 under CTBm block */
    cy_en_ctb_status_t status = Cy_CTB_OpampInit(CTBM0, CY_CTB_OPAMP_1, &CYBSP_CTB_OA1_config);

    if (CY_CTB_SUCCESS != status)
    {
        /* Insert error handling here */
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    if (CY_CTB_SUCCESS == status)
    {
        /* Continuous Time Block mini (CTBm) enabling */
        Cy_CTB_Enable(CTBM0);
    }

    NVIC_ClearPendingIRQ(intrCfg.intrSrc);
    NVIC_EnableIRQ(intrCfg.intrSrc);

}

/*******************************************************************************
* Function Name: AMUX_Start
********************************************************************************
* Summary:
*  Disconnect all channels.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void AMUX_Start(void)
{
    /* Disconnect all BEMF channels - P2.2, P2.3, P2.4 */
    Cy_GPIO_Port_Deinit(GPIO_PRT2);
}

/*******************************************************************************
* Function Name: AMUX_Select
********************************************************************************
* Summary:
*  This functions first disconnects all channels then connects the given channel.
*
* Parameters:
*  channel:  The channel to connect to the common terminal.
*
* Return:
*  void
*
*******************************************************************************/
void AMUX_Select(uint8_t channel)
{
    /* Disconnect all previous connections */
    AMUX_Start();

    switch(channel)
    {
        case BEMF_A:
            /* Connect BEMF_A (P2.2) to AMUXB */
            Cy_GPIO_Pin_FastInit(GPIO_PRT2, P2_2_NUM, CY_GPIO_DM_ANALOG, 1UL, P2_2_AMUXB);
            break;
        case BEMF_B:
            /* Connect BEMF_B (P2.3) to AMUXB */
            Cy_GPIO_Pin_FastInit(GPIO_PRT2, P2_3_NUM, CY_GPIO_DM_ANALOG, 1UL, P2_3_AMUXB);
            break;
        case BEMF_C:
            /* Connect BEMF_C (P2.4) to AMUXB */
            Cy_GPIO_Pin_FastInit(GPIO_PRT2, P2_4_NUM, CY_GPIO_DM_ANALOG, 1UL, P2_4_AMUXB);
            break;
        default:
            break;
    }
}

/*******************************************************************************
* Function Name: OA1_ISR
********************************************************************************
* Summary:
* - This is the interrupt handler for OpAmp OA1
* - It monitors the back EMF signals on line A, B and C to detect zero-crossings
* - Handle commutation process
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void OA1_ISR(void)
{
    bldc_controller_status_t *ptr_bldc_status = bldc_controller_get_status();

    ptr_bldc_status->opamp_intr_cnt++; /* Variable to track the opamp1 interrupts */

    if(ptr_bldc_status->closedLoop && ptr_bldc_status->commutateFlag) /* Condition for commutation */
    {
        /* Check for the threshold delay-1 for commutation
         * Commutation delay to switch the high-side MOSFETs */
        if(ptr_bldc_status->opamp_intr_cnt == ptr_bldc_status->zerocross_delay_cnt)
        {
            switch(ptr_bldc_status->bemf_ch) /* Check the current BEMF channel connected to OpAmp1 */
            {
                case BEMF_A:
                    ptr_bldc_status->sector = SECTOR_6;

                    Cy_TCPWM_TriggerStopOrKill(TCPWM, CYBSP_PWM_BH_MASK);    //BH = 0
                    Cy_TCPWM_TriggerStopOrKill(TCPWM, CYBSP_PWM_CH_MASK);    //CH = 0
                    Cy_TCPWM_TriggerReloadOrIndex(TCPWM, CYBSP_PWM_AH_MASK); //AH = 1

                    if(ptr_bldc_status->directionReverse)
                    {
                        Cy_GPIO_Clr(PWM_AL_PORT, PWM_AL_PIN); //AL = 0
                        Cy_GPIO_Clr(PWM_BL_PORT, PWM_BL_PIN); //BL = 0
                        Cy_GPIO_Set(PWM_CL_PORT, PWM_CL_PIN); //CL = 1

                    }
                    else
                    {
                        Cy_GPIO_Clr(PWM_AL_PORT, PWM_AL_PIN); //AL = 0
                        Cy_GPIO_Clr(PWM_CL_PORT, PWM_CL_PIN); //CL = 0
                        Cy_GPIO_Set(PWM_BL_PORT, PWM_BL_PIN); //BL = 1
                    }
                    break;

                case BEMF_B:
                    ptr_bldc_status->sector = SECTOR_2;
                    
                    Cy_TCPWM_TriggerStopOrKill(TCPWM, CYBSP_PWM_AH_MASK);    //AH = 0
                    Cy_TCPWM_TriggerStopOrKill(TCPWM, CYBSP_PWM_CH_MASK);    //CH = 0
                    Cy_TCPWM_TriggerReloadOrIndex(TCPWM, CYBSP_PWM_BH_MASK); //BH = 1

                    if(ptr_bldc_status->directionReverse)
                    {
                        Cy_GPIO_Clr(PWM_BL_PORT, PWM_BL_PIN); //BL = 0
                        Cy_GPIO_Clr(PWM_CL_PORT, PWM_CL_PIN); //CL = 0
                        Cy_GPIO_Set(PWM_AL_PORT, PWM_AL_PIN); //AL = 1
                    }
                    else
                    {
                        Cy_GPIO_Clr(PWM_AL_PORT, PWM_AL_PIN); //AL = 0
                        Cy_GPIO_Clr(PWM_BL_PORT, PWM_BL_PIN); //BL = 0
                        Cy_GPIO_Set(PWM_CL_PORT, PWM_CL_PIN); //CL = 1
                    }
                    break;

                case BEMF_C:
                    ptr_bldc_status->sector = SECTOR_4;

                    Cy_TCPWM_TriggerStopOrKill(TCPWM, CYBSP_PWM_AH_MASK);    //AH = 0
                    Cy_TCPWM_TriggerStopOrKill(TCPWM, CYBSP_PWM_BH_MASK);    //BH = 0
                    Cy_TCPWM_TriggerReloadOrIndex(TCPWM, CYBSP_PWM_CH_MASK); //CH = 1

                    if(ptr_bldc_status->directionReverse)
                    {
                        Cy_GPIO_Clr(PWM_AL_PORT, PWM_AL_PIN); //AL = 0
                        Cy_GPIO_Clr(PWM_CL_PORT, PWM_CL_PIN); //CL = 0
                        Cy_GPIO_Set(PWM_BL_PORT, PWM_BL_PIN); //BL = 1
                    }
                    else
                    {
                        Cy_GPIO_Clr(PWM_BL_PORT, PWM_BL_PIN); //BL = 0
                        Cy_GPIO_Clr(PWM_CL_PORT, PWM_CL_PIN); //CL = 0
                        Cy_GPIO_Set(PWM_AL_PORT, PWM_AL_PIN); //AL = 1
                    }
                    break;
            }
            ptr_bldc_status->commutateFlag = 0;
        }
    }

    /* Check for the threshold delay-2 for commutation
     * Commutation delay to switch the low-side MOSFETs */
    if(ptr_bldc_status->opamp_intr_cnt == ptr_bldc_status->commutate_delay_cnt)
    {
        if(ptr_bldc_status->closedLoop && !ptr_bldc_status->commutateFlag) /* condition for commutation */
        {
            switch(ptr_bldc_status->sector) /* Commutate the phases based on the current sector of operation (6-sectors in total) */
            {
                case SECTOR_6:
                    Cy_TCPWM_TriggerStopOrKill(TCPWM, CYBSP_PWM_BH_MASK);    //BH = 0
                    Cy_TCPWM_TriggerStopOrKill(TCPWM, CYBSP_PWM_CH_MASK);    //CH = 0
                    Cy_TCPWM_TriggerReloadOrIndex(TCPWM, CYBSP_PWM_AH_MASK); //AH = 1

                    if(ptr_bldc_status->directionReverse)
                    {
                        Cy_GPIO_Clr(PWM_AL_PORT, PWM_AL_PIN); //AL = 0
                        Cy_GPIO_Clr(PWM_CL_PORT, PWM_CL_PIN); //CL = 0
                        Cy_GPIO_Set(PWM_BL_PORT, PWM_BL_PIN); //BL = 1

                        ptr_bldc_status->bemf_ch = BEMF_C;
                    }
                    else
                    {
                        Cy_GPIO_Clr(PWM_AL_PORT, PWM_AL_PIN); //AL = 0
                        Cy_GPIO_Clr(PWM_BL_PORT, PWM_BL_PIN); //BL = 0
                        Cy_GPIO_Set(PWM_CL_PORT, PWM_CL_PIN); //CL = 1

                        ptr_bldc_status->bemf_ch = BEMF_B;
                    }
                    break;

                case SECTOR_2:
                    Cy_TCPWM_TriggerStopOrKill(TCPWM, CYBSP_PWM_AH_MASK);    //AH = 0
                    Cy_TCPWM_TriggerStopOrKill(TCPWM, CYBSP_PWM_CH_MASK);    //CH = 0
                    Cy_TCPWM_TriggerReloadOrIndex(TCPWM, CYBSP_PWM_BH_MASK); //BH = 1

                    if(ptr_bldc_status->directionReverse)
                    {
                        Cy_GPIO_Clr(PWM_AL_PORT, PWM_AL_PIN); //AL = 0
                        Cy_GPIO_Clr(PWM_BL_PORT, PWM_BL_PIN); //BL = 0
                        Cy_GPIO_Set(PWM_CL_PORT, PWM_CL_PIN); //CL = 1

                        ptr_bldc_status->bemf_ch = BEMF_A;
                    }
                    else
                    {
                        Cy_GPIO_Clr(PWM_BL_PORT, PWM_BL_PIN); //BL = 0
                        Cy_GPIO_Clr(PWM_CL_PORT, PWM_CL_PIN); //CL = 0
                        Cy_GPIO_Set(PWM_AL_PORT, PWM_AL_PIN); //AL = 1

                        ptr_bldc_status->bemf_ch = BEMF_C;
                    }
                    break;

                case SECTOR_4:
                    Cy_TCPWM_TriggerStopOrKill(TCPWM, CYBSP_PWM_AH_MASK);    //AH = 0
                    Cy_TCPWM_TriggerStopOrKill(TCPWM, CYBSP_PWM_BH_MASK);    //BH = 0
                    Cy_TCPWM_TriggerReloadOrIndex(TCPWM, CYBSP_PWM_CH_MASK); //CH = 1

                    if(ptr_bldc_status->directionReverse)
                    {
                        Cy_GPIO_Clr(PWM_BL_PORT, PWM_BL_PIN); //BL = 0
                        Cy_GPIO_Clr(PWM_CL_PORT, PWM_CL_PIN); //CL = 0
                        Cy_GPIO_Set(PWM_AL_PORT, PWM_AL_PIN); //AL = 1

                        ptr_bldc_status->bemf_ch = BEMF_B;
                    }
                    else
                    {
                        Cy_GPIO_Clr(PWM_AL_PORT, PWM_AL_PIN); //AL = 0
                        Cy_GPIO_Clr(PWM_CL_PORT, PWM_CL_PIN); //CL = 0
                        Cy_GPIO_Set(PWM_BL_PORT, PWM_BL_PIN); //BL = 1

                    ptr_bldc_status->bemf_ch = BEMF_A;
                    }
                    break;
            }
            /* Trigger a capture event on speed counter */
            Cy_TCPWM_TriggerCaptureOrSwap(TCPWM, CYBSP_CONTROL_TIMER_MASK);
        }
        /* Route the next BEMF channel to OpAmp V+ */
        AMUX_Select(ptr_bldc_status->bemf_ch);

        /* Delay for channel switching */
        Cy_SysLib_DelayUs(50u);

        ptr_bldc_status->commutateFlag = 1;
        ptr_bldc_status->opamp_intr_cnt = 0;
    }

    /* Clear OpAmp1 interrupt */
    Cy_CTB_ClearInterrupt(CTBM0, CY_CTB_OPAMP_1);
}
/* [] END OF FILE */
