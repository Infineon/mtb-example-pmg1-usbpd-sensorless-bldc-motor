/*******************************************************************************
* File Name: bldc_controller.c
* Version 1.0
*
* Description:
*  This file provides the source code to the APIs for the initializing,
*  starting and stopping of the BLDC motor
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
#include "bldc_controller.h"
#include "bemf_comp.h"
#include "startup.h"
#include "control.h"
#include "userinterface.h"
#include "parameters.h"


/*******************************************************************************
* Macros
*******************************************************************************/
#define CYBSP_CONTROL_TIMER_INPUT_DISABLED    (0x7U)

/*******************************************************************************
* Global Variables
*******************************************************************************/
static bldc_controller_status_t gl_bldc_controller_status;

/* Configuration structure of control timer responsible for startup and speed control */
cy_stc_tcpwm_counter_config_t CYBSP_CONTROL_TIMER_myConfig =
{
    .period = 400,
    .clockPrescaler = CY_TCPWM_COUNTER_PRESCALER_DIVBY_1,
    .runMode = CY_TCPWM_COUNTER_CONTINUOUS,
    .countDirection = CY_TCPWM_COUNTER_COUNT_UP,
    .compareOrCapture = CY_TCPWM_COUNTER_MODE_COMPARE,
    .compare0 = 400,
    .compare1 = 16384,
    .enableCompareSwap = false,
    .interruptSources = CY_TCPWM_INT_ON_TC,
    .captureInputMode = CYBSP_CONTROL_TIMER_INPUT_DISABLED & 0x3U,
    .captureInput = CY_TCPWM_INPUT_0,
    .reloadInputMode = CYBSP_CONTROL_TIMER_INPUT_DISABLED & 0x3U,
    .reloadInput = CY_TCPWM_INPUT_0,
    .startInputMode = CYBSP_CONTROL_TIMER_INPUT_DISABLED & 0x3U,
    .startInput = CY_TCPWM_INPUT_0,
    .stopInputMode = CYBSP_CONTROL_TIMER_INPUT_DISABLED & 0x3U,
    .stopInput = CY_TCPWM_INPUT_0,
    .countInputMode = CYBSP_CONTROL_TIMER_INPUT_DISABLED & 0x3U,
    .countInput = CY_TCPWM_INPUT_1,
};

/* Interrupt configuration structure for Control Timer */
cy_stc_sysint_t ctrl_timer_intr_config =
{
    /*.intrSrc =*/ CYBSP_CONTROL_TIMER_IRQ,    /* Interrupt source is PWM interrupt */
    /*.intrPriority =*/ 3UL          /* Interrupt priority is 3 */
};

/* Interrupt configuration structure for motor ON-OFF switch */
const cy_stc_sysint_t ctrl_sw_intr_config =
{
    .intrSrc = CTRL_SW_IRQ,       /* Source of interrupt signal */
    .intrPriority = 3u,                 /* Interrupt priority */
};

/*
 * @brief: Get handle to the structure containing information about the BLDC controller status.
 * @parameter: void.
 * @return: Pointer to the system information structure
 * */
bldc_controller_status_t* bldc_controller_get_status(void)
{
    return &gl_bldc_controller_status;
}

/*
 * @brief: Get handle to the structure containing information about the CYBSP_CONTROL_TIMER configuration.
 * @parameter: void.
 * @return: Pointer to the timer configuration information structure
 * */
cy_stc_tcpwm_counter_config_t* CYBSP_CONTROL_TIMER_get_config(void)
{
    return &CYBSP_CONTROL_TIMER_myConfig;
}

/*
 * @brief: Get handle to the structure containing information about the ctrl_timer_intr_configuration
 * @parameter: void.
 * @return: Pointer to the interrupt configuration information structure
 * */
cy_stc_sysint_t* ctrl_timer_intr_get_config(void)
{
    return &ctrl_timer_intr_config;
}

/*******************************************************************************
* Function Name: BLDC_Peripheral_Init
********************************************************************************
* Summary:
* This function initializes all the peripherals required to operate the BLDC motor
*
* Parameters:
*  void:
*
* Return:
*  void
*
*******************************************************************************/
void BLDC_Peripheral_Init(void)
{
    cy_rslt_t result;

    /* SAR ADC initialization */
    cy_en_sar_status_t sarStatus = Cy_SAR_Init(SAR0, &CYBSP_ADC_config);
    if (CY_SAR_SUCCESS != sarStatus)
    {
        /* insert error handling here */
        CY_ASSERT(CY_ASSERT_FAILED);
    }
    /* Enable SAR ADC */
    Cy_SAR_Enable(SAR0);


    /* Initialize Ctrl_Switch GPIO interrupt */
    result = Cy_SysInt_Init(&ctrl_sw_intr_config, &Ctrl_Switch_ISR);
    if (result != CY_SYSINT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /* Clear any pending interrupt and enable the Ctrl Switch Interrupt */
    NVIC_ClearPendingIRQ(ctrl_sw_intr_config.intrSrc);

    /* Enable the Ctrl_Switch Interrupt */
    NVIC_EnableIRQ(ctrl_sw_intr_config.intrSrc);


    /* Initialize PWM_AH */
    result = Cy_TCPWM_PWM_Init(CYBSP_PWM_AH_HW, CYBSP_PWM_AH_NUM, &CYBSP_PWM_AH_config);
    if(result != CY_TCPWM_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }
    /* Enable the TCPWM as PWM */
    Cy_TCPWM_PWM_Enable(CYBSP_PWM_AH_HW, CYBSP_PWM_AH_NUM);


    /* Initialize PWM_BH */
    result = Cy_TCPWM_PWM_Init(CYBSP_PWM_BH_HW, CYBSP_PWM_BH_NUM, &CYBSP_PWM_BH_config);
    if(result != CY_TCPWM_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }
    /* Enable the TCPWM as PWM */
    Cy_TCPWM_PWM_Enable(CYBSP_PWM_BH_HW, CYBSP_PWM_BH_NUM);


    /* Initialize PWM_CH */
    result = Cy_TCPWM_PWM_Init(CYBSP_PWM_CH_HW, CYBSP_PWM_CH_NUM, &CYBSP_PWM_CH_config);
    if(result != CY_TCPWM_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }
    /* Enable the TCPWM as PWM */
    Cy_TCPWM_PWM_Enable(CYBSP_PWM_CH_HW, CYBSP_PWM_CH_NUM);


    /* Initialize & Enable OpAmp1 as comparator with interrupt trigger */
    BEMF_Comp_Init();

    gl_bldc_controller_status.errorCode = NO_ERROR; /* Defining no initial error (assumption) */
    gl_bldc_controller_status.errorFlag = 0;
    gl_bldc_controller_status.sw_timer_flag = 0; /* Initialize switch-timer-flag */
    gl_bldc_controller_status.directionReverse = 0; /* Set the default direction of rotation */
}


/*******************************************************************************
* Function Name: BLDC_Motor_Init
********************************************************************************
* Summary:
* This function initializes the BLDC motor control variables, flags and required interrupts
*
* Parameters:
*  void:
*
* Return:
*  void
*
*******************************************************************************/
void BLDC_Motor_Init(void)
{
    cy_rslt_t result;

    /* Variable initialization before starting the motor */
    gl_bldc_controller_status.bldc_on_flag = 0; /* Indicates that initially motor is in OFF state */
    gl_bldc_controller_status.startup_cycle_cnt = 0;
    gl_bldc_controller_status.closedLoop = 0; /* Indicates that initially motor operates in open loop */
    gl_bldc_controller_status.ctrl_interval = 0;
    gl_bldc_controller_status.uv_counter = 0;
    gl_bldc_controller_status.ov_counter = 0;

    gl_bldc_controller_status.commutate_delay_cnt = COMMUTATE_DELAY_CNT_INITIAL; /* Initial value of the commutation delay for nominal speed = 2000 RPM */
    gl_bldc_controller_status.zerocross_delay_cnt = (uint8_t)(gl_bldc_controller_status.commutate_delay_cnt/2);

    gl_bldc_controller_status.ctrl_error = 0;
    gl_bldc_controller_status.diff_error = 0;

    gl_bldc_controller_status.closedloop_ctrl_delay_cnt = 0;

    gl_bldc_controller_status.startup_timer_period = FREERUN_TIMER_PERIOD_CNT; /* Indicates the initial value of timer period for startup */
    gl_bldc_controller_status.pwmCompare = PREPOSITION_PWM_COMPARE;

    gl_bldc_controller_status.speed_capture_val = 0;
    gl_bldc_controller_status.speed_capture_old_val = SPEED_CAPTURE_INITIAL;

    gl_bldc_controller_status.commutateFlag = 1;
    gl_bldc_controller_status.opamp_intr_cnt = 0;

    if(gl_bldc_controller_status.directionReverse)
        gl_bldc_controller_status.sectorCnt = SECTOR_T + 1;
    else
        gl_bldc_controller_status.sectorCnt = 0;

    if(gl_bldc_controller_status.directionReverse) /* Assign the initial BEMF channel based on the rotation direction */
        gl_bldc_controller_status.bemf_ch = BEMF_B;
    else
        gl_bldc_controller_status.bemf_ch = BEMF_A;

/*-----------------------------------------------------------------------------------*/
/* INIT: CYBSP_CONTROL_TIMER in StartupTimer mode */

    /* Update PeriClk divider value to 48000 => 1kHz clock frequency */
    Set_PeriClk_Divider_Val(STARTUP_TIMER_CLKDIV);

    /* Initialize the interrupt vector table with the Switch Timer interrupt handler address and assign priority */
    result = Cy_SysInt_Init(&ctrl_timer_intr_config, SwitchTimer_ISR);
    if(result != CY_SYSINT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /* Modify the  Counter parameters for Startup Timer */
    CYBSP_CONTROL_TIMER_myConfig.period = PREPOSITION_TIMER_PERIOD_CNT;
    CYBSP_CONTROL_TIMER_myConfig.compareOrCapture = CY_TCPWM_COUNTER_MODE_COMPARE;
    CYBSP_CONTROL_TIMER_myConfig.interruptSources = CY_TCPWM_INT_ON_TC;

    /* Initialize the counter in Startup Timer mode */
    result = Cy_TCPWM_Counter_Init(CYBSP_CONTROL_TIMER_HW, CYBSP_CONTROL_TIMER_NUM, &CYBSP_CONTROL_TIMER_myConfig);
    if(result != CY_TCPWM_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }
    /* Enable the TCPWM as Counter */
    Cy_TCPWM_PWM_Enable(CYBSP_CONTROL_TIMER_HW, CYBSP_CONTROL_TIMER_NUM);

    /* Clear all the pending interrupts */
    NVIC_ClearPendingIRQ(ctrl_timer_intr_config.intrSrc);

    /* Enable interrupt */
    NVIC_EnableIRQ(ctrl_timer_intr_config.intrSrc);

/* INIT COMPLETE: CYBSP_CONTROL_TIMER in StartupTimer mode */
/*-----------------------------------------------------------------------------------*/


    /* Global interrupt enable */
    __enable_irq();

    /* Clear all the drive signals to LOW */
    Cy_GPIO_Clr(PWM_AH_PORT, PWM_AH_PIN);
    Cy_GPIO_Clr(PWM_BH_PORT, PWM_BH_PIN);
    Cy_GPIO_Clr(PWM_CH_PORT, PWM_CH_PIN);
    Cy_GPIO_Clr(PWM_AL_PORT, PWM_AL_PIN);
    Cy_GPIO_Clr(PWM_BL_PORT, PWM_BL_PIN);
    Cy_GPIO_Clr(PWM_CL_PORT, PWM_CL_PIN);

    /* Set the PWM compare value for preposition */
    Cy_TCPWM_PWM_SetCompare0(CYBSP_PWM_AH_HW, CYBSP_PWM_AH_NUM, gl_bldc_controller_status.pwmCompare);
    Cy_TCPWM_PWM_SetCompare0(CYBSP_PWM_BH_HW, CYBSP_PWM_BH_NUM, gl_bldc_controller_status.pwmCompare);
    Cy_TCPWM_PWM_SetCompare0(CYBSP_PWM_CH_HW, CYBSP_PWM_CH_NUM, gl_bldc_controller_status.pwmCompare);

}

/*******************************************************************************
* Function Name: gl_bldc_controller_status_Start
********************************************************************************
* Summary:
* This function triggers the gl_bldc_controller_status motor to start
*
* Parameters:
*  void:
*
* Return:
*  void
*
*******************************************************************************/
void BLDC_Start(void)
{
    cy_rslt_t result;

    /* Initialize the interrupt vector table with the Startup Timer interrupt handler address and assign priority */
    result = Cy_SysInt_Init(&ctrl_timer_intr_config, StartupTimer_ISR);
    if(result != CY_SYSINT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    gl_bldc_controller_status.bldc_on_flag = 1; /* Indicates the ON status of the motor */

    /* Trigger start the Startup Timer */
    Cy_TCPWM_TriggerReloadOrIndex(TCPWM, CYBSP_CONTROL_TIMER_MASK);
}

/*******************************************************************************
* Function Name: BLDC_Stop
********************************************************************************
* Summary:
* This function stops the BLDC motor
*
* Parameters:
*  void:
*
* Return:
*  void
*
*******************************************************************************/
void BLDC_Stop(void)
{
    gl_bldc_controller_status.bldc_on_flag = 0; /* Indicates the OFF status of the motor */

    /* Trigger stop the Startup Timer */
    Cy_TCPWM_TriggerStopOrKill(TCPWM, CYBSP_CONTROL_TIMER_MASK);

    /* Trigger stop the PWM signals */
    Cy_TCPWM_TriggerStopOrKill(TCPWM, CYBSP_PWM_AH_MASK);
    Cy_TCPWM_TriggerStopOrKill(TCPWM, CYBSP_PWM_BH_MASK);
    Cy_TCPWM_TriggerStopOrKill(TCPWM, CYBSP_PWM_CH_MASK);

    /* Clear all pending interrupts */
    Cy_TCPWM_ClearInterrupt(CYBSP_CONTROL_TIMER_HW, CYBSP_CONTROL_TIMER_NUM, CY_TCPWM_INT_ON_CC_OR_TC);
    Cy_CTB_ClearInterrupt(CTBM0, CY_CTB_OPAMP_1);

    /* Re-initialize the motor controller for the next start */
    BLDC_Motor_Init();

    /* 5ms delay between successive timer triggers to ensure fail-safe operation */
    Cy_SysLib_Delay(5u);

    /* Turn OFF the user LED when control is reset */
    Cy_GPIO_Set(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);
}


/*******************************************************************************
* Function Name: Set_PeriClk_Divider_Val
********************************************************************************
* Summary:
* This function updates the Peripheral Clock Divider value during runtime
*
* Parameters:
*  periDiv
*
* Return:
*  void
*
*******************************************************************************/
void Set_PeriClk_Divider_Val(uint32_t periDiv)
{
    /* Disable the peripheral clock divider */
    Cy_SysClk_PeriphDisableDivider(CY_SYSCLK_DIV_16_BIT, CONTROL_TIMER_CLK_DIV);

    /* Set the divider value for the peripheral clock divider */
    if (CY_SYSCLK_SUCCESS != Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, CONTROL_TIMER_CLK_DIV, periDiv))
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }
    /* Enable the peripheral clock divider */
    Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_16_BIT, CONTROL_TIMER_CLK_DIV);
}

/* [] END OF FILE */
