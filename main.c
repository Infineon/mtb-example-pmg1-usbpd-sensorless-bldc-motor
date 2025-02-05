/******************************************************************************
* File Name: main.c
*
* Description: This is the source code for the PMG1 USBPD sensorless BLDC motor control solution example
*              for ModusToolbox.
*
* Related Document: See README.md
*
******************************************************************************/
#include "cybsp.h"
#include "cy_pdl.h"
#include "control.h"
#include "bldc_controller.h"
#include "bemf_comp.h"
#include "protection.h"
#include "userinterface.h"
#include "parameters.h"
#include "stdio.h"
#include "config.h"
#include "cy_pdutils_sw_timer.h"
#include "cy_usbpd_common.h"
#include "cy_pdstack_common.h"
#include "cy_usbpd_typec.h"
#include "cy_pdstack_dpm.h"
#include "cy_usbpd_vbus_ctrl.h"
#include "cy_usbpd_phy.h"
#include "cy_app_instrumentation.h"
#include "cy_app.h"
#include "cy_app_pdo.h"
#include "cy_app_sink.h"
#include "cy_app_swap.h"
#include "cy_app_vdm.h"
#include "cy_app_battery_charging.h"
#include "cy_app_fault_handlers.h"
#include "mtbcfg_ezpd.h"
#include "cy_app_timer_id.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define UART_PRINT_ENABLE           (1u)    /* To enable/disable the printing of motor run-time data using UART */

/*******************************************************************************
* Global Variables
*******************************************************************************/
cy_stc_pdutils_sw_timer_t gl_TimerCtx;
cy_stc_usbpd_context_t gl_UsbPdPort0Ctx;
cy_stc_pdstack_context_t gl_PdStackPort0Ctx;

/******************************************************************************
 * Structure type declaration
 ******************************************************************************/
/* PD Stack DPM Parameters for Port 0 */
const cy_stc_pdstack_dpm_params_t pdstack_port0_dpm_params =
{
        .dpmSnkWaitCapPeriod = 335,
        .dpmRpAudioAcc = CY_PD_RP_TERM_RP_CUR_DEF,
        .dpmDefCableCap = 300,
        .muxEnableDelayPeriod = 0,
        .typeCSnkWaitCapPeriod = 0,
        .defCur = 90
};

uint32_t gl_discIdResp[7] = {0xFF00A841, 0x184004B4, 0x00000000, 0xF5030000};

/* App Parameters for Port 0 */
const cy_stc_app_params_t port0_app_params =
{
    .appVbusPollAdcId = APP_VBUS_POLL_ADC_ID,
    .appVbusPollAdcInput = APP_VBUS_POLL_ADC_INPUT,
    .prefPowerRole = 2u, /* 0-Sink, 1-Source, 2-No Preference */
    .prefDataRole = 2u, /* 0-UFP, 1- DFP, 2-No Preference */
    .discIdResp = (cy_pd_pd_do_t *)&gl_discIdResp[0],
    .discIdLen = 0x14,
    .swapResponse = 0x3F
};
cy_stc_pdstack_context_t * gl_PdStackContexts[NO_OF_TYPEC_PORTS] =
{
        &gl_PdStackPort0Ctx,
};

bool mux_ctrl_init(uint8_t port)
{
    /* No MUXes to be controlled on the PMG1 proto kits. */
    CY_UNUSED_PARAMETER(port);
    return true;
}

const cy_stc_sysint_t wdt_interrupt_config =
{
    .intrSrc = (IRQn_Type)srss_interrupt_wdt_IRQn,
    .intrPriority = 3U,
};

const cy_stc_sysint_t usbpd_port0_intr0_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port0_IRQ,
    .intrPriority = 3U,
};

const cy_stc_sysint_t usbpd_port0_intr1_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port0_DS_IRQ,
    .intrPriority = 3U,
};

/*******************************************************************************
* Function Name: get_pdstack_context
********************************************************************************
* Summary:
*   Returns the respective port PD Stack Context
*
* Parameters:
*  portIdx - Port Index
*
* Return:
*  cy_stc_pdstack_context_t
*
*******************************************************************************/
cy_stc_pdstack_context_t *get_pdstack_context(uint8_t portIdx)
{
    return (gl_PdStackContexts[portIdx]);
}

/*******************************************************************************
* Function Name: sln_pd_event_handler
********************************************************************************
* Summary:
*   Solution PD Event Handler
*   Handles the Extended message event
*
* Parameters:
*  ctx - PD Stack Context
*  evt - App Event
*  data - Data
*
* Return:
*  None
*
*******************************************************************************/
void sln_pd_event_handler(cy_stc_pdstack_context_t* ctx, cy_en_pdstack_app_evt_t evt, const void *data)
{
    (void)ctx;

    if(evt == APP_EVT_HANDLE_EXTENDED_MSG)
    {
        cy_stc_pd_packet_extd_t * ext_mes = (cy_stc_pd_packet_extd_t * )data;
        if ((ext_mes->msg != CY_PDSTACK_EXTD_MSG_SECURITY_RESP) && (ext_mes->msg != CY_PDSTACK_EXTD_MSG_FW_UPDATE_RESP))
        {
            /* Send Not supported message */
            Cy_PdStack_Dpm_SendPdCommand(ctx, CY_PDSTACK_DPM_CMD_SEND_NOT_SUPPORTED, NULL, true, NULL);
        }
    }
}

/*******************************************************************************
* Function Name: instrumentation_cb
********************************************************************************
* Summary:
*  Callback function for handling instrumentation faults
*
* Parameters:
*  port - Port
*  evt - Event
*
* Return:
*  None
*
*******************************************************************************/
void instrumentation_cb(uint8_t port, uint8_t evt)
{
    uint8_t evt_offset = APP_TOTAL_EVENTS;
    evt += evt_offset;
    sln_pd_event_handler(&gl_PdStackPort0Ctx, (cy_en_pdstack_app_evt_t)evt, NULL);
}

/*******************************************************************************
* Function Name: wdt_interrupt_handler
********************************************************************************
* Summary:
*  Interrupt Handler for Watch Dog Timer
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void wdt_interrupt_handler(void)
{
    /* Clear WDT pending interrupt */
    Cy_WDT_ClearInterrupt();

#if (CY_PDUTILS_TIMER_TICKLESS_ENABLE == 0)
    /* Load the timer match register. */
    Cy_WDT_SetMatch((Cy_WDT_GetCount() + gl_TimerCtx.multiplier));
#endif /* (TIMER_TICKLESS_ENABLE == 0) */

    /* Invoke the timer handler. */
    Cy_PdUtils_SwTimer_InterruptHandler(&(gl_TimerCtx));
}

/*******************************************************************************
* Function Name: cy_usbpd0_intr0_handler
********************************************************************************
* Summary:
*  Interrupt Handler for USBPD0 Interrupt 0
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void cy_usbpd0_intr0_handler(void)
{
    Cy_USBPD_Intr0Handler(&gl_UsbPdPort0Ctx);
}

/*******************************************************************************
* Function Name: cy_usbpd0_intr1_handler
********************************************************************************
* Summary:
*  Interrupt Handler for USBPD0 Interrupt 1
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void cy_usbpd0_intr1_handler(void)
{
    Cy_USBPD_Intr1Handler(&gl_UsbPdPort0Ctx);
}

/*******************************************************************************
* Function Name: get_dpm_connect_stat
********************************************************************************
* Summary:
*  Gets the DPM configuration for Port 0
*
* Parameters:
*  None
*
* Return:
*  cy_stc_pd_dpm_config_t
*
*******************************************************************************/
cy_stc_pd_dpm_config_t* get_dpm_connect_stat(void)
{
    return &(gl_PdStackPort0Ctx.dpmConfig);
}

/*
 * Application callback functions for the DPM. Since this application
 * uses the functions provided by the stack, loading the stack defaults.
 */
const cy_stc_pdstack_app_cbk_t app_callback =
{
    .app_event_handler = Cy_App_EventHandler,
    .vconn_enable = Cy_App_VconnEnable,
    .vconn_disable = Cy_App_VconnDisable,
    .vconn_is_present = Cy_App_VconnIsPresent,
    .vbus_is_present = Cy_App_VbusIsPresent,
    .vbus_discharge_on = Cy_App_VbusDischargeOn,
    .vbus_discharge_off = Cy_App_VbusDischargeOff,
    .psnk_set_voltage = Cy_App_Sink_SetVoltage,
    .psnk_set_current = Cy_App_Sink_SetCurrent,
    .psnk_enable = Cy_App_Sink_Enable,
    .psnk_disable = Cy_App_Sink_Disable,
    .eval_src_cap = Cy_App_Pdo_EvalSrcCap,
    .eval_dr_swap = Cy_App_Swap_EvalDrSwap,
    .eval_pr_swap = Cy_App_Swap_EvalPrSwap,
    .eval_vconn_swap = Cy_App_Swap_EvalVconnSwap,
    .eval_vdm = Cy_App_Vdm_EvalVdmMsg,
    .vbus_get_value = Cy_App_VbusGetValue
};

/*******************************************************************************
* Function Name: app_get_callback_ptr
********************************************************************************
* Summary:
*  Returns pointer to the structure holding the application callback functions
*
* Parameters:
*  context - PD Stack Context
*
* Return:
*  cy_stc_pdstack_app_cbk_t
*
*******************************************************************************/
cy_stc_pdstack_app_cbk_t* app_get_callback_ptr(cy_stc_pdstack_context_t * context)
{
    (void)context;
    /* Solution callback pointer is same for all ports */
    return ((cy_stc_pdstack_app_cbk_t *)(&app_callback));
}


/*******************************************************************************
* Function Name: Main
********************************************************************************
* Summary:
* Main function
*
* Parameters:
*  void:
*
* Return:
*  void
*
*******************************************************************************/
int main()
{
    cy_rslt_t result;
    cy_stc_pdutils_timer_config_t timerConfig;

#if UART_PRINT_ENABLE
    cy_stc_scb_uart_context_t uart_context;
    char_t adc_string[100]; /* Array to store the data for UART printing */
    uint16_t rpm; /* UART printing */
    uint16_t motorCurrent; /* UART printing */
    extern uint16_t gl_adc_result_ch2;

#endif /*UART_PRINT_ENABLE */

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board initialization failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    #if UART_PRINT_ENABLE
        /* Configure and enable the UART peripheral */
        Cy_SCB_UART_Init(CYBSP_UART_HW, &CYBSP_UART_config, &uart_context);
        Cy_SCB_UART_Enable(CYBSP_UART_HW);
    #endif /* UART_PRINT_ENABLE */

    /* For initialization of peripherals required for motor control */
    BLDC_Peripheral_Init();

    /* For initialization of the motor control variables */
    BLDC_Stop();

    /*
     * Register the interrupt handler for the watchdog timer. This timer is used to
     * implement the soft timers required by the USB-PD Stack.
     */
    Cy_SysInt_Init(&wdt_interrupt_config, &wdt_interrupt_handler);
    NVIC_EnableIRQ(wdt_interrupt_config.intrSrc);

    timerConfig.sys_clk_freq = Cy_SysClk_ClkSysGetFrequency();
    timerConfig.hw_timer_ctx = NULL;

    /* Initialize the soft timer module. */
    Cy_PdUtils_SwTimer_Init(&gl_TimerCtx, &timerConfig);

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize the instrumentation related data structures. */
    Cy_App_Instrumentation_Init(&gl_TimerCtx);

    /* Register callback function to be executed when instrumentation fault occurs. */
    Cy_App_Instrumentation_RegisterCb((cy_app_instrumentation_cb_t)instrumentation_cb);

    /* Configure and enable the USBPD interrupts */
    Cy_SysInt_Init(&usbpd_port0_intr0_config, &cy_usbpd0_intr0_handler);
    NVIC_EnableIRQ(usbpd_port0_intr0_config.intrSrc);

    Cy_SysInt_Init(&usbpd_port0_intr1_config, &cy_usbpd0_intr1_handler);
    NVIC_EnableIRQ(usbpd_port0_intr1_config.intrSrc);

    /* Initialize the USBPD driver */
    Cy_USBPD_Init(&gl_UsbPdPort0Ctx, 0, mtb_usbpd_port0_HW, mtb_usbpd_port0_HW_TRIM,
            (cy_stc_usbpd_config_t *)&mtb_usbpd_port0_config, get_dpm_connect_stat);

    /* Initialize the Device Policy Manager. */
    Cy_PdStack_Dpm_Init(&gl_PdStackPort0Ctx,
                       &gl_UsbPdPort0Ctx,
                       &mtb_usbpd_port0_pdstack_config,
                       app_get_callback_ptr(&gl_PdStackPort0Ctx),
                       &pdstack_port0_dpm_params,
                       &gl_TimerCtx);

    /* Perform application level initialization. */
    Cy_App_Init(&gl_PdStackPort0Ctx, &port0_app_params);


    /* Initialize the fault configuration values */
    Cy_App_Fault_InitVars(&gl_PdStackPort0Ctx);


    /* Start any timers or tasks associated with application instrumentation. */
    Cy_App_Instrumentation_Start();

    /* Start the device policy manager operation. This will initialize the USB-PD block and enable connect detection. */
    Cy_PdStack_Dpm_Start(&gl_PdStackPort0Ctx);

    /*
     * After the initialization is complete, keep processing the USB-PD device
     * policy manager task in a loop.  Since this application does not have any
     * other function, the PMG1 device can be placed in "deep sleep" mode for
     * power saving whenever the PD stack and drivers are idle.
     */


    /***************************************************************************
    * Main Loop
    ****************************************************************************/
    for(;;)
    {
        /* Start SAR conversion of analog sample values */
        Cy_SAR_StartConvert(SAR0, CY_SAR_START_CONVERT_SINGLE_SHOT);

        /* Wait for ADC conversion to complete */
        Cy_SAR_IsEndConversion(SAR0, CY_SAR_RETURN_STATUS);//This is non-blocking; BLocking conversion takes around 5ms

        /* Read reference speed from ADC channel-0 */
        RefSpeedCheck();

        /* Read DC bus voltage from ADC channel-1 */
        VoltageCheck();

        /* Read motor winding current from ADC channel-2 */
        CurrentCheck();

    #if UART_PRINT_ENABLE
        bldc_controller_status_t *ptr_bldc_status = bldc_controller_get_status();
        if(ptr_bldc_status->bldc_on_flag)
        {
            /* Speed calculation and UART printing only */
            rpm = (uint16_t)((SPEED_COUNTER_CLK_FREQ * 1000) / ptr_bldc_status->speed_capture_val) * (SECOND_PER_MINUTE / (PHASE_CNT * MOTOR_POLE_PAIR_NUM));    /* Clock frequency is in kHz */

            /* To calculate the current consumption of the motor for UART printing only*/
            motorCurrent = (uint16_t)(((gl_adc_result_ch2 * ((float)(ADC_VOLTAGE_RANGE * 1000)/ADC_RSLT_RANGE)) / (CURRENT_SENSE_RESISTOR)) * 1000); /* Calculated value of current consumption (in mA) by the motor */

            /* UART transmit data is used for analyzing the motor performance */
            /* conversion from int to char_t for UART transmit */
            sprintf(adc_string,"RPM: %d; Current: %d mA\r\n", (int)rpm, (int)motorCurrent);

            /* Send a string through UART peripheral */
            Cy_SCB_UART_PutString(CYBSP_UART_HW, adc_string);
        }
    #endif /* UART_PRINT_ENABLE */

        /* User interface for visualizing error cause */
        ErrorHandler();

        /* Switch control for motor ON/OFF and direction reversal */
        Ctrl_SwitchControl();

        /* Handle the device policy tasks for each PD port. */
        Cy_PdStack_Dpm_Task(&gl_PdStackPort0Ctx);

        /* Perform any application level tasks. */
        Cy_App_Task(&gl_PdStackPort0Ctx);

        /* Perform tasks associated with instrumentation. */
        Cy_App_Instrumentation_Task();

    #if SYS_DEEPSLEEP_ENABLE /* Must be disabled during motor run */
        /* If possible, enter deep sleep mode for power saving. */
        Cy_App_SystemSleep(&gl_PdStackPort0Ctx, NULL);

    #endif /* SYS_DEEPSLEEP_ENABLE */
    }
}

/* [] END OF FILE */
