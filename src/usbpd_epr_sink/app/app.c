/******************************************************************************
* File Name: app.c
*
* Description: This is source code for the PMG1 MCU USB-PD Sink with Code Example.
*                This file implements functions for handling of
*              PDStack event callbacks and power saving.
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

#include "cybsp.h"
#include "cy_pdstack_common.h"
#include "cy_pdstack_dpm.h"
#include "config.h"
#include "psink.h"
#include "swap.h"
#include "vdm.h"
#include "app.h"
#include "cy_pdutils_sw_timer.h"
#include "timer_id.h"
#include "cy_gpio.h"


#if BATTERY_CHARGING_ENABLE
#include <charger_detect.h>
#endif /* BATTERY_CHARGING_ENABLE */


/* Variable to hold Application status for each USB-C port. */
app_status_t app_status[NO_OF_TYPEC_PORTS];

bool set_mux(cy_stc_pdstack_context_t* context, mux_select_t cfg, uint32_t custom_data)
{
    (void)custom_data;

    /* As there are no MUXes to be controlled on the PMG1 proto kits, just save the MUX state. */
    app_status[context->port].curr_mux_state = cfg;
    return true;
}

static void app_cbl_dsc_timer_cb (cy_timer_id_t id, void *callbackContext);
static void app_cbl_dsc_callback (cy_stc_pdstack_context_t *ptrPdStackContext, cy_en_pdstack_resp_status_t resp,
                                const cy_stc_pdstack_pd_packet_t *pkt_ptr)
{
    /* Keep repeating the DPM command until we succeed. */
    if (resp == CY_PDSTACK_SEQ_ABORTED)
    {
        Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                GET_APP_TIMER_ID(ptrPdStackContext, APP_CBL_DISC_TRIGGER_TIMER),
                APP_CBL_DISC_TIMER_PERIOD, app_cbl_dsc_timer_cb);
    }
}


static void app_cbl_dsc_timer_cb (cy_timer_id_t id, void *callbackContext)
{
    cy_stc_pdstack_context_t *pdstack_context = callbackContext;

    if (Cy_PdStack_Dpm_SendPdCommand(pdstack_context, CY_PDSTACK_DPM_CMD_INITIATE_CBL_DISCOVERY, NULL, false, app_cbl_dsc_callback) != CY_PDSTACK_STAT_SUCCESS )
    {
        /* Start timer which will send initiate the DPM command after a delay. */
        app_cbl_dsc_callback(pdstack_context, CY_PDSTACK_SEQ_ABORTED, 0);
    }
}

uint8_t app_task(cy_stc_pdstack_context_t *ptrPdStackContext)
{
    fault_handler_task (ptrPdStackContext);

#if BATTERY_CHARGING_ENABLE
    chgdet_task (ptrPdStackContext);
#endif /* BATTERY_CHARGING_ENABLE */

    return true;
}

#if CY_PD_REV3_ENABLE

#if CHUNKING_NOT_SUPPORTED

void app_not_supported_sent_cb(cy_stc_pdstack_context_t *ptrPdStackContext, cy_en_pdstack_resp_status_t resp,
                                const cy_stc_pdstack_pd_packet_t *pkt_ptr)
{
    if (resp == CY_PDSTACK_CMD_SENT)
    {
        if (ptrPdStackContext->peFsmState == CY_PDSTACK_PE_FSM_READY)
        {
            Cy_PdStack_Dpm_SetPeEvt(ptrPdStackContext, CY_PD_PE_EVT_TIMEOUT);
        }
    }
}

static void app_send_not_supported_cb (cy_timer_id_t id, void *callbackContext)
{
    cy_stc_pdstack_context_t *ptrPdStackContext = callbackContext;

    Cy_PdStack_Dpm_SendPdCommand(ptrPdStackContext, CY_PDSTACK_DPM_CMD_SEND_NOT_SUPPORTED,
            NULL, true, app_not_supported_sent_cb);
}

#else /* !CHUNKING_NOT_SUPPORTED */

static cy_en_pdstack_ams_type_t gl_saved_ams_type[NO_OF_TYPEC_PORTS];

void extd_msg_cb(cy_stc_pdstack_context_t *ptrPdStackContext, cy_en_pdstack_resp_status_t resp,
                                                    const cy_stc_pdstack_pd_packet_t *pkt_ptr)
{
    (void)pkt_ptr;

    if(resp == CY_PDSTACK_CMD_SENT)
    {
        /* Save the AMS type once the command has been sent. */
        gl_saved_ams_type[ptrPdStackContext->port] = ptrPdStackContext->dpmStat.nonIntrResponse;
    }

    if(resp == CY_PDSTACK_RES_RCVD)
    {
        /* Restore the saved AMS type once response has been received. */
        ptrPdStackContext->dpmStat.nonIntrResponse = gl_saved_ams_type[ptrPdStackContext->port];
    }
}

/* Global variable used as dummy data buffer to send Chunk Request messages. */
static uint32_t gl_extd_dummy_data;

#endif /* CHUNKING_NOT_SUPPORTED */

bool app_extd_msg_handler(cy_stc_pdstack_context_t *ptrPdStackContext, cy_stc_pd_packet_extd_t *pd_pkt_p)
{
#if CHUNKING_NOT_SUPPORTED
    /*
     * If we receive any message with more than one chunk, start ChunkingNotSupportedTimer and send NOT_SUPPORTED
     * message on expiry.
     */
    if (
            (pd_pkt_p->hdr.hdr.chunked == true) &&
            (
             (pd_pkt_p->hdr.hdr.chunkNum != 0) ||
             (pd_pkt_p->hdr.hdr.dataSize > ((pd_pkt_p->hdr.hdr.chunkNum + 1) * MAX_EXTD_MSG_LEGACY_LEN))
            )
       )
    {
        Cy_PdUtils_SwTimer_Start(ptrPdStackContext->timerContext, ptrPdStackContext,
                          GET_APP_TIMER_ID(ptrPdStackContext, APP_CHUNKED_MSG_RESP_TIMER), 45, app_send_not_supported_cb);

        /* Stop the PD_GENERIC_TIMER to prevent premature return to ready state. */
        Cy_PdUtils_SwTimer_Stop(ptrPdStackContext->timerContext, CY_PDSTACK_GET_PD_TIMER_ID(ptrPdStackContext, PD_GENERIC_TIMER));
    }
    else
    {
        /*
         * Don't send any response to response messages. Handling here instead of in the stack so that
         * these messages can be used for PD authentication implementation.
         */
        if ((pd_pkt_p->msg != CY_PDSTACK_EXTD_MSG_SECURITY_RESP) && (pd_pkt_p->msg != CY_PDSTACK_EXTD_MSG_FW_UPDATE_RESP))
        {
            /* Send Not supported message */
            Cy_PdStack_Dpm_SendPdCommand(ptrPdStackContext,
                                         CY_PDSTACK_DPM_CMD_SEND_NOT_SUPPORTED, NULL, true, NULL);
        }
    }

#else /* CHUNKING_NOT_SUPPORTED */

    /* If this is a chunked message which is not complete, send another chunk request. */
    if ((pd_pkt_p->hdr.hdr.chunked == true) && (pd_pkt_p->hdr.hdr.dataSize >
               ((pd_pkt_p->hdr.hdr.chunkNum + 1) * CY_PD_MAX_EXTD_MSG_LEGACY_LEN)))
    {
        cy_stc_pdstack_dpm_pd_cmd_buf_t extd_dpm_buf;

        extd_dpm_buf.cmdSop                = (cy_en_pd_sop_t)pd_pkt_p->sop;
        extd_dpm_buf.extdType              = (cy_en_pdstack_extd_msg_t)pd_pkt_p->msg;
        extd_dpm_buf.extdHdr.val           = 0;
        extd_dpm_buf.extdHdr.extd.chunked  = true;
        extd_dpm_buf.extdHdr.extd.request  = true;
        extd_dpm_buf.extdHdr.extd.chunkNum = pd_pkt_p->hdr.hdr.chunkNum + 1;
        extd_dpm_buf.datPtr                = (uint8_t*)&gl_extd_dummy_data;
        extd_dpm_buf.timeout                = ptrPdStackContext->senderRspTimeout;

        /* Send next chunk request */
        Cy_PdStack_Dpm_SendPdCommand(ptrPdStackContext,CY_PDSTACK_DPM_CMD_SEND_EXTENDED,
                                     &extd_dpm_buf, true, extd_msg_cb);
     }
    else
    {
        /*
         * Don't send any response to response messages. Handling here instead of in the stack so that
         * these messages can be used for PD authentication implementation.
         */
        if ((pd_pkt_p->msg != CY_PDSTACK_EXTD_MSG_SECURITY_RESP) && (pd_pkt_p->msg != CY_PDSTACK_EXTD_MSG_FW_UPDATE_RESP))
        {
            /* Send Not supported message */
            Cy_PdStack_Dpm_SendPdCommand(ptrPdStackContext,
                                         CY_PDSTACK_DPM_CMD_SEND_NOT_SUPPORTED, NULL, true, NULL);
        }
    }
#endif /* CHUNKING_NOT_SUPPORTED */

    return true;
}
#endif /* CY_PD_REV3_ENABLE */

static uint8_t gl_app_previous_polarity[NO_OF_TYPEC_PORTS];

#if (CY_PD_EPR_ENABLE && (!CY_PD_SOURCE_ONLY))
void epr_enter_mode_timer_cb (
        cy_timer_id_t id,
        void *ptrContext)
{
    (void)id;
    cy_stc_pdstack_context_t *ptrPdStackContext = (cy_stc_pdstack_context_t *)ptrContext;

    if (Cy_PdStack_Dpm_SendPdCommand (ptrPdStackContext, CY_PDSTACK_DPM_CMD_SNK_EPR_MODE_ENTRY, NULL, false, NULL) != CY_PDSTACK_STAT_SUCCESS)
    {
        /* Retry the EPR Entry. */
        Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                GET_APP_TIMER_ID(ptrPdStackContext, APP_EPR_MODE_TIMER), EPR_SNK_ENTRY_TIMER_PERIOD, epr_enter_mode_timer_cb);
    }
}

#endif /* CY_PD_EPR_ENABLE && (!CY_PD_SOURCE_ONLY */

#if (CY_PD_EPR_ENABLE && (!CY_PD_SOURCE_ONLY))
bool typec_reset_epr[NO_OF_TYPEC_PORTS] = {false};
#endif /* CY_PD_EPR_ENABLE && (!CY_PD_SOURCE_ONLY) */

void app_event_handler(cy_stc_pdstack_context_t *ptrPdStackContext,
               cy_en_pdstack_app_evt_t evt, const void* dat)
{
    const cy_en_pdstack_app_req_status_t* result;
    bool  skip_soln_cb = false;
    bool  hardreset_cplt = false;
    bool  typec_only = false;
    uint8_t port = ptrPdStackContext->port;

    if (port >= NO_OF_TYPEC_PORTS)
    {
        return;
    }

    switch(evt)
    {
        case APP_EVT_TYPEC_STARTED:
            /* Initialize the MUX to its default settings (isolate). */
            mux_ctrl_init (port);
            break;

        case APP_EVT_TYPEC_ATTACH:
            /* Enable the USB data connection through a MUX as applicable. */
            set_mux (ptrPdStackContext, MUX_CONFIG_SS_ONLY, 0);

            /* Clear all fault counters if we have seen a change in polarity from previous connection. */
            if (ptrPdStackContext->dpmConfig.polarity != gl_app_previous_polarity[port])
            {
                fault_handler_clear_counts (port);
            }
            gl_app_previous_polarity[port] = ptrPdStackContext->dpmConfig.polarity ;
#if (CY_PD_EPR_ENABLE && (!CY_PD__SOURCE_ONLY))
            typec_reset_epr[port] = false;
#endif /* CY_PD_EPR_ENABLE && (!CY_PD__SOURCE_ONLY) */
            break;

        case APP_EVT_CONNECT:
            app_status[port].cbl_disc_id_finished = false;
            app_status[port].disc_cbl_pending = false;
            break;

        case APP_EVT_HARD_RESET_COMPLETE:
            hardreset_cplt = true;
            /* Intentional fall-through. */

        case APP_EVT_HARD_RESET_SENT:
        case APP_EVT_PE_DISABLED:
            typec_only = ((ptrPdStackContext->dpmStat.pdConnected == false) || (evt == APP_EVT_PE_DISABLED));
            /* Intentional fall-through. */

        case APP_EVT_HARD_RESET_RCVD:
        case APP_EVT_VBUS_PORT_DISABLE:
        case APP_EVT_DISCONNECT:
        case APP_EVT_TYPE_C_ERROR_RECOVERY:
            app_status[port].cbl_disc_id_finished = false;
            app_status[port].disc_cbl_pending = false;

            if (hardreset_cplt)
            {
                set_mux (ptrPdStackContext, MUX_CONFIG_SS_ONLY, 0);
            }
            else
            {
                /*
                 * Isolate the data lines if this is a PD connection.
                 */
                if (!typec_only)
                {
                    set_mux (ptrPdStackContext, MUX_CONFIG_ISOLATE, 0);
                    Cy_PdUtils_SwTimer_Stop(ptrPdStackContext->ptrTimerContext,
                            GET_APP_TIMER_ID(ptrPdStackContext, APP_AME_TIMEOUT_TIMER));
                }
            }

#if (VBUS_OVP_ENABLE || VBUS_UVP_ENABLE)
            if(evt == APP_EVT_TYPE_C_ERROR_RECOVERY)
            {
                /* Clear port-in-fault flag if all fault counts are within limits. */
                if (!app_port_fault_count_exceeded(ptrPdStackContext))
                {
                    if ((app_get_status(port)->fault_status & APP_PORT_DISABLE_IN_PROGRESS) == 0)
                    {
                        ptrPdStackContext->dpmStat.faultActive = false;
                    }
                }
            }
#endif /* (VBUS_OVP_ENABLE || VBUS_UVP_ENABLE) */
            break;

        case APP_EVT_EMCA_NOT_DETECTED:
        case APP_EVT_EMCA_DETECTED:
            app_status[port].cbl_disc_id_finished = true;
            app_status[port].disc_cbl_pending = false;
            break;

        case APP_EVT_DR_SWAP_COMPLETE:
            result = (const cy_en_pdstack_app_req_status_t*)dat;
            if(*result == CY_PDSTACK_REQ_ACCEPT )
            {
            }
            break;
        case APP_EVT_PD_CONTRACT_NEGOTIATION_COMPLETE:
            /* Set VDM version based on active PD revision. */
#if CY_PD_REV3_ENABLE
            if (ptrPdStackContext->dpmConfig.specRevSopLive >= CY_PD_REV3)
            {
                app_status[port].vdm_version = CY_PD_STD_VDM_VERSION_REV3;
            }
            else
#endif /* CY_PD_REV3_ENABLE */
            {
                app_status[port].vdm_version = CY_PD_STD_VDM_VERSION_REV2;
            }
#if (CY_PD_EPR_ENABLE && (!CY_PD_SOURCE_ONLY))
            if(!typec_reset_epr[port])
            {
                /* Start a timer to attempt EPR entry. */
                if ( (ptrPdStackContext->dpmConfig.curPortRole == CY_PD_PRT_ROLE_SINK) &&
                        (ptrPdStackContext->dpmStat.srcCapP->dat[0].fixed_src.eprModeCapable == true) &&
                        (ptrPdStackContext->dpmExtStat.epr.snkEnable == true))

                {
                    Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                            GET_APP_TIMER_ID(ptrPdStackContext, APP_EPR_MODE_TIMER), EPR_SNK_ENTRY_TIMER_PERIOD, epr_enter_mode_timer_cb);

                    typec_reset_epr[port] = true;
                }
            }

#endif /* CY_PD_EPR_ENABLE && (!CY_PD_SOURCE_ONLY) */
            break;

#if CY_PD_REV3_ENABLE
        case APP_EVT_HANDLE_EXTENDED_MSG:
            {
                if (!(app_extd_msg_handler(ptrPdStackContext, (cy_stc_pd_packet_extd_t *)dat)))
                {
                    skip_soln_cb  = false;
                }
                else
                {
                    skip_soln_cb  = true;
                }
            }
            break;

        case APP_EVT_ALERT_RECEIVED:
            break;
#endif /* CY_PD_REV3_ENABLE */

        case APP_EVT_TYPEC_ATTACH_WAIT:
            break;

        case APP_EVT_TYPEC_ATTACH_WAIT_TO_UNATTACHED:
            break;

        case APP_EVT_DATA_RESET_ACCEPTED:
            app_status[port].cbl_disc_id_finished = false;

            /* Switch to USB only configuration once data reset has been accepted. */
            if (ptrPdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_UFP)
            {
                set_mux (ptrPdStackContext, MUX_CONFIG_SS_ONLY, 0);
            }
            else
            {
                set_mux (ptrPdStackContext, MUX_CONFIG_ISOLATE, 0);
            }
            break;

        case APP_EVT_DATA_RESET_CPLT:
            /* DFP needs to re-enable USBx connections at completion of Data_Reset. */
            if (ptrPdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_DFP)
            {
                set_mux (ptrPdStackContext, MUX_CONFIG_SS_ONLY, 0);
            }
            break;

        case APP_EVT_PD_SINK_DEVICE_CONNECTED:
            break;

        case APP_EVT_PKT_RCVD:
#if CHUNKING_NOT_SUPPORTED
            /* Make sure we stop the timer that initiates NOT_SUPPORTED response sending. */
            timer_stop(port, APP_CHUNKED_MSG_RESP_TIMER);
#endif /* CHUNKING_NOT_SUPPORTED */
            break;

#if (!CY_PD_SINK_ONLY)
        case APP_EVT_HR_PSRC_ENABLE:
            set_mux (port, MUX_CONFIG_SS_ONLY, 0);
            break;
#endif /* (!CY_PD_SINK_ONLY) */

        default:
            /* Nothing to do. */
            break;
    }

    /* Pass the event notification to the fault handler module. */
    if (fault_event_handler (ptrPdStackContext, evt, dat))
    {
        skip_soln_cb = true;
    }

#if BATTERY_CHARGING_ENABLE
    chgdet_pd_event_handler (ptrPdStackContext, evt, dat);
#endif /* BATTERY_CHARGING_ENABLE */

    if (!skip_soln_cb)
    {
        /* Send notifications to the solution */
       sln_pd_event_handler(ptrPdStackContext, evt, dat) ;
    }
}

app_resp_t* app_get_resp_buf(uint8_t port)
{
    return &app_status[port].appResp;
}

app_status_t* app_get_status(uint8_t port)
{
    return &app_status[port];
}

void app_init(cy_stc_pdstack_context_t *ptrPdStackContext)
{
    /* Initialize the VDM responses from the configuration table. */
    vdm_data_init(ptrPdStackContext);

#if BATTERY_CHARGING_ENABLE
    chgdet_init(ptrPdStackContext);
#endif /* BATTERY_CHARGING_ENABLE */

    /* Update custom host config settings to the stack. */
    ptrPdStackContext->dpmStat.typecAccessorySuppDisabled = !(ptrPdStackContext->ptrPortCfg->accessoryEn);
    ptrPdStackContext->dpmStat.typecRpDetachDisabled = !(ptrPdStackContext->ptrPortCfg->rpDetachEn);
}

#if SYS_DEEPSLEEP_ENABLE
/* Implements PMG1 deep sleep functionality for power saving. */
bool system_sleep(cy_stc_pdstack_context_t *ptrPdStackContext, cy_stc_pdstack_context_t *ptrPdStack1Context)
{
    uint32_t intr_state;
    bool dpm_slept = false;
    bool retval = false;
    bool bc_slept = true;

    /* Do one DPM sleep capability check before locking interrupts out. */
    if (
            (Cy_PdStack_Dpm_IsIdle (ptrPdStackContext, &dpm_slept) != CY_PDSTACK_STAT_SUCCESS) ||
            (!dpm_slept)
       )
    {
        return retval;
    }

    intr_state = Cy_SysLib_EnterCriticalSection();

#if BATTERY_CHARGING_ENABLE
    bc_slept = chgdet_prepare_deepsleep(ptrPdStackContext);
#endif /* BATTERY_CHARGING_ENABLE */

    if (bc_slept)
    {
        if (
                ((Cy_PdStack_Dpm_PrepareDeepSleep(ptrPdStackContext, &dpm_slept) == CY_PDSTACK_STAT_SUCCESS) &&
                (dpm_slept))
           )
        {
            Cy_PdUtils_SwTimer_EnterSleep(ptrPdStackContext->ptrTimerContext);

            Cy_USBPD_SetReference(ptrPdStackContext->ptrUsbPdContext, true);

            /* Device sleep entry. */
            Cy_SysPm_CpuEnterDeepSleep();

            Cy_USBPD_SetReference(ptrPdStackContext->ptrUsbPdContext, false);

            retval = true;
        }
    }

    Cy_SysLib_ExitCriticalSection(intr_state);

    /* Call dpm_wakeup() if dpm_sleep() had returned true. */
    if (dpm_slept)
    {
        Cy_PdStack_Dpm_Resume(ptrPdStackContext, &dpm_slept);
    }

#if BATTERY_CHARGING_ENABLE
    if (bc_slept)
    {
        chgdet_resume (ptrPdStackContext);
    }
#endif /* BATTERY_CHARGING_ENABLE */

    (void)ptrPdStack1Context;
    return retval;
}
#endif /* SYS_DEEPSLEEP_ENABLE */

bool vconn_enable(cy_stc_pdstack_context_t *ptrPdStackContext, uint8_t channel)
{
#if (!PMG1_VCONN_DISABLE)
    /* Reset RX Protocol for cable */
    Cy_PdStack_Dpm_ProtResetRx(ptrPdStackContext, CY_PD_SOP_PRIME);
    Cy_PdStack_Dpm_ProtResetRx(ptrPdStackContext, CY_PD_SOP_DPRIME);

    if (Cy_USBPD_Vconn_Enable(ptrPdStackContext->ptrUsbPdContext, channel) != CY_USBPD_STAT_SUCCESS)
    {
        return false;
    }
    return true;
#else
    return false;
#endif /* (!PMG1_VCONN_DISABLE) */
}

void vconn_disable(cy_stc_pdstack_context_t *ptrPdStackContext, uint8_t channel)
{
#if (!PMG1_VCONN_DISABLE)
    Cy_USBPD_Vconn_Disable(ptrPdStackContext->ptrUsbPdContext, channel);
#endif /* (!PMG1_VCONN_DISABLE) */
}

bool vconn_is_present(cy_stc_pdstack_context_t *ptrPdStackContext)
{
#if (!PMG1_VCONN_DISABLE)
    return Cy_USBPD_Vconn_IsPresent(ptrPdStackContext->ptrUsbPdContext, ptrPdStackContext->dpmConfig.revPol);
#else
    return false;
#endif /* (!PMG1_VCONN_DISABLE) */
}

#if PSVP_FPGA_ENABLE
#include "cy_usbpd_pmg1s3_regs.h"
#endif /* PSVP_FPGA_ENABLE */

bool vbus_is_present(cy_stc_pdstack_context_t *ptrPdStackContext, uint16_t volt, int8_t per)
{
#if PSVP_FPGA_ENABLE
    PPDSS_REGS_T pd = ptrPdStackContext->ptrUsbPdContext->base;
    bool ret = false;

    /* Since the PSVP implementation does not support any analog components, we need to use a hack to identify whether
     * the Vbus voltage is under 1 V or higher than 4 V.
     */
    if ((volt + ((per * volt) / 100)) <= 5500)
    {
        pd->adc_ctrl = ((pd->adc_ctrl & 0x7F00) | 0x02);
        Cy_SysLib_DelayUs (100);
        if ((pd->adc_ctrl & (1 << 15)) != 0)
            ret = true;
    }
    else
    {
        if (volt <= 800u)
        {
            pd->adc_ctrl = ((pd->adc_ctrl & 0x7F00) | 0x01);
            Cy_SysLib_DelayUs (100);
            if ((pd->adc_ctrl & (1 << 15)) != 0)
                ret = true;
        }
    }

    return ret;
#else
    uint8_t level;
    uint8_t retVal;

    /*
     * Re-run calibration every time to ensure that VDDD or the measurement
     * does not break.
     */
    Cy_USBPD_Adc_Calibrate(ptrPdStackContext->ptrUsbPdContext, APP_VBUS_POLL_ADC_ID);
    level =  Cy_USBPD_Adc_GetVbusLevel(ptrPdStackContext->ptrUsbPdContext,
                           APP_VBUS_POLL_ADC_ID,
                           volt, per);

    retVal = Cy_USBPD_Adc_CompSample(ptrPdStackContext->ptrUsbPdContext,
                     APP_VBUS_POLL_ADC_ID,
                     APP_VBUS_POLL_ADC_INPUT, level);

    return retVal;
#endif /* PSVP_FPGA_ENABLE */
}


void vbus_discharge_on(cy_stc_pdstack_context_t* context)
{
    Cy_USBPD_Vbus_DischargeOn(context->ptrUsbPdContext);
}

void vbus_discharge_off(cy_stc_pdstack_context_t* context)
{
    Cy_USBPD_Vbus_DischargeOff(context->ptrUsbPdContext);
}

uint16_t vbus_get_value(cy_stc_pdstack_context_t *ptrPdStackContext)
{
    uint16_t retVal;

    /* Measure the actual VBUS voltage. */
    retVal = Cy_USBPD_Adc_MeasureVbus(ptrPdStackContext->ptrUsbPdContext,
                          APP_VBUS_POLL_ADC_ID,
                          APP_VBUS_POLL_ADC_INPUT);

    return retVal;
}

/* End of File */
