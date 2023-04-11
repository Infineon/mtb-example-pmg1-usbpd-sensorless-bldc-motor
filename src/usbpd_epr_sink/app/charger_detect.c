/******************************************************************************
* File Name: charger_detect.c
*
* Description: This source file implements the BC 1.2 (legacy) charger detect
*              functions which are part of the PMG1 MCU USBPD Sink Example for
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

#include "cybsp.h"
#include "charger_detect.h"
#include "cy_pdstack_common.h"
#include "cy_pdstack_dpm.h"
#include "cy_pdutils.h"
#include "cy_usbpd_bch.h"
#include "psink.h"
#include "cy_pdutils_sw_timer.h"
#include "timer_id.h"
#include "app.h"

#if (BATTERY_CHARGING_ENABLE)

chgdet_status_t gl_chgdet_status[NO_OF_TYPEC_PORTS];

#if (defined(CY_IP_MXUSBPD) || defined(CY_IP_M0S8USBPD))

static void chgdet_timer_cb(cy_timer_id_t id, void *cbContext)
{
    cy_stc_pdstack_context_t *stack_ctx = (cy_stc_pdstack_context_t *)cbContext;

    if (stack_ctx->port != 0u)
    {
        id = (id & 0x00FFU) + CY_PDUTILS_TIMER_APP_PORT0_START_ID;
    }

    if (id == APP_BC_GENERIC_TIMER1)
    {
        chgdet_fsm_set_evt((cy_stc_pdstack_context_t *)cbContext, BC_EVT_TIMEOUT1);
    }

    if (id == APP_BC_GENERIC_TIMER2)
    {
        chgdet_fsm_set_evt((cy_stc_pdstack_context_t *)cbContext, BC_EVT_TIMEOUT2);
    }
}

static void chgdet_fsm_off(cy_stc_pdstack_context_t *stack_ctx, chgdet_fsm_evt_t evt)
{
    /* Nothing to do in this state. */
    CY_UNUSED_PARAMETER(stack_ctx);
    CY_UNUSED_PARAMETER(evt);
}

static void chgdet_fsm_sink_start(cy_stc_pdstack_context_t *stack_ctx, chgdet_fsm_evt_t evt)
{
    chgdet_status_t *chgdet_stat = &gl_chgdet_status[stack_ctx->port];

    if (evt == CHGDET_FSM_EVT_ENTRY)
    {
        /* Reset the charger detect hardware block by disabling and enabling it. */
        Cy_USBPD_Bch_Phy_Dis(stack_ctx->ptrUsbPdContext);
        Cy_USBPD_Bch_Phy_En(stack_ctx->ptrUsbPdContext);

        /* Move to the Primary Charger Detect state. */
        chgdet_stat->chgdet_fsm_state = CHGDET_FSM_SINK_PRIMARY_CHARGER_DETECT;
        chgdet_fsm_set_evt (stack_ctx, BC_EVT_ENTRY);
    }
}

/* This state is for primary charger detect. Refer BC 1.2 spec for details. */
static void chgdet_fsm_sink_primary_charger_detect(cy_stc_pdstack_context_t *stack_ctx, chgdet_fsm_evt_t evt)
{
    chgdet_status_t *chgdet_stat = &gl_chgdet_status[stack_ctx->port];

    switch (evt)
    {
        case CHGDET_FSM_EVT_ENTRY:
            /* Apply terminations on D+/- and start VDP_DM_SRC_ON timer to schedule the next step. */
            Cy_USBPD_Bch_Phy_ConfigSnkTerm (stack_ctx->ptrUsbPdContext, CHGB_SINK_TERM_PCD);
            Cy_PdUtils_SwTimer_Start(stack_ctx->ptrTimerContext, (void *)stack_ctx,
                    GET_APP_TIMER_ID(stack_ctx, APP_BC_GENERIC_TIMER1),
                    APP_BC_VDP_DM_SRC_ON_PERIOD, chgdet_timer_cb);
            break;

        case CHGDET_FSM_EVT_TIMEOUT1:
            /* Now measure D- and see if D- is pulled up to VDP_SRC. */
            if (Cy_USBPD_Bch_Phy_Config_Comp (stack_ctx->ptrUsbPdContext, 0, CHGB_COMP_P_DM, CHGB_COMP_N_VREF,
                        CHGB_VREF_0_325V, CHGB_COMP_NO_INTR) == true)
            {
                /* Start timer for source to differentiate between primary and secondary detection */
                Cy_PdUtils_SwTimer_Start(stack_ctx->ptrTimerContext, (void *)stack_ctx,
                        GET_APP_TIMER_ID(stack_ctx, APP_BC_GENERIC_TIMER2),
                        APP_BC_VDMSRC_EN_DIS_PERIOD, chgdet_timer_cb);
            }
            else
            {
                /* TYPE-C only source is connected. */
                chgdet_stat->chgdet_fsm_state = CHGDET_FSM_SINK_TYPE_C_ONLY_SOURCE_CONNECTED;
                chgdet_fsm_set_evt(stack_ctx, BC_EVT_ENTRY);
            }

            /* Remove applied terminations */
            Cy_USBPD_Bch_Phy_RemoveTerm(stack_ctx->ptrUsbPdContext);
            break;

        case CHGDET_FSM_EVT_TIMEOUT2:
            /* Proceed to secondary detection for CDP/DCP detection. */
            chgdet_stat->chgdet_fsm_state = CHGDET_FSM_SINK_SECONDARY_CHARGER_DETECT;
            chgdet_fsm_set_evt(stack_ctx, BC_EVT_ENTRY);
            break;

        default:
            break;
    }
}

static void chgdet_fsm_sink_type_c_only_source_connected(cy_stc_pdstack_context_t *stack_ctx, chgdet_fsm_evt_t evt)
{
    if (evt == CHGDET_FSM_EVT_ENTRY)
    {
        /* Disable charger detect block operation. */
        Cy_USBPD_Bch_Phy_Dis(stack_ctx->ptrUsbPdContext);

#if defined(CY_DEVICE_CCG6)
        /* TBD: Enable DP/DM Mux. */
#endif /* defined(CY_DEVICE_CCG6) */
    }
}

/* This state is used to perform secondary charger detect. Refer BC 1.2 spec for details. */
static void chgdet_fsm_sink_secondary_charger_detect(cy_stc_pdstack_context_t *stack_ctx, chgdet_fsm_evt_t evt)
{
    chgdet_status_t *chgdet_stat = &gl_chgdet_status[stack_ctx->port];

    switch (evt)
    {
        case CHGDET_FSM_EVT_ENTRY:
            /* Apply terminations on D+/-. */
            Cy_USBPD_Bch_Phy_ConfigSnkTerm(stack_ctx->ptrUsbPdContext, CHGB_SINK_TERM_SCD);

            /* Start timer to apply VDM_SRC for TVDM_SRC_ON */
            Cy_PdUtils_SwTimer_Start(stack_ctx->ptrTimerContext, (void *)stack_ctx,
                    GET_APP_TIMER_ID(stack_ctx, APP_BC_GENERIC_TIMER1),
                    APP_BC_VDP_DM_SRC_ON_PERIOD, chgdet_timer_cb);
            break;

        case CHGDET_FSM_EVT_TIMEOUT1:
            /* Now measure D+ and see if D- is pulled up to VDM_SRC. */
            if (Cy_USBPD_Bch_Phy_Config_Comp(stack_ctx->ptrUsbPdContext, 0, CHGB_COMP_P_DP, CHGB_COMP_N_VREF,
                        CHGB_VREF_0_325V, CHGB_COMP_NO_INTR) == true)
            {
                /* DCP connected. */
                chgdet_stat->chgdet_fsm_state = CHGDET_FSM_SINK_DCP_CONNECTED;
            }
            else
            {
                /* CDP connected. */
                chgdet_stat->chgdet_fsm_state = CHGDET_FSM_SINK_CDP_CONNECTED;
            }

            chgdet_fsm_set_evt(stack_ctx, BC_EVT_ENTRY);

            /* Remove applied terminations */
            Cy_USBPD_Bch_Phy_RemoveTerm(stack_ctx->ptrUsbPdContext);
            break;

        default:
            break;
    }
}

/* The Type-C source is a Downstream Charging Port (DCP). */
static void chgdet_fsm_sink_dcp_connected(cy_stc_pdstack_context_t *stack_ctx, chgdet_fsm_evt_t evt)
{
    if (evt == CHGDET_FSM_EVT_ENTRY)
    {
        Cy_USBPD_Bch_Phy_Dis(stack_ctx->ptrUsbPdContext);

#if defined(CY_DEVICE_CCG6)
        /* TBD: Enable DP/DM Mux if not done so far. */
#endif /* defined(CY_DEVICE_CCG6) */

        /* Set current limit to 1.5A (DCP) and enable Sink FET if not already turned ON. */
        psnk_set_current(stack_ctx, CY_PD_I_1P5A);
        psnk_enable(stack_ctx);
    }
}

/* The Type-C source is a Standard Downstream Port (SDP). */
static void chgdet_fsm_sink_sdp_connected(cy_stc_pdstack_context_t *stack_ctx, chgdet_fsm_evt_t evt)
{
    if (evt == CHGDET_FSM_EVT_ENTRY)
    {
        Cy_USBPD_Bch_Phy_Dis(stack_ctx->ptrUsbPdContext);

#if defined(CY_DEVICE_CCG6)
        /* TBD: Enable DP/DM Mux if not done so far. */
#endif /* defined(CY_DEVICE_CCG6) */

        /* Set current limit to 0.5A (SDP) and enable Sink FET if not already turned ON. */
        psnk_set_current(stack_ctx, CY_PD_ISAFE_DEF);
        psnk_enable(stack_ctx);
    }
}

/* The Type-C source is a Charging Downstream Port (CDP). */
static void chgdet_fsm_sink_cdp_connected(cy_stc_pdstack_context_t *stack_ctx, chgdet_fsm_evt_t evt)
{
    if (evt == CHGDET_FSM_EVT_ENTRY)
    {
        Cy_USBPD_Bch_Phy_Dis(stack_ctx->ptrUsbPdContext);

#if defined(CY_DEVICE_CCG6)
        /* TBD: Enable DP/DM Mux if not done so far. */
#endif /* defined(CY_DEVICE_CCG6) */

        /* Set current limit to 1.5A (CDP) and enable Sink FET if not already turned ON. */
        psnk_set_current(stack_ctx, CY_PD_I_1P5A);
        psnk_enable(stack_ctx);
    }
}

void (*const chgdet_fsm_table[CHGDET_FSM_MAX_EVTS]) (cy_stc_pdstack_context_t *stack_ctx, chgdet_fsm_evt_t evt) =
{
    chgdet_fsm_off,                                 /* 0: CHGDET_FSM_OFF */
    chgdet_fsm_sink_start,                          /* 1: CHGDET_FSM_SINK_START */
    chgdet_fsm_sink_primary_charger_detect,         /* 2: CHGDET_FSM_SINK_PRIMARY_CHARGER_DETECT */
    chgdet_fsm_sink_type_c_only_source_connected,   /* 3: CHGDET_FSM_SINK_TYPE_C_ONLY_SOURCE_CONNECTED */
    chgdet_fsm_sink_secondary_charger_detect,       /* 4: CHGDET_FSM_SINK_SECONDARY_CHARGER_DETECT */
    chgdet_fsm_sink_dcp_connected,                  /* 5: CHGDET_FSM_SINK_DCP_CONNECTED */
    chgdet_fsm_sink_sdp_connected,                  /* 6: CHGDET_FSM_SINK_SDP_CONNECTED */
    chgdet_fsm_sink_cdp_connected                   /* 7: CHGDET_FSM_SINK_CDP_CONNECTED */
};

/* Callback from the PDL driver. */
static void chgdet_phy_cbk_handler(void *drv_ctx, uint32_t event)
{
    cy_stc_usbpd_context_t *context = (cy_stc_usbpd_context_t *)drv_ctx;
    cy_stc_pdstack_context_t *stack_ctx = (cy_stc_pdstack_context_t *)(context->pdStackContext);
    chgdet_fsm_set_evt(stack_ctx, event);
}

#endif /* (defined(CY_IP_MXUSBPD) || defined(CY_IP_M0S8USBPD)) */

cy_en_usbpd_status_t chgdet_init(cy_stc_pdstack_context_t *stack_ctx)
{
    cy_en_usbpd_status_t stat = CY_USBPD_STAT_BAD_PARAM;

#if (defined(CY_IP_MXUSBPD) || defined(CY_IP_M0S8USBPD))
    if ((stack_ctx != NULL) && (stack_ctx->port < NO_OF_TYPEC_PORTS))
    {
        if (Cy_USBPD_Bch_Phy_Init (stack_ctx->ptrUsbPdContext, chgdet_phy_cbk_handler) == CY_USBPD_STAT_SUCCESS)
        {
            stat = CY_USBPD_STAT_SUCCESS;
            gl_chgdet_status[stack_ctx->port].chgdet_fsm_state = CHGDET_FSM_OFF;
            gl_chgdet_status[stack_ctx->port].chgdet_evt       = 0;
        }
    }
#else
    CY_UNUSED_PARAMETER(stack_ctx);
#endif /* (defined(CY_IP_MXUSBPD) || defined(CY_IP_M0S8USBPD)) */

    return stat;
}

cy_en_usbpd_status_t chgdet_start(cy_stc_pdstack_context_t *stack_ctx)
{
#if (defined(CY_IP_MXUSBPD) || defined(CY_IP_M0S8USBPD))
    chgdet_status_t *chgdet_stat = &gl_chgdet_status[stack_ctx->port];

    /* Move to start state for sink mode operation. */
    chgdet_stat->chgdet_fsm_state = CHGDET_FSM_SINK_START;
    chgdet_stat->chgdet_evt       = BC_EVT_ENTRY;
    chgdet_stat->connected        = false;
#else
    CY_UNUSED_PARAMETER(stack_ctx);
#endif /* (defined(CY_IP_MXUSBPD) || defined(CY_IP_M0S8USBPD)) */

    return CY_USBPD_STAT_SUCCESS;
}

cy_en_usbpd_status_t chgdet_stop(cy_stc_pdstack_context_t *stack_ctx)
{
#if (defined(CY_IP_MXUSBPD) || defined(CY_IP_M0S8USBPD))
    if (stack_ctx->port >= NO_OF_TYPEC_PORTS)
    {
        return CY_USBPD_STAT_BAD_PARAM;
    }

    chgdet_status_t *chgdet_stat = &gl_chgdet_status[stack_ctx->port];
    cy_stc_usbpd_context_t *drv_ctx = (cy_stc_usbpd_context_t *)(stack_ctx->ptrUsbPdContext);

    Cy_USBPD_Bch_Phy_DisableComp(drv_ctx, 0u);
    Cy_PdUtils_SwTimer_StopRange(stack_ctx->ptrTimerContext,
            GET_APP_TIMER_ID(stack_ctx, APP_BC_GENERIC_TIMER1),
            GET_APP_TIMER_ID(stack_ctx, APP_CDP_DP_DM_POLL_TIMER));
    Cy_USBPD_Bch_Phy_Dis(drv_ctx);

    chgdet_stat->chgdet_fsm_state = CHGDET_FSM_OFF;
    chgdet_stat->chgdet_evt       = 0;
    chgdet_stat->connected        = false;
    chgdet_stat->attach           = false;
    chgdet_stat->cur_volt         = CY_PD_VSAFE_0V;

    /* If there is no PD contract, ensure current limit is set to minimum. */
    if (stack_ctx->dpmConfig.contractExist == 0u)
    {
        psnk_set_current (stack_ctx, CY_PD_ISAFE_0A);
    }

#if defined(CY_DEVICE_CCG6)
    if (stack_ctx->dpmConfig.attach != 0u)
    {
        /* TBD: Enable the DP/DM MUX for USB 2.0 data connection. */
    }
#endif /* defined(CY_DEVICE_CCG6) */
#else
    CY_UNUSED_PARAMETER(stack_ctx);
#endif /* (defined(CY_IP_MXUSBPD) || defined(CY_IP_M0S8USBPD)) */

    return CY_USBPD_STAT_SUCCESS;
}

bool chgdet_is_active(cy_stc_pdstack_context_t *stack_ctx)
{
#if (defined(CY_IP_MXUSBPD) || defined(CY_IP_M0S8USBPD))
    chgdet_status_t *chgdet_stat = &gl_chgdet_status[stack_ctx->port];
    bool ret = false;

    if (chgdet_stat->chgdet_fsm_state != CHGDET_FSM_OFF)
    {
        ret = true;
    }

    return ret;
#else
    CY_UNUSED_PARAMETER(stack_ctx);
    return false;
#endif /* (defined(CY_IP_MXUSBPD) || defined(CY_IP_M0S8USBPD)) */
}

cy_en_usbpd_status_t chgdet_task(cy_stc_pdstack_context_t *stack_ctx)
{
#if (defined(CY_IP_MXUSBPD) || defined(CY_IP_M0S8USBPD))
    chgdet_status_t* chgdet_stat = &gl_chgdet_status[stack_ctx->port];
    uint8_t evt;
    
    if (stack_ctx->dpmConfig.connect == 0u)
    {
        chgdet_stop(stack_ctx);
        return CY_USBPD_STAT_SUCCESS;
    }

    /* Get the next event to be processed. */
    evt = Cy_PdUtils_EventGroup_GetEvent((uint32_t *)&(chgdet_stat->chgdet_evt), true);

    if (evt < CHGDET_FSM_MAX_EVTS)
    {
        /* Call the FSM handler function if we have a valid event. */
        chgdet_fsm_table[chgdet_stat->chgdet_fsm_state](stack_ctx, (chgdet_fsm_evt_t)evt);
    }
#else
    CY_UNUSED_PARAMETER(stack_ctx);
#endif /* (defined(CY_IP_MXUSBPD) || defined(CY_IP_M0S8USBPD)) */

    return CY_USBPD_STAT_SUCCESS;
}

bool chgdet_prepare_deepsleep(cy_stc_pdstack_context_t *stack_ctx)
{
#if (defined(CY_IP_MXUSBPD) || defined(CY_IP_M0S8USBPD))
    chgdet_status_t *chgdet_stat = &gl_chgdet_status[stack_ctx->port];
    bool ret = false;

    if (
            (chgdet_stat->chgdet_evt == 0u) &&
            (!Cy_PdUtils_SwTimer_RangeEnabled (stack_ctx->ptrTimerContext,
                    GET_APP_TIMER_ID(stack_ctx, APP_BC_GENERIC_TIMER1), APP_BC_DP_DM_DEBOUNCE_TIMER))
       )
    {
        Cy_USBPD_Bch_Phy_Config_DeepSleep ((cy_stc_usbpd_context_t *)(stack_ctx->ptrUsbPdContext));
        ret = true;
    }

    return ret;
#else
    CY_UNUSED_PARAMETER(stack_ctx);
    return true;
#endif /* (defined(CY_IP_MXUSBPD) || defined(CY_IP_M0S8USBPD)) */
}

void chgdet_resume(cy_stc_pdstack_context_t *stack_ctx)
{
#if (defined(CY_IP_MXUSBPD) || defined(CY_IP_M0S8USBPD))
    Cy_USBPD_Bch_Phy_Config_Wakeup((cy_stc_usbpd_context_t *)(stack_ctx->ptrUsbPdContext));
#else
    CY_UNUSED_PARAMETER(stack_ctx);
#endif /* (defined(CY_IP_MXUSBPD) || defined(CY_IP_M0S8USBPD)) */
}

const chgdet_status_t* chgdet_get_status(cy_stc_pdstack_context_t *stack_ctx)
{
    chgdet_status_t *chgdet_stat = &gl_chgdet_status[stack_ctx->port];
    return ((const chgdet_status_t *)chgdet_stat);
}

void chgdet_pd_event_handler(cy_stc_pdstack_context_t *stack_ctx, cy_en_pdstack_app_evt_t evt, const void* dat)
{
    CY_UNUSED_PARAMETER(dat);

#if (defined(CY_IP_MXUSBPD) || defined(CY_IP_M0S8USBPD))
    switch (evt)
    {
        case APP_EVT_DISCONNECT:
        case APP_EVT_VBUS_PORT_DISABLE:
        case APP_EVT_TYPE_C_ERROR_RECOVERY:
            chgdet_stop(stack_ctx);
            break;
 
        case APP_EVT_TYPEC_ATTACH:
            /* Start charger detect state machine once in the connected state if Rp detected is default Rp. */
            if (stack_ctx->dpmConfig.snkCurLevel == CY_PD_RD_USB)
            {                        
                chgdet_start(stack_ctx);
            }
            break;

       case APP_EVT_PD_CONTRACT_NEGOTIATION_COMPLETE:
            /* Stop charger detect state machine once PD contract has been established. */
            chgdet_stop(stack_ctx);
            break;

        default:
            break;
    }
#else
    CY_UNUSED_PARAMETER(stack_ctx);
    CY_UNUSED_PARAMETER(evt);
#endif /* (defined(CY_IP_MXUSBPD) || defined(CY_IP_M0S8USBPD)) */
}

void chgdet_fsm_set_evt(cy_stc_pdstack_context_t *stack_ctx, uint32_t evt_mask)
{
    chgdet_status_t *chgdet_stat = &gl_chgdet_status[stack_ctx->port];
    chgdet_stat->chgdet_evt |= evt_mask;
}

void chgdet_fsm_clear_evt(cy_stc_pdstack_context_t *stack_ctx, uint32_t evt_mask)
{
    chgdet_status_t *chgdet_stat = &gl_chgdet_status[stack_ctx->port];
    chgdet_stat->chgdet_evt &= ~evt_mask;
}

#endif /* (BATTERY_CHARGING_ENABLE) */

/* End of File */
