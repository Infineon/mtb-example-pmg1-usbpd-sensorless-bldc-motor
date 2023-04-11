/******************************************************************************
* File Name: pdo.c
*
* Description: This source file implements source capability (PDO) evaluation
*              functions which are part of the PMG1 MCU USB-PD Sink with
*              Capsense Code Example for ModusToolBox.
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

#include "config.h"
#include "cy_pdstack_common.h"
#include "cy_pdstack_dpm.h"
#include "cy_pdutils.h"
#include "pdo.h"
#include "app.h"

#if (!(CY_PD_SOURCE_ONLY))

/* PDO Variables. */
uint32_t gl_max_min_cur_pwr[NO_OF_TYPEC_PORTS];
uint32_t gl_contract_power[NO_OF_TYPEC_PORTS];
uint32_t gl_contract_voltage[NO_OF_TYPEC_PORTS];
uint32_t gl_op_cur_power[NO_OF_TYPEC_PORTS];

#if CY_PD_EPR_ENABLE
uint32_t gl_pdo_power[NO_OF_TYPEC_PORTS];
#endif /* CY_PD_EPR_ENABLE */

static uint32_t calc_power(uint32_t voltage, uint32_t current)
{
    /*
       Voltage is expressed in 50 mV units.
       Current is expressed in 10 mA units.
       Power should be expressed in 250 mW units.
     */
    return (Cy_PdUtils_DivRoundUp(voltage * current, 500));
}

static uint32_t calc_current(uint32_t power, uint32_t voltage)
{
    /*
       Power is expressed in 250 mW units.
       Voltage is expressed in 50 mV units.
       Current should be expressed in 10 mA units.
     */
    return (Cy_PdUtils_DivRoundUp(power * 500, voltage));
}

/**
 * Checks if SRC pdo is acceptable for SNK pdo.
 * @param pdo_src pointer to current SRC PDO
 * @param pdo_snk pointer to current SNK PDO
 * @return True if current src pdo is acceptable for current snk pdo
 */
static bool is_src_acceptable_snk(cy_stc_pdstack_context_t* context, cy_pd_pd_do_t* pdo_src, uint8_t snk_pdo_idx)
{
    cy_pd_pd_do_t* pdo_snk = (cy_pd_pd_do_t*)&(context->dpmStat.curSnkPdo[snk_pdo_idx]);
    uint8_t port = context->port;

    uint32_t snk_supply_type = pdo_snk->fixed_snk.supplyType;
    uint32_t fix_volt;
    uint32_t maxVolt;
    uint32_t minVolt;
    uint32_t out = false;
    uint32_t max_min_temp, compare_temp;
    uint32_t oper_cur_pwr;

    max_min_temp = context->dpmStat.curSnkMaxMin[snk_pdo_idx] & CY_PD_SNK_MIN_MAX_MASK;
#if CY_PD_EPR_ENABLE
    gl_pdo_power[port] = 0;
#endif /* CY_PD_EPR_ENABLE */

    switch(pdo_src->fixed_src.supplyType)
    {
        case CY_PDSTACK_PDO_FIXED_SUPPLY:  /* Fixed supply PDO */
            fix_volt = pdo_src->fixed_src.voltage;
#if CY_PD_EPR_ENABLE
            gl_pdo_power[port] = calc_power(fix_volt, pdo_src->fixed_src.maxCurrent);
#endif /* CY_PD_EPR_ENABLE */
            maxVolt = Cy_PdUtils_DivRoundUp(fix_volt, 20);
            minVolt = fix_volt - maxVolt;
            maxVolt = fix_volt + maxVolt;

            switch(snk_supply_type)  /* Checking sink PDO type */
            {
                case CY_PDSTACK_PDO_FIXED_SUPPLY:
                    if(fix_volt == pdo_snk->fixed_snk.voltage)
                    {
                        compare_temp = CY_PDUTILS_GET_MAX(max_min_temp, pdo_snk->fixed_snk.opCurrent);
                        if (pdo_src->fixed_src.maxCurrent >= compare_temp)
                        {
                            gl_op_cur_power[port] = pdo_snk->fixed_snk.opCurrent;
                            out = true;
                        }
                    }
                    break;

                case CY_PDSTACK_PDO_VARIABLE_SUPPLY:
                    if ((minVolt >= pdo_snk->var_snk.minVoltage) && (maxVolt <= pdo_snk->var_snk.maxVoltage))
                    {
                        compare_temp = CY_PDUTILS_GET_MAX(max_min_temp, pdo_snk->var_snk.opCurrent);
                        if (pdo_src->fixed_src.maxCurrent >= compare_temp)
                        {
                            gl_op_cur_power[port] = pdo_snk->var_snk.opCurrent;
                            out = true;
                        }
                    }
                    break;

                case CY_PDSTACK_PDO_BATTERY:
                    if ((minVolt >= pdo_snk->bat_snk.minVoltage) && (maxVolt <= pdo_snk->bat_snk.maxVoltage))
                    {
                        fix_volt = minVolt;

                        /* Calculate the operating current and min/max current values. */
                        oper_cur_pwr = calc_current(pdo_snk->bat_snk.opPower, minVolt);
                        max_min_temp = calc_current(max_min_temp, minVolt);

                        /* Make sure the source can supply the maximum current that may be required. */
                        compare_temp = CY_PDUTILS_GET_MAX(max_min_temp, oper_cur_pwr);
                        if (pdo_src->fixed_src.maxCurrent >= compare_temp)
                        {
                            gl_op_cur_power[port] = oper_cur_pwr;
                            out = true;
                        }
                    }
                    break;

                default:
                    break;
            }

            if (out)
            {
                gl_contract_voltage[port] = fix_volt;
                gl_contract_power[port]   = calc_power (fix_volt, gl_op_cur_power[port]);
                gl_max_min_cur_pwr[port]  = max_min_temp;
            }
            break;

        case CY_PDSTACK_PDO_BATTERY:   /* SRC is a battery */
            maxVolt = pdo_src->bat_src.maxVoltage;
            minVolt = pdo_src->bat_src.minVoltage;
#if CY_PD_EPR_ENABLE
            gl_pdo_power[port] = pdo_src->bat_src.maxPower;
#endif /* CY_PD_EPR_ENABLE */
            switch(snk_supply_type)
            {
                case CY_PDSTACK_PDO_FIXED_SUPPLY:
                    /* Battery cannot supply fixed voltage
                     * Battery voltage changes with time
                     * This contract if permitted can be un-reliable */
                    break;

                case CY_PDSTACK_PDO_VARIABLE_SUPPLY:
                    if((minVolt >= pdo_snk->var_snk.minVoltage) && (maxVolt <= pdo_snk->var_snk.maxVoltage))
                    {
                        /* Calculate the expected operating power and maximum power requirement. */
                        oper_cur_pwr = calc_power(maxVolt, pdo_snk->var_snk.opCurrent);
                        max_min_temp = calc_power(maxVolt, max_min_temp);

                        compare_temp = CY_PDUTILS_GET_MAX(oper_cur_pwr, max_min_temp);
                        if (pdo_src->bat_src.maxPower >= compare_temp)
                        {
                            gl_op_cur_power[port] = oper_cur_pwr;
                            out = true;
                        }
                    }
                    break;

                case CY_PDSTACK_PDO_BATTERY:
                    /* Battery connected directly to a battery.
                     * This combination is unreliable */
                    if((minVolt >= pdo_snk->bat_snk.minVoltage) && (maxVolt <= pdo_snk->bat_snk.maxVoltage))
                    {
                        compare_temp = CY_PDUTILS_GET_MAX(max_min_temp, pdo_snk->bat_snk.opPower);
                        if (pdo_src->bat_src.maxPower >= compare_temp)
                        {
                            gl_op_cur_power[port] = pdo_snk->bat_snk.opPower;
                            out = true;
                        }
                    }
                    break;

                default:
                    break;
            }

            if (out)
            {
                gl_contract_voltage[port] = maxVolt;
                gl_max_min_cur_pwr[port]  = max_min_temp;
                gl_contract_power[port]   = gl_op_cur_power[port];
            }
            break;

        case CY_PDSTACK_PDO_VARIABLE_SUPPLY:   /* Variable supply PDO */
            maxVolt = pdo_src->var_src.maxVoltage;
            minVolt = pdo_src->var_src.minVoltage;
#if (CY_PD_EPR_ENABLE)
            gl_pdo_power[port] = calc_power(maxVolt, pdo_src->var_src.maxCurrent);
            /* Minimum Voltage Shall Not be less than 80% of the Maximum Voltage. */
            if(minVolt < Cy_PdUtils_DivRoundUp(maxVolt * 4u, 5u))
            {
                break;
            }
#endif /* CY_PD_EPR_ENABLE */
            switch (snk_supply_type) /* Checking sink PDO type */
            {
                case CY_PDSTACK_PDO_FIXED_SUPPLY:
                    /* This connection is not feasible
                     * A variable source cannot provide a fixed voltage */
                    break;

                case CY_PDSTACK_PDO_VARIABLE_SUPPLY:
                    if((minVolt >= pdo_snk->var_snk.minVoltage) && (maxVolt <= pdo_snk->var_snk.maxVoltage))
                    {
                        compare_temp = CY_PDUTILS_GET_MAX(pdo_snk->var_snk.opCurrent, max_min_temp);

                        if (pdo_src->var_src.maxCurrent >= compare_temp)
                        {
                            gl_contract_power[port] = calc_power(minVolt, pdo_snk->var_snk.opCurrent);
                            gl_op_cur_power[port]   = pdo_snk->var_snk.opCurrent;
                            out = true;
                        }
                    }
                    break;

                case CY_PDSTACK_PDO_BATTERY:
                    if((minVolt >= pdo_snk->bat_snk.minVoltage) && (maxVolt <= pdo_snk->bat_snk.maxVoltage))
                    {
                        /* Convert from power to current. */
                        oper_cur_pwr = calc_current(pdo_snk->bat_snk.opPower, minVolt);
                        max_min_temp = calc_current(max_min_temp, minVolt);

                        compare_temp = CY_PDUTILS_GET_MAX(oper_cur_pwr, max_min_temp);
                        if (pdo_src->var_src.maxCurrent >= compare_temp)
                        {
                            gl_contract_power[port] = pdo_snk->bat_snk.opPower;
                            gl_op_cur_power[port]   = oper_cur_pwr;
                            out = true;
                        }
                    }
                    break;

                default:
                    break;
            }

            if (out)
            {
                gl_contract_voltage[port] = maxVolt;
                gl_max_min_cur_pwr[port]  = max_min_temp;
            }
            break;

        default:
            break;
    }

    return out;
}

static cy_pd_pd_do_t form_rdo(cy_stc_pdstack_context_t* context, uint8_t pdo_no, bool capMisMatch, bool giveBack)
{
#if (CY_PD_REV3_ENABLE)
    const cy_stc_pd_dpm_config_t *dpm = &(context->dpmConfig);
    const cy_stc_pdstack_dpm_ext_status_t *dpmExtStat = &(context->dpmExtStat);
#endif /* CY_PD_REV3_ENABLE */

    cy_pd_pd_do_t snkRdo;
    uint8_t port = context->port;

    snkRdo.val = 0u;
    snkRdo.rdo_gen.noUsbSuspend = context->dpmStat.snkUsbSuspEn;
    snkRdo.rdo_gen.usbCommCap = context->dpmStat.snkUsbCommEn;
    snkRdo.rdo_gen.capMismatch = capMisMatch;
#if (CY_PD_EPR_ENABLE)
    /* In request PDO index is SPR 1...7, EPR 8...13 */
    if(pdo_no > CY_PD_MAX_NO_OF_PDO)
    {
        /* if pdo index > 7, set the bit31 and limit EPR obj_pos in 0...5 range */
        snkRdo.rdo_gen.eprPdo = true;
        pdo_no &= CY_PD_MAX_NO_OF_PDO;
    }
#endif /* CY_PD_EPR_ENABLE */
    snkRdo.rdo_gen.objPos = pdo_no;
    snkRdo.rdo_gen.giveBackFlag = (capMisMatch) ? false : giveBack;
    snkRdo.rdo_gen.opPowerCur = gl_op_cur_power[port];
    snkRdo.rdo_gen.minMaxPowerCur = gl_max_min_cur_pwr[port];

    if (
            (snkRdo.rdo_gen.giveBackFlag == false) &&
            (snkRdo.rdo_gen.opPowerCur > snkRdo.rdo_gen.minMaxPowerCur)
       )
    {
        snkRdo.rdo_gen.minMaxPowerCur = snkRdo.rdo_gen.opPowerCur;
    }

#if (CY_PD_REV3_ENABLE)
    /* We will always support unchunked extended messages in PD 3.0 mode. */
    if (dpm->specRevSopLive >= CY_PD_REV3)
    {
        snkRdo.rdo_gen.unchunkSup = true;

#if (CY_PD_EPR_ENABLE)

        snkRdo.rdo_gen.eprModeCapable = dpmExtStat->epr.snkEnable;
#endif /* CY_PD_EPR_ENABLE */
    }

#endif /* (CY_PD_REV3_ENABLE) */

    return snkRdo;
}

static cy_en_pdstack_pdo_sel_alg_t src_pdo_selection_algorithm = (cy_en_pdstack_pdo_sel_alg_t)PD_PDO_SEL_ALGO;

/*
 * Evaluate the source capabilities listed by the source and pick the appropriate one to request for.
 */
void eval_src_cap(cy_stc_pdstack_context_t* context, const cy_stc_pdstack_pd_packet_t* srcCap,
        cy_pdstack_app_resp_cbk_t app_resp_handler)
{
    uint8_t src_pdo_index, snk_pdo_index;
    cy_pd_pd_do_t* snkPdo = (cy_pd_pd_do_t*)&context->dpmStat.curSnkPdo[0];
    uint8_t port = context->port;
    cy_stc_pdstack_dpm_status_t *dpm = &(context->dpmStat);
    cy_stc_pdstack_dpm_ext_status_t *dpmExt = &(context->dpmExtStat);
    uint16_t src_vsafe5_cur = srcCap->dat[0].fixed_src.maxCurrent; /* Source max current for first PDO */
    cy_pd_pd_do_t snkRdo;
    uint32_t highest_gl_contract_power = 0u;
    bool match = false;
    uint8_t src_pdo_len = srcCap->len;
    uint8_t snk_pdo_len = dpm->curSnkPdocount;
   
#if (CY_PD_EPR_ENABLE)
    bool eprActive = false;
    bool eprSpr = false;
    if(srcCap->hdr.hdr.extd && dpmExt->eprActive)
    {
        src_pdo_len = srcCap->hdr.hdr.dataSize / 4u;
        Cy_PdStack_Dpm_IsEprSpr(context, &eprSpr);
        if((src_pdo_len > CY_PD_MAX_NO_OF_PDO) && !eprSpr)
        {
            snk_pdo_len = CY_PD_MAX_NO_OF_PDO + dpmExt->curEprSnkPdoCount;
        }
    }
        Cy_PdStack_Dpm_ChangeEprToSpr(context, false);
#endif /* CY_PD_EPR_ENABLE */

    for(snk_pdo_index = 0u; snk_pdo_index < snk_pdo_len; snk_pdo_index++)
    {
        for(src_pdo_index = 0u; src_pdo_index < src_pdo_len; src_pdo_index++)
        {
#if (CY_PD_EPR_ENABLE)
            if((src_pdo_index >= CY_PD_MAX_NO_OF_PDO) && (srcCap->dat[src_pdo_index].fixed_src.supplyType == CY_PDSTACK_PDO_BATTERY))
            {
                continue;
            }
#endif /* CY_PD_EPR_ENABLE */
            if(is_src_acceptable_snk(context, (cy_pd_pd_do_t*)(&srcCap->dat[src_pdo_index]), snk_pdo_index))
            {
                bool max_cond = false;

                /*
                 * We support 4 different algorithms based on which the most appropriate Source PDO is selected:
                 * CY_PDSTACK_HIGHEST_POWER   : Pick the Fixed Source PDO which delivers maximum amount of power.
                 * CY_PDSTACK_HIGHEST_VOLTAGE : Pick the Fixed Source PDO which delivers power at max. voltage.
                 * CY_PDSTACK_HIGHEST_CURRENT : Pick the Fixed Source PDO which delivers the max. current.
                 * Default (none of the above): Pick the Source PDO which delivers max. amount of power.
                 */
                switch (src_pdo_selection_algorithm)
                {
                    case CY_PDSTACK_HIGHEST_POWER:
                        /* Contract_power is based on SRC PDO */
                        if (srcCap->dat[src_pdo_index].fixed_src.supplyType == CY_PDSTACK_PDO_FIXED_SUPPLY)
                        {
                            uint32_t temp_power = calc_power(srcCap->dat[src_pdo_index].fixed_src.voltage,
                                    srcCap->dat[src_pdo_index].fixed_src.maxCurrent);
                            if (temp_power >= highest_gl_contract_power)
                            {
                                highest_gl_contract_power = temp_power;
                                max_cond = true;
                            }
                        }
                        break;

                    case CY_PDSTACK_HIGHEST_VOLTAGE:
                        /* Only fixed pdo takes part */
                        if ((srcCap->dat[src_pdo_index].fixed_src.supplyType == CY_PDSTACK_PDO_FIXED_SUPPLY)
                                && (gl_contract_voltage[port] >= highest_gl_contract_power))
                        {
                            highest_gl_contract_power = gl_contract_voltage[port];
                            max_cond = true;
                        }
                        break;

                    case CY_PDSTACK_HIGHEST_CURRENT:
                        /* Only fixed pdo takes part */
                        if ((srcCap->dat[src_pdo_index].fixed_src.supplyType == CY_PDSTACK_PDO_FIXED_SUPPLY)
                                && (srcCap->dat[src_pdo_index].fixed_src.maxCurrent >= highest_gl_contract_power))
                        {
                            highest_gl_contract_power = srcCap->dat[src_pdo_index].fixed_src.maxCurrent;
                            max_cond = true;
                        }
                        break;

                    default:
                        /* Contract_power is calculated in is_src_acceptable_snk() */
                        if (gl_contract_power[port] >= highest_gl_contract_power)
                        {
                            highest_gl_contract_power = gl_contract_power[port];
                            max_cond = true;
                        }
                        break;
                }

                if (max_cond)
                {
                    /* Check if sink needs higher capability */
                    if (
                            (snkPdo[0].fixed_snk.highCap) &&
                            (gl_contract_voltage[port] == (CY_PD_VSAFE_5V/CY_PD_VOLT_PER_UNIT))
                       )
                    {
                        /* 5V contract isn't acceptable with highCap = 1 */
                        continue;
                    }

                    snkRdo = form_rdo(context, (src_pdo_index + 1u), false,
                            (context->dpmStat.curSnkMaxMin[snk_pdo_index] & CY_PD_GIVE_BACK_MASK));
                    match = true;
                }
            }
#if (CY_PD_EPR_ENABLE)
            /* Sink should not process received EPR_Source_Capabilites message with a
             * PDO greater than 100W in any of the first seven object positions.
             * 100000mW in 250mW units. */
            Cy_PdStack_Dpm_IsEprModeActive(context, &eprActive);
            if(eprActive && (srcCap->hdr.hdr.extd)
                && (src_pdo_index < CY_PD_MAX_NO_OF_PDO) && (gl_pdo_power[port] > 100000u/250u))
            {
                (app_get_resp_buf(port))->reqStatus = CY_PDSTACK_REQ_SEND_HARD_RESET;
                app_resp_handler(context, app_get_resp_buf(port));
                return;
            }
            else
            {
                /* Clear req_status */
                (app_get_resp_buf(port))->reqStatus = (cy_en_pdstack_app_req_status_t)0;
            }
#endif /* CY_PD_EPR_ENABLE */
        }
    }

    if(match == false)
    {
        /* Capability mismatch: Ask for vsafe5v PDO with CapMismatch */
        gl_contract_voltage[port] = snkPdo[0].fixed_snk.voltage;
        gl_op_cur_power[port] = snkPdo[0].fixed_snk.opCurrent;
        gl_contract_power[port] = Cy_PdUtils_DivRoundUp(
                gl_contract_voltage[port] * gl_op_cur_power[port], 500u);

        if(src_vsafe5_cur < gl_op_cur_power[port])
        {
            /* SNK operation current can't be bigger than SRC maxCurrent */
            gl_op_cur_power[port] = src_vsafe5_cur;
        }

        gl_max_min_cur_pwr[port] = context->dpmStat.curSnkMaxMin[0];
        snkRdo = form_rdo(context, 1u, true, false);
    }
#if (!CY_PD_EPR_ENABLE)
    /* Config table setting to always request max current instead of operation current */
    if (dpm->curSnkMaxMin != 0)
    {
        uint32_t max_current = srcCap->dat[snkRdo.rdo_gen.objPos - 1].fixed_src.maxCurrent;
        snkRdo.rdo_gen.opPowerCur = max_current;
        snkRdo.rdo_gen.minMaxPowerCur = max_current;
    }
#endif /* !CY_PD_EPR_ENABLE */
    (app_get_resp_buf(port))->respDo = snkRdo;
    app_resp_handler(context, app_get_resp_buf(context->port));
}

#endif /* (!(CY_PD_SOURCE_ONLY)) */

#if (!CY_PD_SINK_ONLY)

/*
 * Verify that the Request RDO received from the sink is valid and can be supported.
 */
void eval_rdo(cy_stc_pdstack_context *ptrPdStackContext,
        pd_do_t rdo, app_resp_cbk_t app_resp_handler)
{
    if (Cy_PdStack_Dpm_IsRdoValid(ptrPdStackContext, rdo) == 0)
    {
        app_get_resp_buf(context->port)->reqStatus = REQ_ACCEPT;
    }
    else
    {
        app_get_resp_buf(context->port)->reqStatus = REQ_REJECT;
    }

    app_resp_handler(port, app_get_resp_buf(context->port));
}

#endif /* (!CY_PD_SINK_ONLY) */

/* End of File */

