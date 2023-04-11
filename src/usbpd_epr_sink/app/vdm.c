/******************************************************************************
* File Name: vdm.c
*
* Description: This source file implements handlers for Vendor Defined Messages (VDMs)
*              as part of the PMG1 MCU USB-PD Sink Code Example for ModusToolBox.
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
#include "cy_pdutils.h"
#include "cy_pdutils_sw_timer.h"
#include "vdm.h"
#include "app.h"

/* Stores Discover ID response VDO count */
static uint8_t  gl_vdm_id_vdo_cnt[NO_OF_TYPEC_PORTS];

/* Stores the actual Discover ID response data */
static cy_pd_pd_do_t gl_vdm_id_vdo_resp[NO_OF_TYPEC_PORTS][CY_PD_MAX_NO_OF_DO];

const vdm_info_config_t vdm_info[NO_OF_TYPEC_PORTS] =
{
    {
        .discId = {
#if CY_PD_REV3_ENABLE
            0xFF00A041UL,               /* VDM Header. */
#else
            0xFF008041UL,               /* VDM Header. */
#endif /* CY_PD_REV3_ENABLE */

            0x18400000UL,               /* ID Header. */
            0x00000000UL,               /* Cert Stat. */
            0x00000000UL,               /* Product VDO. */
            0x00000000UL,               /* Not Used. */
            0x00000000UL,               /* Not Used. */
            0x00000000UL                /* Not Used. */
        },

        .respLength = 4u
    }
};

/* Update the VDM response information with ID information from the config structure. */
void vdm_data_init(cy_stc_pdstack_context_t *context)
{
    uint8_t port = context->port;

    /* Length of the response. */
    gl_vdm_id_vdo_cnt[port] = vdm_info[port].respLength;

    /* Copy the actual Discover Identity response. */
    memcpy ((uint8_t *)gl_vdm_id_vdo_resp[port], (const uint8_t *)vdm_info[port].discId,
            vdm_info[port].respLength * 4u);

    /* Update the vendor and product IDs from the configuration data. */
    gl_vdm_id_vdo_resp[port][ID_HEADER_DO_INDEX].std_id_hdr.usbVid = context->ptrPortCfg->mfgVid;
    gl_vdm_id_vdo_resp[port][PRODUCT_VDO_DO_INDEX].std_prod_vdo.usbPid = context->ptrPortCfg->mfgPid;
}

void eval_vdm(cy_stc_pdstack_context_t * context, const cy_stc_pdstack_pd_packet_t *vdm, cy_pdstack_vdm_resp_cbk_t vdm_resp_handler)
{
    app_status_t* app_stat = app_get_status(context->port);
    uint8_t port = context->port;

    cy_pd_pd_do_t* dobj = NULL;
    uint8_t i, count = 0u;
    bool pd3_live = false;

#if CY_PD_REV3_ENABLE
    if (context->dpmConfig.specRevSopLive >= CY_PD_REV3)
    {
        pd3_live = true;
    }
#endif /* CY_PD_REV3_ENABLE */

    if (
            (vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.vdmType == CY_PDSTACK_VDM_TYPE_STRUCTURED) &&
            (vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmdType == CY_PDSTACK_CMD_TYPE_INITIATOR)
       )
    {
        /* Handler for Structured VDMs received from an Initiator. */

        /* Copy received VDM Header data to VDM response Header*/
        app_stat->vdmResp.respBuf[CY_PD_VDM_HEADER_IDX].val = vdm->dat[CY_PD_VDM_HEADER_IDX].val;

#if CY_PD_REV3_ENABLE
        /* Use the minimum VDM version from among the partner's revision and the live revision. */
        app_stat->vdm_version = CY_PDUTILS_GET_MIN(app_stat->vdm_version, vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.stVer);
#endif /* CY_PD_REV3_ENABLE */

        /* Set a NAK response by default. */
        app_stat->vdmResp.noResp  = CY_PDSTACK_VDM_AMS_RESP_READY;
        app_stat->vdmResp.doCount = 1 ;
        app_stat->vdmResp.respBuf[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmdType = CY_PDSTACK_CMD_TYPE_RESP_NAK;

        /* Respond to VDMs only if we are UFP or if a PD 3.0 contract is in place. */
        if ((context->dpmConfig.curPortType == CY_PD_PRT_TYPE_UFP) || (pd3_live))
        {
            /* Valid VMD commands should only have one Data Object. Send NAK response otherwise. */
            if (vdm->len == 1)
            {
                switch(vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmd)
                {
                    case CY_PDSTACK_VDM_CMD_DSC_IDENTITY:
                        count = gl_vdm_id_vdo_cnt[port];
                        if((vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.svid == CY_PD_STD_SVID) && (count != 0))
                        {
                            app_stat->vdmResp.doCount = count;
                            dobj = gl_vdm_id_vdo_resp[port];
                            for(i = 0 ; i < count; i++)
                            {
                                app_stat->vdmResp.respBuf[i] = dobj[i];
                            }

#if CY_PD_REV3_ENABLE
                            /* Mask Product Type (DFP) field when VDM version is 1.0. */
                            if (app_stat->vdm_version == 0)
                            {
                                cy_pd_pd_do_t id_hdr = app_stat->vdmResp.respBuf[ID_HEADER_DO_INDEX];
                                uint8_t max_do_cnt = 4;

                                /* Make sure to clear fields that are reserved under PD 2.0. */
                                id_hdr.val &= 0xFC1FFFFF;

                                /* Make sure to not use invalid product types. */
                                if (id_hdr.std_id_hdr.prodType == CY_PDSTACK_PROD_TYPE_PSD)
                                {
                                    id_hdr.std_id_hdr.prodType = CY_PDSTACK_PROD_TYPE_UNDEF;
                                }

                                /* AMAs may be providing one extra VDO. */
                                if (id_hdr.std_id_hdr.prodType == CY_PDSTACK_PROD_TYPE_AMA)
                                {
                                    max_do_cnt++;
                                }

                                /* Ensure that the size of the response is limited based on the ID header. */
                                if (app_stat->vdmResp.doCount > max_do_cnt)
                                {
                                    app_stat->vdmResp.doCount = max_do_cnt;
                                }

                                /* Store the updated ID header VDO value. */
                                app_stat->vdmResp.respBuf[ID_HEADER_DO_INDEX] = id_hdr;
                            }
#else
                            /* Make sure that any reserved fields are zeroed out. */
                            app_stat->vdmResp.respBuf[ID_HEADER_DO_INDEX].std_id_hdr.rsvd1 = 0;
#endif /* CY_PD_REV3_ENABLE */

                            /* Set VDM Response ACKed */
                            app_stat->vdmResp.respBuf[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmdType =
                                CY_PDSTACK_CMD_TYPE_RESP_ACK;
                        }
                        break;

                    case CY_PDSTACK_VDM_CMD_DSC_SVIDS:
                    case CY_PDSTACK_VDM_CMD_DSC_MODES:
                    case CY_PDSTACK_VDM_CMD_ENTER_MODE:
                    case CY_PDSTACK_VDM_CMD_EXIT_MODE:
                        /* Since no alternate modes are supported, NAK these requests. */
                        break;

                    case CY_PDSTACK_VDM_CMD_ATTENTION:
                        /* Any attention VDMs can be ignored. */
                        app_stat->vdmResp.noResp = CY_PDSTACK_VDM_AMS_RESP_NOT_REQ;
                        break;

                    default:
                        /* Unknown command. Send a NAK response. */
                        break;
                }
            }
        }
        else
        {
            /* No response to VDMs received on PD 2.0 connection while in DFP state. */
            app_stat->vdmResp.noResp = CY_PDSTACK_VDM_AMS_RESP_NOT_REQ;
        }

        /* Set the VDM version for the response. */
        app_stat->vdmResp.respBuf[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.stVer = app_stat->vdm_version;
    }
    else
    {
        if (!pd3_live)
        {
            /* Ignore the VDM when in the DFP role. */
            app_stat->vdmResp.noResp = CY_PDSTACK_VDM_AMS_RESP_NOT_REQ;
        }
        else
        {
            /* PD 3.0 Contract: Respond with Not_Supported. */
            app_stat->vdmResp.noResp = CY_PDSTACK_VDM_AMS_RESP_NOT_SUPP;
        }
    }

    /* Notify the stack with the desired response. */
    vdm_resp_handler(context, &app_stat->vdmResp);
}

/* End of File */
