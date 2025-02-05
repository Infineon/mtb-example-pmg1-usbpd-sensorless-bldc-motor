/*******************************************************************************
* File Name: protection.c
* Version 1.0
*
* Description:
*  This file provides the source code to the Motor Protection implementation
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
#include "protection.h"
#include "bldc_controller.h"
#include "parameters.h"


/*******************************************************************************
* Global Variables
*******************************************************************************/
uint16_t gl_adc_result_ch2 = 0u;

/*******************************************************************************
* Function Name: VoltageCheck
********************************************************************************
* Summary:
* Check DC bus voltage periodically for over-voltage (OV) or under-voltage (UV) event
*
* Parameters:
*  void:
*
* Return:
*  void
*
*******************************************************************************/
void VoltageCheck(void)
{
    bldc_controller_status_t* ptr_bldc_status = bldc_controller_get_status();
    uint16_t adc_result_ch1;

    /* Read the ADC channel 'VOLTAGE_MEASURE_CHAN' corresponding to the DC BUS voltage sample */
    adc_result_ch1 = Cy_SAR_GetResult16(SAR0, VOLTAGE_MEASURE_CHAN);

    /* Check if the motor is switched ON */
    if(ptr_bldc_status->bldc_on_flag)
    {
        /* Under-voltage protection */
        if(adc_result_ch1 < UV_ADC_THRESHOLD)    /* 1528 = 0.8955 V => 18V on DC bus */
        {
            ptr_bldc_status->uv_counter++;

            /* Threshold count to interpret an under-voltage condition */
            if(ptr_bldc_status->uv_counter == UV_CNT_THRESHOLD)
            {
                ptr_bldc_status->errorCode = UNDER_VOLTAGE;
                ptr_bldc_status->errorFlag = 1;
                BLDC_Stop();
            }
        }
        else if(ptr_bldc_status->uv_counter > 0)
            ptr_bldc_status->uv_counter--;

        /* Over-voltage protection */
        if(adc_result_ch1 > OV_ADC_THRESHOLD)    /* 2547 = 1.4925 V => 30V on DC bus */
        {
            ptr_bldc_status->ov_counter++;

            /* Threshold count to interpret an over-voltage condition */
            if(ptr_bldc_status->ov_counter == OV_CNT_THRESHOLD)
            {
                ptr_bldc_status->errorCode = OVER_VOLTAGE;
                ptr_bldc_status->errorFlag = 1;
                BLDC_Stop();
            }

        }
        else if(ptr_bldc_status->ov_counter > 0)
            ptr_bldc_status->ov_counter--;
    }
}

/*******************************************************************************
* Function Name: CurrentCheck
********************************************************************************
* Summary:
* Check the motor winding current periodically for over-current (OC)
*
* Parameters:
*  void:
*
* Return:
*  void
*
*******************************************************************************/
void CurrentCheck(void)
{
    bldc_controller_status_t* ptr_bldc_status = bldc_controller_get_status();

    /* Read the ADC channel 'CURRENT_MEASURE_CHAN' corresponding to the motor winding current sample */
    gl_adc_result_ch2 = Cy_SAR_GetResult16(SAR0, CURRENT_MEASURE_CHAN);

    /* Check if motor is ON */
    if(ptr_bldc_status->bldc_on_flag)
    {
        /* Over-current protection */
        if(gl_adc_result_ch2 > OC_ADC_THRESHOLD)    /* 179 = 105mV => 3.5A */
        {
            ptr_bldc_status->OC_Counter++;

            /* Threshold count to interpret an over-current condition */
            if(ptr_bldc_status->OC_Counter == OC_CNT_THRESHOLD)
            {
                ptr_bldc_status->errorCode = OVER_CURRENT;
                ptr_bldc_status->errorFlag = 1;
                BLDC_Stop();
            }
        }
        else if(ptr_bldc_status->OC_Counter > 0)
            ptr_bldc_status->OC_Counter--;
    }
}

/* [] END OF FILE */
