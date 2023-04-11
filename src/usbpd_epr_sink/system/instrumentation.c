/******************************************************************************
* File Name: instrumentation.c
*
* Description: This source file implements functions to monitor CPU resource
*              (execution time and stack usage) usage in the PMG1 MCU USB-PD
*              Sink Code Example for ModusToolBox.
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
#include "instrumentation.h"
#include "cy_pdl.h"
#include "cy_pdutils_sw_timer.h"
#include "cy_pdstack_common.h"
#include <stddef.h>

/* Run-time stack lower limit defined in linker script. */
#if defined(__ARMCC_VERSION)
    extern unsigned long Image$$ARM_LIB_STACK$$ZI$$Base;
#elif defined (__ICCARM__)
    #pragma language=extended
    #pragma segment="CSTACK"
#else
    extern int __StackLimit;
#endif /* defined(__ARMCC_VERSION) */

instrumentation_cb_t gl_instrumentation_cb = NULL;

#if RESET_ON_ERROR_ENABLE

/* RAM based signature and offset used to check whether reset data is valid. */
#define RESET_DATA_VALID_OFFSET         (0)
#define RESET_DATA_VALID_SIG            (0xDEADBEEF)

/* RAM based signature and offset used to check whether watchdog reset has happened. */
#define WATCHDOG_RESET_OFFSET           (1)
#define WATCHDOG_RESET_SIG              (0xC003300C)

/* RAM offset where the watchdog reset count is maintained. */
#define RESET_COUNT_OFFSET              (2)

/* Size of the reset tracking data structure in DWORDs. */
#define RESET_DATA_STRUCT_SIZE          (3)

/* Address of the run-time instrumentation data structure. */
#if defined(__ARMCC_VERSION)
    volatile uint32_t *gl_runtime_data_addr = (uint32_t *)&Image$$ARM_LIB_STACK$$ZI$$Base;
#elif defined (__ICCARM__)
    #pragma language=extended
    #pragma segment="CSTACK"
    volatile uint32_t *gl_runtime_data_addr = (uint32_t volatile *)__sfb( "CSTACK" );
#else
    volatile uint32_t *gl_runtime_data_addr = (uint32_t volatile *)&__StackLimit;
#endif /* defined(__ARMCC_VERSION) */

/* Variable used to identify whether main loop has been run. */
volatile uint32_t gl_main_loop_delay = 0;

/* Margin (in ms) available until watchdog reset. */
volatile uint16_t gl_min_reset_margin = WATCHDOG_RESET_PERIOD_MS;

extern cy_stc_pdstack_context_t gl_PdStackPort0Ctx;

/* Timer callback to reset device if main loop has not been run as expected. */
void watchdog_timer_cb (
    cy_timer_id_t id,           /**< Timer ID for which callback is being generated. */
    void *callbackContext)       /**< Timer module Context. */
{
    (void)callbackContext;
    (void)id;

    /*
     * It is possible that this timer is the only reason for the device to wake from sleep.
     * Hence allow three consecutive timer expiries before resetting the device.
     */
    gl_main_loop_delay++;
    if (gl_main_loop_delay >= 3)
    {
        if(gl_instrumentation_cb != NULL)
        {
            gl_instrumentation_cb(0, INST_EVT_WDT_RESET);
        }
        /* Store the reset signature into RAM. */
        gl_runtime_data_addr[WATCHDOG_RESET_OFFSET] = WATCHDOG_RESET_SIG;
        NVIC_SystemReset ();
    }

    /* Start the timer again. */
    Cy_PdUtils_SwTimer_Start (gl_PdStackPort0Ctx.ptrTimerContext, NULL, CY_PDUTILS_WATCHDOG_TIMER,
            WATCHDOG_RESET_PERIOD_MS, watchdog_timer_cb);
}

#endif /* RESET_ON_ERROR_ENABLE */

#if STACK_USAGE_CHECK_ENABLE

/*
 * Minimum run-time stack usage value. This many bytes at the top of the stack
 * will not be tracked for usage.
 */
#define MIN_STACK_USAGE     (256u)

/* Signature used to track stack usage. */
#define STACK_UNUSED_SIG    (0x00555500)

/* Address of the bottom location of the run-time stack. */
uint32_t *gStackBottom  = (uint32_t *)CYDEV_SRAM_BASE;
volatile uint16_t gMinStackMargin   = 0;

#endif /* STACK_USAGE_CHECK_ENABLE */

void instrumentation_init(void)
{
    uint32_t wdr_cnt = 0;

    /* Added to avoid compiler warning if all features are disabled. */
    (void)wdr_cnt;

#if STACK_USAGE_CHECK_ENABLE
    uint32_t *addr_p;

    /* Store the stack bottom location. */
#if defined(__ARMCC_VERSION)
    gStackBottom = (uint32_t *)&Image$$ARM_LIB_STACK$$ZI$$Base;
#elif defined (__ICCARM__)
    gStackBottom = (uint32_t *)__sfb( "CSTACK" );
#else
    gStackBottom = (uint32_t *)&__cy_stack_limit;
#endif /* defined(__ARMCC_VERSION) */

#if RESET_ON_ERROR_ENABLE
    /* If we have watchdog reset tracking enabled, the lowest twelve bytes of stack cannot be used. */
    gStackBottom += RESET_DATA_STRUCT_SIZE;
#endif /* RESET_ON_ERROR_ENABLE */

    /* Fill the stack memory with unused signature. */
    for (addr_p = gStackBottom; addr_p < (uint32_t *)((CYDEV_SRAM_BASE + CYDEV_SRAM_SIZE) - MIN_STACK_USAGE); addr_p++)
    {
        *addr_p = STACK_UNUSED_SIG;
    }

    /* Initialize the stack margin value. */
    gMinStackMargin = (uint16_t)((uint32_t)addr_p - (uint32_t)gStackBottom);

#endif /* STACK_USAGE_CHECK_ENABLE */

#if RESET_ON_ERROR_ENABLE
    if (gl_runtime_data_addr[RESET_DATA_VALID_OFFSET] == RESET_DATA_VALID_SIG)
    {
        wdr_cnt = gl_runtime_data_addr[RESET_COUNT_OFFSET];
        if (gl_runtime_data_addr[WATCHDOG_RESET_OFFSET] == WATCHDOG_RESET_SIG)
            wdr_cnt++;
    }

    /*
     * Store the reset data valid signature and current reset count.
     * Also clear the reset detected signature.
     */
    gl_runtime_data_addr[RESET_DATA_VALID_OFFSET] = RESET_DATA_VALID_SIG;
    gl_runtime_data_addr[WATCHDOG_RESET_OFFSET]   = 0;
    gl_runtime_data_addr[RESET_COUNT_OFFSET]      = wdr_cnt;
#endif /* RESET_ON_ERROR_ENABLE */
}

void instrumentation_start(void)
{
#if RESET_ON_ERROR_ENABLE
    /* Start the timer used for watchdog reset. */
    Cy_PdUtils_SwTimer_Start (gl_PdStackPort0Ctx.ptrTimerContext, NULL, CY_PDUTILS_WATCHDOG_TIMER,
            WATCHDOG_RESET_PERIOD_MS, watchdog_timer_cb);
#endif /* RESET_ON_ERROR_ENABLE */

#if WATCHDOG_HARDWARE_RESET_ENABLE
    /*
     * Enable WDT hardware reset.
     * WDT interrupt flag is expected to be cleared by software timer module
     * (At the least CY_PDUTILS_WATCHDOG_TIMER is active always).
     * If WDT interrupt handler is not executed because of CPU lock up and
     * the WDT interrupt flag is not cleared for the three consecutive
     * interrupts, a hardware reset is triggered for the recovery.
     */
    Cy_WDT_Enable();
#endif /* WATCHDOG_HARDWARE_RESET_ENABLE */
}

void instrumentation_task(void)
{
#if STACK_USAGE_CHECK_ENABLE
    uint32_t *addr_p = gStackBottom;
#endif /* STACK_USAGE_CHECK_ENABLE */

#if RESET_ON_ERROR_ENABLE
    /* Clear the variable to indicate main loop has been run. */
    gl_main_loop_delay = 0;
#endif /* RESET_ON_ERROR_ENABLE */

#if STACK_USAGE_CHECK_ENABLE
    for (addr_p = gStackBottom; addr_p < (uint32_t *)((CYDEV_SRAM_BASE + CYDEV_SRAM_SIZE) - MIN_STACK_USAGE); addr_p++)
    {
        if (*addr_p != STACK_UNUSED_SIG)
        {
            break;
        }
    }

    /* Calculate the minimum stack availability margin and update debug register. */
    gMinStackMargin = CY_PDUTILS_GET_MIN(gMinStackMargin, ((uint32_t)addr_p - (uint32_t)gStackBottom));
#endif /* STACK_USAGE_CHECK_ENABLE */
}

void CyBoot_IntDefaultHandler_Exception_EntryCallback(void)
{
    if (gl_instrumentation_cb != NULL)
    {
        gl_instrumentation_cb(0, INST_EVT_HARD_FAULT);
    }

#if RESET_ON_ERROR_ENABLE
    /* Store the reset signature into RAM. */
    gl_runtime_data_addr[WATCHDOG_RESET_OFFSET] = WATCHDOG_RESET_SIG;
    NVIC_SystemReset ();
#endif /* RESET_ON_ERROR_ENABLE */
}

void instrumentation_register_cb(instrumentation_cb_t cb)
{
    if(cb != NULL)
    {
        gl_instrumentation_cb = cb;
    }
}

/* End of file */
