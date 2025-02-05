/*******************************************************************************
* File Name: parameters.h
* Version 1.0
*
* Description:
*  This file contains the parameters and specifications of the motor and the various peripherals used
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
#ifndef __PARAMETER_H__
#define __PARAMETER_H__

/******************************************************************************
 * Header file including
 ******************************************************************************/

/******************************************************************************
 * Macro declaration
 ******************************************************************************/
/*============================================================================
 * macro for motor physical parameters
 *============================================================================*/
/* motor pole-pairs number */
#define MOTOR_POLE_PAIR_NUM                              ((uint8_t)(4))    /* 8 poles; 4 pole-pairs */

/* Number of phase windings */
#define PHASE_CNT                                        ((uint8_t)(3))    /* 3 phase */


/*============================================================================
 * macro for solution general setting
 *============================================================================*/
/* VDD range, unit: V */
#define VDD_RANGE                                        ((uint8_t)(3.3))    /* 3.3V */

/* resistor divider scale for DC bus voltage */
#define DC_BUS_SAMPLE_SCALE                              ((float)((1+19.1)/1))    /* 20.1 -  based on the voltage divider network used at the input */

/* value of current sense resistor connected to half-bridge, unit: mohm */
#define CURRENT_SENSE_RESISTOR                           ((uint8_t)(30))    /* 30 mohm */

/* IMO frequency, unit: kHz */
#define IMO_FREQ                                         ((uint32_t)(48000))    /* 48MHz */

/* BEMF-A channel input to AMUXBUS_B */
#define BEMF_A                                           ((uint8_t)(1))    /* P2.2 - BEMF_A */

/* BEMF-B channel input to AMUXBUS_B */
#define BEMF_B                                           ((uint8_t)(2))    /* P2.3 - BEMF_B */

/* BEMF-C channel input to AMUXBUS_B */
#define BEMF_C                                           ((uint8_t)(3))    /* P2.4 - BEMF_C */

/* seconds per minutes */
#define SECOND_PER_MINUTE                                ((uint8_t)(60))

/* PD sink voltage on DC bus for EPR contract, unit: V */
#define PD_EPR_SNK_VOLTAGE                               ((uint8_t)(28))    /* 28V */

/* Normal PD sink voltage on DC bus, unit: V */
#define PD_SNK_VOLTAGE                                   ((uint8_t)(20))    /* 20V */


/*============================================================================
 * macro for TCPWM block settings
 *============================================================================*/
/* PMW frequency, unit: kHz */
#define PWM_CLK_FREQ                                     ((uint16_t)(24000))    /* 24MHz */

/* PMW period count */
#define PWM_PERIOD_CNT                                   ((uint16_t)(1000))    /* 1000 counts */

/* PMW frequency, unit: kHz */
#define PWM_FREQ                                         ((uint16_t)(PWM_CLK_FREQ / PWM_PERIOD_CNT))    /* 24kHz */

/* Startup Timer clock frequency, unit: kHz */
#define STARTUP_TIMER_CLK_FREQ                           ((uint32_t)(1))    /* 1kHz */

/* Speed Counter clock frequency, unit: kHz */
#define SPEED_COUNTER_CLK_FREQ                           ((uint32_t)(960))    /* 960kHz */

/* Startup Timer clock divider value */
#define STARTUP_TIMER_CLKDIV                             ((uint32_t)((IMO_FREQ / STARTUP_TIMER_CLK_FREQ) - 1))    /* 47999 */

/* Speed Counter clock divider value */
#define SPEED_COUNTER_CLKDIV                             ((uint32_t)((IMO_FREQ / SPEED_COUNTER_CLK_FREQ) - 1))    /* 49 */

/* Peripheral clock divider number */
#define CONTROL_TIMER_CLK_DIV                            ((uint32_t)(3))    /* Peri Clk Divider 3 */


/*============================================================================
 * macro for ADC sampling
 *============================================================================*/
/* Maximum ADC result range in single-ended mode */
#define ADC_RSLT_RANGE                                   ((uint16_t)(4096))    /* 12-bit SAR ADC: 0 - 4095 */

/* Maximum input voltage to ADC in single-ended mode, unit: V */
#define ADC_VOLTAGE_RANGE                                ((float)(2.4))    /* (0 - 2.4V) - single-ended mode */

/* ADC channel number for speed reference potentiometer */
#define SPEED_REF_CHAN                                   ((uint32_t)(0))    /* Channel 0 */

/* ADC channel number for DC bus voltage sample */
#define VOLTAGE_MEASURE_CHAN                             ((uint32_t)(1))    /* Channel 1 */

/* ADC channel number for winding current sample */
#define CURRENT_MEASURE_CHAN                             ((uint32_t)(2))    /* Channel 2 */

/* define the maximum resistor value for potentiometer, unit: ohm */
#define SPEED_POT_VALUE                                  (uint16_t)(10000)    /* 10kohm */


/*============================================================================
 * macro for preposition
 *============================================================================*/
/* default initial PWM duty cycle for pole alignment */
#define PREPOSITION_PWM_DUTY                             ((uint16_t)(5))    /* 5% */

/* Default initial PWM compare value for motor pre-position (alignment) */
#define PREPOSITION_PWM_COMPARE                          ((uint16_t)(PREPOSITION_PWM_DUTY * PWM_PERIOD_CNT / 100))    /* 90 counts */

/* default alignment period for Startup Timer during preposition, unit: ms */
#define PREPOSITION_TIMER_PERIOD                         ((uint16_t)(400))    /* 400ms */

/* default alignment period count for Startup Timer during preposition */
#define PREPOSITION_TIMER_PERIOD_CNT                     ((uint32_t)(PREPOSITION_TIMER_PERIOD * STARTUP_TIMER_CLK_FREQ))    /* 400 */


/*============================================================================
 * macro for startup
 *============================================================================*/
/* default duty cycle for free-run in open loop */
#define FREERUN_PWM_DUTY                                 ((uint16_t)(30))    /* 30% */

/* Default PWM compare value during free-run */
#define FREERUN_PWM_COMPARE                              ((uint16_t)(FREERUN_PWM_DUTY * PWM_PERIOD_CNT / 100))    /* 300 counts */

/* default initial period for Startup Timer during free-run in open loop, unit: ms */
#define FREERUN_TIMER_PERIOD                             ((uint16_t)(6))    /* 6ms */

/* default minimum period for Startup Timer during free-run in open loop, unit: ms */
#define FREERUN_TIMER_PERIOD_MIN                         ((uint16_t)(1))    /* 1ms */

/* default initial period count for Startup Timer during free-run in open loop */
#define FREERUN_TIMER_PERIOD_CNT                         ((uint16_t)(FREERUN_TIMER_PERIOD * STARTUP_TIMER_CLK_FREQ))    /* 6 */

/* default minimum period count for Startup Timer during free-run in open loop */
#define FREERUN_TIMER_PERIOD_CNT_MIN                     ((uint16_t)(FREERUN_TIMER_PERIOD_MIN * STARTUP_TIMER_CLK_FREQ))    /* 1 */

/* default period reduction step for Startup Timer during free run to accelerate the motor */
#define FREERUN_TIMER_PERIOD_STEP                        ((uint16_t)(1))    /* 1 count per cycle */

/* Number of commutation cycles performed in open-loop during startup phase. This cavlue is used to control the open-loop duration */
#define STARTUP_CYCLE_CNT                                ((uint8_t)(24))    /* 20 cycles */


/*============================================================================
 * macro for speed PID control
 *============================================================================*/
/* Number of delay cycles to start control after closing the loop */
#define CONTROL_START_DELAY                              ((uint8_t)(4))    /* 4 commutation cycles delay */

/* Interval (in cycles) to update the control variable */
#define CONTROL_UPDATE_INTERVAL                          ((uint8_t)(2))    /* every 2 cycles of SpeedCounter interrupt */

/* default value of Proportional gain, Kp */
#define KP_VALUE                                         ((float)(0.015))

/* default value of Integral gain, Ki */
#define KI_VALUE                                         ((float)(1))

/* default value of Differential gain, Kd */
#define KD_VALUE                                         ((float)(0.008))

/* maximum duty cycle for driving PWM */
#define MAX_PWM_DUTY                                     ((uint16_t)(80))    /* 80% */

/* Maximum PWM compare value corresponding to MAX_PWM_DUTY */
#define PWM_COMPARE_MAX                                  ((uint16_t)(MAX_PWM_DUTY * PWM_PERIOD_CNT / 100))    /* 800 counts */

/* Minimum duty cycle for driving PWM */
#define MIN_PWM_DUTY                                     ((uint16_t)(10))    /* 10% */

/* Minimum PWM compare value corresponding to MAX_PWM_DUTY */
#define PWM_COMPARE_MIN                                  ((uint16_t)(MIN_PWM_DUTY * PWM_PERIOD_CNT / 100))    /* 100 counts */


/*============================================================================
 * macro for speed calculation
 *============================================================================*/
/* Maximum capture value for minimum speed */
#define SPEED_CAPTURE_MAX                                ((uint32_t)(7000))    /* 7000 counts - corresponds to around 10% PWM duty cycle*/

/* Minimum capture value for maximum speed */
#define SPEED_CAPTURE_MIN                                ((uint32_t)(600))    /* 600 counts - corresponds to around 80% PWM duty cycle*/

/* Initial capture value assigned when closed-loop control starts */
#define SPEED_CAPTURE_INITIAL                            ((uint32_t)(2400))    /* 2400 counts */

/* Intermediate capture value to break the commutation delays */
#define SPEED_CAPTURE_MID                                ((uint32_t)(2400))    /* 2400 counts */

/* Threshold for the lower limit of the motor speed */
#define SPEED_CAPTURE_THRESHOLD_MAX                      ((uint32_t)(8500))    /* 8500 counts */

/* Threshold for the upper limit of the motor speed */
#define SPEED_CAPTURE_THRESHOLD_MIN                      ((uint32_t)(600))    /* 600 counts */

/* Threshold for maximum step change in the motor speed during run-time */
#define SPEED_CAPTURE_STEP_THRESHOLD                     ((int32_t)(2500))    /* 2500 counts */

/* default period count for Speed Counter */
#define SPEED_COUNTER_PERIOD_CNT                         ((uint32_t)(65535))    /* 65535 counts */

/* Maximum reference RPM for the motor */
#define REF_RPM_MAX                                      ((uint16_t)((SPEED_COUNTER_CLK_FREQ * 1000) / SPEED_CAPTURE_MIN) * (SECOND_PER_MINUTE / (PHASE_CNT * MOTOR_POLE_PAIR_NUM)))    /* 8000 */

/* Minimum reference RPM for the motor */
#define REF_RPM_MIN                                      ((uint16_t)((SPEED_COUNTER_CLK_FREQ * 1000) / SPEED_CAPTURE_MAX) * (SECOND_PER_MINUTE / (PHASE_CNT * MOTOR_POLE_PAIR_NUM)))    /* 685 */

/* Initial reference RPM for the motor during startup */
#define REF_RPM_INITIAL                                  ((uint16_t)((SPEED_COUNTER_CLK_FREQ * 1000) / SPEED_CAPTURE_INITIAL) * (SECOND_PER_MINUTE / (PHASE_CNT * MOTOR_POLE_PAIR_NUM)))    /* 2000 */


/*============================================================================
 * macro for commutation
 *============================================================================*/
/* Time period (us) of PWM signal, unit: us */
#define PWM_PERIOD_US                                   ((float)((float)PWM_PERIOD_CNT / (float)PWM_FREQ))    /* 41.666us */

/* Time period (us) for terminal count of the Speed Counter, unit: us  */
#define SPEED_COUNTER_PERIOD_US                         ((float)((float)SPEED_COUNTER_PERIOD_CNT / (float)SPEED_COUNTER_CLK_FREQ * 1000))    /* 68265.625us */

/* No. of PWM periods in SPEED_COUNTER_PERIOD_CNT  */
#define CAPTURE_DELAY_CONVERT_FACTOR                    ((uint16_t)(SPEED_COUNTER_PERIOD_US / PWM_PERIOD_US))    /* Capture value to Delay conversion factor = 1638 */

/* 
 * No. of PWM periods in SPEED_COUNTER_PERIOD_CNT
 * Lesser than calculated value is used at low speed to reduce the motor vibration due to non-linearity at lower speeds
 * SPEED_CAPTURE_MID value is used to split the lower speed ranges  to assign this value for calculation
 * Directly proportional to the PWM clock frequency used
 */
#define LS_CAPTURE_DELAY_CONVERT_FACTOR                 ((uint16_t)(1500))    /* Capture value to Delay conversion factor at low speed = 1500 (for 24kHz PWM clock) */

/* Initial value of commutation delay (in count of OA1 interrupts) to be assigned at the beginning of closed-loop control  */
#define COMMUTATE_DELAY_CNT_INITIAL                     ((uint8_t)(SPEED_CAPTURE_INITIAL * CAPTURE_DELAY_CONVERT_FACTOR / SPEED_COUNTER_PERIOD_CNT))

/* Fine tuning parameter for delay in terms of opamp interrupt count (within +/-6) */
#define DELAY_OFFSET_TUNE                               ((int8_t)(3))

/* Total number of sectors */
#define SECTOR_T                                        ((uint8_t)(6))

/* Defining six sectors for commutation */
typedef enum
{
    SECTOR_1 = 1, SECTOR_2, SECTOR_3, SECTOR_4, SECTOR_5, SECTOR_6
}en_commutation_sectors_t;

/*============================================================================
 * macro for protection
 *============================================================================*/
/* Under-voltage threshold to turn off the motor, unit: V */
#define  UV_THRESHOLD                                  ((uint8_t)(18))    /* 18V */

/* Over-voltage threshold to turn off the motor, unit: V */
#define  OV_THRESHOLD                                  ((uint8_t)(30))    /* 30V */

/* Over-current threshold to turn off the motor, unit: A */
#define  OC_THRESHOLD                                  ((float)(3.5))    /* 3.5A */

/* Under-voltage threshold sample value extracted from DC Vbus voltage, unit: V */
#define  UV_THRESHOLD_SAMPLE                           ((float)((float)UV_THRESHOLD / DC_BUS_SAMPLE_SCALE))    /* 0.8955V */

/* Over-voltage threshold sample value extracted from DC Vbus voltage, unit: V */
#define  OV_THRESHOLD_SAMPLE                           ((float)((float)OV_THRESHOLD / DC_BUS_SAMPLE_SCALE))    /* 1.4925V */

/* Over-current threshold sample value extracted from current sense resistor voltage, unit: V */
#define  OC_THRESHOLD_SAMPLE                           ((float)(((float)OC_THRESHOLD * CURRENT_SENSE_RESISTOR))/1000)    /* 0.105V */

/* ADC result equivalent to  the under-voltage threshold */
#define  UV_ADC_THRESHOLD                              ((uint16_t)((UV_THRESHOLD_SAMPLE * ADC_RSLT_RANGE) / ADC_VOLTAGE_RANGE))    /* 1528 */

/* ADC result equivalent to  the over-voltage threshold */
#define  OV_ADC_THRESHOLD                              ((uint16_t)(OV_THRESHOLD_SAMPLE * ADC_RSLT_RANGE / ADC_VOLTAGE_RANGE))    /* 2547 */

/* ADC result equivalent to  the over-current threshold */
#define  OC_ADC_THRESHOLD                              ((uint16_t)((OC_THRESHOLD_SAMPLE * ADC_RSLT_RANGE) / (ADC_VOLTAGE_RANGE)))    /* 179 */

/* Number of counts of under-voltage detection required to trigger a UV condition */
#define  UV_CNT_THRESHOLD                              ((uint8_t)(5))

/* Number of counts of under-voltage detection required to trigger a OV condition */
#define  OV_CNT_THRESHOLD                              ((uint8_t)(5))

/* Number of counts of under-voltage detection required to trigger a OC condition */
#define  OC_CNT_THRESHOLD                              ((uint8_t)(5))

/*============================================================================
 * macro for error detection
 *============================================================================*/
/* Error codes corresponding to various detectable errors */
#define UNDER_VOLTAGE                                  ((uint8_t)(0))
#define OVER_VOLTAGE                                   ((uint8_t)(1))
#define OVER_CURRENT                                   ((uint8_t)(2))
#define OVERLOAD                                       ((uint8_t)(3))
#define COMMUTATION_ERROR                              ((uint8_t)(4))
#define NO_ERROR                                       ((uint8_t)(5))

/* Parameters to set the user LED blinks for various types of errors */
#define UV_ERROR_BLNK_CNT                              ((uint8_t)(2))    /* Blink LED in sets of 2 */
#define OV_ERROR_BLNK_CNT                              ((uint8_t)(3))    /* Blink LED in sets of 3 */
#define OC_ERROR_BLNK_CNT                              ((uint8_t)(4))    /* Blink LED in sets of 4 */
#define OL_ERROR_BLNK_CNT                              ((uint8_t)(6))    /* Blink LED in sets of 6 */
#define COMM_ERROR_BLNK_CNT                            ((uint8_t)(0))    /* Blink LED continuously */

/* Parameters to set the user LED blink-rates */
#define LED_BLNK_INTRVL                                ((uint8_t)(200))    /* Time interval (ms) between LED blinks */
#define LED_BLNK_PERIOD                                ((uint16_t)(1000))    /* Time period (ms) for LED blinking */

/*============================================================================
 * macro for switch operations
 *============================================================================*/
/* default switch press & hold time for motor direction reversal, unit: ms */
#define SW_HOLD_TIME                         ((uint16_t)(3000))    /* 3000ms */

/* Switch hold time conversion to ControlTimer interrupt counts */
#define SW_HOLD_CNT                         ((uint8_t)(SW_HOLD_TIME/PREPOSITION_TIMER_PERIOD))    /* 7 counts */

/* Number of times the user LED blinks when the switch is long pressed for direction reversal */
#define SW_TIMER_BLNK_CNT                   ((uint8_t)(3))    /* Blink LED three times */

/* Number of times the switch needs to be pressed to reset the controller on occurrence of an error */
#define SW_RESET_CNT                         ((uint8_t)(2))    /* 2 times */

/******************************************************************************
 * Global variables declaration
 ******************************************************************************/

/******************************************************************************
 * Global function declaration
 ******************************************************************************/

/******************************************************************************
 * End of declaration
 ******************************************************************************/

#endif  /* __PARAMETER_H__ */
/* [] END OF FILE */
