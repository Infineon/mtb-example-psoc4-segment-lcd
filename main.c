/******************************************************************************
 * File Name:   main.c
 *
 * Description: This is the source code for the PSoC 4 Segment LCD example
 *              for ModusToolbox.
 *
 * Related Document: See README.md
 *
 *
 *******************************************************************************
 * Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
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

/******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg_seglcd.h"

/******************************************************************************
 * Include header files
 ******************************************************************************/
/* Delay */
#define BUTTON_DELAY_MS  (200u)

/*******************************************************************************
 * Global Variables
 ********************************************************************************/
volatile uint32_t timer = 0u;  /* The unsigned integer number to be displayed */
volatile uint32_t state = 0u;  /* User button interrupt state */
uint32_t temp = 0u;            /* Variable to save active or Deep Sleep mode */

/*******************************************************************************
 * Function Prototypes
 ********************************************************************************/
void button_isr(void);
void timer_isr(void);
cy_en_syspm_status_t deep_sleep_callback(cy_stc_syspm_callback_params_t *callbackParams, cy_en_syspm_callback_mode_t mode);

/******************************************************************************
 * Interrupt configuration structure
 *******************************************************************************/
cy_stc_sysint_t button_intr_cfg =
{
    .intrSrc = WAKEUP_PIN_IRQ,   /* Interrupt source is button interrupt */
    .intrPriority = 3UL          /* Interrupt priority is 3 */
};

cy_stc_sysint_t timer_intr_cfg =
{
    .intrSrc = TIMER_IRQ,   /* Interrupt source is timer interrupt */
    .intrPriority = 3UL     /* Interrupt priority is 3 */
};

/*******************************************************************************
 * Function Name: main
 ********************************************************************************
 * Summary:
 * System entrance point. This function performs
 *    1. Initializes the BSP.
 *    2. Initializes and enables the button interrupt.
 *    3. Initializes and enables the TIMER and segment LCD peripherals.
 *    4. Puts the chip into Deep Sleep mode.
 *    5. Restores work after Deep Sleep mode.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  int
 *
 *******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize and enable button interrupt */
    result = Cy_SysInt_Init(&button_intr_cfg, button_isr);
    if(result != CY_SYSINT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    NVIC_EnableIRQ(button_intr_cfg.intrSrc);

    /* Initialize and enable timer interrupt */
    result = Cy_SysInt_Init(&timer_intr_cfg, timer_isr);
    if(result != CY_SYSINT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    NVIC_EnableIRQ(timer_intr_cfg.intrSrc);

    /* Initialize the TIMER */
    if (CY_TCPWM_SUCCESS != Cy_TCPWM_Counter_Init(TIMER_HW, TIMER_NUM, &TIMER_config))
    {
        /* Handle possible errors */
        CY_ASSERT(0);
    }

    /* Check if the desired interrupt is enabled prior to triggering */
    if (0UL != (CY_TCPWM_INT_ON_TC & Cy_TCPWM_GetInterruptMask(TIMER_HW, TIMER_NUM)))
    {
        Cy_TCPWM_SetInterrupt(TIMER_HW, TIMER_NUM, CY_TCPWM_INT_ON_TC);
    }

    /* enables the TIMER */
    Cy_TCPWM_Counter_Enable(TIMER_HW, TIMER_NUM);

    /* Initialize the segment LCD */
    if (CY_SEGLCD_SUCCESS == Cy_SegLCD_Init(LCD, &LCD_DISP_config))
    {
        if (CY_SEGLCD_SUCCESS == Cy_SegLCD_ClrFrame(LCD, LCD_DISP_commons))
        {
            /* enables the segment LCD */
            Cy_SegLCD_Enable(LCD);

            /* Now the block generates LCD signals (all the pixels are off) and is ready to turn on any pixel
             * (or many pixels) using any of the frame/pixel/character/display management API functions.
             */
        }
        else
        {
            /* error handling */
            CY_ASSERT(0);
        }
    }
    else
    {
        /* error handling */
        CY_ASSERT(0);
    }

    Cy_TCPWM_TriggerStart(TCPWM, TIMER_MASK);

    /* SysPm callback params */
    cy_stc_syspm_callback_params_t callbackParams = {
            /*.base       =*/ NULL,
            /*.context    =*/ NULL
        };

    /* SysClk context structure */
    cy_stc_sysclk_context_t sysClkContext;

    /* SysClk callback params */
    cy_stc_syspm_callback_params_t sysClkCallbackParams =
    {
        .base       = NULL,
        .context    = (void*)&sysClkContext
    };

    /* Callback declaration for SysClk Deep Sleep mode */
    cy_stc_syspm_callback_t sysClkCallback =
    {
        .callback       = &Cy_SysClk_DeepSleepCallback,
        .type           = CY_SYSPM_DEEPSLEEP,
        .skipMode       = 0UL,
        .callbackParams = &sysClkCallbackParams,
        .prevItm        = NULL,
        .nextItm        = NULL,
        .order          = 0
    };

    /* Callback declaration for Deep Sleep mode */
    cy_stc_syspm_callback_t deep_sleep_cb = {
        deep_sleep_callback,        /* Callback function */
        CY_SYSPM_DEEPSLEEP,         /* Callback type */
        0,                          /* Skip mode */
        &callbackParams,            /* Callback params */
        NULL, NULL};                /* For internal usage */


    /* Register SysClk Deep Sleep callback */
    Cy_SysPm_RegisterCallback(&sysClkCallback);

    /* Register Deep Sleep callback */
    Cy_SysPm_RegisterCallback(&deep_sleep_cb);

    for (;;)
    {
        if (0u == state)
        {
            /* Displaying the timer value */
        }
        else
        {
            if (0u == temp)
            {
                temp = 1u;

                /* Turn the LED connected to LP_OUT_PIN OFF to indicate low power mode */
                Cy_GPIO_Write(LP_OUT_PIN_PORT, LP_OUT_PIN_PIN, 1u);

                /* Put the chip into Deep Sleep mode */
                Cy_SysPm_CpuEnterDeepSleep();
            }
            else
            {
                /* Turn the LED connected to LP_OUT_PIN ON to indicate active mode */
                Cy_GPIO_Write(LP_OUT_PIN_PORT, LP_OUT_PIN_PIN, 0u);

                temp = 0u;
                state = 0u;
            }
        }
    }
}

/*******************************************************************************
 * Function Name: button_isr
 ********************************************************************************
 * Summary:
 * Interrupt service routine for the button interrupt:
 *   1. Sets a state variable.
 *   2. Clears interrupt from the button.
 *
 * Parameters:
 *  None
 *
 * Return
 *  void
 *
 *******************************************************************************/
void button_isr(void)
{
    Cy_SysLib_Delay(BUTTON_DELAY_MS);
    state = 1u;
    Cy_GPIO_ClearInterrupt(WAKEUP_PIN_PORT, WAKEUP_PIN_NUM);
}

/*******************************************************************************
 * Function Name: timer_isr
 ********************************************************************************
 * Summary:
 * TIMER interrupt service routine:
 *   1. Increases a timer value.
 *   2. Displays a current timer value on the segment LCD glass.
 *   3. Clears interrupts from the TIMER.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void timer_isr(void)
{
    timer++;
    Cy_SegLCD_WriteNumber(LCD, timer, 0, &LCD_DISP_Display0, false, false);
    Cy_TCPWM_ClearInterrupt(TIMER_HW, TIMER_NUM, CY_TCPWM_INT_ON_TC);
}

/*******************************************************************************
 * Function Name: deep_sleep_callback
 ********************************************************************************
 *
 * Summary:
 * Deep Sleep callback implementation. It disables the timer before
 * entering Deep Sleep power mode. After waking up, the timer gets restored.
 *
 * Parameters:
 *  callbackParams: The pointer to the callback parameters structure
 *                  cy_stc_syspm_callback_params_t.
 *  mode: Callback mode, see cy_en_syspm_callback_mode_t
 *
 * Return:
 *  Entered status, see cy_en_syspm_status_t.
 *
 *******************************************************************************/
cy_en_syspm_status_t deep_sleep_callback(cy_stc_syspm_callback_params_t *callbackParams, cy_en_syspm_callback_mode_t mode)
{
    cy_en_syspm_status_t ret_val = CY_SYSPM_FAIL;

    switch (mode)
    {
        case CY_SYSPM_CHECK_READY:

            ret_val = CY_SYSPM_SUCCESS;
            break;

        case CY_SYSPM_CHECK_FAIL:

            ret_val = CY_SYSPM_SUCCESS;
            break;

        case CY_SYSPM_BEFORE_TRANSITION:

            /* Disable the TIMER */
            Cy_TCPWM_Counter_Disable(TCPWM, TIMER_NUM);

            ret_val = CY_SYSPM_SUCCESS;
            break;

        case CY_SYSPM_AFTER_TRANSITION:

            /* Enable the Start counter */
            Cy_TCPWM_Counter_Enable(TCPWM, TIMER_NUM);

            Cy_TCPWM_TriggerStart(TCPWM, TIMER_MASK);

            ret_val = CY_SYSPM_SUCCESS;
            break;

        default:
            /* Don't do anything in the other modes */
            ret_val = CY_SYSPM_SUCCESS;
            break;
    }
    return ret_val;
}


/* [] END OF FILE */
