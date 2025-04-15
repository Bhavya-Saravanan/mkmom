/***********************************************************************
 * @file      timer.c
 * @version   1.0
 * @brief     Source file for timer configuration and interrupt setup.
 *
 * @author    Bhavya Saravanan, bhsa3618@colorado.edu
 * @date      13-02-2025
 *
 * @institution University of Colorado Boulder (UCB)
 * @course      ECEN 5823: IoT Embedded Firmware
 * @instructor  Chris Choi
 *
 * @resources  Referred Silicon Labs' Peripheral Libraries
 *
 ***********************************************************************/


#include "em_letimer.h"
#include "em_cmu.h"
#include "gpio.h"
#include "timer.h"
#include "src/oscillator.h"
#include "src/scheduler.h"
#include "em_core.h"
#include "app.h"

#define INCLUDE_LOG_DEBUG 1
#include "log.h"



/**
 * @brief  Initializes LETIMER0 for periodic operation.
 *
 * This function configures LETIMER0 for periodic interrupts based on
 * the selected energy mode.
 *
 * @param  None
 * @return None
 */
void TimerInit(void)
{
    uint32_t VALUE_TO_LOAD_3000MS;

    if (LOWEST_ENERGY_MODE == 3) {
        VALUE_TO_LOAD_3000MS = PERIOD_VALUE_EM3;
    } else {
        VALUE_TO_LOAD_3000MS = PERIOD_VALUE;
    }

    const LETIMER_Init_TypeDef letimerInit = {
        .enable = false,          // Enable later after setup
        .debugRun = true,         // Keep running while debugging
        .comp0Top = true,         // Load COMP0 into CNT on underflow
        .bufTop = false,          // Do not load COMP1 into COMP0
        .out0Pol = 0,             // Default output pin value
        .out1Pol = 0,             // Default output pin value
        .ufoa0 = letimerUFOANone, // No underflow output action
        .ufoa1 = letimerUFOANone, // No underflow output action
        .repMode = letimerRepeatFree, // Free-running mode
        .topValue = VALUE_TO_LOAD_3000MS // Set COMP0 value for period
    };

    LETIMER_Init(LETIMER0, &letimerInit);
    LETIMER_CompareSet(LETIMER0, 0, VALUE_TO_LOAD_3000MS);  // Set full period
    LETIMER_IntClear(LETIMER0, 0xFFFFFFFF);  // Clear all pending interrupts
    LETIMER_IntEnable(LETIMER0, LETIMER_IF_UF|LETIMER_IF_COMP1); // Enable UF and COMP1 interrupts
    LETIMER_Enable(LETIMER0, true);  // Start the timer
}
/**
 * @brief  Provides a blocking delay in microseconds.
 *
 * This function uses the LETIMER0 counter to create a blocking delay
 * for the specified duration.
 *
 * @param  us - The delay time in microseconds.
 * @return None
 */
void timerWaitUs(uint32_t us_wait)
{

    if (us_wait < 10 || us_wait > 1000000) {
        LOG_ERROR("out of range values and please check the values");
        return;
    }
    if(LOWEST_ENERGY_MODE ==3)
    {
       // Computing compare values
       CURRENT_LFA_FREQ = ULFRCO_CLK_FREQ ;
    }else{
       CURRENT_LFA_FREQ = ACTUAL_CLK_FREQ;
    }
    uint32_t timer_freq =  CURRENT_LFA_FREQ;
    uint32_t ticks = (us_wait * timer_freq) / 1000000;

    uint32_t start_count = LETIMER_CounterGet(LETIMER0); // Read current LETIMER count
    uint32_t end_count = (start_count > ticks) ? (start_count - ticks) : 1;

    while (LETIMER_CounterGet(LETIMER0) >= end_count);
}

/**
 * @brief Non-blocking microsecond delay using LETIMER0 COMP1.
 *
 * This function generates a delay in microseconds by setting the COMP1 register.
 * It puts the MCU into sleep mode and wakes it up using an interrupt.
 *
 * @param us_wait - Delay duration in microseconds.
 * @return None
 */
void timerWaitUs_irq(uint32_t us_wait)
{
    uint32_t timer_freq;
    uint32_t ticks;
    if (us_wait < 10 || us_wait > 1000000) {
            LOG_ERROR("out of range values ");
            return;
        }
    if (LOWEST_ENERGY_MODE == 3) {
        timer_freq = ULFRCO_CLK_FREQ;
    } else {
        timer_freq = ACTUAL_CLK_FREQ;
    }

    ticks = (us_wait * timer_freq) / 1000000;  // Convert microseconds to ticks
    ticks = LETIMER_CounterGet(LETIMER0) - ticks;
    LETIMER_CompareSet(LETIMER0, 1, ticks );
    LETIMER_IntEnable(LETIMER0, LETIMER_IF_COMP1);  // Enable COMP1 once
}







