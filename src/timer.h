/***********************************************************************
 * @file      timer.h
 * @version   1.0
 * @brief     Header file for timer configuration and initialization.
 *
 * @author    Bhavya Saravanan, bhsa3618@colorado.edu
 * @date      13-02-2025
 *
 * @institution University of Colorado Boulder (UCB)
 * @course      ECEN 5823: IoT Embedded Firmware
 * @instructor  Chris Choi
 *
 * @resources  Referred Silicon Labs' Peripheral Libraries
 ***********************************************************************/

#ifndef TIMER_H_
#define TIMER_H_
#include "app.h"
#include "src/oscillator.h"


#define PERIOD_VALUE_EM3      ((TIMER_PERIOD_MS * ULFRCO_CLK_FREQ) / 1000)
#define PERIOD_VALUE     ((TIMER_PERIOD_MS * ACTUAL_CLK_FREQ) / 1000)

#define LFXO_FREQUENCY     (32768) //frequency used in the EM0/EM1/EM2 MODE
#define ULFRC0_FREQUENCY   (1000)  //frequency used in the EM3 mode
#define ACTUAL_CLK_FREQ    (LFXO_FREQUENCY / PRESCALER_VALUE)  //Period formula for EM0/EM1/EM2 MODE
#define ULFRCO_CLK_FREQ    (ULFRC0_FREQUENCY/PRESCALER_VALUE) // Period formula for EM3 mode
#define TIMER_PERIOD_MS    (3000) // period in ms
extern volatile uint32_t milliseconds;  // Declare as extern

/**
 * @brief  Initializes LETIMER0 for periodic operation.
 *
 * This function configures LETIMER0 for periodic interrupts based on
 * the selected energy mode.
 *
 * @param  None
 * @return None
 */
void TimerInit(void);
/**
 * @brief  Provides a blocking delay in microseconds.
 *
 * This function uses the LETIMER0 counter to create a blocking delay
 * for the specified duration.
 *
 * @param  us - The delay time in microseconds.
 * @return None
 */
void timerWaitUs(uint32_t us);
/**
 * @brief Non-blocking microsecond delay using LETIMER0 COMP1.
 *
 * This function generates a delay in microseconds by setting the COMP1 register.
 * It puts the MCU into sleep mode and wakes it up using an interrupt.
 *
 * @param us_wait - Delay duration in microseconds.
 * @return None
 */
void timerWaitUs_irq(uint32_t us_wait);



#endif /* TIMER_H_ */
