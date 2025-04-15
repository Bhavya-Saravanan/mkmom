/***********************************************************************
 * @file      oscillator.h
 * @version   1.0
 * @brief     Header file for oscillator configuration.
 *
 * @author    Bhavya Saravanan, bhsa3618@colorado.edu
 * @date      30-01-2025
 *
 * @institution University of Colorado Boulder (UCB)
 * @course      ECEN 5823: IoT Embedded Firmware
 * @instructor  Chris Choi
 *
 * @resources  Referred Silicon Labs' Peripheral Libraries
 *
 ***********************************************************************/

#ifndef OSCILLATOR_H_
#define OSCILLATOR_H_


#define PRESCALER_VALUE cmuClkDiv_2  // Setting the prescalar value to 2

extern uint32_t CURRENT_LFA_FREQ;




/**
 * @brief  Initializes the system oscillator for LETIMER0 operation.
 *
 * This function configures and enables the appropriate oscillator
 * (LFXO or ULFRCO) based on the selected energy mode for LETIMER0.
 *
 * - Enables the required oscillator.
 * - Waits for the oscillator to stabilize.
 * - Selects the oscillator as the clock source for LETIMER0.
 * - Sets the required prescaler for LETIMER0 operation.
 *
 * @param  None
 * @return None
 */

void oscillator_init(void);

#endif /* OSCILLATOR_H_ */
