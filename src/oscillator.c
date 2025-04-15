/***********************************************************************
 * @file      oscillator.c
 * @version   1.0
 * @brief     Source file for oscillator configuration and initialization.
 *
 * @author    Bhavya Saravanan, bhsa3618@colorado.edu
 * @date      30-01-2025
 *
 * @institution University of Colorado Boulder (UCB)
 * @course      ECEN 5823: IoT Embedded Firmware
 * @instructor  Chris Choi
 *
 * @resources  Referred Silicon Labs' Peripheral Libraries
 ***********************************************************************/

#include "em_cmu.h"
#include "app.h"
#include "oscillator.h"
#include "src/timer.h"
#define INCLUDE_LOG_DEBUG 1
#include "log.h"

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

void oscillator_init(void)
{
  // Select oscillator based on the energy mode
  if (LOWEST_ENERGY_MODE == 3) {
     CMU_OscillatorEnable(cmuOsc_ULFRCO, true, true); // Enable ULFRCO for EM3
     CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);

  }else{
     CMU_OscillatorEnable(cmuOsc_LFXO, true, true);// Enable LFXO for other modes
     CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);

   }


  // Set prescaler and enable LETIMER0 clock
   CMU_ClockDivSet(cmuClock_LETIMER0, PRESCALER_VALUE);
   CMU_ClockEnable(cmuClock_LETIMER0, true);
}
