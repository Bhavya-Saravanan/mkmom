/*
   gpio.h
  
    Created on: Dec 12, 2018
        Author: Dan Walkes

    Updated by Dave Sluiter Sept 7, 2020. moved #defines from .c to .h file.
    Updated by Dave Sluiter Dec 31, 2020. Minor edits with #defines.

    Editor: Feb 26, 2022, Dave Sluiter
    Change: Added comment about use of .h files.

 *
 * Student edit: Add your name and email address here:
 * @student    Bhavya Saravanan, bhsa3618@Colorado.edu
 *
 
 */


// Students: Remember, a header file (a .h file) generally defines an interface
//           for functions defined within an implementation file (a .c file).
//           The .h file defines what a caller (a user) of a .c file requires.
//           At a minimum, the .h file should define the publicly callable
//           functions, i.e. define the function prototypes. #define and type
//           definitions can be added if the caller requires theses.


#ifndef SRC_GPIO_H_
#define SRC_GPIO_H_
#include "em_gpio.h"
#define PB0_PORT gpioPortF
#define PB0_PIN  6
#define PB1_PORT gpioPortF
#define PB1_PIN  7
#define LCD_EXTCOMIN_PORT (gpioPortD)
// Function prototypes
/**
 * @brief Initializes the GPIO pins.
 *
 * This function configures the GPIO pins for LED control, including setting
 * the drive strength and mode for the appropriate ports and pins.
 */
void gpioInit();

/**
 * @brief Turns on LED0.
 *
 * Sets the GPIO pin associated with LED0 to a high state.
 *
 * @param None
 * @return None
 */
void gpioLed0SetOn();

/**
 * @brief Turns off LED0.
 *
 * Clears the GPIO pin associated with LED0, setting it to a low state.
 *
 * @param None
 * @return None
 */
void gpioLed0SetOff();

/**
 * @brief Turns off LED1.
 *
 * Clears the GPIO pin associated with LED1, setting it to a low state.
 *
 * @param None
 * @return None
 */
void gpioLed1SetOn();

/**
 * @brief Turns off LED1.
 *
 * Clears the GPIO pin associated with LED1, setting it to a low state.
 *
 * @param None
 * @return None
 */
void gpioLed1SetOff();
/**
 * @brief Controls the LCD EXTCOMIN signal.
 *
 * Sets or clears the EXTCOMIN signal for the LCD display based on the input value.
 *
 * @param value A boolean value where true sets the pin high, and false clears the pin.
 * @return None
 */
void gpioSetDisplayExtcomin(bool value);
#endif /* SRC_GPIO_H_ */
