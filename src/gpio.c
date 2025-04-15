/*
  gpio.c
 
   Created on: Dec 12, 2018
       Author: Dan Walkes
   Updated by Dave Sluiter Dec 31, 2020. Minor edits with #defines.

   March 17
   Dave Sluiter: Use this file to define functions that set up or control GPIOs.
   
   Jan 24, 2023
   Dave Sluiter: Cleaned up gpioInit() to make it less confusing for students regarding
                 drive strength setting. 

 *
 * Student edit: Add your name and email address here:
 * @student   Bhavya Saravanan, bhsa3618@Colorado.edu
 *
 
 */


// *****************************************************************************
// Students:
// We will be creating additional functions that configure and manipulate GPIOs.
// For any new GPIO function you create, place that function in this file.
// *****************************************************************************

#include <stdbool.h>
#include "em_gpio.h"
#include <string.h>

#include "gpio.h"


// Student Edit: Define these, 0's are placeholder values.
//
// See the radio board user guide at https://www.silabs.com/documents/login/user-guides/ug279-brd4104a-user-guide.pdf
// and GPIO documentation at https://siliconlabs.github.io/Gecko_SDK_Doc/efm32g/html/group__GPIO.html
// to determine the correct values for these.
// If these links have gone bad, consult the reference manual and/or the datasheet for the MCU.
// Change to correct port and pins:
#define LED_port   (gpioPortF) // GPIO port for LEDs gpioPortF ==5
#define LED0_pin   (4)  // Pin number for LED0 on the portF
#define LED1_pin   (5)  // Pin number for LED1 on the portF



#define LCD_EXTCOMIN_PIN (13)

// Set GPIO drive strengths and modes of operation
/**
 * @brief Initializes the GPIO pins.
 *
 * Configures the GPIO pins for LED control by setting the appropriate drive strength
 * and mode for each port and pin.
 *
 * @param None
 * @return None
 */
void gpioInit()
{

	//GPIO_DriveStrengthSet(LED_port, gpioDriveStrengthStrongAlternateStrong); // Strong, 10mA
   GPIO_DriveStrengthSet(LED_port, gpioDriveStrengthWeakAlternateWeak); // Weak, 1mA
	
	// Set the 2 GPIOs mode of operation and button mode
	GPIO_PinModeSet(LED_port, LED0_pin, gpioModePushPull, false);
	GPIO_PinModeSet(LED_port, LED1_pin, gpioModePushPull, false);
	GPIO_PinModeSet(PB0_PORT, PB0_PIN, gpioModeInputPullFilter, 1);
  GPIO_PinModeSet(PB1_PORT, PB1_PIN, gpioModeInputPullFilter, 1);
  // Enable glitch filtering & interrupt on both edges
  GPIO_ExtIntConfig(PB0_PORT, PB0_PIN, PB0_PIN, true, true, true);
  GPIO_ExtIntConfig(PB1_PORT, PB1_PIN, PB1_PIN, true, true, true);
  // Enable EVEN and odd GPIO Interrupts in NVIC (PB0 is even)
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
} // gpioInit()

/**
 * @brief Turns on LED0.
 *
 * Sets the GPIO pin associated with LED0 to a high state, turning the LED on.
 *
 * @param None
 * @return None
 */
void gpioLed0SetOn()
{
	GPIO_PinOutSet(LED_port, LED0_pin);
}

/**
 * @brief Turns off LED0.
 *
 * Clears the GPIO pin associated with LED0, setting it to a low state and turning the LED off.
 *
 * @param None
 * @return None
 */
void gpioLed0SetOff()
{
	GPIO_PinOutClear(LED_port, LED0_pin);
}
/**
 * @brief Turns on LED1.
 *
 * Sets the GPIO pin associated with LED1 to a high state, turning the LED on.
 *
 * @param None
 * @return None
 */

void gpioLed1SetOn()
{
	GPIO_PinOutSet(LED_port, LED1_pin);
}
/**
 * @brief Turns off LED1.
 *
 * Clears the GPIO pin associated with LED1, setting it to a low state and turning the LED off.
 *
 * @param None
 * @return None
 */

void gpioLed1SetOff()
{
	GPIO_PinOutClear(LED_port, LED1_pin);
}
/**
 * @brief Controls the LCD EXTCOMIN signal.
 *
 * Sets or clears the EXTCOMIN signal for the LCD display based on the input value.
 *
 * @param value A boolean value where true sets the pin high, and false clears the pin.
 * @return None
 */
void gpioSetDisplayExtcomin(bool value) {
    (value) ? GPIO_PinOutSet(LCD_EXTCOMIN_PORT, LCD_EXTCOMIN_PIN)
            : GPIO_PinOutClear(LCD_EXTCOMIN_PORT, LCD_EXTCOMIN_PIN);
}






