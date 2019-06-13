/*! @file

 *
 *  @brief Routines to implement and initialize LEDs functions.
 *
 *  This contains the functions for operating the LEDs.
 *
 *  @author Mohamed Hamed 12919508
 *  @date 2019-04-10
 */

/****************************************HEADER FILES****************************************************/
#include "LEDs.h"
#include "MK70F12.h"
#include "types.h"

/****************************************GLOBAL VARS*****************************************************/
// Macro for a mask with all LED control bits set
#define ALL_LEDS (LED_ORANGE | LED_YELLOW | LED_GREEN | LED_BLUE)

/****************************************PUBLIC FUNCTION DEFINITION***************************************/

/*! @brief Sets up the LEDs before first use.
 *
 *  @return bool - TRUE if the LEDs were successfully initialized.
 */
bool LEDs_Init(void)
{
  // Enable PORT A clock
  SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;

  // Drive Strength Enabled = 1, p.318
  PORTA_PCR10 |= PORT_PCR_DSE_MASK; // Blue
  PORTA_PCR11 |= PORT_PCR_DSE_MASK; // Orange
  PORTA_PCR28 |= PORT_PCR_DSE_MASK; // Yellow
  PORTA_PCR29 |= PORT_PCR_DSE_MASK; // Green
  
  // Configure multiplexed PINs in Port A for GPIO OUT usage p.317
  PORTA_PCR10 |= PORT_PCR_MUX(1); // Blue
  PORTA_PCR11 |= PORT_PCR_MUX(1); // Orange
  PORTA_PCR28 |= PORT_PCR_MUX(1); // Yellow
  PORTA_PCR29 |= PORT_PCR_MUX(1); // Green
 
  //p.2146
  GPIOA_PDDR |= ALL_LEDS;

  // Switch the LEDs off as default
  LEDs_Off(ALL_LEDS);

  return true;
}
 
/*! @brief Turns an LED on.
 *
 *  @param color The color of the LED to turn on.
 *  @note Assumes that LEDs_Init has been called.
 */
void LEDs_On(const TLED color)
{
  //p.2145
  GPIOA_PCOR = color;
}
 
/*! @brief Turns off an LED.
 *
 *  @param color THe color of the LED to turn off.
 *  @note Assumes that LEDs_Init has been called.
 */
void LEDs_Off(const TLED color)
{
  //p.2145
  GPIOA_PSOR = color;
}

/*! @brief Toggles an LED.
 *
 *  @param color THe color of the LED to toggle.
 *  @note Assumes that LEDs_Init has been called.
 */
void LEDs_Toggle(const TLED color)
{
  //p.2145
  GPIOA_PTOR = color;
}
