/*! @file
 *
 *  @brief Routines for controlling Periodic Interrupt Timer (PIT) on the TWR-K70F120M.
 *
 *  This contains the functions for operating the periodic interrupt timer (PIT).
 *
 *  @author PMcL
 *  @date 2015-08-22
 */

#ifndef PIT_H
#define PIT_H

// new types
#include "types.h"

typedef struct
{
  int16_t myArray[16]; // Array of sixteen samples
  int16_t vrmsValue; // Value of the VRMS for this channel
}
channelSample;

/*! @brief Sets up the PIT before first use.
 *
 *  Enables the PIT and freezes the timer when debugging.
 *  @param moduleClk The module clock rate in Hz.
 *  @return bool - TRUE if the PIT was successfully initialized.
 *  @note Assumes that moduleClk has a period which can be expressed as an integral number of nanoseconds.
 */
bool PIT_Init(const uint32_t moduleClk);

/*! @brief Sets the value of the desired period of the PIT0.
 *
 *  @param period The desired value of the timer period in nanoseconds.
 *  @param restart TRUE if the PIT is disabled, a new value set, and then enabled.
 *                 FALSE if the PIT will use the new value after a trigger event.
 *  @note The function will enable the timer and interrupts for the PIT.
 */
void PIT_Set(const uint32_t period, const bool restart);


/*! @brief Sets the value of the desired period of the PIT1.
 *
 *  @param period The desired value of the timer period in nanoseconds.
 *  @param restart TRUE if the PIT is disabled, a new value set, and then enabled.
 *                 FALSE if the PIT will use the new value after a trigger event.
 *  @note The function will enable the timer and interrupts for the PIT.
 */
void PIT_Set1(const uint64_t period, const bool restart);

/*! @brief Sets the value of the desired period of the PIT1.
 *
 *  @param period The desired value of the timer period in nanoseconds.
 *  @param restart TRUE if the PIT is disabled, a new value set, and then enabled.
 *                 FALSE if the PIT will use the new value after a trigger event.
 *  @note The function will enable the timer and interrupts for the PIT.
 */
void PIT_Set2(const uint32_t period, const bool restart);

/*! @brief Enables or disables the PIT0.
 *
 *  @param enable - TRUE if the PIT is to be enabled, FALSE if the PIT is to be disabled.
 */
void PIT_Enable(const bool enable);

/*! @brief Enables or disables the PIT1.
 *
 *  @param enable - TRUE if the PIT is to be enabled, FALSE if the PIT is to be disabled.
 */
void PIT_Enable1(const bool enable);

/*! @brief Enables or disables the PIT2.
 *
 *  @param enable - TRUE if the PIT is to be enabled, FALSE if the PIT is to be disabled.
 */
void PIT_Enable2(const bool enable);

/*! @brief Interrupt service routine for the PIT0.
 *
 *  The periodic interrupt timer has timed out.
 *  @note Assumes the PIT has been initialized.
 */
void __attribute__ ((interrupt)) PIT_ISR(void);

/*! @brief Interrupt service routine for the PIT1.
 *
 *  The periodic interrupt timer has timed out.
 *  @note Assumes the PIT has been initialized.
 */
void __attribute__ ((interrupt)) PIT1_ISR(void);

/*! @brief Interrupt service routine for the PIT2.
 *
 *  The periodic interrupt timer has timed out.
 *  @note Assumes the PIT has been initialized.
 */
void __attribute__ ((interrupt)) PIT2_ISR(void);


#endif
