/*! @file PIT.c
 *
 *  @brief This file contents the implementation of the Periodic Interrupt Timer (PIT) functions
 *
 *  @author 12919508
 *  @date 19/06/2019
 */

 /*!
  *  @addtogroup PIT_module PIT module documentation
  *  @{
  */

#include "PIT.h"
#include "types.h"
#include "IO_Map.h"
#include "OS.h"
#include "analog.h"

static uint32_t ModuleClk_Global_MHz; /*!< Module clock value in MHz */
static uint32_t Clkperiod;
extern OS_ECB *PIT_Semaphore; /*!< Binary semaphore for signaling clock update */
extern OS_ECB *PIT_Semaphore1; /*!< Binary semaphore for signaling clock update */
extern OS_ECB *PIT_Semaphore2; /*!< Binary semaphore for signaling clock update */

int arrayPosition;
int arrayPosition1;

extern channelSample samples [3]; // Array of structures with two positions, one for the single channel and one for the two in parallel


/*! @brief Sets up the PIT before first use.
 *
 *  Enables the PIT and freezes the timer when debugging.
 *  @param moduleClk The module clock rate in Hz.
 *  @return bool - TRUE if the PIT was successfully initialized.
 *  @note Assumes that moduleClk has a period which can be expressed as an integral number of nanoseconds.
 */
bool PIT_Init(const uint32_t moduleClk)
{

  ModuleClk_Global_MHz = moduleClk / 1000000; //It converts the clock module to MHz
  Clkperiod = 1000000000 / moduleClk ;

  SIM_SCGC6 |= SIM_SCGC6_PIT_MASK; //Enable PIT
  PIT_MCR &= ~PIT_MCR_FRZ_MASK; // Run timers when in debug mode (PIT_MCR = 0)
  PIT_MCR &= ~PIT_MCR_MDIS_MASK; // Enable clock for PIT

  NVICICPR2 = (1<<4); //Interrupt clear instruction PIT0
  NVICISER2 = (1<<4); //Interrupt enable instruction PIT0
  NVICICPR2 = (1<<5); //Interrupt clear instruction PIT1
  NVICISER2 = (1<<5); //Interrupt enable instruction PIT1
  NVICICPR2 = (1<<6); //Interrupt clear instruction PIT2
  NVICISER2 = (1<<6); //Interrupt enable instruction PIT2

  PIT_Semaphore = OS_SemaphoreCreate(0); //PIT semaphore initialized to zero
  PIT_Semaphore1 = OS_SemaphoreCreate(0);
  PIT_Semaphore2 = OS_SemaphoreCreate(0);

  return true;
}


/*! @brief Sets the value of the desired period of the PIT0.
 *
 *  @param period The desired value of the timer period in nanoseconds.
 *  @param restart TRUE if the PIT is disabled, a new value set, and then enabled.
 *                 FALSE if the PIT will use the new value after a trigger event.
 *  @note The function will enable the timer and interrupts for the PIT.
 */
void PIT_Set(const uint32_t period, const bool restart)
{
  if (restart == true)
  {
    PIT_Enable(false);
    PIT_LDVAL0 = ((period/1000) * ModuleClk_Global_MHz) -1; // Setup timer0 for 12500000 cycles
  }

  PIT_Enable(true);
  PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK; // Enable interrupts for PIT
}

/*! @brief Sets the value of the desired period of the PIT1.
 *
 *  @param period The desired value of the timer period in nanoseconds.
 *  @param restart TRUE if the PIT is disabled, a new value set, and then enabled.
 *                 FALSE if the PIT will use the new value after a trigger event.
 *  @note The function will enable the timer and interrupts for the PIT.
 */
void PIT_Set1(const uint64_t period, const bool restart)
{
  if (restart == true)
  {
    PIT_Enable1(false);
    PIT_LDVAL1 = ((period) / Clkperiod) -1;
    PIT_Enable1(true);
    PIT_TCTRL1 |= PIT_TCTRL_TIE_MASK;
  }

  else
  {
    PIT_LDVAL1 = ((period) / Clkperiod) -1;
    PIT_Enable1(true);
    PIT_TCTRL1 |= PIT_TCTRL_TIE_MASK;
  }
}

/*! @brief Sets the value of the desired period of the PIT0.
 *
 *  @param period The desired value of the timer period in nanoseconds.
 *  @param restart TRUE if the PIT is disabled, a new value set, and then enabled.
 *                 FALSE if the PIT will use the new value after a trigger event.
 *  @note The function will enable the timer and interrupts for the PIT.
 */
void PIT_Set2(const uint32_t period, const bool restart)
{
  if (restart == true)
  {
    PIT_Enable2(false);
    PIT_LDVAL2 = ((period/1000) * ModuleClk_Global_MHz) -1; // Setup timer0 for 12500000 cycles
  }

  PIT_Enable2(true);
  PIT_TCTRL2 |= PIT_TCTRL_TIE_MASK; // Enable interrupts for PIT
}


/*! @brief Enables or disables the PIT0.
 *
 *  @param enable - TRUE if the PIT is to be enabled, FALSE if the PIT is to be disabled.
 */
void PIT_Enable(const bool enable)
{
  if (enable == true) // Enable PIT
  {
    PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;
  }
  else // Disable PIT
  {
    PIT_TCTRL0 &= ~PIT_TCTRL_TEN_MASK;
  }
}

/*! @brief Enables or disables the PIT1.
 *
 *  @param enable - TRUE if the PIT is to be enabled, FALSE if the PIT is to be disabled.
 */
void PIT_Enable1(const bool enable)
{
  if (enable == true) // Enable PIT
  {
    PIT_TCTRL1 |= PIT_TCTRL_TEN_MASK;
  }
  else // Disable PIT
  {
    PIT_TCTRL1 &= ~PIT_TCTRL_TEN_MASK;
  }
}

/*! @brief Enables or disables the PIT2.
 *
 *  @param enable - TRUE if the PIT is to be enabled, FALSE if the PIT is to be disabled.
 */
void PIT_Enable2(const bool enable)
{
  if (enable == true) // Enable PIT
  {
    PIT_TCTRL2 |= PIT_TCTRL_TEN_MASK;
  }
  else // Disable PIT
  {
    PIT_TCTRL2 &= ~PIT_TCTRL_TEN_MASK;
  }
}


/*! @brief Interrupt service routine for the PIT0.
 *
 *  The periodic interrupt timer has timed out.
 *  The user callback function will be called.
 *  @note Assumes the PIT has been initialized.
 */
void __attribute__ ((interrupt)) PIT_ISR(void)
{

  OS_ISREnter();

  if (PIT_TFLG0 & PIT_TFLG_TIF_MASK) // Check if timeout has occurred
  {
    PIT_TFLG0 |= PIT_TFLG_TIF_MASK; // Clear timer interrupt flag
    Analog_Get(0, &(samples[0].myArray[arrayPosition])); // Read channel 0
    arrayPosition++;

    if (arrayPosition == 16 )
    {
      arrayPosition = 0;
      OS_SemaphoreSignal(PIT_Semaphore);
    }
     // Signal PIT thread to tell it can run
  }

  OS_ISRExit(); // End of servicing interrupt

}

/*! @brief Interrupt service routine for the PIT1.
 *
 *  The periodic interrupt timer has timed out.
 *  The user callback function will be called.
 *  @note Assumes the PIT has been initialized.
 */
void __attribute__ ((interrupt)) PIT1_ISR(void)
{

  OS_ISREnter();

  if (PIT_TFLG1 & PIT_TFLG_TIF_MASK) // Check if timeout has occurred
  {
    PIT_TFLG1 |= PIT_TFLG_TIF_MASK; // Clear timer interrupt flag
    OS_SemaphoreSignal(PIT_Semaphore1); // Signal PIT thread to tell it can run
  }

  OS_ISRExit(); // End of servicing interrupt

}

/*! @brief Interrupt service routine for the PIT0.
 *
 *  The periodic interrupt timer has timed out.
 *  The user callback function will be called.
 *  @note Assumes the PIT has been initialized.
 */
void __attribute__ ((interrupt)) PIT2_ISR(void)
{

  OS_ISREnter();

  if (PIT_TFLG2 & PIT_TFLG_TIF_MASK) // Check if timeout has occurred
  {
    PIT_TFLG2 |= PIT_TFLG_TIF_MASK; // Clear timer interrupt flag
    Analog_Get(1, &(samples[1].myArray[arrayPosition1])); // Read channel 1
    Analog_Get(2, &(samples[2].myArray[arrayPosition1])); // Read channel 2
    arrayPosition1++;

    if (arrayPosition1 == 16 )
    {
      arrayPosition1 = 0;
      OS_SemaphoreSignal(PIT_Semaphore2);
    }
     // Signal PIT thread to tell it can run
  }

  OS_ISRExit(); // End of servicing interrupt

}

/*!
 * @}
*/


