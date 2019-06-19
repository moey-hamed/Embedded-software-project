/*! @file
 *
 *  @brief Routines for controlling the Real Time Clock (RTC) on the TWR-K70F120M.
 *
 *  This implements the functions for operating the real time clock (RTC).
 *  @author  12919508- Mohamed Hamed
 *  @author  99178434- Yupeng Guo
 *  @date 2019-04-29
 */
/************************Header Files************************/
#include "RTC.h"
#include "MK70F12.h"
#include "types.h"
#include "PE_Types.h"
#include "OS.h"
/***********************Global variables*********************/
static void *RTC_Arguments;
static void (*RTC_Callback)(void*);
extern OS_ECB *Update_Clock_Semaphore; /*!< Binary semaphore for signaling clock update */
/************************Public Functions****************************/
bool RTC_Init(void (*userFunction)(void*), void* userArguments)
{
  // Set the function and arguments
  RTC_Arguments = userArguments;
  RTC_Callback = userFunction;

  SIM_SCGC6 |= SIM_SCGC6_RTC_MASK; //Enable RTC clock gate

  RTC_CR |= RTC_CR_OSCE_MASK; //Enable 32.768 kHz Oscillator
  for (uint32_t i = 0; i < 0x60000; i++) ; //wait for the oscillator startup time

  RTC_IER |= RTC_IER_TSIE_MASK; //Enable Time seconds interrupt
  RTC_LR &= ~RTC_LR_CRL_MASK;  // Lock the control register(no more writing)

  NVICICPR2 |= NVIC_ICPR_CLRPEND(1 << 3); //Clear interrupts (67 % 32 = 3)
  NVICISER2 |= NVIC_ISER_SETENA(1 << 3); // Enable interrupt (67 % 32 = 3)

  return true; //True if RTC successfully initialized
}

void RTC_Set(const uint8_t hours, const uint8_t minutes, const uint8_t seconds)
{
  RTC_TPR &= ~RTC_TPR_TPR_MASK;//Reset Prescaler Register
  RTC_SR &= ~RTC_SR_TCE_MASK; //Disable Time counter to set Time

  RTC_TSR = (hours * 3600) + (minutes * 60) + seconds; //set the Times seconds register

  RTC_SR |= RTC_SR_TCE_MASK; // Enable the Timer counter
}

void RTC_Get(uint8_t* const hours, uint8_t* const minutes, uint8_t* const seconds)
{
  uint32_t time = RTC_TSR; //stores the time

  *hours = (time / ( 60 * 60)) % 24;
  *minutes = (time % (60 * 60) / 60);
  *seconds = (time % (60 * 60) % 60);
}

void __attribute__ ((interrupt)) RTC_ISR(void)
{
  OS_ISREnter();
  OS_SemaphoreSignal(Update_Clock_Semaphore); // Signal RTC thread to update clock value
  OS_ISRExit();
}

