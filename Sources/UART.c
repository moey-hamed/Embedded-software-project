/*! @file
 *
 *  @brief I/O routines for UART communications on the TWR-K70F120M.
 *
 * this contains UART initialisation , UART poll and UART I/O
 *
 *  @author  12919508- Mohamed Hamed
 *  @author  99178434- Yupeng Guo
 *  @date 2019-03-28
 */


/****************************************HEADER FILES****************************************************/
#include "UART.h"
#include "types.h"
#include "FIFO.h"
#include "MK70F12.h"
#include "Cpu.h"
#include "PE_Types.h"
#include "OS.h"

/****************************************GLOBAL VARS*****************************************************/
#define THREAD_STACK_SIZE 100

TFIFO TFIFO1;
TFIFO RFIFO1;

// Prototypes
static void ReceiveThread(void* pData);
static void TransmitThread(void* pData);

static uint32_t TransmitThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08))); /*!< The stack for the transmit thread. */
static uint32_t ReceiveThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08))); /*!< The stack for the receive thread */
static OS_ECB *TransmitSemaphore; /*!< Binary semaphore for signaling that data transmission */
static OS_ECB *ReceiveSemaphore;  /*!< Binary semaphore for signaling receiving of data */
/****************************************PUBLIC FUNCTION DEFINITION***************************************/

bool UART_Init(const uint32_t baudRate, const uint32_t moduleClk)
{
  OS_ERROR error;             /*!< Thread content */

  uint16union_t sbr;   	      /*!< The SBR value */
  uint16_t baudRateDivisor;   /*!< The SBR and BRFD value as a whole number */
  uint8_t brfa;   // Define baud rate fine adjustment variable


  SIM_SCGC4 |= SIM_SCGC4_UART2_MASK;  //Enable UART2 in SIM_SCGC4(lab1 hints)

  SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;  //Enable pin routing for port E(lab1 hints)

  //Shares pins with PortE bits 16 and 17(p.280).(lab1 hints)

  PORTE_PCR16 |= PORT_PCR_MUX(3);   //Set Pin16 to MUX model ALT3
  PORTE_PCR17 |= PORT_PCR_MUX(3);   //Set Pin17 to MUX model ALT3

  //Initialize the FIFO buffers for transform data

  FIFO_Init(&TFIFO1);                //Initialize the Transmit FIFO
  FIFO_Init(&RFIFO1);                //Initialize the Receive FIFO

  //Baud rate generation(k70 pdf p.1973)
  //UART baud rate = UART module clock / (16  (SBR[12:0] + BRFD)
  //Define a variable ZF = (UART module clock / UART baud rate) for helping calculate

  const uint32_t ZF = (moduleClk / baudRate);

  //Calculate the baud rate
  sbr.l = ZF / 16;

  if (sbr.l > 8191)                  //The max value of 13-bit number is 8191
    {
      return false;
    }

  //Set the baud rate

  UART2_BDH = sbr.s.Hi;             //Set 5 high bits
  UART2_BDL = sbr.s.Lo;             //Set 8 low bits

  //Calculate the baud rate fine adjust value
  brfa = (2 * ZF) % 32;

  //Set the baud rate fine 5 bits fine adjust value
  UART2_C4 |= UART_C4_BRFA_MASK & brfa;

  UART2_C2 |= UART_C2_TIE_MASK;
  UART2_C2 |= UART_C2_RIE_MASK;
  
  UART2_C2 |= UART_C2_TE_MASK;		//Enables UART transmitter
  UART2_C2 |= UART_C2_RE_MASK;		//Enables UART receiver

  NVICICPR1 = NVIC_ICPR_CLRPEND(1 << (49 % 32));
  NVICISER1 = NVIC_ISER_SETENA(1 << (49 % 32));
  
  FIFO_Init(&RFIFO1); // Initialize receiver FIFO
  
  FIFO_Init(&TFIFO1); // Initialize transmitter FIFO

  ReceiveSemaphore = OS_SemaphoreCreate(0); // Receive semaphore initialized to 0
  TransmitSemaphore = OS_SemaphoreCreate(0); // Transmit semaphore initialized to 0

  error = OS_ThreadCreate(ReceiveThread, // 2nd highest priority thread
                          NULL,
                          &ReceiveThreadStack[THREAD_STACK_SIZE - 1],
                          1);

  error = OS_ThreadCreate(TransmitThread, // 3rd highest priority thread
                          NULL,
                          &TransmitThreadStack[THREAD_STACK_SIZE - 1],
                          2);
  return true;
}


bool UART_InChar(uint8_t* const dataPtr)
{
  //Get the data store in receive FIFO
  return FIFO_Get(&RFIFO1, dataPtr);
}


bool UART_OutChar(const uint8_t data)
{
  EnterCritical();
  bool success;
  
  UART2_C2 &= ~UART_C2_TIE_MASK;
  success = FIFO_Put(&TFIFO1, data);
  UART2_C2 |= UART_C2_TIE_MASK;
  
  ExitCritical();
  //Place data to the transmit FIFO
  return success;
}

/*! @brief Poll the UART status register to try and receive and/or transmit one character.
 *
 *  @return void
 *  @note Assumes that UART_Init has been called.
 */
//void UART_Poll(void)  //UART poll to be placed in the main loop
//{
//If TDRE is set, transmit data to the data register
// if (UART2_S1 & UART_S1_TDRE_MASK)
//  {
//   FIFO_Get(&TFIFO1, (uint8_t *) &UART2_D);
//  }

//If RDRF is set, receive data from the data register
// if (UART2_S1 & UART_S1_RDRF_MASK)
// {
//    FIFO_Put(&RFIFO1, UART2_D);
// }
//}

 /*! @brief Thread that looks after transmitting data.
  *
  *  @param pData Thread parameter.
  *  @note Assumes that semaphores are created and communicate properly.
  */
 void TransmitThread(void *data)
 {
   for (;;)
   {
	 OS_SemaphoreWait(TransmitSemaphore, 0); // Wait for transmit semaphore to signal
	 if (UART2_S1 & UART_S1_TDRE_MASK) // Clear TDRE flag by reading it
	 {
	   FIFO_Get(&TFIFO1,(uint8_t* )&UART2_D);
	   UART2_C2 |= UART_C2_TIE_MASK; // Re-enable transmission interrupt
	 }
   }
 }
 
/*! @brief Thread that looks after receiving data.
 *
 *  @param pData Thread parameter.
 *  @note Assumes that semaphores are created and communicate properly.
 */
static void ReceiveThread(void* pData)
{
  for (;;)
  {
    OS_SemaphoreWait(ReceiveSemaphore, 0); // Wait for receive semaphore to signal
    FIFO_Put(&RFIFO1, UART2_D); // Put byte into RxFIFO
    UART2_C2 |= UART_C2_RIE_MASK; // Re-enable receive interrupt
  }
}


 void __attribute__ ((interrupt)) UART_ISR(void)
{
  OS_ISREnter();

  {
    UART2_C2 &= ~UART_C2_RIE_MASK; // Receive interrupt disabled
    OS_SemaphoreSignal(ReceiveSemaphore); // Signal receive thread
  }

  if (UART2_C2 & UART_C2_TIE_MASK) // Clear TDRE flag by reading it
  {
    UART2_C2 &= ~UART_C2_TIE_MASK; // Transmit interrupt disabled
    OS_SemaphoreSignal(TransmitSemaphore); // Signal transmit thread
  }

  OS_ISRExit();

}


