/*! @file
 *
 *  @brief I/O routines for UART communications on the TWR-K70F120M.
 *
 *  This contains the functions for operating the UART (serial port).
 *
 *  @author Mohamed Hamed 12919508
 *  @date 2019-03-28
 */


/****************************************HEADER FILES****************************************************/
#include "FIFO.h"
#include "cpu.h"
#include "PE_Types.h"
#include "OS.h"

/****************************************PUBLIC FUNCTION DEFINITION***************************************/

bool FIFO_Init(TFIFO * const fifo)
{
  OS_DisableInterrupts();
  // Initialize start index = 0
  fifo->Start = 0;

  // Initialize end index = 0
  fifo->End = 0;

  // Initialize store bytes in FIFO to 0
  fifo->NbBytes = 0;

  //Create a Semaphore for space available in FIFO
  fifo->NotFullSemaphore = OS_SemaphoreCreate(0);

  //Create a Semaphore to wait for space available
  fifo->NotEmptySemaphore = OS_SemaphoreCreate(0);
  
  return true;

}


bool FIFO_Put(TFIFO * const fifo, const uint8_t data)
{
  // Check if there is enough space in buffer
  if (fifo->NbBytes == FIFO_SIZE)
  {
    OS_SemaphoreWait(fifo->NotFullSemaphore,0); // Wait for signal saying FIFO is not full
  }

  else
  {
    fifo->Buffer[fifo->End] = data;  //Put data into FIFO buffer

    //If the FIFO is full, reset
  if (fifo->End == FIFO_SIZE - 1)
  {
    fifo->End = 0;
  }
    
  else
  {
      fifo->End++;                     //Increase the end index
  }
  fifo->NbBytes++;                    //Increase the number of bytes in FIFO
  OS_SemaphoreSignal(fifo->NotEmptySemaphore);   // Signal that there is available space
  return true;
  }

}


bool FIFO_Get(TFIFO * const fifo, uint8_t * const dataPtr)
{
  //Check whether there is data in the buffer or not
  if(fifo->NbBytes == 0) //No data in the buffer
  {
    OS_SemaphoreWait(fifo->NotEmptySemaphore);
  }

  else //There is already some data in buffer
  {
   *dataPtr = fifo->Buffer[fifo->Start];    //Store data in buffer
                     //decrease the number of bytes in FIFO
  if (fifo->Start == FIFO_SIZE - 1)
  {
    fifo->Start = 0;
  }
  else 
  {
      fifo->Start++;                         //increase start index
  }
  fifo->NbBytes--;    
  OS_SemaphoreSignal(fifo->NotFullSemaphore); // Signal saying FIFO is not full

  return true;

  }

}
