/*! @file
 *
 *  @brief I/O routines for UART communications on the TWR-K70F120M.
 *
 *  This contains the functions for operating the UART (serial port).
 *
 *  @author Mohamed Hamed 12919508
 *  @author: 99178434- Yupeng Guo
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

  // Initialize start index = 0
  fifo->Start = 0;

  // Initialize end index = 0
  fifo->End = 0;

  // Initialize store bytes in FIFO to 0
  fifo->NbBytes = 0;

  //Create a Semaphore to access the buffer
  fifo->BufferSem = OS_SemaphoreCreate(1);

  //Create a Semaphore for space available in FIFO
  fifo->SpaceSem = OS_SemaphoreCreate(FIFO_SIZE);

  //Create a Semaphore to wait for space available
  fifo->AvailableSem = OS_SemaphoreCreate(0);
  return true;
}


bool FIFO_Put(TFIFO * const fifo, const uint8_t data)
{
  OS_SemaphoreWait(fifo->SpaceSem,0);  // wait for space available
  OS_SemaphoreWait(fifo->BufferSem,0); // wait for buffer access
  // Check if there is enough space in buffer
  if (fifo->NbBytes >= FIFO_SIZE)
    {
      return false;
    }

  else
    {
      fifo->Buffer[fifo->End] = data;  //Put data into FIFO buffer
      fifo->NbBytes++;                //Increase the number of bytes in FIFO
      fifo->End++;                     //Increase the end index

      //If the FIFO is full, reset
     if (fifo->End == FIFO_SIZE - 1)
	{
	  fifo->End = 0;
	}

     OS_SemaphoreSignal(fifo->BufferSem);      // Signal that there is access to buffer
     OS_SemaphoreSignal(fifo->AvailableSem);   // Signal that there is available space
    return true;

  }
}


bool FIFO_Get(TFIFO * const fifo, uint8_t * const dataPtr)
{
     OS_SemaphoreWait(fifo->AvailableSem,0);  //wait for available items
     OS_SemaphoreWait(fifo->BufferSem,0);     //wait for buffer access
     //Check whether there is data in the buffer or not
     if(fifo->NbBytes == 0) //No data in the buffer
       {
	       return false;
       }

     else //There is already some data in buffer
      {
	 *dataPtr = fifo->Buffer[fifo->Start];    //Store data in buffer
	 fifo->Start++;                           //increase start index
	 fifo->NbBytes--;                         //decrease the number of bytes in FIFO
     if (fifo->Start == FIFO_SIZE - 1)
      {
	fifo->Start = 0;
      }

     OS_SemaphoreSignal(fifo->BufferSem);   // Signal that there is access to buffer
     OS_SemaphoreSignal(fifo->SpaceSem);    // Signal that there is space in FIFO

     return true;
     }
}

