/*
 * @file
 *
 *  @created on: 28/03/2019
 *  @brief: Packet module, This module contains the code for managing incoming and outgoing packets
 *  @author: 12919508- Mohamed Hamed
 *  @date: 17/04/2019
 */

/****************************************HEADER FILES****************************************************/
#include "packet.h"
#include "UART.h"
#include "MK70F12.h"
#include "types.h"
#include "PE_Types.h"
#include "Cpu.h"
#include "OS.h"
/****************************************GLOBAL VARS*****************************************************/
uint8_t packet_position = 0;
/****************************************PRIVATE FUNCTION DECLARATION***********************************/
static bool IsChecksumValid(void);
void ShiftPacket(void);

/****************************************PUBLIC FUNCTION DEFINITION***************************************/

bool Packet_Init(const uint32_t baudRate, const uint32_t moduleClk)
{
  // Initialise the UART and receive/transmit buffers
  return UART_Init(baudRate, moduleClk);
}


bool Packet_Get(void)
{
  uint8_t data;

  //Checks whether there is data in the receive FIFO and stores it the address pointed by Data
  if (UART_InChar(&data))
  {
      switch(packet_position)
      {
	//Command
	case 0:
	  Packet_Command = data;
	  packet_position++;
	  return false; //incomplete packet
	  break;

	//Parameter 1
	case 1:
	  Packet_Parameter1 = data;
	  packet_position++;
	  return false; //incomplete packet
	  break;

        //Parameter 2
	case 2:
	  Packet_Parameter2 = data;
	  packet_position++;
	  return false; //incomplete packet
	  break;

	//Parameter 3
	case 3:
	  Packet_Parameter3 = data;
	  packet_position++;
	  return false; //incomplete packet

	//Checksum
	case 4:
	  Packet_Checksum = data;
	  if (IsChecksumValid())
	  {
	      return true;
	  }

	  ShiftPacket();  //shift packet private function
	  return false;

	default:
	  packet_position = 0;
	  break;
      }
  }
  return false;
}

bool Packet_Put(const uint8_t command, const uint8_t parameter1, const uint8_t parameter2, const uint8_t parameter3)
{
  OS_DisableInterrupts();
  // Calculate check sum
  uint8_t checkSum = command ^ parameter1 ^ parameter2 ^ parameter3;
  if (!UART_OutChar(command))
    return false;		    //Place Command byte in TxFIFO
  if (!UART_OutChar(parameter1))
    return false;			//Place Parameter1 byte in TxFIFO
  if (!UART_OutChar(parameter2))
    return false;			//Place Parameter2 byte in TxFIFO
  if (!UART_OutChar(parameter3))
    return false;			//Place Parameter3 byte in TxFIFO
  if (!UART_OutChar(checkSum))
    return false;	//Place Checksum byte in TxFIFO
  return true;
  OS_EnableInterrupts();
}
/****************************************PRIVATE FUNCTION DEFINITION***************************************
/*! @brief Determine whether the value of the checksum we calculated is equal to the value carried by its own packet
 *
 *  @return bool - True if the calculated checksum is equal to the packet checksum
 */

static bool IsChecksumValid(void)
{
  //Calculate the packet checksum in theory
  uint8_t calculated_checksum = Packet_Command ^ Packet_Parameter1 ^ Packet_Parameter2 ^ Packet_Parameter3;
  //Verify that the current packet format is correct or not
  return (calculated_checksum == Packet_Checksum);
}

/*! @brief Throw away the first byte and then move the whole packet to the left
 *
 *  @return - void
 */
 void ShiftPacket(void)
{
  //Shift packet down
  Packet_Command = Packet_Parameter1;
  Packet_Parameter1 = Packet_Parameter2;
  Packet_Parameter2 = Packet_Parameter3;
  Packet_Parameter3 = Packet_Checksum;
}
