/*! @file
 *
 *  @brief Routines to implement packet encoding and decoding for the serial port.
 *
 *  This contains the functions for implementing the "Tower to PC Protocol" 5-byte packets.
 *
 *  @author PMcL
 *  @date 2015-07-23
 */

#ifndef PACKET_H
#define PACKET_H

// New types
#include "types.h"

// Packet structure
#define PACKET_NB_BYTES 5

#pragma pack(push)
#pragma pack(1)

typedef union
{
  uint8_t bytes[PACKET_NB_BYTES];     /*!< The packet as an array of bytes. */
  struct
  {
    uint8_t command;		      /*!< The packet's command. */
    union
    {
      struct
      {
        uint8_t parameter1;	      /*!< The packet's 1st parameter. */
        uint8_t parameter2;	      /*!< The packet's 2nd parameter. */
        uint8_t parameter3;	      /*!< The packet's 3rd parameter. */
      } separate;
      struct
      {
        uint16_t parameter12;         /*!< Parameter 1 and 2 concatenated. */
        uint8_t parameter3;
      } combined12;
      struct
      {
        uint8_t paramater1;
        uint16_t parameter23;         /*!< Parameter 2 and 3 concatenated. */
      } combined23;
    } parameters;
    uint8_t checksum;
  } packetStruct;
} TPacket;

#pragma pack(pop)

#define Packet_Command     Packet.packetStruct.command
#define Packet_Parameter1  Packet.packetStruct.parameters.separate.parameter1
#define Packet_Parameter2  Packet.packetStruct.parameters.separate.parameter2
#define Packet_Parameter3  Packet.packetStruct.parameters.separate.parameter3
#define Packet_Parameter12 Packet.packetStruct.parameters.combined12.parameter12
#define Packet_Parameter23 Packet.packetStruct.parameters.combined23.parameter23
#define Packet_Checksum    Packet.packetStruct.checksum

extern TPacket Packet;
/******************Commands:PC to tower*********************************/

//0x04 Get startup values
#define PC_GET_STARTUP 0x04
#define PC_GET_STARTUP_PAR1 0
#define PC_GET_STARTUP_PAR2 0
#define PC_GET_STARTUP_PAR3 0

//0x07 Program byte
#define PC_PROGRAM_BYTE 0x07
#define PC_PROGRAM_BYTE_PAR1
#define PC_PROGRAM_BYTE_PAR2 0
#define PC_PROGRAM_BYTE_PAR3

//0X07 Read byte
#define PC_READ_BYTE 0x08
#define PC_READ_BYTE_PAR1
#define PC_READ_BYTE_PAR2 0
#define PC_READ_BYTE_PAR3 0

//0x09 Get version
#define PC_GET_VERSION 0x09
#define PC_GET_VERSION_PAR1 'v'
#define PC_GET_VERSION_PAR2 'x'
#define PC_GET_VERSION_PAR3 'CR'

//0x0B Tower Number
#define PC_TOWER_NUMBER 0x0B
#define PC_TOWER_NUMBER_PAR1_GET 1
#define PC_TOWER_NUMBER_PAR1_SET 2
#define PC_TOWER_NUMBER_PAR2_GET 0
//#define TOWER_NUMBER_PAR2_SET LSB
#define PC_TOWER_NUMBER_PAR3_GET 0
//#define TOWER_NUMBER_PAR3_SET MSB

//0x0D Tower Mode
#define PC_TOWER_MODE 0x0D
#define PC_TOWER_MODE_PAR1_GET 1
#define PC_TOWER_MODE_PAR1_SET 2
#define PC_TOWER_MODE_PAR2_GET 0
//#define TOWER_NUMBER_PAR2_SET LSB
#define PC_TOWER_MODE_PAR3_GET 0
//#define TOWER_NUMBER_PAR3_SET MSB

/******************Commands: Tower to PC *********************************/
//0x04 Tower startup
#define TOWER_STARTUP 0x04
#define TOWER_STARTUP_PAR1 0
#define TOWER_STARTUP_PAR2 0
#define TOWER_STARTUP_PAR3 0

//0x08 Read bit
#define TOWER_READ_BIT 0x08
#define TOWER_READ_BIT_PAR1
#define TOWER_READ_BIT_PAR2 0
#define TOWER_READ_BIT_PAR3

//0x09 Special-Tower version
#define TOWER_VERSION 0x09
#define TOWER_VERSION_PAR1 'v'
#define TOWER_VERSION_PAR2 1
#define TOWER_VERSION_PAR3 0

//0x0B Tower number
#define TOWER_NUMBER 0x0B
#define TOWER_NUMBER_PAR1 1
#define TOWER_NUMBER_PAR2
#define TOWER_NUMBER_PAR3

//0x0C Time
#define PC_TIME 0x0C

//0X0D TOWER Mode
#define TOWER_MODE 0x0D
#define TOWER_MODE_PAR1 1
#define TOWER_MODE_PAR2
#define TOWER_MODE_PAR3

// Acknowledgment bit mask
extern const uint8_t PACKET_ACK_MASK;

/*! @brief Initializes the packets by calling the initialization routines of the supporting software modules.
 *
 *  @param baudRate The desired baud rate in bits/sec.
 *  @param moduleClk The module clock rate in Hz.
 *  @return bool - TRUE if the packet module was successfully initialized.
 */
bool Packet_Init(const uint32_t baudRate, const uint32_t moduleClk);

/*! @brief Attempts to get a packet from the received data.
 *
 *  @return bool - TRUE if a valid packet was received.
 */
bool Packet_Get(void);

/*! @brief Builds a packet and places it in the transmit FIFO buffer.
 *
 *  @return bool - TRUE if a valid packet was sent.
 */
bool Packet_Put(const uint8_t command, const uint8_t parameter1, const uint8_t parameter2, const uint8_t parameter3);

#endif
