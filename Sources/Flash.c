/*! @file
 *  Flash.c
 *  @brief Routines to implement functions that are used to access internal Flash.
 *
 *  This contains the functions to write,modify and erase to the Flash.
 *
 *  @author Mohamed Hamed 12919508
 *  @author Yupeng Guo    99178434
 *  @date 2019-04-10
 */
/****************************************HEADER FILES****************************************************/
#include "types.h"
#include "MK70F12.h"
#include "packet.h"
#include "Flash.h"

/****************************************GLOBAL VARS*****************************************************/

#define ACCERR_FPVIOL_ERROR (FTFE_FSTAT & 0x30) // Bits showing ACCER Error or FPVIOL Error
#define CHECK_BIT(x,n) ((x >> (n)) & 1)  // Macro to compare each bit with LSB
#define FLASH_COMMAND_PROGRME_PHRASE 0x07
#define FLASH_COMMAND_ERASE_SECTOR 0x09

typedef struct
{
  struct{
    uint8_t Command;        /*!< Stores FTFE command */
    uint8_t FlashAddressHi;  /*!< Stores starting address bits [23:16] */
    uint8_t FlashAddressMd;  /*!< Stores starting address bits [15:8]  */
    uint8_t FlashAddressLo;  /*!< Stores starting address bits [7:0]   */
  }ADD;

  struct{
    uint8_t DataByte0;      /*!< Stores phrase bits [63:56] */
    uint8_t DataByte1;      /*!< Stores phrase bits [55:48] */
    uint8_t DataByte2;      /*!< Stores phrase bits [47:40] */
    uint8_t DataByte3;      /*!< Stores phrase bits [39:32] */
    uint8_t DataByte4;      /*!< Stores phrase bits [31:24] */
    uint8_t DataByte5;      /*!< Stores phrase bits [23:16] */
    uint8_t DataByte6;      /*!< Stores phrase bits [15:8]  */
    uint8_t DataByte7;      /*!< Stores phrase bits [7:0]   */
  }DATA;
} TFCCOB;

uint8_t Occupied = 0x00; /*!< Occupied space in flash memory where each bit represents a byte in a phrase */

/****************************************PRIVATE FUNCTION DECLARATION***********************************/
static bool EraseSector(const uint32_t address);
static bool LaunchCommand(TFCCOB* commonCommandObject);
static bool AllocateVar(volatile void **data, const uint8_t end,const uint8_t size);
static bool WritePhrase(const uint32_t address, const uint64union_t phrase);
static bool ModifyPhrase(const uint32_t address, const uint64union_t phrase);
static void WaitCCIF(void);

/****************************************PUBLIC FUNCTION DEFINITION***************************************/

/*! @brief Enables the Flash module.
 *
 *  @return bool - TRUE if the Flash was setup successfully.
 */
bool Flash_Init()
{
  while(!(FTFE_FSTAT & FTFE_FSTAT_CCIF_MASK)) {} // Wait for command completion
	if(ACCERR_FPVIOL_ERROR) // Check for ACCERR flag and FPVIOL flag
  		SIM_SCGC3 |= SIM_SCGC3_NFC_MASK;  	/* !Initialize the Flash Clock  */
  return true;
}

/*! @brief Allocates space for a non-volatile variable in the Flash memory.
 *
 *  @param variable is the address of a pointer to a variable that is to be allocated space in Flash memory.
 *         The pointer will be allocated to a relevant address:
 *         If the variable is a byte, then any address.
 *         If the variable is a half-word, then an even address.
 *         If the variable is a word, then an address divisible by 4.
 *         This allows the resulting variable to be used with the relevant Flash_Write function which assumes a certain memory address.
 *         e.g. a 16-bit variable will be on an even address
 *  @param size The size, in bytes, of the variable that is to be allocated space in the Flash memory. Valid values are 1, 2 and 4.
 *  @return bool - TRUE if the variable was allocated space in the Flash memory.
 *  @note Assumes Flash has been initialized.
 */
bool Flash_AllocateVar(volatile void** variable, const uint8_t size)
{
  uint8_t position = 0;		/*!< Position in the 8 byte sector */
  uint8_t availableSlots = 0;	/*!< Number of available positions */
  uint8_t found = 0 ;           /*!< Number of empty locations found */

  while (position < 8) // Cycle through the locations in the sector to find free space to allocate data
    {
      if (CHECK_BIT(Occupied,position++) == 0) // Macro to compare each bit to find available positions
	{
	  availableSlots++; // Increase counter variable if the current array slot is empty
	  found++;
	}
      else
	{
	  found = 0; // No empty locations found
	  continue; // Return to the start of the loop to increment count
	  }

      if ((availableSlots % size == 0) && (found == size))
	{
	  if (AllocateVar(variable,position-1,size)) // Place variable into the allocated free spaces
	    break;
	  else
	    return false;
	}
    }
  return true;
}

/*! @brief Writes a 32-bit number to Flash.
 *
 *  @param address The address of the data.
 *  @param data The 32-bit data to write.
 *  @return bool - TRUE if Flash was written successfully, FALSE if address is not aligned to a 4-byte boundary or if there is a programming error.
 *  @note Assumes Flash has been initialized.
 */
bool Flash_Write32(volatile uint32_t* const address, const uint32_t data)
{
  uint64union_t phrase; /*!< Stores the hi and lo components of the phrase */
  uint32_t theAddress = (uint32_t)address; /*!< Stores the starting address to store data */

  if ((theAddress/4) % 2 == 0) // Setting up a phrase
    {
      phrase.s.Lo = data; // Combine data in address+4 with current word
      phrase.s.Hi = _FW(theAddress+4);
      return ModifyPhrase(theAddress,phrase); // Return 64 bit phrase
      }
  else
    {
      phrase.s.Lo = _FW(theAddress-4); // Combine data in address-4 with current word
      phrase.s.Hi = data;
      return ModifyPhrase(theAddress-4,phrase); // Return 64 bit phrase
      }
}

/*! @brief Writes a 16-bit number to Flash.
 *
 *  @param address The address of the data.
 *  @param data The 16-bit data to write.
 *  @return bool - TRUE if Flash was written successfully, FALSE if address is not aligned to a 2-byte boundary or if there is a programming error.
 *  @note Assumes Flash has been initialized.
 */
bool Flash_Write16(volatile uint16_t* const address, const uint16_t data)
{
  uint32union_t word; /*!< Stores the hi and lo components of the word */
  uint32_t theAddress = (uint32_t)address; /*!< Stores the starting address to store data */

  if (theAddress % 4 == 0) // Setting up a word
    {
      word.s.Lo = data; // Combine data in address+2 with current half word
      word.s.Hi = _FH(theAddress+2);
      return Flash_Write32(&(_FW(theAddress)),word.l); // Return 32 bit word
      }
  else
    {
      word.s.Lo = _FH(theAddress-2); // Combine data in address-2 with current half word
      word.s.Hi = data;
      return Flash_Write32(&(_FW(theAddress-2)),word.l); // Return 32 bit word
      }
}

/*! @brief Writes an 8-bit number to Flash.
 *
 *  @param address The address of the data.
 *  @param data The 8-bit data to write.
 *  @return bool - TRUE if Flash was written successfully, FALSE if there is a programming error.
 *  @note Assumes Flash has been initialized.
 */
bool Flash_Write8(volatile uint8_t* const address, const uint8_t data)
{
  uint16union_t halfWord; /*!< Stores the hi and lo components of the half word */
  uint32_t theAddress = (uint32_t)address; /*!< Stores the starting address to store data */

  if (theAddress % 2 == 0) // Setting up a half word
    {
      halfWord.s.Lo = data; // Combine data in address+1 with current byte
      halfWord.s.Hi = _FB(theAddress+1);
      return Flash_Write16(&(_FH(theAddress)),halfWord.l); // Return 16 bit half word
      }
  else
    {
      halfWord.s.Lo = _FB(theAddress-1); // Combine data in address-1 with current byte
      halfWord.s.Hi = data;
      return Flash_Write16(&(_FH(theAddress-1)),halfWord.l); // Return 16 bit half word
      }
}

/*! @brief Erases the entire Flash sector.
 *
 *  @return bool - TRUE if the Flash "data" sector was erased successfully.
 *  @note Assumes Flash has been initialized.
 */
bool Flash_Erase(void)
{
  return EraseSector(FLASH_DATA_START); // Erase block 2's sector 0 in flash
}

/*! @brief Writes the 64 bit phrase and the 32 bit address to the structure
 *
 *  @return bool - TRUE if the Flash "data" sector was erased successfully
 *  @param address is the starting address of the data to be written at
 *  @param data is the 64 bit phrase
 */

/****************************************PRIVATE FUNCTION DEFINITION***************************************/

/*! @brief Erases the entire Flash sector
 *
 *  @return bool - TRUE if the Flash "data" sector was erased successfully
 *  @param address is the starting address of the sector
 */

static bool EraseSector(const uint32_t address)
{
  TFCCOB Erase;

  Erase.ADD.Command = FLASH_COMMAND_ERASE_SECTOR; // Command to erase sector
  Erase.ADD.FlashAddressHi = address >> 16; // Bits [23:16] of starting address
  Erase.ADD.FlashAddressLo = address >> 8; // Bits [15:8] of starting address
  Erase.ADD.FlashAddressMd = address; // Bits [7:0] of starting address
  return LaunchCommand(&Erase);
}

/*! @brief Updates FCCOB registers to write to flash
 *
 *  @return bool - TRUE if the the writing was completed successfully
 *  @param commonCommandObject is the structure which contains the stored values
 */
static bool LaunchCommand(TFCCOB* commonCommandObject)
{
  if (ACCERR_FPVIOL_ERROR) // Check for ACCERR flag and FPVIOL flag
    {
      FTFE_FSTAT = FTFE_FSTAT_ACCERR_MASK | FTFE_FSTAT_FPVIOL_MASK; // Clear past errors (0x30)
    }

  FTFE_FCCOB0 = commonCommandObject->ADD.Command; // Place structure content into FCCOB registers
  FTFE_FCCOB1 = commonCommandObject->ADD.FlashAddressHi;
  FTFE_FCCOB2 = commonCommandObject->ADD.FlashAddressMd;
  FTFE_FCCOB3 = commonCommandObject->ADD.FlashAddressLo;

  FTFE_FCCOB8 = commonCommandObject->DATA.DataByte0;
  FTFE_FCCOB9 = commonCommandObject->DATA.DataByte1;
  FTFE_FCCOBB = commonCommandObject->DATA.DataByte2;
  FTFE_FCCOBA = commonCommandObject->DATA.DataByte3;
  FTFE_FCCOB4 = commonCommandObject->DATA.DataByte4;
  FTFE_FCCOB5 = commonCommandObject->DATA.DataByte5;
  FTFE_FCCOB6 = commonCommandObject->DATA.DataByte6;
  FTFE_FCCOB7 = commonCommandObject->DATA.DataByte7;

  FTFE_FSTAT = FTFE_FSTAT_CCIF_MASK; // Launch command sequence
  WaitCCIF(); // Wait for command completion
  return true;
}

/*! @brief Allocates space for a non-volatile variable in the Flash memory
 *
 *  @param data is the address of a pointer to a variable that is to be allocated space in flash memory
 *  @param end is the position of the free starting location in the sector (position-1)
 *  @param size is the size, in bytes, of the variable that is to be allocated space in the flash memory (1, 2 or 4)
 *
 *  @return bool - TRUE if the variable was allocated space in the Flash memory.
 */
static bool AllocateVar(volatile void **data, const uint8_t end,const uint8_t size)
{
  switch (size)
  {
    case 1:
      *(volatile uint8_t**) data = &(_FB(FLASH_DATA_START+end)); // Starting address to place data(1 byte)
      Occupied |= 0x1 << end; // Set flag to signal one byte being occupied
      break;

    case 2:
      *(volatile uint16_t**) data = &(_FH(FLASH_DATA_START+(end-1))); // Starting address to place data(2 bytes)
      Occupied |= 0x3 << end-1; // Set flag to signal two bytes being occupied
      break;

    case 4:
      *(volatile uint32_t**) data = &(_FW(FLASH_DATA_START+(end-3))); // Starting address to place data(4 bytes)
      Occupied |= 0xF << end-3; // Set flag to signal four bytes being occupied
      break;

    default:
      return false; // Error
      }
  return true; // Address declaration was successfully completed
  }

/*!
 * @brief prepares the FCCOB struct to write to flash
 *
 * @param address is the location in flash to save the phrase
 * @param phrase is the data to be saved to the flash
 * @return bool - True if no errors (ACCERR/FPVIOL) occur
 */
static bool WritePhrase(const uint32_t address, const uint64union_t phrase)
{
  TFCCOB Write;

  Write.ADD.Command = FLASH_COMMAND_PROGRME_PHRASE; // Command to program phrase
  Write.ADD.FlashAddressHi = address >> 16; // Bits [23:16] of starting address
  Write.ADD.FlashAddressMd = address >> 8; // Bits [15:8] of starting address
  Write.ADD.FlashAddressLo = address; // Bits [7:0] of starting address

  Write.DATA.DataByte0 = phrase.l >> 56; // Store the phrase (separated into bytes) in the fccob structure
  Write.DATA.DataByte1 = phrase.l >> 48;
  Write.DATA.DataByte2 = phrase.l >> 40;
  Write.DATA.DataByte3 = phrase.l >> 32;
  Write.DATA.DataByte4 = phrase.l >> 24;
  Write.DATA.DataByte5 = phrase.l >> 16;
  Write.DATA.DataByte6 = phrase.l >> 8;
  Write.DATA.DataByte7 = phrase.l;

  return (LaunchCommand(&Write));
}

/*!
 * @brief modify phrase by erasing the sector and then calling write phrase
 *
 * @param address is the location in flash to modify
 * @param phrase is the data to be written to the flash
 * @return bool - True if no errors (ACCERR/FPVIOL) occur
 */
static bool ModifyPhrase(const uint32_t address, const uint64union_t phrase)
{
  //Call erase
  if (EraseSector(FLASH_DATA_START)) //If successful
    {
      //Write to flash
      return WritePhrase(address, phrase);
    }
  return false;
}

/* @brief Wait for the CCIF register to be set to 1.
 *
 */
void WaitCCIF(void)
{
  //wait for the command to complete.
  while (!(FTFE_FSTAT & FTFE_FSTAT_CCIF_MASK)); //(https://community.nxp.com/thread/329360)
}


