/* ###################################################################
 **     Filename    : main.c
 **     Project     : Lab6
 **     Processor   : MK70FN1M0VMJ12
 **     Version     : Driver 01.01
 **     Compiler    : GNU C Compiler
 **     Date/Time   : 2015-07-20, 13:27, # CodeGen: 0
 **     Abstract    :
 **         Main module.
 **         This module contains user's application code.
 **     Settings    :
 **     Contents    :
 **         No public methods
 **
 ** ###################################################################*/
/*!
 ** @file main.c
 ** @version 6.0
 ** @brief
 **         Main module.
 **         This module contains user's application code.
 */
/*!
 **  @addtogroup main_module main module documentation
 **  @{
 */
/* MODULE main */

// CPU module - contains low level hardware initialization routines
#include "Cpu.h"
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "Packet.h"
#include "UART.h"
#include "FIFO.h"
#include "LEDs.h"
#include "Flash.h"

// Simple OS
#include "OS.h"

// Analog functions
#include "analog.h"
/****************************************GLOBAL VARS*****************************************************/
TPacket Packet;
const uint8_t PACKET_ACK_MASK = 0x80u; //Used to mask out the Acknowledgment bit
const uint32_t baudRate = 115200;     //Baudrate set to 115200
const uint32_t moduleClk = CPU_BUS_CLK_HZ; //set teh moduleclock to CPU clock

static volatile uint16union_t * NvTowerNb;   // The Tower's Number
static volatile uint16union_t * NvTowerMode; // The Tower's Mode
/****************************************PRIVATE FUNCTION DECLARATION***********************************/
static bool HandlePacket(void);
static bool HandleStartup(void);
static bool HandleSpecial(void);
static bool HandleTowerNumber(void);
static bool HandleTowerMode(void);
static bool InitializeComponents (void);
static void AllocateAndSet(volatile uint16union_t ** const addressPtr, uint16_t const dataIfEmpty);
void PacketThread(void* pData);
static void InitModulesThread(void* pData);
// ----------------------------------------
// Thread set up
// ----------------------------------------
// Arbitrary thread stack size - big enough for stacking of interrupts and OS use.
#define THREAD_STACK_SIZE 100
#define NB_ANALOG_CHANNELS 2
OS_ECB* PacketSemaphore;
// Thread stacks
OS_THREAD_STACK(InitModulesThreadStack, THREAD_STACK_SIZE); /*!< The stack for the LED Init thread. */
static uint32_t AnalogThreadStacks[NB_ANALOG_CHANNELS][THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
OS_THREAD_STACK(PacketThreadStack , THREAD_STACK_SIZE);
OS_THREAD_STACK(TransmitThreadStack , THREAD_STACK_SIZE);
OS_THREAD_STACK(ReceiveThreadStack , THREAD_STACK_SIZE);

// ----------------------------------------
// Thread priorities
// 0 = highest priority
// ----------------------------------------
const uint8_t ANALOG_THREAD_PRIORITIES[NB_ANALOG_CHANNELS] = {3, 4};

/*! @brief Data structure used to pass Analog configuration to a user thread
 *
 */
typedef struct AnalogThreadData
{
  OS_ECB* semaphore;
  uint8_t channelNb;
} TAnalogThreadData;

/*! @brief Analog thread configuration data
 *
 */
static TAnalogThreadData AnalogThreadData[NB_ANALOG_CHANNELS] =
{
  {
    .semaphore = NULL,
    .channelNb = 0
  },
  {
    .semaphore = NULL,
    .channelNb = 1
  }
};
/****************************************PRIVATE FUNCTION DEFINITION***********************************/
/*! @brief Handles the received and verified packet based on its command byte.
 *
 *  @return bool - TRUE if the packet was successfully handled.
 */
static bool HandlePacket(void)
{
  bool error = false;

  // Switch on Packet Command after zeroing acknowledgment bit
  switch (Packet_Command & ~PACKET_ACK_MASK)
  {
    //Respond to a Startup Packet
    case PC_GET_STARTUP:
      if (HandleStartup())
  {
    error = true;
  }
      break;

    //Respond to a Special Packet
    case PC_GET_VERSION:
      if (HandleSpecial())
      {
    error = true;
      }
      break;


    //Respond to a Get/Set Tower Number Packer
    case PC_TOWER_NUMBER:
      if(HandleTowerNumber());
      {
    error = true;
      }
      break;
    //Respond to a Tower Mode Packet
    case PC_TOWER_MODE:
      if (HandleTowerMode());
      {
    error = true;
      }
      break;

    default:      // Received invalid or unimplemented packet
      break;
  }

  //Check whether the Acknowledgment bit is set
  if (Packet_Command & PACKET_ACK_MASK)
    {
      //Create a new command response packet
      uint8_t maskedPacket = 0;

      if (error == true)
  {
    //If there are errors, the Acknowledgment bit should be 0
    maskedPacket = Packet_Command & ~PACKET_ACK_MASK;
  }
      else
  {
    //If there are no errors, the Acknowledgment bit should be 1
    maskedPacket = Packet_Command | PACKET_ACK_MASK;
  }
      //Place the Acknowledgment Packet in the TxFIFO
      Packet_Put(maskedPacket, Packet_Parameter1, Packet_Parameter2, Packet_Parameter3);
    }
}

/*! @brief Handles the "Get startup values" packet
 *
 * Command: 0x04
 * Parameter 1: 0
 * Parameter 2: 0
 * Parameter 3: 0
 *  @return bool - TRUE if the packet was successfully handled.
 */
static bool HandleStartup(void)
{
  // Verify that the received command was for "Get Startup"
  if (Packet_Parameter1 == PC_GET_STARTUP_PAR1 && Packet_Parameter2 == PC_GET_STARTUP_PAR2 && Packet_Parameter3 == PC_GET_STARTUP_PAR3)
    {
      // Transmit the four required packets to the PC
      (void) Packet_Put(TOWER_STARTUP, TOWER_STARTUP_PAR1, TOWER_STARTUP_PAR2,TOWER_STARTUP_PAR3);
      (void) Packet_Put(TOWER_VERSION, TOWER_VERSION_PAR1, TOWER_VERSION_PAR2, TOWER_VERSION_PAR3);
      (void) Packet_Put(TOWER_NUMBER, TOWER_NUMBER_PAR1, NvTowerNb->s.Lo, NvTowerNb->s.Hi);;
      (void) Packet_Put(TOWER_MODE, TOWER_MODE_PAR1, NvTowerMode->s.Lo, NvTowerMode->s.Hi);;
      return true;
    }
  return false;
}


/*! @brief Handles the Special Command (currently only Get version implemented)
 *
 * Sends the version number to the PC.
 *
 * Command: 0x09
 * Parameter 1: d
 * Parameter 2: j
 * Parameter 3: CR
 *
 *  @return bool - TRUE if the packet was successfully handled.
 */
static bool HandleSpecial(void)
{
  // Verify that the received command was for "Get version"
  if (Packet_Parameter1 == PC_GET_VERSION_PAR1 && Packet_Parameter2 == PC_GET_VERSION_PAR2)
    {
      // Transmit the version number to the PC
      (void) Packet_Put(TOWER_VERSION, TOWER_VERSION_PAR1, TOWER_VERSION_PAR2, TOWER_VERSION_PAR3);
      return true;
    }

  // Invalid command
  return false;
}

/*! @brief Handles the Tower Number PC To Tower Command
 *
 * Command: 0x0B
 * Parameter 1: 1 = get Tower number
 *              2 = set Tower number
 * Parameter 2: LSB for set, 0 for a get
 * Parameter 3: MSB for set, 0 for a get
 * @note The Tower number is an unsigned 16-bit number
 *
 *  @return bool - TRUE if the packet was successfully handled.
 */
static bool HandleTowerNumber(void)
{
  //Verify that the received command was for "Tower Number"
  if (Packet_Parameter1 == PC_TOWER_NUMBER_PAR1_GET && Packet_Parameter23 == PC_TOWER_NUMBER_PAR2_GET && Packet_Parameter23 == PC_TOWER_NUMBER_PAR2_GET)
    {
      // Transmit the Tower Number to the PC
      (void) Packet_Put(TOWER_NUMBER, TOWER_NUMBER_PAR1, NvTowerNb->s.Lo, NvTowerNb->s.Hi);;
      return true;
    }

  //Verify that the received command was for setting  tower number
  else if (Packet_Parameter1 == PC_TOWER_NUMBER_PAR1_SET)
    {
     //Write the tower number we set
      return Flash_Write16((uint16_t *) NvTowerNb, Packet_Parameter23);
    }

  // Invalid packet, likely called get with non zeroed parameter 2/3
  return false;
}


/*! @brief Handles the "Tower Number" packet
 *
 * Command: 0x0D
 * Parameter 1:  1 = get Tower mode
 *               2 = set Tower mode
 * Parameter 2: LSB for set, 0 for a get
 * Parameter 3: MSB for set, 0 for a get
 *
 * Response: None
 *
 * @return bool - TRUE if the packet was successfully handled.
 */
static bool HandleTowerMode(void)
{
  //Verify that the received command was for "Tower Mode"
  if (Packet_Parameter1 == PC_TOWER_MODE_PAR1_GET && Packet_Parameter2 == PC_TOWER_MODE_PAR2_GET&& Packet_Parameter3 == PC_TOWER_MODE_PAR3_GET)
    {
      (void) Packet_Put(TOWER_MODE, TOWER_MODE_PAR1, NvTowerMode->s.Lo, NvTowerMode->s.Hi);;
      return true;
    }
 //Verify that the received command was for setting Tower Mode
  else if (Packet_Parameter1 == PC_TOWER_MODE_PAR1_SET)
    {
     //Write the tower mode we set
      return Flash_Write16((uint16_t *)NvTowerMode, Packet_Parameter23);
    }

  return false;
}


/*! @brief Initialises the Packet, Flash and LEDs modules.
 *
 * @return bool - true if all modules were successfully initialised
 */

static bool InitializeComponents (void)
{
  return Packet_Init(baudRate, moduleClk)
         & Flash_Init()
         & LEDs_Init();
}



/*! @brief Allocates a block of Flash Memory and sets a default if the block is empty
 *
 *  Maps an addressPtr to a block of FLASH memory of the given size.
 *  If the block of memory is empty a default is set.
 *
 *  @param addressPtr Pointer to the address to be mapped to memory
 *  @param dataIfEmpty The data that should be set to the block if empty
 */
static void AllocateAndSet(volatile uint16union_t ** const addressPtr, uint16_t const dataIfEmpty)
{
  // Allocate block in memory for the passed in size
  bool allocatedAddress = Flash_AllocateVar((volatile void **)addressPtr, sizeof(**addressPtr));

  if (allocatedAddress && (*addressPtr)->l == 0xFFFF)
    {
      // Memory "empty", set default
      Flash_Write16((uint16 *)*addressPtr, dataIfEmpty);
    }
}

/*! @brief The main entry point into the program
 *
 *  @return int - Hopefully never (embedded software never ends!)
 */

void LPTMRInit(const uint16_t count)
{
  // Enable clock gate to LPTMR module
  SIM_SCGC5 |= SIM_SCGC5_LPTIMER_MASK;

  // Disable the LPTMR while we set up
  // This also clears the CSR[TCF] bit which indicates a pending interrupt
  LPTMR0_CSR &= ~LPTMR_CSR_TEN_MASK;

  // Enable LPTMR interrupts
  LPTMR0_CSR |= LPTMR_CSR_TIE_MASK;
  // Reset the LPTMR free running counter whenever the 'counter' equals 'compare'
  LPTMR0_CSR &= ~LPTMR_CSR_TFC_MASK;
  // Set the LPTMR as a timer rather than a counter
  LPTMR0_CSR &= ~LPTMR_CSR_TMS_MASK;

  // Bypass the prescaler
  LPTMR0_PSR |= LPTMR_PSR_PBYP_MASK;
  // Select the prescaler clock source
  LPTMR0_PSR = (LPTMR0_PSR & ~LPTMR_PSR_PCS(0x3)) | LPTMR_PSR_PCS(1);

  // Set compare value
  LPTMR0_CMR = LPTMR_CMR_COMPARE(count);

  // Initialize NVIC
  // see p. 91 of K70P256M150SF3RM.pdf
  // Vector 0x65=101, IRQ=85
  // NVIC non-IPR=2 IPR=21
  // Clear any pending interrupts on LPTMR
  NVICICPR2 = NVIC_ICPR_CLRPEND(1 << 21);
  // Enable interrupts from LPTMR module
  NVICISER2 = NVIC_ISER_SETENA(1 << 21);

  //Turn on LPTMR and start counting
  LPTMR0_CSR |= LPTMR_CSR_TEN_MASK;
}

void __attribute__ ((interrupt)) LPTimer_ISR(void)
{
  // Clear interrupt flag
  LPTMR0_CSR |= LPTMR_CSR_TCF_MASK;

  // Signal the analog channels to take a sample
  for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
    (void)OS_SemaphoreSignal(AnalogThreadData[analogNb].semaphore);
}

/*! @brief Initialises modules.
 *
 */
static void InitModulesThread(void* pData)
{
  OS_DisableInterrupts();
  // Analog
  (void)Analog_Init(CPU_BUS_CLK_HZ);
  // Generate the global analog semaphores
  for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
    AnalogThreadData[analogNb].semaphore = OS_SemaphoreCreate(0);

  // Initialse the components on the Tower Board (UART, Flash, LEDs etc.)
    if (InitializeComponents())
      {
        // If Tower Board is successful in starting up (all peripherals initialised), then the orange LED should be turned on.
        LEDs_On(LED_ORANGE);
      }

    // Allocate flash memory for Tower Mode and Number, and set defaults if empty
      AllocateAndSet(&NvTowerMode, 1); // default to 1 as per spec
      AllocateAndSet(&NvTowerNb, 9508); // default to last 4 digits of student number as per spec

  // Initialise the low power timer to tick every 10 ms
  LPTMRInit(10);

  // We only do this once - therefore delete this thread
  OS_EnableInterrupts();
  OS_SemaphoreSignal(PacketSemaphore);
  HandleStartup();

  OS_ThreadDelete(OS_PRIORITY_SELF);
}
 void PacketThread(void* data)
{
  OS_SemaphoreWait(PacketSemaphore, 0);
  for (;;)
  {
    if (Packet_Get()) // Check for received packets from PC
     HandlePacket(); // Handle received packet
  }
}

/*! @brief Samples a value on an ADC channel and sends it to the corresponding DAC channel.
 *
 */
void AnalogLoopbackThread(void* pData)
{
  // Make the code easier to read by giving a name to the typecast'ed pointer
  #define analogData ((TAnalogThreadData*)pData)

  for (;;)
  {
    int16_t analogInputValue;

    (void)OS_SemaphoreWait(analogData->semaphore, 0);
    // Get analog sample
    Analog_Get(analogData->channelNb, &analogInputValue);
    // Put analog sample
    Analog_Put(analogData->channelNb, analogInputValue);
  }
}

/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
  /* Write your local variable definition here */

  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  PE_low_level_init();
  /*** End of Processor Expert internal initialization.                    ***/
  OS_ERROR error;
  // Initialise low-level clocks etc using Processor Expert code
  PE_low_level_init();

  // Initialize the RTOS
  OS_Init(CPU_CORE_CLK_HZ, false);

  // Create module initialisation thread
  error = OS_ThreadCreate(InitModulesThread,
                          NULL,
                          &InitModulesThreadStack[THREAD_STACK_SIZE - 1],
                          0); // Highest priority
  error = OS_ThreadCreate(ReceiveThread, // 2nd highest priority thread
                          NULL,
                          &ReceiveThreadStack[THREAD_STACK_SIZE - 1],
                          1);

  error = OS_ThreadCreate(TransmitThread, // 3rd highest priority thread
                          NULL,
                          &TransmitThreadStack[THREAD_STACK_SIZE - 1],
                          2);

  // Create threads for 2 analog loopback channels
  for (uint8_t threadNb = 0; threadNb < NB_ANALOG_CHANNELS; threadNb++)
  {
    error = OS_ThreadCreate(AnalogLoopbackThread,
                            &AnalogThreadData[threadNb],
                            &AnalogThreadStacks[threadNb][THREAD_STACK_SIZE - 1],
                            ANALOG_THREAD_PRIORITIES[threadNb]);

  }

  error = OS_ThreadCreate(PacketThread,
                          NULL,
                          &PacketThreadStack[THREAD_STACK_SIZE - 1],
                          5); // Highest priority


  PacketSemaphore = OS_SemaphoreCreate(0);
  // Start multithreading - never returns!
  OS_Start();
}

/*!
 ** @}
 */
