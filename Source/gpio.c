/****************************************************************************
 Module
   gpio.c

 Revision
   1.0.1

 Description
   The following module holds functions associated with setting the direction
   of GPIO pins setup for digital use. there are separate functions for
   reading and setting the value of a pin.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
01/27/2020 21:56 ram    initial release
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
// the common headers for C99 types
#include <stdint.h>
#include <stdbool.h>

// the headers to access the GPIO subsystem
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"

#include "gpio.h"

/*----------------------------- Module Defines ----------------------------*/
#define NIBBLE_0_M 0x0f
#define PORT_NUM 6

#define SET 0
#define CHK 1

typedef struct PinInfo
{
  bool Valid;
  uint8_t Port;
  uint32_t PortAddr;
  uint8_t Bit;
  uint32_t BitAddr;
}PinInfo_t;
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/
static void PortStringDecode(char *PinName, PinInfo_t *pPin);

/*---------------------------- Module Variables ---------------------------*/
static const uint32_t PortAddrBase[PORT_NUM] =
{
  GPIO_PORTA_BASE,
  GPIO_PORTB_BASE,
  GPIO_PORTC_BASE,
  GPIO_PORTD_BASE,
  GPIO_PORTE_BASE,
  GPIO_PORTF_BASE
};

static const uint32_t Clocks[PORT_NUM][2] =
{
  { SYSCTL_RCGCGPIO_R0, SYSCTL_PRGPIO_R0 },
  { SYSCTL_RCGCGPIO_R1, SYSCTL_PRGPIO_R1 },
  { SYSCTL_RCGCGPIO_R2, SYSCTL_PRGPIO_R2 },
  { SYSCTL_RCGCGPIO_R3, SYSCTL_PRGPIO_R3 },
  { SYSCTL_RCGCGPIO_R4, SYSCTL_PRGPIO_R4 },
  { SYSCTL_RCGCGPIO_R5, SYSCTL_PRGPIO_R5 }
};

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
    GPIO_ConfigPin

 Parameters
   Pin name,Direction

 Returns
   status flag

 Description
   Sets the pin in question as a digital input or ouput as specified by the
   direction argument.
 Notes

 Author
   R. Merchant
****************************************************************************/
bool GPIO_ConfigPin(char *PinName, PinState_t Dir)
{
  bool      ReturnFlag = false;
  PinInfo_t ThisPin;
  PortStringDecode(PinName, &ThisPin);
  //If pin value is valid
  if (ThisPin.Valid)
  {
    //If peripheral not ready
    if ((HWREG(SYSCTL_PRGPIO) & Clocks[ThisPin.Port][CHK]) == 0)
    {
      // Start Clock
      HWREG(SYSCTL_RCGCGPIO) |= Clocks[ThisPin.Port][SET];
      // Wait for clock to come up
      while ((HWREG(SYSCTL_PRGPIO) & Clocks[ThisPin.Port][CHK])
          != Clocks[ThisPin.Port][CHK])
      {}
    }
    // end if
    // Shift 1 over by value to get the right port bit
    uint32_t RegVal = 1 << ThisPin.Bit;
    // Set up digital IO
    HWREG(ThisPin.PortAddr + GPIO_O_DEN) |= RegVal;
    // if input, invert shifted value
    if (Dir == Input)
    {
      //Set Direction
      HWREG(ThisPin.PortAddr + GPIO_O_DIR) &= ~RegVal;
      ReturnFlag = true;
    }
    else if (Dir == Output)
    {
      //Set Direction
      HWREG(ThisPin.PortAddr + GPIO_O_DIR) |= RegVal;
      ReturnFlag = true;
    }
  }
  //endif

  // return flag
  return ReturnFlag;
}//end function

/****************************************************************************
 Function
    GPIO_SetPin

 Parameters
   Pin Name, State (e.g. "PA5", High)

 Returns
   status flag

 Description
   Sets the port bit in qurestion high or low.

 Notes
  This utilizes the atomic write addr space of the Cortex peripheral
  and does not rely on RMW of the all bits address space.

 Author
   R. Merchant
****************************************************************************/
bool GPIO_SetPin(char *PinName, PinState_t State)
{
  bool              ReturnFlag = false;
  static PinInfo_t  ThisPin; //static for speed
  PortStringDecode(PinName, &ThisPin);
  //If pin is value is valid
  if (ThisPin.Valid)
  {
    HWREG(ThisPin.PortAddr + ThisPin.BitAddr) = State;
    //set return flag true
    ReturnFlag = true;
  }
  //endif

  // return flag
  return ReturnFlag;
}//end function

/****************************************************************************
 Function
    GPIO_ReadPin

 Parameters
   Pin name

 Returns
   Pin value

 Description
   Reads the port bit in qurestion. If an invalid port name is given,
   a 0xFF is returned. If valid, returns 1 if pin is in the high state and
   0 if in the low state.

 Notes
  This utilizes the atomic read addr space of the Cortex peripheral

 Author
   R. Merchant
****************************************************************************/
uint8_t GPIO_ReadPin(char *PinName)
{
  uint8_t           BitValue = 0xFF;
  static PinInfo_t  ThisPin; //static for speed
  PortStringDecode(PinName, &ThisPin);
  //If pin is value is valid
  if (ThisPin.Valid)
  {
    BitValue = HWREG(ThisPin.PortAddr + ThisPin.BitAddr);
    //Shift read right by bit amount
    BitValue = BitValue >> ThisPin.Bit;
  }
  //endif

  // return bit value
  return BitValue;
}// end function

/***************************************************************************
 private functions
 ***************************************************************************/
/****************************************************************************
 Function
    PortStringDecode

 Parameters
   Pin Name, PinInfo structure pointer

 Returns
   None

 Description
   The following function will read the port name string and parse it to
   get the port index (0-5). If the port letter is valid, it will grab
   the base address from the array constant and right it to the structure
   passed to the function via a pointer. It then decodes the bit number
   and assigns the bit name to the bit memember if the bit value is valid.
   If all is valid, the valid member is set true.
 Notes

 Author
   R. Merchant
****************************************************************************/
static void PortStringDecode(char *PinName, PinInfo_t *pPin)
{
  pPin->Valid = false;
  static uint8_t Val; // speed!
  // Get the port in question (2nd char), mask char
  Val = (uint8_t)(*(PinName + 1));
  // Mask nibble and subtract 1
  Val = (Val & NIBBLE_0_M) - 1;
  //If valid port (0-5)
  if (Val < 6)
  {
    pPin->Port = Val;
    // select port base from said number
    pPin->PortAddr = PortAddrBase[Val];

    // Mask 3rd char into int
    // Mask char to int and grab nubble
    Val = (uint8_t)(*(PinName + 2));
    Val &= NIBBLE_0_M;
    if (Val < 8)
    {
      pPin->Bit = Val;
      // Shift 1 over by bitvalue+2 to get the right port bit address
      // Add data reg offset and store to member
      pPin->BitAddr = GPIO_O_DATA +  (1 << (Val + 2));
      //all passed? Set valid true!
      pPin->Valid = true;
    }
  }
}

/*------------------------------- Footnotes -------------------------------*/
/* module is intended for digital operation.
*/
/*------------------------------ End of file------------------------------*/
