/****************************************************************************
 Module
   AccelerometerFSM.c

 Revision
   1.0.1

 Description
   State machine that comunicates constantly with the accelerometer

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 02/13/20 22:30 ram		Updated template for use with accelerometer servicing
****************************************************************************/
#pragma diag_suppress 550 // stop showing warning that purge isn't used

/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

//Header files for hw access
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ssi.h"
#include "inc/hw_nvic.h"

#include "gpio.h"
#include "EncoderLib.h"

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_DeferRecall.h"
#include "TestHarnessService0.h"
#include "AccelerometerFSM.h"
#include "BumperFSM.h"

/*----------------------------- Module Defines ----------------------------*/
#define SCR_VAL 39 // 500kHz clock (back off if cables prove WAY too long)
#define INIT_CMD_NUM 6
#define CMD_BYTE_NUM 2
#define QUERY_BYTE_NUM 7
#define SAMPLE_NUM 16

// Intilization definitions
#define BW_RATE 0x2C
#define RATE 0x0D //sets to 800Hz
#define DATA_FORMAT 0x31
#define FORMAT 0x00 // sets to 4wire SPI right justified
#define TAP_REG 0x1D
#define TAP_VAL 0x04 //tap threshold of 1/2g
#define DUR_REG 0x21
#define DUR_VAL 0xFF // tap time of 10ms
#define MEAS_REG 0x2D
#define MEASURE BIT3HI // Sets to measure mode

#define X0_REG 0xF2     // multibyte read starting at x0
#define PUMP_BYTE 0xFF  // multibyte pump to generate clock

// Indices of the bytes in the data queue
#define X0_INDEX 1
#define X1_INDEX 2
#define Y0_INDEX 3
#define Y1_INDEX 4

// Vector calculation definitions
#define BYTE_S 8
#define SIGN_EXT 0xFC00 //to sign extend 10 bit value
#define X_COMP 0
#define Y_COMP 1

#define DATA_PIN "PE5"
#define TAP_THRESH_HI 100
#define TAP_THRESH_LO 50

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/
static void AcclSPI_HWInit(void);
static void StartInitTx(uint8_t CMDindex);
static void StartQueryTx(void);
static void GetAcclVector(void);
static int16_t Register2Value(uint8_t HiByte, uint8_t LoByte);

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
static AccelerometerState_t CurrentState;

// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;

// add a deferral queue for up to 3 pending deferrals +1 to allow for overhead
static ES_Event_t     DeferralQueue[3 + 1];

static const uint8_t  InitCmds[INIT_CMD_NUM][CMD_BYTE_NUM] =
{
  { BW_RATE, RATE },        // sets up 200Hz sampling
  { DATA_FORMAT, FORMAT },  // makes sure data is correctly formated
  // yea it's the defualt, but better safe than sorry!
  { 0x2A, 0x0A },         // set up single tap on y-axis
  { 0x2F, 0xEF },         // Set data_ready to interrupt 1
  { MEAS_REG, MEASURE },  // put meter into measure mode
  { 0x2E, BIT7HI } // Enable data_ready interrupt
};

static uint8_t  DataQueue[QUERY_BYTE_NUM];

static int16_t  a_VecOffset[2] = { 12, 1 };
static int16_t  a_Vec[2] = { 0, 0 };

static int16_t  SampleQueue[SAMPLE_NUM][2];

static uint8_t  SampleCounter = 0;

static uint8_t  LastDataVal;
static bool     TapDetect = false;
static bool     TapActive = false;
/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitAccelerometerFSM

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this state machine
 Notes

 Author
     J. Edward Carryer, 10/23/11, 18:55
****************************************************************************/
bool InitAccelerometerFSM(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  // initialize hardware
  AcclSPI_HWInit();
  // put us into the Initial PseudoState
  CurrentState = InitAcclState;
  // post the initial transition event

  ThisEvent.EventType = ES_INIT;
  if (ES_PostToService(MyPriority, ThisEvent) == true)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/****************************************************************************
 Function
     PostAccelerometerFSM

 Parameters
     EF_Event_t ThisEvent , the event to post to the queue

 Returns
     boolean False if the Enqueue operation failed, True otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostAccelerometerFSM(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunAccelerometerFSM

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event_t, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   Runs state machine for accelerometer querying
 Notes
   uses nested switch/case to implement the machine.
 Author
   R. Merchant
****************************************************************************/
ES_Event_t RunAccelerometerFSM(ES_Event_t ThisEvent)
{
  ES_Event_t      ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  static uint8_t  InitCnt = 0;

  switch (CurrentState)
  {
    case InitAcclState:        // If current state is initial Psedudo State
    {
      if (ThisEvent.EventType == ES_INIT)    // only respond to ES_Init
      {
        // Setup deferral queue
        ES_InitDeferralQueueWith(DeferralQueue, ARRAY_SIZE(DeferralQueue));
        // Start init transmission
        StartInitTx(InitCnt);
        //increment initialization counter
        InitCnt++;
        // change state to intializing meter
        CurrentState = InitializingMeter;
      }
    }
    break;

    case InitializingMeter:      // If current state IntializingMeter
    {
      if (ThisEvent.EventType == EI_ACCL_EOT)
      {
        //Purge Rx FIFO with dummy read
        uint8_t purge;
        uint8_t index = 0; //for Rx FIFO reading loops
        for (index = 0; index < CMD_BYTE_NUM; index++)
        {
          purge = HWREG(SSI3_BASE + SSI_O_DR);
        }
        // If intialization sequence is over
        if (InitCnt >= INIT_CMD_NUM)
        {
          // Start a query transmission
          StartQueryTx();
          // Set state to wating for action.
          CurrentState = SendingQueryBytes;
        }
        else
        {
          // Send the next init command
          StartInitTx(InitCnt);
          //Increment the init counter
          InitCnt++;
        }
      }
    }
    break;
    // If current state is sending bytes
    case SendingQueryBytes:
    {
      switch (ThisEvent.EventType)
      {
        case EI_ACCL_EOT:  //If event is EOT
        {
          // Read data from DR and populate data queue
          uint8_t index = 0; //for Rx FIFO reading loops
          for (index = 0; index < QUERY_BYTE_NUM; index++)
          {
            DataQueue[index] = HWREG(SSI3_BASE + SSI_O_DR);
          }

          // Parse and get acceleration vector to store in sample queue
          GetAcclVector();
          // Recall any deferred events
          ES_RecallEvents(MyPriority, DeferralQueue);
          // restart timer
          ES_Timer_InitTimer(ACCL_TIMER, 5);
          // Set state to waiting for action
          CurrentState = Waiting4Action;  //Decide what the next state will be
        }
        break;

        case EV_QUERY_ACCL:  //If event is query request

        {
          // Defer the event
          ES_DeferEvent(DeferralQueue, ThisEvent);
        }
        break;

        // repeat cases as required for relevant events
        default:
          ;
      }  // end switch on CurrentEvent
    }
    break;
    case Waiting4Action:      // If current state is Waiting4Action
    {
      if ((ThisEvent.EventType == EV_QUERY_ACCL) || (ThisEvent.EventType == ES_TIMEOUT))   //If event is event one
      {
        // Start a query transmission
        StartQueryTx();
        // Set state to sending bytes
        CurrentState = SendingQueryBytes;
      }  // end switch on CurrentEvent
    }
    break;
    // repeat state pattern as required for other states
    default:
      ;
  }                                   // end switch on Current State
  return ReturnEvent;
}

/****************************************************************************
 Function
     QueryAccelerometerSM

 Parameters
     None

 Returns
     TemplateState_t The current state of the state machine

 Description
     returns the current state of the  state machine
 Notes

 Author
     R. Merchant
****************************************************************************/
AccelerometerState_t QueryAccelerometerSM(void)
{
  return CurrentState;
}

/****************************************************************************
Function
   AcclFSM_QueryVector

Parameters
  pointer to vector

Returns
  Nothing

Description
  Takes the average of the sample queue and puts the compoenents into the
  passed in array.

Notes

Author
  R. Merchant
****************************************************************************/
void AcclFSM_QueryVector(int16_t *pVector)
{
  // Average each componenet
  // declare placeholder for the sums
  int16_t sumX = 0, sumY = 0;
  uint8_t i = 0;

  // Sum all the elements
  for (i = 0; i < SAMPLE_NUM; i++)
  {
    //Add the elements up
    sumX += SampleQueue[i][X_COMP];
    sumY += SampleQueue[i][Y_COMP];
  }
  // Place x-component average into index 0
  *pVector = sumX / SAMPLE_NUM;
  // Place y-component average into index 1
  *(pVector + 1) = sumY / SAMPLE_NUM;
}

/****************************************************************************
Function
   AcclFSM_EnableTap/AcclFSM_DisableTap

Parameters
  None

Returns
  Nothing

Description
  Eanbales/disables tap event checker. Prevent spurious events when we run
 into the miner.

Notes

Author
  R. Merchant
****************************************************************************/
void AcclFSM_EnableTap(void)
{
  // Set tap flag high
  TapDetect = true;
}

void AcclFSM_DisableTap(void)
{
  // Set tap  flag low
  TapDetect = false;
}

/***************************************************************************
 Service ISR functions ADD NAMES TO STARTUP FILE!!!
 ***************************************************************************/

void AcclFSM_EOTISR(void)
{
  static ES_Event_t EOTEvent = { EI_ACCL_EOT, 0 }; //static for speed
  // Disable NVIC
  HWREG(NVIC_DIS1) = BIT26HI;

  // Post ACCL_EOT
  PostAccelerometerFSM(EOTEvent);
}

/***************************************************************************
 Service event checker(s). Add to wrapper
 ***************************************************************************/
/****************************************************************************
Function
   Check4Data

Parameters
  None

Returns
  Event flag

Description
  Checks if the INT1 pin from the accelerometer has produced a rising edge.
  This signals there is data ready to be read from the meter.

Notes

Author
  R. Merchant
****************************************************************************/
bool Check4Data(void)
{
  bool    ReturnVal = false;
  // Read the data pin
  uint8_t CurDataVal = GPIO_ReadPin(DATA_PIN);
  // If there is a rising edge
  if ((CurDataVal != LastDataVal) && (CurDataVal == 1))
  {
    // Generate and post the query event
    ES_Event_t DataEvent = { EV_QUERY_ACCL, 0 };
    PostAccelerometerFSM(DataEvent);
    // Set return value to true
    ReturnVal = true;
  }
  //endif
  // Update last value to match current
  LastDataVal = CurDataVal;
  return ReturnVal;
}

/****************************************************************************
Function
   Check4Tap

Parameters
  None

Returns
  Event flag

Description
  Checks if there is an acceleration spike. Reads the value of the y-comp to
  determine where the tap came from. Only runs if tap detection is enabled.
  Also checks to see when the accleration has dropped below the lower
  threshold.

Notes

Author
  R. Merchant
****************************************************************************/
bool Check4Tap(void)
{
  bool ReturnVal = false;
  // If looking for tap events
  if (TapDetect)
  {
    // take snapshot of y-comp to save vector for later calculations
    int16_t   tapCompy = a_Vec[Y_COMP];
    // Calculate the magnitude of the acceleration
    uint16_t  aMag = abs(a_Vec[1]);
    // If the value is >= to the upper threshold and there is no tap active
    if ((aMag > TAP_THRESH_HI) && (!TapActive))
    {
      // Set tap activity true
      TapActive = true;

      // If y-comp < 0
      if (tapCompy < 0)
      {
        // post front down to debounce service
        ES_Event_t TapEvent = { EV_BUMPERPRESS_F, 0 };
        PostBumperFSM(TapEvent);
      }
      // else if y-comp > 0
      else if (tapCompy > 0)
      {
        // post rear down to debounce service
        ES_Event_t TapEvent = { EV_BUMPERPRESS_R, 0 };
        PostBumperFSM(TapEvent);
      }
      //endif
      // set return val to true
      ReturnVal = true;
    }
    // else if the value is <= lower threshold and tap is active
    else if ((aMag < TAP_THRESH_LO) && TapActive)
    {
      // Set tap inactive
      TapActive = false;
    }
    // endif
  }
  //endif
  return ReturnVal;
}

/***************************************************************************
 private functions
 ***************************************************************************/
/****************************************************************************
 Function
    AcclSPI_HWInit

 Parameters
   None

 Returns
   None

 Description
   Sets up SSI3 to be a freescale SPI port with clock 500kHz (/80) and
   to generate interrupt on EOT. This will communicate with the accelerometer.

 Notes

 Author
   R. Merchant
****************************************************************************/
static void AcclSPI_HWInit(void)
{
  // Start SSI3 clock
  HWREG(SYSCTL_RCGCSSI) |= SYSCTL_RCGCSSI_R3;
  // Start PORTD clock
  HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R4;
  // Wait for PORTD clock
  while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R4) != SYSCTL_PRGPIO_R4)
  {}

  // Set AFSEL high for PD0-PD3
  HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) |= (BIT0HI | BIT1HI | BIT2HI | BIT3HI);
  // Set PTCL to 0x00001111
  HWREG(GPIO_PORTD_BASE + GPIO_O_PCTL) =
      (HWREG(GPIO_PORTD_BASE + GPIO_O_PCTL) & 0xffff0000) | (0x1111);
  // Set PD0-PD3 has digital I/O
  HWREG(GPIO_PORTD_BASE + GPIO_O_DEN) |= (BIT0HI | BIT1HI | BIT2HI | BIT3HI);
  // Set PD0(clk), PD1(Fss), PD3(Tx) as outputs
  HWREG(GPIO_PORTD_BASE + GPIO_O_DIR) |= (BIT0HI | BIT1HI | BIT3HI);
  // Set PD3 (Rx) as input
  HWREG(GPIO_PORTD_BASE + GPIO_O_DIR) &= BIT3LO;
  // Enable pull-up on clock
  HWREG(GPIO_PORTD_BASE + GPIO_O_PUR) |= BIT0HI;

  // Setup data pin to be an input
  GPIO_ConfigPin(DATA_PIN, Input);
  // get the intial state of the pin
  LastDataVal = GPIO_ReadPin(DATA_PIN);

  // Wait for SSI3 to be ready
  while ((HWREG(SYSCTL_PRSSI) & SYSCTL_PRSSI_R3) != SYSCTL_PRSSI_R3)
  {}
  // Disable SSI3
  HWREG(SSI3_BASE + SSI_O_CR1) &= ~SSI_CR1_SSE;
  // Select master mode
  HWREG(SSI3_BASE + SSI_O_CR1) &= ~SSI_CR1_MS;
  // Configure clock prescaler = 2
  HWREG(SSI3_BASE + SSI_O_CC) = SSI_CC_CS_SYSPLL;
  HWREG(SSI3_BASE + SSI_O_CPSR) = 2;

  // Configure CR0 for Freescale SPI
  // set SCR
  HWREG(SSI3_BASE + SSI_O_CR0) = (HWREG(SSI3_BASE + SSI_O_CR0) & ~SSI_CR0_SCR_M)
      | (SCR_VAL << SSI_CR0_SCR_S);
  // set mode to SPI
  HWREG(SSI3_BASE + SSI_O_CR0) =
      (HWREG(SSI3_BASE + SSI_O_CR0) & ~SSI_CR0_FRF_M) | SSI_CR0_FRF_MOTO;
  //set SPO = 1, SPH = 1
  HWREG(SSI3_BASE + SSI_O_CR0) |= (SSI_CR0_SPH | SSI_CR0_SPO);
  // set data size to 8bits,
  HWREG(SSI3_BASE + SSI_O_CR0) =
      (HWREG(SSI3_BASE + SSI_O_CR0) & ~SSI_CR0_DSS_M) | SSI_CR0_DSS_8;

  // Locally enable EOT interrupt mask
  HWREG(SSI3_BASE + SSI_O_CR1) |= SSI_CR1_EOT;
  // Locally enable TXIM in SSIIM
  HWREG(SSI3_BASE + SSI_O_IM) |= (SSI_IM_TXIM);
  // Enable SSI3 module
  HWREG(SSI3_BASE + SSI_O_CR1) |= SSI_CR1_SSE;
  // Lower priority for EOT interrupt
  HWREG(NVIC_PRI14) = (HWREG(NVIC_PRI14) & ~NVIC_PRI14_INTC_M) |
      (5 << NVIC_PRI14_INTC_S);
  // enable interrupts globally
  __enable_irq();

  return;
}

/****************************************************************************
Function
   StartInitTx

Parameters
  CMDindex

Returns
  None

Description
  Sends the needed intilization information to the accelerometer based on
  the index provided in the argument.

Notes

Author
  R. Merchant
****************************************************************************/
static void StartInitTx(uint8_t CMDindex)
{
  uint8_t lpcnt = 0;
  // Load FIFO with needed intialization instructions
  for (lpcnt = 0; lpcnt < CMD_BYTE_NUM; lpcnt++)
  {
    HWREG(SSI3_BASE + SSI_O_DR) = InitCmds[CMDindex][lpcnt];
  }
  // Enable the NVIC
  HWREG(NVIC_EN1) = BIT26HI;
}

/****************************************************************************
Function
   StartQueryTx

Parameters
  None

Returns
  Nothing

Description
  Sends the needed bytes to query x0 - y1 in a multi-byte read. Uses pumping
  bytes (0x00) to continue getting data from accelerometer.

Notes

Author
  R. Merchant
****************************************************************************/
static void StartQueryTx(void)
{
  // Init loop count
  uint8_t lpcnt = 0;
  // Load FIFO first with the start register
  HWREG(SSI3_BASE + SSI_O_DR) = X0_REG;
  // Load FIFO with pumps
  for (lpcnt = 0; lpcnt < (QUERY_BYTE_NUM - 1); lpcnt++)
  {
    HWREG(SSI3_BASE + SSI_O_DR) = PUMP_BYTE;
  }
  // Enable NVIC
  HWREG(NVIC_EN1) = BIT26HI;
}

/****************************************************************************
Function
   GetAcclVector

Parameters
  None

Returns
  Nothing

Description
  Converts the data in the data buffer into an acceleration vector that
  can be used to determine heading

Notes

Author
  R. Merchant
****************************************************************************/
static void GetAcclVector(void)
{
  // get each component value
  int16_t a_x = Register2Value(DataQueue[X1_INDEX], DataQueue[X0_INDEX]);
  int16_t a_y = Register2Value(DataQueue[Y1_INDEX], DataQueue[Y0_INDEX]);

  // Calculate queue position
  uint8_t index = SampleCounter % SAMPLE_NUM;

  // populate single vector, correcting for offset
  a_Vec[X_COMP] = a_x - a_VecOffset[X_COMP];
  a_Vec[Y_COMP] = a_y - a_VecOffset[Y_COMP];

  // populate sample queue, correcting for offset
  SampleQueue[index][X_COMP] = a_x - a_VecOffset[X_COMP];
  SampleQueue[index][Y_COMP] = a_y - a_VecOffset[Y_COMP];

  SampleCounter++;
}

/****************************************************************************
Function
   Register2Value

Parameters
  None

Returns
  Nothing

Description
  Converts the data in the pseudo X and Y registers into a signed integer.
  To be safe, data from the accelerometer is treated as a 10-bit 2s comp
  value, so this function converts the 10-bit signed to 16-bit signed type.

Notes

Author
  R. Merchant
****************************************************************************/
static int16_t Register2Value(uint8_t HiByte, uint8_t LoByte)
{
  // Concat bytes
  uint16_t  ConcatVal = (HiByte << BYTE_S) + LoByte;
  // Allocate for signed value
  int16_t   SignedVal;
  // Mask to 10-bit value
  ConcatVal &= 0x3FF;

  // if MSb is high, value is negative
  if (ConcatVal & BIT9HI)
  {
    //sign extend negative
    SignedVal = ConcatVal | SIGN_EXT;
  }
  // else, value is positive
  else
  {
    //sign extend positive
    SignedVal = ConcatVal & ~SIGN_EXT;
  }
  //endif

  // spit back
  return SignedVal;
}

/*------------------------------- Footnotes -------------------------------*/

/*------------------------------ End of file ------------------------------*/
