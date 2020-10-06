/****************************************************************************
 Module
  SPUD_Query.c

 Revision
   1

 Description
   The following module holds functions associated with transmitting to
   the SPUD and reading the transmissions.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
02/18/2020      gab      initial release
****************************************************************************/

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

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_DeferRecall.h"

//other header files
#include "SPUD_Query.h"
#include "SPUDHSM.h"

/*----------------------------- Module Defines ----------------------------*/

//SPI initialization
#define CPSDVSR 14  //Clock preescaler 14
#define SCR 222     // Serial clock rate 222

#define IDLE_CMD 0xff   //invalid command
#define SPI_POLL_TIME 2 //2ms between byte transmissions
#define TotalBytes 3    //number total of bytes to send
//#define DEBUG

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine. They should be functions
   relevant to the behavior of this state machine
*/
static void SPUDSPI_Init(void);
static void SPUD_StartTx(uint8_t CMD);

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match that of enum in header file
static SPUDQueryState_t CurrentState;

static volatile uint8_t DataQueue[TotalBytes];

/*------------------------------ Module Code ------------------------------*/

/****************************************************************************
 Function
     StartSPUDQuerySM

 Parameters
     ES_Event CurrentEvent

 Returns
     None

 Description
     Does any required initialization for this state machine
 Notes

 Author
     G. Bravo.
****************************************************************************/
void StartSPUDQuerySM(ES_Event_t CurrentEvent)
{
  //Configure registers for SPI communication
  SPUDSPI_Init();
  CurrentState = Waiting4Query;   // always start in waiting for query
  // call the entry function (if any) for the ENTRY_STATE
  // there is no entry function for Waiting4Query
}

/****************************************************************************
 Function
    RunSPUDQuerySM

 Parameters
   ES_Event CurrentEvent: the event to process

 Returns
   ES_Event

 Description
  Implement a state machine that communicates a command to the SPUD
 Notes
   uses nested switch/case to implement the machine.
 Author
  G. Bravo/
****************************************************************************/
ES_Event_t RunSPUDQuerySM(ES_Event_t CurrentEvent)
{
  /* are we making a state transition? */
  bool              MakeTransition = false;
  SPUDQueryState_t  NextState = CurrentState;
  // assume we are not consuming event
  ES_Event_t        ReturnEvent = CurrentEvent; 

  switch (CurrentState)
  {
    case Waiting4Query:
    {
      // No During function for this state
      //process any events
      if (CurrentEvent.EventType != ES_NO_EVENT) //If an event is active
      {
        switch (CurrentEvent.EventType)
        {
          case NEW_SPUD_QUERY:       //New query request
          {
            //Get command to send from parameter
            uint8_t Command = ReturnEvent.EventParam;

            //Start transmission with that command
            SPUD_StartTx(Command);
			//Set next state to SendingBytes
            NextState = SendingBytes;   
            //mark that we are taking a transition
			MakeTransition = true;      
            // consume this event
            ReturnEvent.EventType = ES_NO_EVENT;
          }
          break;
        }
      }
    }
    break;

    case SendingBytes:
    {
      // No During function for this state
      //process any events
      if (CurrentEvent.EventType != ES_NO_EVENT) //If an event is active
      {
        switch (CurrentEvent.EventType)
        {
          case SPUD_EOT: //End of transmission
          {
            // Read data from DR and populate data queue
            uint8_t index = 0; //for Rx FIFO reading loops
            for (index = 0; index < TotalBytes; index++)
            {
              DataQueue[index] = HWREG(SSI0_BASE + SSI_O_DR);
            }

            // Set next state to Wait2SendNextByte 
			NextState = Wait2SendNextByte;  
             //mark that we are taking a transition
			MakeTransition = true;         
            // consume this event
            ReturnEvent.EventType = ES_NO_EVENT;

            //start timer
            ES_Timer_InitTimer(SPUDQuery_TIMER, SPI_POLL_TIME);
          }
          break;
        }
      }
    }
    break;

    case Wait2SendNextByte:
    {
      // Execute During function for counting state. ES_ENTRY & ES_EXIT are
      //  No During function for this state
      //process any events
      if (CurrentEvent.EventType != ES_NO_EVENT)  //If an event is active
      {
        switch (CurrentEvent.EventType)
        {
          case ES_TIMEOUT:    //Timer Expired
          {
             //if time out is from SPUDQuery_TIMER
			if (CurrentEvent.EventParam == SPUDQuery_TIMER)    
            {
              //Set the next to Waiting4Query
			  NextState = Waiting4Query;  
              //mark that we are taking a transition
			  MakeTransition = true;      
              // consume this event
              ReturnEvent.EventType = ES_NO_EVENT;

              //post the result of the query
              ES_Event_t NewEvent;
              NewEvent.EventType = SPUD_ANSWER;

              NewEvent.EventParam = DataQueue[TotalBytes-1];
            }
          }
          break;
        }
      }
    }
    break;
  }
  //   If we are making a state transition
  if (MakeTransition == true)
  {
    //   Execute exit function for current state
    //no exit function

    CurrentState = NextState;    //Modify state variable

    //   Execute entry function for new state
    //no entry function
  }

//  ReturnEvent.EventType = ES_NO_EVENT;   //delete
  return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/

/****************************************************************************
Function
   SPUDSPI_Init

Parameters
  None

Returns
  None

Description
  Sets up SSI0 to be a freescale SPI port with clock ~13.8kHz and
  to generate interrupt on EOT
Notes

Author
  G.Bravo
****************************************************************************/
static void SPUDSPI_Init(void)
{
  // Start SSI0 clock
  HWREG(SYSCTL_RCGCSSI) |= SYSCTL_RCGCSSI_R0;
  // Wait for SSI0 to be ready
  while ((HWREG(SYSCTL_PRSSI) & SYSCTL_PRSSI_R0) != SYSCTL_PRSSI_R0)
  {}

  // Start PORTA clock
  HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R0;
  // Wait for PORTA clock
  while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R0) != SYSCTL_PRGPIO_R0)
  {}
  // Program the GPIO to use the alternate function on pins PA2-PA5
  HWREG(GPIO_PORTA_BASE + GPIO_O_AFSEL) |= (BIT2HI | BIT3HI | BIT4HI | BIT5HI);

  // Set mux position in GPIO_O_PCTL to select the SSI use of the pins
  HWREG(GPIO_PORTA_BASE + GPIO_O_PCTL) =
      (HWREG(GPIO_PORTA_BASE + GPIO_O_PCTL) & 0xff0000ff) |
      ((2 << 8) + (2 << 12) + (2 << 16) + (2 << 20));
  // Set PA2-PA5 has digital I/O
  HWREG(GPIO_PORTA_BASE + GPIO_O_DEN) |= (BIT2HI | BIT3HI | BIT4HI | BIT5HI);
  // Set PA2, PA3, PA5 as outputs
  HWREG(GPIO_PORTA_BASE + GPIO_O_DIR) |= (BIT2HI | BIT3HI | BIT5HI);

  // Set PA4 as input
  HWREG(GPIO_PORTA_BASE + GPIO_O_DIR) &= BIT4LO;
  // Enable pull-up on PA2 (clock idles high)
  HWREG(GPIO_PORTA_BASE + GPIO_O_PUR) |= BIT2HI;

  // Disable SSI0
  HWREG(SSI0_BASE + SSI_O_CR1) &= ~SSI_CR1_SSE;
  // Select master mode
  HWREG(SSI0_BASE + SSI_O_CR1) &= ~SSI_CR1_MS;
  //Choose System clock as clock source
  HWREG(SSI0_BASE + SSI_O_CC) = SSI_CC_CS_SYSPLL;
  // Configure clock prescaler
  HWREG(SSI0_BASE + SSI_O_CPSR) = CPSDVSR;

  // Configure CR0 for Freescale SPI
  HWREG(SSI0_BASE + SSI_O_CR0) &= ~SSI_CR0_FRF_M;

  // set clock rate SCR
  HWREG(SSI0_BASE + SSI_O_CR0) = (HWREG(SSI0_BASE + SSI_O_CR0) & ~SSI_CR0_SCR_M)
      | (SCR << SSI_CR0_SCR_S);
  //set SPO = 1, SPH = 1
  HWREG(SSI0_BASE + SSI_O_CR0) |= (SSI_CR0_SPH | SSI_CR0_SPO);
  // set data size to 8bits,
  HWREG(SSI0_BASE + SSI_O_CR0) =
      (HWREG(SSI0_BASE + SSI_O_CR0) & ~SSI_CR0_DSS_M) | SSI_CR0_DSS_8;

  // Locally enable EOT interrupt mask
  HWREG(SSI0_BASE + SSI_O_CR1) |= SSI_CR1_EOT;
  // Locally enable TXIM in SSIIM
  HWREG(SSI0_BASE + SSI_O_IM) |= (SSI_IM_TXIM);
  // Enable SSI0 module
  HWREG(SSI0_BASE + SSI_O_CR1) |= SSI_CR1_SSE;

  // Set interrupt priority to 7
  HWREG(NVIC_PRI1) = (HWREG(NVIC_PRI1) & ~NVIC_PRI1_INT7_M)
    | (7u << NVIC_PRI1_INT7_S);
  // enable interrupts globally
  __enable_irq();
}

/****************************************************************************
 Function
    SPUD_StartTx

 Parameters
   None

 Returns
   None

 Description
   Send Command to SPUD
 Notes

 Author
   G.Bravo
****************************************************************************/
static void SPUD_StartTx(uint8_t CMD)
{
  // Load FIFO first with the desired command
  HWREG(SSI0_BASE + SSI_O_DR) = CMD;
  // Init loop count
  uint8_t lpcnt = 0;
  // Load FIFO with pumps (0x00)
  for (lpcnt = 0; lpcnt < (TotalBytes - 1); lpcnt++)
  {
    HWREG(SSI0_BASE + SSI_O_DR) = 0x00;
  }

  // Enable EOT interrupt, Set NVIC_EN0 bit 7 high
  HWREG(NVIC_EN0) = BIT7HI;
}

/***************************************************************************
 Library ISR functions ADD NAMES TO STARTUP FILE!!!
 ***************************************************************************/

/****************************************************************************
  Function
     SPUD_EOTISR

  Parameters
    None

  Returns
    None

  Description
    End of transmission interrupt. Will post that transmission ended

  Author
    G.Bravo
 ****************************************************************************/

void SPUD_EOTISR(void)
{
  static ES_Event_t SPIEvent;     //static for speed
  // Post EOT to self
  SPIEvent.EventType = SPUD_EOT;

  //Post to higher level state machine
  PostSPUDHSM(SPIEvent);

  //Disable interrupt
  HWREG(NVIC_DIS0) = BIT7HI;
}

/*------------------------------- Footnotes -------------------------------*/
/*
1. add timer: SPUDQuery_TIMER
2. Add interrupt to start file
*/
/*------------------------------ End of file ------------------------------*/
