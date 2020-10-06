/****************************************************************************
 Module
   EncoderLib.c

 Revision
   1.0.1

 Description
   The following module holds functions associated with reading the encoder
   pulses to determine motor speed and direction.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
01/28/2020      ram      initial release
****************************************************************************/
#pragma diag_suppress 177 //Since test operation is not always called.
/*----------------------------- Include Files -----------------------------*/
// the common headers for C99 types
#include <stdint.h>
#include <stdbool.h>

// the headers to access the GPIO subsystem
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"

//header for timer operations
#include "inc/hw_timer.h"
#include "inc/hw_nvic.h"

#include "ES_Port.h"
#include "BITDEFS.H"
#include "gpio.h"
#include "EncoderLib.h"
#include "termio.h"
/*----------------------------- Module Defines ----------------------------*/
#define NUM_ENCS 2
#define PPR 3
#define ENC_PORT GPIO_PORTD_BASE

//PD6 atomic read
#define ENCB_0_ADDR (ENC_PORT + GPIO_O_DATA + (BIT7HI << 2))
#define ENCB_0_S 7
#define ENCB_0_PIN "PD6"

//PD7 atomic read
#define ENCB_1_ADDR (ENC_PORT + GPIO_O_DATA + (BIT6HI << 2))
#define ENCB_1_S 6
#define ENCB_1_PIN "PD7"

#define ONE_SHOT_PS 8 //pre scale for 1-shot timers
#define STALL_TIME_US 20000

//for calculating RPM
#define TICKS_US 40
#define TICKS_MS 40000
#define MS_MIN 60000
#define GEAR_RATIO 50
#define PERIOD2RPM(x) (16000000 / x)

#define BYTE3_M 0xFF000000
#define SHIFT_3_BYTES_L(x) (x << 24)

#define TOGGLE_ON() HWREG(GPIO_PORTE_BASE + GPIO_O_DATA + (BIT2HI << 2)) = BIT2HI
#define TOGGLE_OFF() HWREG(GPIO_PORTE_BASE + GPIO_O_DATA + (BIT2HI << 2)) = BIT2LO

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this library.They should be functions
   relevant to the behavior of this library.
*/
static int32_t Period2RPM(uint8_t Which);
static void TestOperation(void);

/*---------------------------- Module Variables ---------------------------*/
static const uint8_t      ReverseCond[NUM_ENCS] = { 1, 0 };

static volatile uint32_t  Periods[NUM_ENCS] = { 0, 0 };
static volatile uint16_t  Counts[NUM_ENCS] = { 0, 0 };
static volatile uint8_t   Directions[NUM_ENCS] = { 0, 0 };
static volatile bool      StoppedFlags[NUM_ENCS] = { false, false };

static volatile uint32_t  stallTime;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
    EncLib_InitCapture

 Parameters
   None

 Returns
   None

 Description
   Sets up the input capture timers to measure the period between
   encoder pulses. Sets up B channel pins to read direction.
 Notes

 Author
   R. Merchant
****************************************************************************/
void EncLib_InitCapture(void)
{
  // Start clock to WTIMER1
  HWREG(SYSCTL_RCGCWTIMER) |= SYSCTL_RCGCWTIMER_R1;
  while ((HWREG(SYSCTL_PRWTIMER) & SYSCTL_PRWTIMER_R1) != SYSCTL_PRWTIMER_R1)
  {}
  // Enable port c clock
  HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R2;
  while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R2) != SYSCTL_PRGPIO_R2)
  {}

  // disable tmrs
  HWREG(WTIMER1_BASE + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
  HWREG(WTIMER1_BASE + TIMER_O_CTL) &= ~TIMER_CTL_TBEN;
  // Set up timers in 32-bit mode
  HWREG(WTIMER1_BASE + TIMER_O_CFG) = TIMER_CFG_16_BIT;

  // Set TIRL to max value (all 1's)
  HWREG(WTIMER1_BASE + TIMER_O_TAILR) = TIMER_TAILR_M;
  HWREG(WTIMER1_BASE + TIMER_O_TBILR) = TIMER_TBILR_M;
  // Set up TMRs to be input capture mode (TA/BMR = 0x3), count up
  HWREG(WTIMER1_BASE + TIMER_O_TAMR) =
      (HWREG(WTIMER1_BASE + TIMER_O_TAMR) & ~TIMER_TAMR_TAAMS) |
      (TIMER_TAMR_TACDIR | TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP);
  HWREG(WTIMER1_BASE + TIMER_O_TBMR) =
      (HWREG(WTIMER1_BASE + TIMER_O_TBMR) & ~TIMER_TBMR_TBAMS) |
      (TIMER_TBMR_TBCDIR | TIMER_TBMR_TBCMR | TIMER_TBMR_TBMR_CAP);
  // Set capture to rising edge
  HWREG(WTIMER1_BASE + TIMER_O_CTL) &= ~TIMER_CTL_TAEVENT_M;
  HWREG(WTIMER1_BASE + TIMER_O_CTL) &= ~TIMER_CTL_TBEVENT_M;

  // Set PC6 to WT1CCP0 and PC7 to WT1CCP1
  // First set AFSEL bit7 and bit6 flag high
  HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) |= (BIT7HI | BIT6HI);
  // Write 7 to 7th and 6th nibble in PCTL
  HWREG(GPIO_PORTC_BASE + GPIO_O_PCTL) =
      ((HWREG(GPIO_PORTC_BASE + GPIO_O_PCTL) & ~BYTE3_M)
      | SHIFT_3_BYTES_L(0x77));
  // Make PC7 and PC6 DIO
  HWREG(GPIO_PORTC_BASE + GPIO_O_DEN) |= (BIT7HI | BIT6HI);
  // Set PC7 and PC6 to input
  HWREG(GPIO_PORTC_BASE + GPIO_O_DIR) &= (BIT7LO & BIT6LO);

  // Set up direction pins as digital inputs
  GPIO_ConfigPin( ENCB_0_PIN, Input);
  GPIO_ConfigPin( ENCB_1_PIN, Input);

  //Setup Debug pin
  GPIO_ConfigPin( "PE2",      Output);
  GPIO_SetPin("PE2", Low);

  // Clear pending capture interrupt
  HWREG(WTIMER1_BASE + TIMER_O_ICR) = TIMER_ICR_CAECINT;
  HWREG(WTIMER1_BASE + TIMER_O_ICR) = TIMER_ICR_CBECINT;
  // Enable capture event interrupt mask
  HWREG(WTIMER1_BASE + TIMER_O_IMR) |= TIMER_IMR_CAEIM;
  HWREG(WTIMER1_BASE + TIMER_O_IMR) |= TIMER_IMR_CBEIM;

  // Enable NVIC_EN3 bit 1 annd 0
  HWREG(NVIC_EN3) |= (BIT1HI | BIT0HI);
  // Enable interrupts globally
  __enable_irq();
  return;
}

/****************************************************************************
 Function
    EncLib_Init1ShotTimers

 Parameters
   None
 Returns
   Period value

 Description
   Sets up the 1-shot timers to check for a 0 rpm condition.
 Notes

 Author
   R. Merchant
****************************************************************************/
void EncLib_Init1ShotTimers(void)
{
  // Start clock to WTIMER2
  HWREG(SYSCTL_RCGCWTIMER) |= SYSCTL_RCGCWTIMER_R2;
  while ((HWREG(SYSCTL_PRWTIMER) & SYSCTL_PRWTIMER_R2) != SYSCTL_PRWTIMER_R2)
  {}

  //disable timers
  HWREG(WTIMER2_BASE + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
  HWREG(WTIMER2_BASE + TIMER_O_CTL) &= ~TIMER_CTL_TBEN;
  //Set to 16-bit mode
  HWREG(WTIMER2_BASE + TIMER_O_CFG) = TIMER_CFG_16_BIT;

  // Set timers IRL to stall time
  stallTime = STALL_TIME_US * TICKS_US;
  HWREG(WTIMER2_BASE + TIMER_O_TAILR) = stallTime;
  HWREG(WTIMER2_BASE + TIMER_O_TBILR) = stallTime;
  // Set timers to countdown, 1 shot mode
  HWREG(WTIMER2_BASE + TIMER_O_TAMR) =
      (HWREG(WTIMER2_BASE + TIMER_O_TAMR) &
      (~TIMER_TAMR_TAAMS & ~TIMER_TAMR_TACDIR)) |
      TIMER_TAMR_TAMR_1_SHOT;
  HWREG(WTIMER2_BASE + TIMER_O_TBMR) =
      (HWREG(WTIMER2_BASE + TIMER_O_TBMR) &
      (~TIMER_TBMR_TBAMS & ~TIMER_TBMR_TBCDIR)) |
      TIMER_TBMR_TBMR_1_SHOT;

  // Clear pending timout interrupts
  HWREG(WTIMER2_BASE + TIMER_O_ICR) = TIMER_ICR_TATOCINT;
  HWREG(WTIMER2_BASE + TIMER_O_ICR) = TIMER_ICR_TBTOCINT;
  // Enable timout event interrrupt mask
  HWREG(WTIMER2_BASE + TIMER_O_IMR) |= TIMER_IMR_TATOIM;
  HWREG(WTIMER2_BASE + TIMER_O_IMR) |= TIMER_IMR_TBTOIM;
  // Enable NVIC_EN3 bit 2 annd 3
  HWREG(NVIC_EN3) |= (BIT2HI | BIT3HI);
  // Enable interrupts globally
  __enable_irq();
}

/****************************************************************************
 Function
    EncLib_GetPeriod

 Parameters
   Which encoder

 Returns
   Period value

 Description
   Starts all timers associated with the speed measurement
 Notes

 Author
   R. Merchant
****************************************************************************/
void EncLib_StartHWTimers(void)
{
  // Enable capture timers
  HWREG(WTIMER1_BASE + TIMER_O_CTL) |= TIMER_CTL_TAEN;
  HWREG(WTIMER1_BASE + TIMER_O_CTL) |= TIMER_CTL_TBEN;

  // Enable 1-shot timers
  HWREG(WTIMER2_BASE + TIMER_O_CTL) |= TIMER_CTL_TAEN;
  HWREG(WTIMER2_BASE + TIMER_O_CTL) |= TIMER_CTL_TBEN;
}

/****************************************************************************
 Function
    EncLib_GetPeriod

 Parameters
   Which encoder, Pointer to struct to store data

 Returns
   Encoder Data struct

 Description
   Returns period, direction and stop status of the selected encoder as
   members of a structure.
 Notes

 Author
   R. Merchant
****************************************************************************/
void EncLib_GetData(uint8_t Which, EncData_t *pData)
{
  // Store data to struct fields
  pData->RPM = Period2RPM(Which);
  pData->Count = Counts[Which];
}

/***************************************************************************
 Library ISR functions ADD NAMES TO STARTUP FILE!!!
 ***************************************************************************/
/****************************************************************************
ISR Name
   EncLib_Capture#ISR

Description
  Gets value in the timer register after getting a capture event. Stores
  period into array based on the #isr it is. Sets stop flag low as well.
Notes

Author
  R. Merchant
****************************************************************************/
void EncLib_Capture0ISR(void)  //WT1CCP0
{
  //Intialize data
  static uint32_t LastRiseTime;
  static uint32_t CurrentRiseTime; //static for speed

  // Clear interrupt flag
  HWREG(WTIMER1_BASE + TIMER_O_ICR) = TIMER_ICR_CAECINT;
  // Restart WT2A by reloading ILR
  HWREG(WTIMER2_BASE + TIMER_O_TAILR) = stallTime;

  #if 0 // Test operation
  TestOperation();
  #endif

  //Read B Channel
  Directions[0] = HWREG(ENCB_0_ADDR) >> ENCB_0_S;
  // Clear motorStopped flag
  StoppedFlags[0] = false;
  // Get TAR value
  CurrentRiseTime = HWREG(WTIMER1_BASE + TIMER_O_TAR);
  // Calculate dt (current - last)
  Periods[0] = CurrentRiseTime - LastRiseTime;
  // Increment the encoder tick count
  Counts[0]++;
  // Set LastRiseTime to current rime
  LastRiseTime = CurrentRiseTime;
}

void EncLib_Capture1ISR(void)  //WT1CCP1
{
  //Intialize data
  static uint32_t LastRiseTime;
  static uint32_t CurrentRiseTime; //static for speed

  // Clear interrupt flag
  HWREG(WTIMER1_BASE + TIMER_O_ICR) = TIMER_ICR_CBECINT;
  // Restart WT2B by reloading ILR
  HWREG(WTIMER2_BASE + TIMER_O_TBILR) = stallTime;

  #if 0 // Test operation
  TestOperation();
  #endif

  //Read B Channel
  Directions[1] = HWREG(ENCB_1_ADDR) >> ENCB_1_S;
  // Clear motorStopped flag
  StoppedFlags[1] = false;
  // Get TAV value
  CurrentRiseTime = HWREG(WTIMER1_BASE + TIMER_O_TBR);
  // Calculate dt (current - last)
  Periods[1] = CurrentRiseTime - LastRiseTime;
  // Increment the encoder tick count
  Counts[1]++;
  // Set LastRiseTime to current rime
  LastRiseTime = CurrentRiseTime;
}

/****************************************************************************
ISR Name
   EncLib_Stall#ISR

Description
  Goes off them stall timer# reaches 0 to indicate motor# has stopped
  turning. Sets the corresponding stop flag and restarts the timer.
Notes

Author
  R. Merchant
****************************************************************************/
void EncLib_StallISR0(void)
{
  // Clear interrupt flag
  HWREG(WTIMER2_BASE + TIMER_O_ICR) = TIMER_ICR_TATOCINT;

  #if 0 // Test operation
  TestOperation();
  #endif

  // Set motor stop flag
  StoppedFlags[0] = true;
  // Restart timer
  HWREG(WTIMER2_BASE + TIMER_O_CTL) |= TIMER_CTL_TAEN;
}

void EncLib_StallISR1(void)
{
  // Clear interrupt flag
  HWREG(WTIMER2_BASE + TIMER_O_ICR) = TIMER_ICR_TBTOCINT;

  #if 0 // Test operation
  TestOperation();
  #endif

  // Set motor stop flag
  StoppedFlags[1] = true;
  // Restart timer
  HWREG(WTIMER2_BASE + TIMER_O_CTL) |= TIMER_CTL_TBEN;
}

/***************************************************************************
 private functions
 ***************************************************************************/
/****************************************************************************
Function
   EncLib_Period2RPM

Parameters
  Period

Returns
  RPM

Description
  converts Period value into an RPM value based on encoder and capture
  timer settings
Notes
  THIS FUNCTION RETURNS THE RPM OF THE SHAFT THE MOTOR IS ATTACHED TO!
  Be sure to perform necessary calculations to covert shaft RPM to
  ouput RPM! Values are unsigned as well, use direction flag to set sign.

Author
  R. Merchant
****************************************************************************/
static int32_t Period2RPM(uint8_t Which)
{
  static int32_t RPM; //static for speed
  //If motor stopped, set RPM to 0
  if (StoppedFlags[Which])
  {
    RPM = 0;
  }
  // Else calculate RPM
  else
  {
    RPM = PERIOD2RPM(Periods[Which]);
    // check to see if direction pin indicates reverse rotation
    if (Directions[Which] == ReverseCond[Which])
    {
      // Negate RPM value
      RPM = (-1) * RPM;
    }
  }
  return RPM;
}

static void TestOperation(void)
{
  static bool TestFlag = true;
  if (TestFlag)
  {
    TOGGLE_ON();
  }
  else
  {
    TOGGLE_OFF();
  }
  TestFlag = !TestFlag;
}

/*------------------------------- Footnotes -------------------------------*/
/* Designed for drive motor encoders.
   Make sure to add ISR labels to startup file for vector placement
*/
/*------------------------------ End of file ------------------------------*/
