/****************************************************************************
 Module
   MinerLib.c

 Revision
   1.0.3

 Description
   Contains functions for IR Beacon detection and for tape detection.
   Contains the ISR for both functions as well.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
02/03/2020      ram      initial release
02/12/2020		  ram		   Updating library for project use. Adjsuting event
                         event checker to look for the mine selected.
02/27/2020      ram      Updating to include "gripper" actuation
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
// the common headers for C99 types
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// the headers to access the GPIO subsystem
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"

// Timer interrupts
#include "inc/hw_timer.h"
#include "inc/hw_nvic.h"

//Framework files
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Port.h"

#include "gpio.h"
#include "GeneralPWM.h"
#include "Miner.h"
#include "GP_MasterGameHSM.h"

/*----------------------------- Module Defines ----------------------------*/
#define TICKS_US 40
#define TARGET_RANGE 20 * TICKS_US
#define NUM_MINERS 4
#define NUM_TEAM_MINERS 2
#define PROX_PIN "PB0" // for reading tape sensor circuit

#define EM_PIN "PB1" // for electromagnet control
#define EM_ON High
#define EM_OFF Low

#define SERVO_L 0
#define SERVO_L_OPEN 0
#define SERVO_L_CLOSE 180

#define SERVO_R 1
#define SERVO_R_OPEN 180
#define SERVO_R_CLOSE 0

//#define DEBUG
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this library.They should be functions
   relevant to the behavior of this library.
*/
/*---------------------------- Module Variables ---------------------------*/
static volatile uint32_t BeaconPeriod;
static volatile bool Hunting = false;
static uint8_t EdgeCounter = 0;
static uint32_t MinerPeriods_us[NUM_MINERS] =
	{
		300, // CHK_Miner1
		500, // CHK_Miner2
		700, // GHI_Miner1
		1100 // GHI_Miner2
	};


static Miner_t MinerToFind = None;
static bool MinerGrabbed = false;
static bool NoBeacon = true;
static uint8_t LastProxValue;
/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
    IRLib_BeaconHWInit

 Parameters
   None

 Returns
   None

 Description
   Sets up hardware for the IR beacon capture ISR
 Notes

 Author
   R. Merchant
****************************************************************************/
void Miner_HWInit(void)
{
  // Start WTMR0 clock
  HWREG(SYSCTL_RCGCWTIMER) |= SYSCTL_RCGCWTIMER_R0;
  // Set PC4 as digital input
  GPIO_ConfigPin("PC4", Input);
  // Set AFSEL for Bit 4 high
  HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) |= BIT4HI;
  // Set PCTL for 4th nibble to 7
  HWREG(GPIO_PORTC_BASE + GPIO_O_PCTL) =
      (HWREG(GPIO_PORTC_BASE + GPIO_O_PCTL) & 0xFFF0FFFF) | 0x00070000;

  // make sure wtmr0 clock is ready
  while ((HWREG(SYSCTL_PRWTIMER) & SYSCTL_PRWTIMER_R0) != SYSCTL_PRWTIMER_R0)
  {}
  // disable timer
  HWREG(WTIMER0_BASE + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
  // Set up timer in 32-bit mode
  HWREG(WTIMER0_BASE + TIMER_O_CFG) = TIMER_CFG_16_BIT;

  // Set TAIRL0 to max value (all 1's)
  HWREG(WTIMER0_BASE + TIMER_O_TAILR) = TIMER_TAILR_M;
  // Set up TMRA0 to be input capture mode (TAMR = 0x3), count up
  HWREG(WTIMER0_BASE + TIMER_O_TAMR) =
      (HWREG(WTIMER0_BASE + TIMER_O_TAMR) & ~TIMER_TAMR_TAAMS) |
      (TIMER_TAMR_TACDIR | TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP);
  // Set capture to both edges
  HWREG(WTIMER0_BASE + TIMER_O_CTL) =
      (HWREG(WTIMER0_BASE + TIMER_O_CTL) & ~TIMER_CTL_TAEVENT_M) |
      TIMER_CTL_TBEVENT_POS;

  // Clear pending capture interrupt
  HWREG(WTIMER0_BASE + TIMER_O_ICR) = TIMER_ICR_CAECINT;
  // Enable capture event interrupt mask
  HWREG(WTIMER0_BASE + TIMER_O_IMR) |= TIMER_IMR_CAEIM;

  // Set priority for  interrupt to 2
  HWREG(NVIC_PRI23) = (HWREG(NVIC_PRI23) & ~NVIC_PRI23_INTC_M) |
      (2 << NVIC_PRI23_INTC_S);

  //Set up electromagnet control pin
  GPIO_ConfigPin(EM_PIN, Output);
  // Make sure electromagnet is off
  GPIO_SetPin(EM_PIN, EM_OFF);
  
  // Setup pin to read tape sensor circuit
  GPIO_ConfigPin(PROX_PIN, Input);
  // Store the startup value of the pin
  LastProxValue = GPIO_ReadPin(PROX_PIN);


  return;
}

/****************************************************************************
 Function
    StartFindingMiner / StopFindingMiner

 Parameters
   None

 Returns
   None

 Description
   Starts/stops the input capture timer to look for edges. Enables/disables the
   NVIC and sets the enable flag high/low to (de)activate the event checker.
   Sets the selection flag and clears it
 Notes

 Author
   R. Merchant
****************************************************************************/
void StartFindingMiner(Miner_t MinerName)
{
	// Set selection flag
	MinerToFind = MinerName;
	// Enable NVIC_EN2 bit 30
	HWREG(NVIC_EN2) = BIT30HI;
   
   #ifdef DEBUG
   printf("Miner Name: %d \r\n", MinerName);
   #endif
   
	// Start timer
	HWREG(WTIMER0_BASE + TIMER_O_CTL) |= TIMER_CTL_TAEN;
	// Set Beacon period to 0
	BeaconPeriod = 0;
   // Set edge counts to zero
   EdgeCounter = 0;
	// Set hunting flag high
	Hunting = true;
	return;
}

void StopFindingMiner(void)
{

	// Disable NVIC_DIS2 bit 30
	HWREG(NVIC_DIS2) = BIT30HI;
	// Stop timer
	HWREG(WTIMER0_BASE + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
	// Set enable flag low
	Hunting = false;
  // Set no beacon flag false
  NoBeacon = false;
	// Clear hunting flag
	MinerToFind = None;
	return;
}
/****************************************************************************
 Function
    MinerLib_EnableEM / MinerLib_DisableEM

 Parameters
   None

 Returns
   None

 Description
   Sets the pin controling the electromagnet to either enable it or disable it.
 Notes
   To change the on and off state of the pin, check the #define statments

 Author
   R. Merchant
****************************************************************************/
void MinerLib_EnableEM(void)
{
	//Set EM pin to on state
	GPIO_SetPin(EM_PIN, EM_ON);
	return;
}

void MinerLib_DisableEM(void)
{
	//Set EM pin to off state
	GPIO_SetPin(EM_PIN, EM_OFF);
	return;
}

void MinerLib_Grab(void)
{
  //set servos to closed position
  ServoPWM_SetPosition(SERVO_L, SERVO_L_CLOSE);
  ServoPWM_SetPosition(SERVO_R, SERVO_R_CLOSE);
  // set grabber value true
  MinerGrabbed = true;
}

void MinerLib_Release(void)
{
  //set servoes to open position
  ServoPWM_SetPosition(SERVO_L, SERVO_L_OPEN);
  ServoPWM_SetPosition(SERVO_R, SERVO_R_OPEN);
  // set grabber value false
  MinerGrabbed = false;
}

bool MinerLib_QueryGrab(void)
{
  //return grab flag
  return(MinerGrabbed);
}

/****************************************************************************
 Function
    Query_Miner_Number

 Parameters
   None

 Returns
		Miner number

 Description
   Measures the period of the miner and returns which miner it is
 Notes
   To change the on and off state of the pin, check the #define statments

 Author
   D. Petrakis
****************************************************************************/

Miner_t Query_Miner_Number(void)
{
	// Initailizes the miner variable that will get returned 
	Miner_t My_Miner =None;
	
	// if Period matches miner Red1
  if (abs(BeaconPeriod - TICKS_US*MinerPeriods_us[0]) < TARGET_RANGE)
	{
		My_Miner = CHK_Miner1;		
	}
	// if Period matches miner Red2
	else if (abs(BeaconPeriod - TICKS_US*MinerPeriods_us[1]) < TARGET_RANGE)
	{
		My_Miner = CHK_Miner2;	
	}	
	// if Period matches miner Blue1
	else if (abs(BeaconPeriod - TICKS_US*MinerPeriods_us[2]) < TARGET_RANGE)
	{
		My_Miner = GHI_Miner1;	
	}	
	// if Period matches miner Blue2
	else if (abs(BeaconPeriod - TICKS_US*MinerPeriods_us[3]) < TARGET_RANGE)
	{
		My_Miner = GHI_Miner2;	
	}	
	
  return (My_Miner);
}


/****************************************************************************
 Function
    Beacon Event Checker

 Parameters
   None

 Returns
   bool

 Description
   Checks for a valid period to know beacon has been detected.
 Notes

 Author
   R. Merchant
****************************************************************************/
bool Check4Beacon(void)
{
  bool ReturnFlag = false;

  //If hunting and there is no beacon found
  if (Hunting & NoBeacon)
  {
    // Calculate error
	 uint8_t index = (uint8_t)MinerToFind; // cast just in case
    int32_t Error = BeaconPeriod - (TICKS_US*MinerPeriods_us[index]);
     
    #ifdef DEBUG
    printf("Period: %d\r\n", BeaconPeriod);
    #endif
		ES_Event_t BeaconEvent;
    // If error in range and we have enough edges
    if ((abs(Error) < TARGET_RANGE) && (EdgeCounter > 10))
    {
      // Post Beacon found!
      #ifdef DEBUG
      printf("Beacon Found!");
      #endif
       
      BeaconEvent.EventType = MINER_FOUND;
      BeaconEvent.EventParam = MinerToFind;
      PostMasterGameHSM(BeaconEvent);
      
      // Set no beacon flag low
      NoBeacon = false;

      // Set flag to true
      ReturnFlag = true;
			
    }
    // endif
  }
	
  //endif
  //return flag
  return ReturnFlag;
}
/****************************************************************************
 Function
    Miner Proximity Event Checker

 Parameters
   None

 Returns
   bool

 Description
   Checks ferrous ring of the miner is close enough the electromagnet.
 Notes

 Author
   R. Merchant
****************************************************************************/
bool Check4MinerRing(void)
{
  bool ReturnFlag = false;
  uint8_t CurrentProxValue;

  //If hunting
  if (true)
  {
    // Read pin
	 CurrentProxValue = GPIO_ReadPin(PROX_PIN);
    // If pin value has changed
    if (CurrentProxValue != LastProxValue)
    {
      // Set flag to true
      ReturnFlag = true;
      // Allocate memory for posting an event
      ES_Event_t MinerRingEvent;
      // if the pin state is low
      if(CurrentProxValue == 1)
      {
        //Post ring found if it is the miner of interest
        if(Query_Miner_Number()==QueryMinerOfInterest()){
        MinerRingEvent.EventType = MINER_REACHED;
        PostMasterGameHSM(MinerRingEvent);
        }else{//else act if this was a collision
        //post collition with param front 
							ES_Event_t NewEvent;
							NewEvent.EventType=COLLISION;
							NewEvent.EventParam=Coll_Front;
							PostMasterGameHSM(NewEvent);
          
        }
      }
      // if pin state is high
      else
      {
        //Post ring lost
        MinerRingEvent.EventType = MINERRING_LOST;
      }
      //Disable beacon interrupt
      StopFindingMiner();
    }
    // endif
  }
  //endif
  // Set last value to current value
  LastProxValue = CurrentProxValue;
  //return flag
  return ReturnFlag;
}

/***************************************************************************
 Library ISR functions ADD NAMES TO STARTUP FILE!!!
 ***************************************************************************/
/****************************************************************************
 ISR
    BeaconISR
 Description
   calculates the beacon period and stores to module variable
 Notes

 Author
   R. Merchant
****************************************************************************/
void MinerLib_BeaconISR(void)
{
	static uint32_t LastTime;
	static uint32_t CurrentTime; //static for speed
	// Clear interrupt
	HWREG(WTIMER0_BASE + TIMER_O_ICR) = TIMER_ICR_CAECINT;
	//get current time
	CurrentTime = HWREG(WTIMER0_BASE + TIMER_O_TAR);
	// calculate period
	BeaconPeriod = CurrentTime - LastTime;
	// increment edge counter
	EdgeCounter++;
	// Set last time = to currnet time
	LastTime = CurrentTime;
}

 /***************************************************************************
 private functions
 ***************************************************************************/

 /*------------------------------- Footnotes -------------------------------*/
/* Designed for drive motor encoders.
   Make sure to add ISR labels to startup file for vector placement
*/
/*------------------------------ End of file ------------------------------*/
