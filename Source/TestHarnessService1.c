/****************************************************************************
 Module
   TestHarnessService1.c

 Revision
   1.0.1

 Description
   This is the first service for the Test Harness under the
   Gen2 Events and Services Framework.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 10/26/17 18:26 jec     moves definition of ALL_BITS to ES_Port.h
 10/19/17 21:28 jec     meaningless change to test updating
 10/19/17 18:42 jec     removed referennces to driverlib and programmed the
                        ports directly
 08/21/17 21:44 jec     modified LED blink routine to only modify bit 3 so that
                        I can test the new new framework debugging lines on PF1-2
 08/16/17 14:13 jec      corrected ONE_SEC constant to match Tiva tick rate
 11/02/13 17:21 jec      added exercise of the event deferral/recall module
 08/05/13 20:33 jec      converted to test harness service
 01/16/12 09:58 jec      began conversion from TemplateFSM.c
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
// This module
#include "TestHarnessService1.h"

// Hardware
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"

// Event & Services Framework
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_DeferRecall.h"
#include "ES_ShortTimer.h"
#include "ES_Port.h"

#include "AccelerometerFSM.h"
#include "GeneralPWM.h"
#include "EncoderLib.h"
#include "DriveService.h"

/*----------------------------- Module Defines ----------------------------*/
// these times assume a 1.000mS/tick timing
#define ONE_SEC 1000
#define HALF_SEC (ONE_SEC / 2)
#define TWO_SEC (ONE_SEC * 2)
#define FIVE_SEC (ONE_SEC * 5)
#define QUARTER_SEC (HALF_SEC / 2)
#define EIGHTH_SEC (QUARTER_SEC / 2)
#define SIXTEENTH_SEC (EIGHTH_SEC / 2)


#define clrScrn() printf("\x1b[2J")
#define clrLine() printf("\x1b[K")

#define ENCODER_ZERO 0
#define ENCODER_ONE 1
#define MOTOR_ZERO 0
#define MOTOR_ONE 1
#define MOTOR_ZERO_DUTY_CYCLE 70
#define MOTOR_ONE_DUTY_CYCLE 50

// RAMPUP Defines and Vars-------------------------------- 
// RampUp state incrimental value
#define INCRIMENT_VAL 25
#define INCRIMENT_TIME (QUARTER_SEC)
// Ramp Up state variables
static uint8_t RampIndex = 1;
static uint8_t OriginalParam = 100;
//--------------------------------------------------------




// #define ALL_BITS (0xff<<2)   Moved to ES_Port.h
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
// type of state variable should match that of enum in header file
 static TestRampUp_t CurrentState;
 
 static uint8_t TargetWheelRPM = 140;  // 140 MAX SPEED.  OBEY THE SPEED LIMIT ---Dimitri.
 

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitTestHarnessService1

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and does any
     other required initialization for this service
 Notes

 Author
     J. Edward Carryer, 01/16/12, 10:00
****************************************************************************/
bool InitTestHarnessService1(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  /********************************************
   in here you write your initialization code
   *******************************************/


    EncLib_StartHWTimers();
    
    // Init Motors 
      DCMotorPWM_SetDutyCycle(MOTOR_ZERO, 0);
      DCMotorPWM_SetDutyCycle(MOTOR_ONE, 0);
      DCMotorPWM_EnableOutputs();
      
      
 // put us into the Initial PseudoState
  CurrentState = InitState;
   
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
     PostTestHarnessService1

 Parameters
     EF_Event ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostTestHarnessService1(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunTestHarnessService1

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes

 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event_t RunTestHarnessService1(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  
  switch (CurrentState)
  {
    case InitState:        // If current state is initial Psedudo State
    {
      printf("InitState \n\r");
      if (ThisEvent.EventType == ES_INIT)    // only respond to ES_Init
      {
        // this is where you would put any actions associated with the
        // transition from the initial pseudo-state into the actual
        // initial state
       
        // Post Drive CCW
        ThisEvent.EventType = EM_STOP;
        ThisEvent.EventParam = TargetWheelRPM/5;
        PostDriveService(ThisEvent);
        
        // Simulate Looking for Beacon
        ES_Timer_InitTimer(SERVICE1_TIMER, 100);
        
        // now put the machine into the actual initial state
        CurrentState = Orienting2Beacon;
      }
    }
    break;
    
    case Orienting2Beacon:
    {
      printf("Orienting2Beacon \n\r");      
      if (((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == SERVICE1_TIMER))
          || ((ThisEvent.EventType == ES_NEW_KEY) && (ThisEvent.EventParam == '2')))
      {
        
        // Simulate Waiting2Drive
        ES_Timer_InitTimer(SERVICE1_TIMER, ONE_SEC);
        CurrentState = Waiting2Drive;
      }  
    }
    break;
    
    case Waiting2Drive:
    {
      printf("Waiting2Drive \n\r");
      if (((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == SERVICE1_TIMER))
            || ((ThisEvent.EventType == ES_NEW_KEY) && (ThisEvent.EventParam == '3')))
      {
        // Start driving forwared
//          ThisEvent.EventType = EM_DRIVE_F;
//          ThisEvent.EventParam = TargetWheelRPM;
//          PostDriveService(ThisEvent);
        
        CurrentState = Ramping;
        ES_Timer_InitTimer(RAMP_UP_TIMER,INCRIMENT_TIME);        
      }
    }
    break;
    
    
    case Ramping:
    {
        uint8_t RampIncriments;
        
      
        //Example: Target RPM (OriginalParam) = 100 RPM, Incriment_Val = 25
        // With RampIndex = 1, loops over 3 times, 25, 50, and 75
        if (RampIndex == 1)
        {  
          printf("RampIndex:    %i     \n\r", RampIndex);
          //OriginalParam = ThisEvent.EventParam;
          RampIncriments = TargetWheelRPM/INCRIMENT_VAL; //Ex: 100/25 = 4 
          ThisEvent.EventType = EM_DRIVE_F;
          ThisEvent.EventParam = INCRIMENT_VAL * RampIndex;
          printf("RPM: %i\r\n", ThisEvent.EventParam);
          PostDriveService(ThisEvent);
          RampIndex++;
          ES_Timer_InitTimer(RAMP_UP_TIMER,INCRIMENT_TIME);
        }
        else if (RampIndex <= RampIncriments)
        {
          if (((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == RAMP_UP_TIMER))
                || ((ThisEvent.EventType == ES_NEW_KEY) && (ThisEvent.EventParam == '4')))
          {
          printf("RampIndex:    %i     \n\r", RampIndex);
          //Set target RPM to RampUp Incriments for both motors 
          ThisEvent.EventType = EM_DRIVE_F;
          ThisEvent.EventParam = INCRIMENT_VAL * RampIndex;
          printf("RPM: %i\r\n", ThisEvent.EventParam);
          PostDriveService(ThisEvent);
          RampIndex++;
          ES_Timer_InitTimer(RAMP_UP_TIMER,INCRIMENT_TIME);
          }
        }
        else
        {
          printf("RampIndex:    %i     \n\r", RampIndex);
          ThisEvent.EventType = EM_DRIVE_F;
          ThisEvent.EventParam = OriginalParam;
          printf("RPM: %i\r\n", ThisEvent.EventParam);
          PostDriveService(ThisEvent);
          // Reset ramp index
          RampIndex = 1;
          // Start Timer for Going to miner
          ES_Timer_InitTimer(SERVICE1_TIMER, TWO_SEC);
          // Set Next State
          CurrentState = Going2Miner;
          
        }
        
        
        // Simulate MinerFound Event
        if ((ThisEvent.EventType == ES_NEW_KEY) && (ThisEvent.EventParam == '6'))
          {
            CurrentState = GrabbingMiner;
            
            
          }
    }
    break;
    
    case Going2Miner:
    {
      printf("Going2Miner \n\r");
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == SERVICE1_TIMER))
      {
        
        ES_Event_t NewEvent = {EM_STOP, 0};
        PostDriveService(NewEvent);
        ES_Timer_InitTimer(SERVICE1_TIMER, FIVE_SEC);
        CurrentState = GrabbingMiner;  
      }
    }
    break;
    
    case GrabbingMiner:
    {
      printf("GrabbingMiner \n\r");
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == SERVICE1_TIMER))
      {
        
        CurrentState = GrabbingMiner;  
      }
    }
    break;
  }

  return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/



/****************************************************************************
 Function
     RampUp

 Parameters
    ThisEvent

 Returns
	Nothing

 Description
    Sets the motors to start driving forward at a slower rate than the target to ensure
    that the motors accelerate at more or less the same rate and more or less drive straight

 Notes

 Author
    Dimitri Petrakis 2/22/2020
****************************************************************************/


#if 0
static void RampUp(ES_Event_t ThisEvent)
{
  uint8_t RampIncriments;
  
  //Example: Target RPM (OriginalParam) = 100 RPM, Incriment_Val = 25
  // With RampIndex = 1, loops over 3 times, 25, 50, and 75
  if (RampIndex == 1)
  {  
    OriginalParam = ThisEvent.EventParam;
    RampIncriments = OriginalParam/INCRIMENT_VAL; //Ex: 100/25 = 4 
    ThisEvent.EventType = EM_DRIVE_F;
    ThisEvent.EventParam = INCRIMENT_VAL * RampIndex;
    PostDriveService(ThisEvent);
    RampIndex++;
    ES_Timer_InitTimer(RAMP_UP_TIMER,INCRIMENT_TIME);
  }
  else if (RampIndex < RampIncriments)
  {
    if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == RAMP_UP_TIMER))
    {
    //Set target RPM to RampUp Incriments for both motors
      
    ThisEvent.EventType = EM_DRIVE_F;
    ThisEvent.EventParam = INCRIMENT_VAL * RampIndex;
    PostDriveService(ThisEvent);
    RampIndex++;
    ES_Timer_InitTimer(RAMP_UP_TIMER,INCRIMENT_TIME);
    }
  }
  else
  {
    ThisEvent.EventType = EM_DRIVE_F;
    ThisEvent.EventParam = OriginalParam;
    PostDriveService(ThisEvent);
    RampIndex = 1;
  }
  return;
}
#endif
/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

