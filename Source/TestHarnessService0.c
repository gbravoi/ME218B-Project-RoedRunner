/****************************************************************************
 Module
   TestHarnessService0.c

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
#include "TestHarnessService0.h"

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

#include "SPUD_Query.h"
#include "SPUDHSM.h"

#include "GP_MasterGameHSM.h"


#include "EncoderLib.h"
#include "DriveService.h"
#include "Miner.h"

/*----------------------------- Module Defines ----------------------------*/
// these times assume a 1.000mS/tick timing
#define ONE_SEC 1000
#define HALF_SEC (ONE_SEC / 2)
#define TWO_SEC (ONE_SEC * 2)
#define FIVE_SEC (ONE_SEC * 5)

#define ENTER_POST ((MyPriority << 3) | 0)
#define ENTER_RUN ((MyPriority << 3) | 1)
#define ENTER_TIMEOUT ((MyPriority << 3) | 2)

#define DEBUG

// #define ALL_BITS (0xff<<2)   Moved to ES_Port.h
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/



/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t    MyPriority;
// add a deferral queue for up to 3 pending deferrals +1 to allow for ovehead
static ES_Event_t DeferralQueue[3 + 1];

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitTestHarnessService0

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
bool InitTestHarnessService0(uint8_t Priority)
{

  ES_Event_t ThisEvent;

  MyPriority = Priority;
  /********************************************
   in here you write your initialization code
   *******************************************/
  // initialize deferral queue for testing Deferal function
  ES_InitDeferralQueueWith(DeferralQueue, ARRAY_SIZE(DeferralQueue));

  // init timer
	 ES_Timer_InitTimer(SERVICE0_TIMER, FIVE_SEC);
	
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
     PostTestHarnessService0

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
bool PostTestHarnessService0(ES_Event_t ThisEvent)
{
#ifdef _INCLUDE_BYTE_DEBUG_
  _HW_ByteDebug_SetValueWithStrobe( ENTER_POST);
  _HW_ByteDebug_SetValueWithStrobe( END_SERVICE);
#endif
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunTestHarnessService0

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
ES_Event_t RunTestHarnessService0(ES_Event_t ThisEvent)
{
  ES_Event_t  ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
 // static char DeferredChar = '1';
  

#ifdef _INCLUDE_BYTE_DEBUG_
  _HW_ByteDebug_SetValueWithStrobe(ENTER_RUN);
#endif
  switch (ThisEvent.EventType)
  {
    case ES_INIT:
    {
      ES_Timer_InitTimer(SERVICE0_TIMER, HALF_SEC);
      puts("Service 00:");
      EncLib_StartHWTimers();
      printf("\rES_INIT received in Service %d\r\n", MyPriority);
    }
    break;
    case ES_TIMEOUT:   // re-start timer & announce
    {
#ifdef _INCLUDE_BYTE_DEBUG_
      _HW_ByteDebug_SetValueWithStrobe( ENTER_TIMEOUT );
#endif  
      ES_Timer_InitTimer(SERVICE0_TIMER, FIVE_SEC);
     printf("ES_TIMEOUT received from Timer %d in Service %d\r\n",
          ThisEvent.EventParam, MyPriority);
    }
    
    break;
    case ES_NEW_KEY:   // announce
    {
			ES_Event_t NewEvent;
      printf("ES_NEW_KEY received with -> %c <- in Service 0\r\n",
          (char)ThisEvent.EventParam);

			if ('a' == ThisEvent.EventParam)
			{
				NewEvent.EventType = EM_STOP;
				PostDriveService(NewEvent);
			}
			if ('b' == ThisEvent.EventParam)
			{
				NewEvent.EventType = GO_TO_MINER;
				NewEvent.EventParam = 0;
				PostMasterGameHSM(NewEvent);
				printf("GO_TO_MINER \n\r");
			}
			if ('c' == ThisEvent.EventParam)
			{
				NewEvent.EventType = MINER_FOUND;
				PostMasterGameHSM(NewEvent);
				printf("MINER FOUND \n\r");
			}
			if ('d' == ThisEvent.EventParam)
			{
				NewEvent.EventType = MINER_REACHED;
				PostMasterGameHSM(NewEvent);
				printf("MINER REACHED \n\r");
			}
    

      if ('e' == ThisEvent.EventParam)
			{
				ES_Event_t CheckOffEvent;
        CheckOffEvent.EventType = EC_COLLISION_F;
        PostMasterGameHSM(CheckOffEvent);
				printf("COLLISION \n\r");
			}
			if ('f' == ThisEvent.EventParam)
			{
				ES_Event_t CheckOffEvent;
        CheckOffEvent.EventType = MINER_LOC_UPDATE_OURS;// for desicion
				CheckOffEvent.EventParam=((Unknown<<4)| Unknown  );
				printf("MINER location updated \n\r");
        PostMasterGameHSM(CheckOffEvent);
			}
			if ('g' == ThisEvent.EventParam)
			{
				ES_Event_t CheckOffEvent;
        CheckOffEvent.EventType = ALL_OUR_PERMITS_UPDATE;// for desicion
				CheckOffEvent.EventParam=(BIT0HI| BIT1HI| BIT5HI);
        PostMasterGameHSM(CheckOffEvent);
			}
			if ('h' == ThisEvent.EventParam)
			{
				ES_Event_t CheckOffEvent;
        CheckOffEvent.EventType = EM_AT_POS;// for desicion
				
        PostMasterGameHSM(CheckOffEvent);
			}
      if ('i' == ThisEvent.EventParam)
			{
				ES_Event_t CheckOffEvent;
        CheckOffEvent.EventType =ALL_OUR_PERMITS_UPDATE;// for desicion
				
        PostMasterGameHSM(CheckOffEvent);
			}
      if ('j' == ThisEvent.EventParam)
			{
				ES_Event_t CheckOffEvent;
        CheckOffEvent.EventType =DEBUG_EV1;// for desicion
        CheckOffEvent.EventParam =7;// for desicion
				
        PostMasterGameHSM(CheckOffEvent);
			}
			if ('r' == ThisEvent.EventParam)
			{
				printf("SERVO RELEASE \r\n");
				MinerLib_Release();
			}
			if ('s' == ThisEvent.EventParam)
			{
				printf("SERVO GRAB \r\n");
				MinerLib_Grab();
			}
			if ('t' == ThisEvent.EventParam)
			{
				ES_Event_t ThisEvent;
        ThisEvent.EventType = INFO_QUERY;
        ThisEvent.EventParam = ALL_OUR_PERMITS;
        PostSPUDHSM(ThisEvent);
				printf("NAV: ASKed SPUD for permits \n\r");
			}
			else if ('q' == ThisEvent.EventParam)
			{
				ES_Event_t ThisEvent;
				ThisEvent.EventType = INFO_QUERY;
				ThisEvent.EventParam = MINER_LOC_OURS;
				PostSPUDHSM(ThisEvent);
				
				ThisEvent.EventType = INFO_QUERY;
				ThisEvent.EventParam = MINER_LOC_THEIRS;
				PostSPUDHSM(ThisEvent);
				printf("Asking for THEIR Miner location\r\n");
			}
			else if ('p' == ThisEvent.EventParam)
			{
				ES_Event_t ThisEvent;
				ThisEvent.EventType = INFO_QUERY;
				ThisEvent.EventParam = ALL_OUR_PERMITS;
				PostSPUDHSM(ThisEvent);
				
				ThisEvent.EventType = INFO_QUERY;
				ThisEvent.EventParam = ALL_THEIR_PERMITS;
				PostSPUDHSM(ThisEvent);
			}
     // printf("query %X\r\n", Comand);
    }  
		
		

    break;
		case MINER_LOC_UPDATE_THEIRS:
		{
			#ifdef DEBUG
      #pragma diag_suppress 188 // Doesn't like printing and enum type for some reason
      printf("Miner of Interest: %d\r\n", QueryMINERLocation(QueryMinerOfInterest()));
			printf("Miner 0: %u\r\n", (uint16_t)QueryMINERLocation(0));
			printf("Miner 1: %u\r\n", (uint16_t)QueryMINERLocation(1));
			printf("Miner 2: %u\r\n", (uint16_t)QueryMINERLocation(2));
			printf("Miner 3: %u\r\n", (uint16_t)QueryMINERLocation(3));
      #endif

		}
		break;
		case ALL_THEIR_PERMITS_UPDATE:
		{
			Color_t OurPermitLocs[3];
			QueryOursLocations(OurPermitLocs);
			printf("OUR: First Loc: %d, Second Loc: %d, Third Loc: %d \r\n", OurPermitLocs[0], OurPermitLocs[1], OurPermitLocs[2]);
			
			Color_t TheirPermitLocs[3];
			QueryTheirsLocations(TheirPermitLocs);
			printf("THEIR: First Loc: %d, Second Loc: %d, Third Loc: %d \r\n", TheirPermitLocs[0], TheirPermitLocs[1], TheirPermitLocs[2]);
		}
    default:
    {}
    break;
  }
#ifdef _INCLUDE_BYTE_DEBUG_
  _HW_ByteDebug_SetValueWithStrobe(END_SERVICE);
#endif

  return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/
