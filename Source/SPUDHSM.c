/****************************************************************************
 Module
   SPUDHSM.c

 Description
   This is the higher level of the SPUD system

 Notes

****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
// Basic includes for a program using the Events and Services Framework
#include "ES_Configure.h"
#include "ES_Framework.h"

/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "SPUDHSM.h"
#include "NavigationLibrary.h"
#include "ES_DeferRecall.h"
#include "SPUD_Query.h"
#include "TypesDefinition.h"
#include "PinsLib.h"
#include "GP_MasterGameHSM.h"

/*----------------------------- Module Defines ----------------------------*/
// define constants for the states for this machine
// and any other local defines

#define SPUD_WAIT 40

// these are the addresses to send to the SPUD
#define STATUS_ADDR 0xC0
#define C1MLOC1_ADDR 0xC1
#define C1MLOC2_ADDR 0xC2
#define C2MLOC1_ADDR 0xC3
#define C2MLOC2_ADDR 0xC4
#define C1RESH_ADDR 0xC5
#define C1RESL_ADDR 0xC6
#define C2RESH_ADDR 0xC7
#define C2RESL_ADDR 0xC8
#define PERMIT1_ADDR 0xC9
#define PERMIT2_ADDR 0xCA

#define STATUS_MASK (BIT0HI | BIT1HI)
#define WAITING4PERMITS 0x0
#define PERMITS_ISSUED BIT0HI
#define SUDDEN_DEATH BIT1HI
#define PERMITS_EXPIRED (BIT0HI | BIT1HI)

#define UNKNOWN_LOC_BIT BIT4HI
#define BLACK_DETECTION_BIT BIT5HI
#define WHITE_DETECTION_BIT BIT6HI
#define LOC_GOOD_BIT BIT7HI
#define LOC_MASK (BIT0HI | BIT1HI | BIT2HI | BIT3HI)

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine, things like during
   functions, entry & exit functions.They should be functions relevant to the
   behavior of this state machine
*/
static ES_Event_t DuringWaiting4InfoQuery(ES_Event_t Event);
static ES_Event_t DuringQueryingStatus(ES_Event_t Event);
static ES_Event_t DuringQueryingOther(ES_Event_t Event);
static Color_t GetRegion(uint16_t answer);
static Color_t GetLoc1(uint16_t answer);
static Color_t GetLoc2(uint16_t answer);

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well
static SPUDHSMState_t CurrentState;
static uint8_t        MyPriority;
static uint16_t       LastSPUDQuery;
static uint16_t       LastInfoQuery;

// add a deferral queue for up to 3 pending deferrals +1 to allow for overhead
static ES_Event_t DeferralQueue[3 + 1];

// for multi-byte queries
static uint16_t multi_byte_info;

//arrays with all posistions with permits of a company
static Color_t Permits_C1[3]; //0:exclusive, 1 and 2: neutral
static Color_t Permits_C2[3]; //0:exclusive, 1 and 2: neutral

//Color of miners
static Color_t C1_MINER1_color;
static Color_t C1_MINER2_color;
static Color_t C2_MINER1_color;
static Color_t C2_MINER2_color;

// either 0 or 1
static Team_t OurTeam; 

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitSPUDHSM

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this HSM
 Notes

 Author
     rpt
****************************************************************************/
bool InitSPUDHSM(uint8_t Priority)
{
	
  // associate with correct queue
  MyPriority = Priority;

  // initialize a deferral queue
  ES_InitDeferralQueueWith(DeferralQueue, ARRAY_SIZE(DeferralQueue));

	// get team information by reading state of team select switch
	OurTeam=ReadTeam();
	
  // call start function for top level state machine
  ES_Event_t CurrentEvent;
  CurrentEvent.EventType = ES_ENTRY;
  StartSPUDHSM(CurrentEvent);
	
  
  return true;
}

/****************************************************************************
 Function
     PostSPUDHSM

 Parameters
     EF_Event_t ThisEvent , the event to post to the queue

 Returns
     boolean False if the Enqueue operation failed, True otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     rpt
****************************************************************************/
bool PostSPUDHSM(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunSPUDHSM

 Parameters
   ES_Event_t: the event to process

 Returns
   ES_Event_t: an event to return

 Description
   add your description here

 Notes
   uses nested switch/case to implement the machine.

 Author
   rpt
****************************************************************************/
ES_Event_t RunSPUDHSM(ES_Event_t CurrentEvent)
{
  // Initialize MakeTransition as False
  bool            MakeTransition = false;
  // Initialize NextState as CurrentState
  SPUDHSMState_t  NextState = CurrentState;
  // Default to normal entry to new state
  ES_Event_t      EntryEventKind = { ES_ENTRY, 0 };
  // We're a top level, so always return no event
  ES_Event_t      ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT;
  uint16_t        answer;
  ES_Event_t      ThisEvent;
  
  switch (CurrentState)
  {
	// If current state is Waiting4InfoQuery
    case Waiting4InfoQuery:
    { 
      CurrentEvent = DuringWaiting4InfoQuery(CurrentEvent);
      //If an event is active
      if (CurrentEvent.EventType != ES_NO_EVENT)
      {
        switch (CurrentEvent.EventType)
        {
          case ES_TIMEOUT:
          {
			// SPUD timer times out
            if (CurrentEvent.EventParam == SPUD_TIMER)         
            { 
              // Go to QueryingStatus
              NextState = QueryingStatus;
			  // Mark that we're making a transition
              MakeTransition = true;
            }
          }
          break;

          case INFO_QUERY:
          {
            // Stop the SPUD Timer
            ES_Timer_StopTimer(SPUD_TIMER);
            // If querying status
            if (CurrentEvent.EventParam == STATUS)
            {
              // Go to QueryingStatus
              NextState = QueryingStatus;
							// Mark that we're making a transition
              MakeTransition = true;
            }
            // If querying literally anything else
            else if (CurrentEvent.EventParam != STATUS)
            {
              // Go to QueryingOther state
              NextState = QueryingOther;
			  // Mark that we're making a transition
              MakeTransition = true;
              
              // Save away what is being asked for as LastInfoQuery
              LastInfoQuery = CurrentEvent.EventParam;
             
            }
          }
          break;
					
		  default:
		  {
			;
		  }
        }
      }
    }
    break;
		
	// If current state is QueryingStatus
    case QueryingStatus:           
    {    
	  // Execute During function for QueryingStatus
      CurrentEvent = DuringQueryingStatus(CurrentEvent);
	  // Save the SPUDs answer to a variable
      answer = CurrentEvent.EventParam;
      // If an event is active
      if (CurrentEvent.EventType != ES_NO_EVENT)      
      {
        switch (CurrentEvent.EventType)
        {
					// If event is SPUD_ANSWER
          case SPUD_ANSWER:       
          {
            // Create a STATUS_UPDATE event 
            ThisEvent.EventType = STATUS_UPDATE;
            
            // If the SPUDs answer was waiting for permits
            if ((answer & STATUS_MASK) == WAITING4PERMITS)
            {  
			  // Set event parameter appropriately
              ThisEvent.EventParam = W4P;
            }
			// Else if the SPUDs answer was permits issued
            else if ((answer & STATUS_MASK) == PERMITS_ISSUED)
            {
			  // Set event parameter appropriately
              ThisEvent.EventParam = PI;
            }
			// Else if the SPUDs answer was sudden death 
            else if ((answer & STATUS_MASK) == SUDDEN_DEATH)
            {
			  // Set event parameter appropriately
              ThisEvent.EventParam = SD;
            }
			// Else if the SPUDs answer was permits expired
            else if ((answer & STATUS_MASK) == PERMITS_EXPIRED)
            {
			  // Set event parameter appropriately
              ThisEvent.EventParam = PE;
            }
			// End If
            
            // Post the status update event
            ES_PostAll(ThisEvent);

            // Recall any deferred events
            ES_RecallEvents(MyPriority, DeferralQueue);
            
            // Go back to Waiting4InfoQuery
            NextState = Waiting4InfoQuery;
			// Mark that we're making a transition
            MakeTransition = true;
          }
          break;
          
		  // If recieve a new query while not in Waiting4InfoQuery
          case INFO_QUERY:
          {       
			// Add it to the deferral queue
            ES_DeferEvent(DeferralQueue, CurrentEvent);
          }
          break;
					
		  default:
		  {
			;
		  }
        }
      }
    }
    break;

		// If current state is QueryingOther
    case QueryingOther:           
    {    
			// Execute During function for QueryingOther.
      CurrentEvent = DuringQueryingOther(CurrentEvent);
      //If an event is active
      if (CurrentEvent.EventType != ES_NO_EVENT)      
      {
        switch (CurrentEvent.EventType)
        {
		  //If SPUD answered
          case SPUD_ANSWER:       
          {       
			// Save SPUDs answer as a variable
            answer = CurrentEvent.EventParam;
            switch (LastSPUDQuery)
            {
			  // If LastSPUDQuery is C1MLOC1_ADDR
              case C1MLOC1_ADDR:
              {
                // Save away information about location of first MINER
                C1_MINER1_color = GetRegion(answer);

                // We'll now query location of 2nd MINER
                // So stay in QueryingOther state
                NextState = QueryingOther;

                // Post NEW_SPUD_QUERY with param C1MLOC2 to self
                ES_Event_t ThisEvent;
                ThisEvent.EventType = NEW_SPUD_QUERY;
                ThisEvent.EventParam = C1MLOC2_ADDR;
                PostSPUDHSM(ThisEvent);
                
                // Update LastSPUDQuery
                LastSPUDQuery = ThisEvent.EventParam;
              }
              break;
              
			  // If LastSPUDQuery is C1MLOC2_ADDR
              case C1MLOC2_ADDR:
              {
				// Save miner color
				C1_MINER2_color = GetRegion(answer);
				
				// If OurTeam is CHK
				if (OurTeam == CHK)
				{
					// Set event type to MINER_LOC_UPDATE_OURS
					ThisEvent.EventType = MINER_LOC_UPDATE_OURS;
				}
				// if OurTeam is GHI
				else if (OurTeam == GHI)
				{
					// Set event type to MINER_LOC_UPDATE_THEIRS
					ThisEvent.EventType = MINER_LOC_UPDATE_THEIRS;
				}
				// End If
								
                // Post miner location update
                ES_PostAll(ThisEvent);

                // Set NextState to QueryingStatus
                NextState = QueryingStatus;
								// Markt that we're making a transition
                MakeTransition = true;
                
              }
              break;
							
			  // If LastSPUDQuery is C2MLOC1_ADDR
              case C2MLOC1_ADDR:
              {
                // Save away information about location of first MINER
				C2_MINER1_color = GetRegion(answer);
								
                // we'll now query location of 2nd MINER
                // so stay in QueryingOther state
                NextState = QueryingOther;

                // Post NEW_SPUD_QUERY with param C2MLOC2 to self
                ES_Event_t ThisEvent;
                ThisEvent.EventType = NEW_SPUD_QUERY;
                ThisEvent.EventParam = C2MLOC2_ADDR;
                PostSPUDHSM(ThisEvent);
                
                // Update LastSPUDQuery
                LastSPUDQuery = ThisEvent.EventParam;
              }
              break;
              
			  // If LastSPUDQuery is C2MLOC2_ADDR
              case C2MLOC2_ADDR:
              {
				// Save Miner color
				C2_MINER2_color = GetRegion(answer);
				
				// If OurTeam is CHK
				if (OurTeam == CHK)
				{
					// Make event type MINER_LOC_UPDATE_THEIRS
					ThisEvent.EventType = MINER_LOC_UPDATE_THEIRS;
				}
				// If OurTeam is GHI
				else if (OurTeam == GHI)
				{
					// Make event type MINER_LOC_UPDATE_OURS
					ThisEvent.EventType = MINER_LOC_UPDATE_OURS;
				}
								
                // Post miner location update
                ES_PostAll(ThisEvent);

                // Set NextState to QueryingStatus
                NextState = QueryingStatus;
								// Mark that we're making a transition
                MakeTransition = true;
                
              }
              
              break;
							
			  // If LastSPUDQuery is C1RESH_ADDR
              case C1RESH_ADDR:
              {
                // Save away high resource information
                multi_byte_info = answer;
                
                // We'll now query SPUD for low resource information
                // So stay in QueryingOther state
                NextState = QueryingOther;

                // Post NEW_SPUD_QUERY with param C1RESL_ADDR to self
                ES_Event_t ThisEvent;
                ThisEvent.EventType = NEW_SPUD_QUERY;
                ThisEvent.EventParam = C1RESL_ADDR;
                PostSPUDHSM(ThisEvent);
                
                // Update LastSPUDQuery
                LastSPUDQuery = ThisEvent.EventParam;
              }
              break;
              
			  // If LastSPUDQuery is C1RESH_ADDR
              case C1RESL_ADDR:
              {
				// If OurTeam is CHK
				if (OurTeam == CHK)
				{
					// Make event type RES_UPDATE_OURS
					ThisEvent.EventType = RES_UPDATE_OURS;
				}
				// Else if OurTeam is GHI
				else if (OurTeam == GHI)
				{
					// Make event type RES_UPDATE_THEIRS
					ThisEvent.EventType = RES_UPDATE_THEIRS;
				}
								
                // Combine hi and lo resource information
                multi_byte_info = ( (multi_byte_info << 8) | answer );
                ThisEvent.EventParam = multi_byte_info;
                
                // Post resource update
                ES_PostAll(ThisEvent);

                // Set NextState to QueryingStatus
                NextState = QueryingStatus;
								// Mark that we're making a transition
                MakeTransition = true;
                
              }
              break;
              
			  // If LastSPUDQuery is C2RESH_ADDR
              case C2RESH_ADDR:
              {
                // Save away high resource information
                multi_byte_info = answer;
                
                // We'll now query SPUD for low resource information
                // So stay in QueryingOther state
                NextState = QueryingOther;

                // Post NEW_SPUD_QUERY with param C2RESL_ADDR to self
                ES_Event_t ThisEvent;
                ThisEvent.EventType = NEW_SPUD_QUERY;
                ThisEvent.EventParam = C2RESL_ADDR;
                PostSPUDHSM(ThisEvent);
                
                //Update LastSPUDQuery
                LastSPUDQuery = ThisEvent.EventParam;
              }
              break;
              
			  // If LastSPUDQuery is C2RESL_ADDR
              case C2RESL_ADDR:
              {
				// If OurTeam is CHK
				if (OurTeam == CHK)
				{
					// Make event type to RES_UPDATE_THEIRS
					ThisEvent.EventType = RES_UPDATE_THEIRS;
				}
				// Else if OurTeam is GHI
				else if (OurTeam == GHI)
				{
					// Make event type to RES_UPDATE_OURS
					ThisEvent.EventType = RES_UPDATE_OURS;
				}
								
                // Combine hi and lo resource information
                multi_byte_info = ( (multi_byte_info << 8) | answer );
                ThisEvent.EventParam = multi_byte_info;
                
                // Post resource update
                ES_PostAll(ThisEvent);

                // Set NextState to QueryingStatus
                NextState = QueryingStatus;
								// Mark that we're making a transition
                MakeTransition = true;
                
              }
              break;
              
							// If LastSPUDQuery is PERMIT1_ADDR
              case PERMIT1_ADDR:
              {
				//Save location in company's permit array
				Permits_C1[0]=GetLoc1(answer);
				Permits_C2[0]=GetLoc2(answer);
								
                // If we are looking for OUR exclusive permit location
                if (LastInfoQuery == EX_PERMIT_OURS)
                {
                  // Create an event with type EX_PERMIT_UPDATE_OURS
                  ThisEvent.EventType = EX_PERMIT_UPDATE_OURS;
                  
                  // Set parameter equal to exclusive permit location for OUR team
				  if (OurTeam == CHK)
				  {
					ThisEvent.EventParam = Permits_C1[0];
				  }
				  else if (OurTeam == GHI)
				  {
					ThisEvent.EventParam = Permits_C2[0];
				  }
                  
                  // Post the event
                  ES_PostAll(ThisEvent);
                  
                  // Set NextState to QueryingStatus
                  NextState = QueryingStatus;
				  // Mark that we're making a transition
                  MakeTransition = true;
                  
                }
                // Else if we are looking for THEIR exclusive permit location
                else if (LastInfoQuery == EX_PERMIT_THEIRS)
                {
                  // Create an event with type EX_PERMIT_UPDATE_THEIRS
                  ThisEvent.EventType = EX_PERMIT_UPDATE_THEIRS;
									
				  // Set parameter equal to exclusive permit location for THIER team
				  if (OurTeam == CHK)
				  {
					ThisEvent.EventParam = Permits_C2[0];
				  }
				  else if (OurTeam == GHI)
				  {
					ThisEvent.EventParam = Permits_C1[0];
				  }
                                    
                  // Post this event 
                  ES_PostAll(ThisEvent);
                  
                  // Set NextState to QueryingStatus
                  NextState = QueryingStatus;
									// Mark that we're making a transition
                  MakeTransition = true;
                  
                }
				// Else if we're looking for all our or their permits
				else if ( (LastInfoQuery == ALL_OUR_PERMITS) || (LastInfoQuery == ALL_THEIR_PERMITS) ){  
					
				  // We'll now query SPUD for neutral permits
				  NextState = QueryingOther;
`				  
				  // Post NEW_SPUD_QUERY with param NEUTRAL_PERMITS to self
				  ES_Event_t ThisEvent;
				  ThisEvent.EventType = NEW_SPUD_QUERY;
				  ThisEvent.EventParam = PERMIT2_ADDR;
				  PostSPUDHSM(ThisEvent);
				
				  // Update LastSPUDQuery
				  LastSPUDQuery = ThisEvent.EventParam;
				}
								
              
              }
              break;
              
			  // If LastSPUDQuery is PERMIT2_ADDR
              case PERMIT2_ADDR:
              {
				//S ave location in company's permit array
				Permits_C1[1]=GetLoc1(answer);
				Permits_C1[2]=GetLoc2(answer);
				Permits_C2[1]=GetLoc1(answer);
				Permits_C2[2]=GetLoc2(answer);
				
				// If we're looking for Netrual Permits
				if (LastInfoQuery == NEUTRAL_PERMITS)
				{
				  // Post neutral permit locations update
				  ThisEvent.EventType = NEUTRAL_PERMIT_UPDATE;
				  ThisEvent.EventParam = answer;
					
				  ES_PostAll(ThisEvent);
					
				  // Set NextState to QueryingStatus
				  NextState = QueryingStatus;
				  // Mark that we're making a transition
				  MakeTransition = true;

				}
				
				// Else if we're looking for all our permits
				else if (LastInfoQuery == ALL_OUR_PERMITS)
				{
				  // Post that array with OUR locations is ready
				  ThisEvent.EventType = ALL_OUR_PERMITS_UPDATE;
				  ES_PostAll(ThisEvent);
				
				  // Set NextState to QueryingStatus
				  NextState = QueryingStatus;
				  // Mark that we're making a transition
				  MakeTransition = true;
				}
					
				// Else if we're looking for all their permits
				else if (LastInfoQuery == ALL_THEIR_PERMITS)
				{
				  // Post that array with THEIR locations is ready
				  ThisEvent.EventType = ALL_THEIR_PERMITS_UPDATE;
				  ES_PostAll(ThisEvent);
				
				  // Set NextState to QueryingStatus
				  NextState = QueryingStatus;
				  // Mark that we're making a transition
				  MakeTransition = true;
				} 
								
              }
              break;
							
			  default:
			  {
				;
			  }
								
            }
          }
          break;
		  //If recieve a new query while not in Waiting4InfoQuery
          case INFO_QUERY:       
          {          
			// add it to the deferral queue
            ES_DeferEvent(DeferralQueue, CurrentEvent);
          }
          break;
					
		  default:
		  {
			;
		  }
        }
      }
    }
    break;
    default:
	  ;
						
  }
	// If we're making a transition
  if (MakeTransition == true)
  {
    // update CurrentState
    CurrentState = NextState;
	// and run state machine with entry event
    RunSPUDHSM(EntryEventKind);
  }
  return ReturnEvent;
}

/****************************************************************************
 Function
     StartSPUDHSM

 Parameters
     CurrentEvent

 Returns
     None

 Description
     Does any required initialization for this state machine
 Notes

 Author
     rpt
****************************************************************************/
void StartSPUDHSM(ES_Event_t CurrentEvent)
{
  // We aren't doing anything with history, so CurrentState is just Waiting4InfoQuery
  CurrentState = Waiting4InfoQuery;

  // Call the entry function (if any) for the ENTRY_STATE
  RunSPUDHSM(CurrentEvent);
}

/****************************************************************************
 Function
     QuerySPUDHSM

 Parameters
     None

 Returns
     SPUDHSMState_t The current state of the Template state machine

 Description
     returns the current state of the SPUDHSM
 Notes

 Author
     rpt
****************************************************************************/
SPUDHSMState_t QuerySPUDHSM(void)
{
  return CurrentState;
}

/****************************************************************************
 Function
     QueryMINERLocation

 Parameters
     Miner_t MinerID

 Returns
     Color_t the color of the miner requested

 Description
     returns the current color of a Miner
 Notes

 Author
     rpt
****************************************************************************/
Color_t QueryMINERLocation(Miner_t MinerID)
{
	 Color_t ReturnVal=Unknown;
  if(MinerID==CHK_Miner1)
	{
		ReturnVal= C1_MINER1_color;
	}
	else if(MinerID==CHK_Miner2)
	{
		ReturnVal= C1_MINER2_color;
	}
	else if(MinerID==GHI_Miner1)
	{
		ReturnVal= C2_MINER1_color;
	}
	else if(MinerID==GHI_Miner2)
	{
		ReturnVal= C2_MINER2_color;
	}
	return ReturnVal;	
}

/****************************************************************************
 Function
     QueryLocationsTheirs

 Parameters
		 Return pointer to fisrt element of array with all permited locations for THEIR team
     None

 Returns
     None

 Description
     Allows to access all locations for THEIR team, by giving the pointer to the array
 Notes

 Author
     gab
****************************************************************************/
void QueryTheirsLocations(Color_t * pPermits)
{
	// If OurTeam is CHK
	if (OurTeam == CHK)
	{
		// Populate array with elements of Corporation 2's permits
		*pPermits=Permits_C2[0]; 
		*(pPermits+1)=Permits_C2[1];
		*(pPermits+2)=Permits_C2[2];
	}
	// If OurTeam is GHI
	else if (OurTeam == GHI)
	{
		// Populate array with elements of Corporation 1's permits
		*pPermits=Permits_C1[0];
		*(pPermits+1)=Permits_C1[1];
		*(pPermits+2)=Permits_C1[2];
	}
  return;
}

/****************************************************************************
 Function
     QueryLocationsC2

 Parameters
     None

 Returns
     Return pointer to fisrt element of array with all permited locations for OUR Team

 Description
     Allows to access all locations for OUR team, by giving the pointer to the array
 Notes

 Author
     gab
****************************************************************************/
void QueryOursLocations(Color_t * pPermits)
{
	// If OurTeam is CHK
	if (OurTeam == CHK)
	{
		// Populate array with elements of Corporation 1's permits
		*pPermits=Permits_C1[0];
		*(pPermits+1)=Permits_C1[1];
		*(pPermits+2)=Permits_C1[2];
	}
	// Else if OurTeam is CHK
	else if (OurTeam == GHI)
	{
		// Populate array with elements of Corporation 2's permits
		*pPermits=Permits_C2[0];
		*(pPermits+1)=Permits_C2[1];
		*(pPermits+2)=Permits_C2[2];
	}
  return;
}

/***************************************************************************
 private functions
 ***************************************************************************/

static ES_Event_t DuringWaiting4InfoQuery(ES_Event_t Event)
{
  ES_Event_t ReturnEvent;
  ReturnEvent = Event;

  // Process ES_ENTRY & ES_EXIT events
  if (Event.EventType == ES_ENTRY)
  {
    // Start a timer to trigger status updates
    ES_Timer_InitTimer(SPUD_TIMER, SPUD_WAIT);

    // No lower state machines for this state
  }
  return ReturnEvent;
}

static ES_Event_t DuringQueryingStatus(ES_Event_t Event)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; 

  // Process ES_ENTRY & ES_EXIT events
  if (Event.EventType == ES_ENTRY)
  {
    // Start SPUDQuery state machine
    StartSPUDQuerySM(Event);
    
    // Post NEW_SPUD_QUERY with param STATUS to self
    ES_Event_t ThisEvent;
    ThisEvent.EventType = NEW_SPUD_QUERY;
    ThisEvent.EventParam = STATUS_ADDR;
    PostSPUDHSM(ThisEvent);
    
  }
  else if (Event.EventType == ES_EXIT)
  {
    // give the lower levels a chance to clean up first
    RunSPUDQuerySM(Event);
    // There's no local exit functionality
  }
  else
  // Do the 'during' function for this state
  {
    // Run any lower level state machine
    ReturnEvent = RunSPUDQuerySM(Event);
  }
	// Return event from lower level state machine
  return ReturnEvent;
}

static ES_Event_t DuringQueryingOther(ES_Event_t Event)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT;

  // Process ES_ENTRY & ES_EXIT events
  if (Event.EventType == ES_ENTRY)
  {
    // Start SPUDQuery state machine
    StartSPUDQuerySM(Event);
    
    // Create NEW_SPUD_QUERY
    ES_Event_t ThisEvent;
    ThisEvent.EventType = NEW_SPUD_QUERY;
    
    // Translate info_query param to appropriate NEW_SPUD_QUERY param
    switch (LastInfoQuery)
    {
	  // if LastInfoQuery is MINER_LOC_OURS
      case MINER_LOC_OURS :
	  {
		// If OurTeam is CHK
		if (OurTeam == CHK) 
		{
			// Set Event parameter to C1MLOC1_ADDR
			ThisEvent.EventParam = C1MLOC1_ADDR;
		}
		// else if OurTeam is GHI
		else if (OurTeam == GHI)
		{
			// Set Event parameter to C2MLOC1_ADDR
			ThisEvent.EventParam = C2MLOC1_ADDR;
		}
	  }
      break;
      
	  // if LastInfoQuery is MINER_LOC_THEIRS
      case MINER_LOC_THEIRS :
      {
		// if OurTeam is CHK
		if (OurTeam == CHK) 
		{
			// Set Event parameter to C2MLOC1_ADDR
			ThisEvent.EventParam = C2MLOC1_ADDR;
		}
		// if OurTeam is GHI
		else if (OurTeam == GHI)
		{
			// Set Event parameter to C1MLOC1_ADDR
			ThisEvent.EventParam = C1MLOC1_ADDR;
		}
	  }
      break;
    
	  // if LastInfoQuery is RES_OURS
      case RES_OURS :
      {
		// if OurTeam is CHK
		if (OurTeam == CHK) 
		{
			// Set Event parameter to C1RESH_ADDR
			ThisEvent.EventParam = C1RESH_ADDR;
		}
		// if OurTeam is GHI
		else if (OurTeam == GHI)
		{
			// Set Event parameter to C2RESH_ADDR
			ThisEvent.EventParam = C2RESH_ADDR;
		}
	  }
      break;
      
	  // if LastInfoQuery is RES_THEIRS
      case RES_THEIRS :
      {
		// if OurTeam is CHK
		if (OurTeam == CHK) 
		{
			// Set Event parameter to C2RESH_ADDR
			ThisEvent.EventParam = C2RESH_ADDR;
		}
		// if OurTeam is GHI
		else if (OurTeam == GHI)
		{
			// Set Event parameter to C1RESH_ADDR
			ThisEvent.EventParam = C1RESH_ADDR;
		}
	  }
      break;
      
	  // if LastInfoQuery is EX_PERMIT_OURS
      case EX_PERMIT_OURS :
	  {
		// Set Event parameter to PERMIT1_ADDR
        ThisEvent.EventParam = PERMIT1_ADDR;
	  }
      break;
      
	  // Else if LastInfoQuery is EX_PERMIT_THEIRS
      case EX_PERMIT_THEIRS :
	  {
		// Set Event parameter to PERMIT1_ADDR
        ThisEvent.EventParam = PERMIT1_ADDR;
	  }
      break;
      
	  // Else if LastInfoQuery is NEUTRAL_PERMITS
      case NEUTRAL_PERMITS:
			{
				// Set Event parameter to PERMIT2_ADDR
        ThisEvent.EventParam = PERMIT2_ADDR;
			}
      break;
			
	  // Else if LastInfoQuery is ALL_OUR_PERMITS
	  case ALL_OUR_PERMITS:
	  {
		// Set Event parameter to PERMIT1_ADDR
        ThisEvent.EventParam = PERMIT1_ADDR;
	  }
      break;
			
			// Else if LastInfoQuery is ALL_OUR_PERMITS
	  case ALL_THEIR_PERMITS:
	  {
		// Set Event parameter to PERMIT1_ADDR
        ThisEvent.EventParam = PERMIT1_ADDR;
	  }
      break;
      
    }
    
    // Save away what is being queried in LastSPUDQuery
    LastSPUDQuery = ThisEvent.EventParam;
    // Post this NEW_SPUD_QUERY event with appropriate param to self
    PostSPUDHSM(ThisEvent);
    
  }
  else if (Event.EventType == ES_EXIT)
  {
    // On exit, give the lower levels a chance to clean up first
    RunSPUDQuerySM(Event);
    // There's no local exit functionality
  }
  else
  // Do the 'during' function for this state
  {
    // Run any lower level state machine
    ReturnEvent = RunSPUDQuerySM(Event);
  }
  // Return either Event, if you don't want to allow the lower level machine
  // to remap the current event, or ReturnEvent if you do want to allow it.
  return ReturnEvent;
}

static Color_t GetRegion(uint16_t answer)
{
  Color_t ReturnVal;
  // If MINER location is known
  if ((answer & LOC_GOOD_BIT) == LOC_GOOD_BIT)
  {
	// Set return value to answer masked with the location mask
    ReturnVal = (Color_t)(answer & LOC_MASK);
  }
  // Else if MINER is on white
  else if ((answer & WHITE_DETECTION_BIT) == WHITE_DETECTION_BIT)
  {
	// Return White
    ReturnVal = White;
  }
  // Else if the MINER is on black
  else if ((answer & BLACK_DETECTION_BIT) == BLACK_DETECTION_BIT)
  {
	// Return Black
    ReturnVal = Black;
  }
	// Else
  else
  {
	// Return Unknown
    ReturnVal = Unknown;
  }
  return ReturnVal;
}

static Color_t GetLoc1(uint16_t answer)
{
  // Return 4 LSB of answer
  return (Color_t)(answer & (BIT0HI | BIT1HI| BIT2HI | BIT3HI) );
}

static Color_t GetLoc2(uint16_t answer)
{
  // Shift asnwer right by 4 then return 4 LSB of answer
  return (Color_t)((answer &(BIT4HI | BIT5HI| BIT6HI | BIT7HI)) >> 4);
}

/*------------------------------- Footnotes -------------------------------*/

/*------------------------------ End of file ------------------------------*/