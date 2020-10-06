/****************************************************************************
 Module
   GP_NavigationSM.c

 Revision
   2.0.1

 Description
   Lower level SM of MasterGame.
	 Implement navigation of the robot once it acquired the MINER to a designated color

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 2/27/20      G.Bravo, D.Petrakis   firts code

****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
// Basic includes for a program using the Events and Services Framework
#include "ES_Configure.h"
#include "ES_Framework.h"

/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "GP_MasterGameHSM.h"
#include "GP_NavigationSM.h"
#include "Color.h"
#include "SPUDHSM.h"
#include "TypesDefinition.h"
#include "NavigationLibrary.h"
#include "DriveService.h"
#include "Miner.h"
#include "AccelerometerFSM.h"
#include "NavigationLibrary.h"

/*----------------------------- Module Defines ----------------------------*/
// define constants for the states for this machine
// and any other local defines

#define ENTRY_STATE CheckLocations
#define BACK_AWAY_FROM_MINER_TICKS 150
#define DRIVE_F_RPM 40

#define INCREMENT_TIME 250
#define MAX_SPEED 50
#define INCREMENT_VAL 25

#define ORIENT_SPEED 40
#define ORIENT_THRESH 2
#define ORIENT_CNTMAX 3
#define STABILIZING_WAIT 500

#define WALL_TIME 5000


/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine, things like during
   functions, entry & exit functions. They should be functions relevant to the
   behavior of this state machine
*/
static ES_Event_t DuringCheckLocations(ES_Event_t Event);
static ES_Event_t DuringWait2Stabilize(ES_Event_t Event);
static ES_Event_t DuringOrient(ES_Event_t Event);
static ES_Event_t DuringQueryMiner(ES_Event_t Event);
static ES_Event_t DuringMove2Color(ES_Event_t Event);
static ES_Event_t DuringMovingFromMiner(ES_Event_t Event);
static Color_t LookForClosestPermit(void);

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well
static NavState_t CurrentState;
static Color_t    PermitLocation_Goal = Unknown;
static Color_t    TractorColor;
static Color_t    LocationsArray[3];
static int16_t    CurrentSpeed = 0;
static uint8_t    OrientCnt = 0;
/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
 RunNavigationSM

 Parameters
   ES_Event_t: the event to process

 Returns
   ES_Event_t: an event to return

 Description
   Run function of navigation SM
 Notes
   uses nested switch/case to implement the machine.
 Author
   G.Bravo, D.Petrakis
****************************************************************************/
ES_Event_t RunNavigationSM(ES_Event_t CurrentEvent)
{
  bool        MakeTransition = false;/* are we making a state transition? */
  NavState_t  NextState = CurrentState;
  ES_Event_t  EntryEventKind = { ES_ENTRY, 0 }; // default to normal entry to new state
  ES_Event_t  ReturnEvent = CurrentEvent;       // assume we are not consuming event

  switch (CurrentState)
  {
    case CheckLocations:
    {
      // If current state is CheckLocations
      ReturnEvent = CurrentEvent = DuringCheckLocations(CurrentEvent);
      //process any events
      if (CurrentEvent.EventType != ES_NO_EVENT)      //If an event is active
      {
        switch (CurrentEvent.EventType)
        {
          //If we get notice that our permit locations have been updatd
		  case ALL_OUR_PERMITS_UPDATE: 
          {
            // Query the Spud and populate the array
            //with the Colors of the Permit Locations			
            QueryOursLocations(LocationsArray);

            // Query the color of our current location
            TractorColor = Query_Color();

            // if it's unknown or white or black, change to last color
            if ((TractorColor == White) || (TractorColor == Black) ||
			(TractorColor == Unknown))
            {
              TractorColor = Query_LastColor();
            }

            // Save the previous location
            Color_t PreviousPermitLocation_Goal = PermitLocation_Goal;

            // Update the current location of the closest permit
            PermitLocation_Goal = LookForClosestPermit();

            // if permit locations have changed
            if (PreviousPermitLocation_Goal != PermitLocation_Goal)
            {
              // move to Wait2Stabilize
              NextState = Wait2Stabilize;

              // we are making a transition
              MakeTransition = true;

              // consume
              ReturnEvent.EventType = ES_NO_EVENT;
            }
            // if permit locations haven't changed
            else if (PreviousPermitLocation_Goal == PermitLocation_Goal)
            {
              // go to QueryMiner state
              NextState = QueryMiner;

              // we're making a transition
              MakeTransition = true;

              // consume
              ReturnEvent.EventType = ES_NO_EVENT;
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

    case Wait2Stabilize:
    {
      ReturnEvent = CurrentEvent = DuringWait2Stabilize(CurrentEvent);
      if (CurrentEvent.EventType != ES_NO_EVENT)   //If an event is active
      {
        switch (CurrentEvent.EventType)
        {
          case ES_TIMEOUT:
          {
            if (CurrentEvent.EventParam == NAVIGATION_TIMER)
            {
              // now that we've stabilized we can go ahead and orient
              NextState = Orient;
              // mark that we're making a transition
              MakeTransition = true;
							// consume
              ReturnEvent.EventType = ES_NO_EVENT;
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

    case Orient:
    {
      ReturnEvent = CurrentEvent = DuringOrient(CurrentEvent);
      //process any events
      if (CurrentEvent.EventType != ES_NO_EVENT)      //If an event is active
      {
        switch (CurrentEvent.EventType)
        {
          case EM_AT_POS: // if we think we've finished orientating
          {
            // check if we're close enough
            // first get new orientation vector
            int16_t CurrentVec[2];
            AcclFSM_QueryVector(CurrentVec);
            int16_t DesiredVec[2];
            Query_DesiredVector(DesiredVec);

            // check if each element matches to some threshold
            int16_t x_diff = abs(CurrentVec[0] - DesiredVec[0]);
            int16_t y_diff = abs(CurrentVec[1] - DesiredVec[1]);

            // if we oriented fine or have exceeded max attempt
            if (((x_diff <= ORIENT_THRESH) && (y_diff <= ORIENT_THRESH)) || 
			(OrientCnt >= ORIENT_CNTMAX))
            {
              // Zero the orient counter
              OrientCnt = 0;
              // check if our tractor is already in the goal region
              TractorColor = Query_Color();

              if (TractorColor == PermitLocation_Goal)
              {
                // go to QueryMiner
                NextState = QueryMiner;
                MakeTransition = true;
              }
              // if we aren't already in the goal region
              else
              {
                // go to Move2Color
                NextState = Move2Color;
                MakeTransition = true;
              }
            }
            // if we didn't orient well enough and haven't exceeded max attempts
            else
            {
              NextState = Wait2Stabilize;
              OrientCnt++;
              MakeTransition = true;
            }

            // Consume Event
            ReturnEvent.EventType = ES_NO_EVENT;
          }
          break;
					case MINER_LOC_UPDATE_OURS :
					{
				    // make sure I know what miner I care about
            Miner_t MyMiner = QueryMinerOfInterest();
            // check what color it's on
            Color_t MinerColor = QueryMINERLocation(MyMiner);

            // if neither miner nor tractor is in the right color
            if ((MinerColor != PermitLocation_Goal))
            {
              //query miner color again
								//query location of miner
						ES_Event_t OtherEvent;
						OtherEvent.EventType = INFO_QUERY;
						OtherEvent.EventParam = MINER_LOC_OURS;
						PostSPUDHSM(OtherEvent);

            }

            // if either miner is in right color
            else if (MinerColor == PermitLocation_Goal)
            {

              // go to MovingAwayFromMiner
              NextState = MovingAwayFromMiner;
              // mark that we're making a transition
              MakeTransition = true;
              // Consume Event
              ReturnEvent.EventType = ES_NO_EVENT;
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

    case QueryMiner:
    {
      ReturnEvent = CurrentEvent = DuringQueryMiner(CurrentEvent);
      //process any events
      if (CurrentEvent.EventType != ES_NO_EVENT)      //If an event is active
      {
        switch (CurrentEvent.EventType)
        {
          // if our miner locations have been updated
          case MINER_LOC_UPDATE_OURS:
          {
            // make sure I know what miner I care about
            Miner_t MyMiner = QueryMinerOfInterest();
            // check what color it's on
            Color_t MinerColor = QueryMINERLocation(MyMiner);

            // make sure we know where we currently are

            // if neither miner nor tractor is in the right color
            if ((MinerColor != PermitLocation_Goal))
            {
              // go back to Move2Color
              NextState = Move2Color;

              // mark that we're making a transition
              MakeTransition = true;

              // consume the event
              ReturnEvent.EventType = ES_NO_EVENT;
            }

            // if either miner or tractor is in right color
            else if ((MinerColor == PermitLocation_Goal))
            {

              // go to MovingAwayFromMiner
              NextState = MovingAwayFromMiner;

              // mark that we're making a transition
              MakeTransition = true;

              // Consume Event
              ReturnEvent.EventType = ES_NO_EVENT;
            }
          }
          break;
          default:
          {
            ;
          }
        }
        // repeat cases as required for relevant events
      }
    }
    break;

    case Move2Color:           // If current state is Move2Color
    {

      ReturnEvent = CurrentEvent = DuringMove2Color(CurrentEvent);
      //process any events
      if (CurrentEvent.EventType != ES_NO_EVENT)      //If an event is active
      {
        // if we get a new color event
        if (CurrentEvent.EventType == EV_NEW_COLOR)
        {
					// Get the current tractor color
          TractorColor = (Color_t)CurrentEvent.EventParam;

          // if the tractor is in the goal color
          if (TractorColor == PermitLocation_Goal)
          {
						    // Stop Driving
							ES_Event_t ThisEvent;
							ThisEvent.EventType = EM_STOP;
							PostDriveService(ThisEvent);

            // move to CheckLocations
            NextState = CheckLocations;

            // mark that we're making a transition
            MakeTransition = true;
          }
          // if the tractor isn't in the goal color
          else
          {
            // Check if the tractor has overshot its position (true or false)
            bool PathCheck = NavLib_CheckPath(TractorColor);

            // If it's still on the right path
            if (PathCheck)
            {
               //start Wall Timer (if this timer expires, mean we are stuck)
								ES_Timer_InitTimer(WALL_TIMER, WALL_TIME);

            }

            // If the tractor has overshot
            else
            {
              // go back to Wait2Stabilize
              NextState = Wait2Stabilize;
              // mark that we're making a transition
              MakeTransition = true;

            }
          }

          // Consume Event
          ReturnEvent.EventType = ES_NO_EVENT;
        }
				//when we finish querying miner position
				else if(CurrentEvent.EventType==MINER_LOC_UPDATE_OURS){
				    // make sure I know what miner I care about
            Miner_t MyMiner = QueryMinerOfInterest();
            // check what color it's on
            Color_t MinerColor = QueryMINERLocation(MyMiner);

            // if neither miner nor tractor is in the right color
            if ((MinerColor != PermitLocation_Goal))
            {
              //query miner color again
								//query location of miner
						ES_Event_t OtherEvent;
						OtherEvent.EventType = INFO_QUERY;
						OtherEvent.EventParam = MINER_LOC_OURS;
						PostSPUDHSM(OtherEvent);

            }

            // if either miner is in right color
            else if (MinerColor == PermitLocation_Goal)
            {

              // go to MovingAwayFromMiner
              NextState = MovingAwayFromMiner;
              // mark that we're making a transition
              MakeTransition = true;
              // Consume Event
              ReturnEvent.EventType = ES_NO_EVENT;
            }
          }
        // if we get a timeout from the wall timer
        else if ((CurrentEvent.EventType == ES_TIMEOUT) 
			&& (CurrentEvent.EventParam == WALL_TIMER))
				{
				    // Post a collision from front
						ES_Event_t ThisEvent;
            ThisEvent.EventType = EC_COLLISION_F;
            PostMasterGameHSM(ThisEvent);
					// Consume Event
              ReturnEvent.EventType = ES_NO_EVENT;
				}
        // if we get a timeout from the ramp timer
        else if ((CurrentEvent.EventType == ES_TIMEOUT) 
			&& (CurrentEvent.EventParam == RAMP_TIMER))
        {
          ES_Event_t ThisEvent;
          // increment speed
          int16_t NewSpeed = CurrentSpeed + INCREMENT_VAL;

          // if incrementing brings us up to max speed
          if (NewSpeed >= MAX_SPEED)
          {
            // Set speed to MAX_SPEED
            ThisEvent.EventType = EM_DRIVE_F;
            ThisEvent.EventParam = MAX_SPEED;
            PostDriveService(ThisEvent);

            // update CurrentSpeed
            CurrentSpeed = MAX_SPEED;
          }

          else
          {
            // Set speed to NewSpeed
            ThisEvent.EventType = EM_DRIVE_F;
            ThisEvent.EventParam = NewSpeed;
            PostDriveService(ThisEvent);

            // update CurrentSpeed
            CurrentSpeed = NewSpeed;

            // restart RAMP_TIMER
            ES_Timer_InitTimer(RAMP_TIMER, INCREMENT_TIME);
          }
          // Consume Event
          ReturnEvent.EventType = ES_NO_EVENT;
        }
      }
    }
    break;

    case MovingAwayFromMiner:   // If current state is MovingAwayFromMiner
    {

      ReturnEvent = CurrentEvent = DuringMovingFromMiner(CurrentEvent);
      //process any events
      if (CurrentEvent.EventType != ES_NO_EVENT)      //If an event is active
      {
        if (CurrentEvent.EventType == EM_AT_POS)
        {
          // Stop Driving
          ES_Event_t ThisEvent;
          ThisEvent.EventType = EM_STOP;
          PostDriveService(ThisEvent);

          // post that navigation has ended
          ES_Event_t NavigationEnded;
          NavigationEnded.EventType = NAVIGATION_ENDED;
          PostMasterGameHSM(NavigationEnded);
        }
      }
    }
    break;
    default:
    {
      ;
    }
  }


  //   If we are making a state transition
  if (MakeTransition == true)
  {
    //   Execute exit function for current state
    CurrentEvent.EventType = ES_EXIT;
    RunNavigationSM(CurrentEvent);

    CurrentState = NextState;    //Modify state variable

    //   Execute entry function for new state
    // this defaults to ES_ENTRY
    RunNavigationSM(EntryEventKind);
  }
  return ReturnEvent;
}

/****************************************************************************
 Function
     StartNavigationSM

 Parameters
     None

 Returns
     None

 Description
     Does any required initialization for this state machine
 Notes

 Author
    G.Bravo, D.Petrakis
****************************************************************************/
void StartNavigationSM(ES_Event_t CurrentEvent)
{
  // to implement entry to a history state or directly to a substate
  // you can modify the initialization of the CurrentState variable
  // otherwise just start in the entry state every time the state machine
  // is started
//always enter entry state, with or without history
  CurrentState = ENTRY_STATE;

  // call the entry function (if any) for the ENTRY_STATE
  RunNavigationSM(CurrentEvent);
}

/****************************************************************************
 Function
     QueryNavSM

 Parameters
     None

 Returns
     NavState_t The current state of the nav state machine

 Description
     returns the current state of the nav state machine
 Notes

 Author
     gab
****************************************************************************/
NavState_t QueryNavSM(void)
{
  return CurrentState;
}

/***************************************************************************
 private functions
 ***************************************************************************/

static ES_Event_t DuringCheckLocations(ES_Event_t Event)
{
  ES_Event_t ReturnEvent = Event;   // assume no re-mapping or consumption

  // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
  if ((Event.EventType == ES_ENTRY) ||
      (Event.EventType == ES_ENTRY_HISTORY))
  {
    // query our permit locations from the SPUD
    ES_Event_t ThisEvent;
    ThisEvent.EventType = INFO_QUERY;
    ThisEvent.EventParam = ALL_OUR_PERMITS;
    PostSPUDHSM(ThisEvent);

  }
  else if (Event.EventType == ES_EXIT)
  {
    // no exit functionality
  }
  else
  // do the 'during' function for this state
  {
    // no during functionality
  }
  return ReturnEvent;
}

static ES_Event_t DuringOrient(ES_Event_t Event)
{
  ES_Event_t ReturnEvent = Event;   // assume no re-mapping or consumption

  // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
  if ((Event.EventType == ES_ENTRY) ||
      (Event.EventType == ES_ENTRY_HISTORY))
  {
    ES_Event_t ThisEvent;

    // Create an empty array to be populated with the accelerometer data
    int16_t CurrentVector[2];

    // Populates the Current vector with data from the accelerometer
    AcclFSM_QueryVector(CurrentVector);

    //Gets the Event that will rotated to the desired position
    ThisEvent = NavLib_GetOrientEvent(PermitLocation_Goal, 
	  TractorColor, CurrentVector);
    Drive_SetRotPosSpeed(ORIENT_SPEED);
    PostDriveService(ThisEvent);

	//query location of miner
    ES_Event_t OtherEvent;
    OtherEvent.EventType = INFO_QUERY;
    OtherEvent.EventParam = MINER_LOC_OURS;
    PostSPUDHSM(OtherEvent);

  }
  else if (Event.EventType == ES_EXIT)
  {
    // no exit functionality
  }
  else
  // do the 'during' function for this state
  {
    // no during functionality
  }

  return ReturnEvent;
}

static ES_Event_t DuringWait2Stabilize(ES_Event_t Event)
{
  ES_Event_t ReturnEvent = Event;

  // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
  if ((Event.EventType == ES_ENTRY) ||
      (Event.EventType == ES_ENTRY_HISTORY))
  {

    // make sure nav timer is stopped
    ES_Timer_StopTimer(NAVIGATION_TIMER);

    // start navigation timer with stabilizing wait
    ES_Timer_InitTimer(NAVIGATION_TIMER, STABILIZING_WAIT);
  }
  else if (Event.EventType == ES_EXIT)
  {}
  else
  // do the 'during' function for this state
  {}

  return ReturnEvent;
}

static ES_Event_t DuringQueryMiner(ES_Event_t Event)
{
  ES_Event_t ReturnEvent = Event;   // assume no re-mapping or consumption

  // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
  if ((Event.EventType == ES_ENTRY) ||
      (Event.EventType == ES_ENTRY_HISTORY))
  {
    // implement any entry actions required for this state machine

    //query location of miner
    ES_Event_t ThisEvent;
    ThisEvent.EventType = INFO_QUERY;
    ThisEvent.EventParam = MINER_LOC_OURS;
    PostSPUDHSM(ThisEvent);

  }
  else if (Event.EventType == ES_EXIT)
  {
    // no exit functionality
  }
  else
  // do the 'during' function for this state
  {
    // no during functionality
  }
  return ReturnEvent;
}

static ES_Event_t DuringMove2Color(ES_Event_t Event)
{
  ES_Event_t ReturnEvent = Event;   // assume no re-mapping or consumption

  // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
  if ((Event.EventType == ES_ENTRY) ||
      (Event.EventType == ES_ENTRY_HISTORY))
  {

    // start moving forward
    ES_Event_t ThisEvent;
    ThisEvent.EventType = EM_DRIVE_F;
    ThisEvent.EventParam = INCREMENT_VAL;
    PostDriveService(ThisEvent);

    // initialize current speed
    CurrentSpeed = INCREMENT_VAL;

    //start ramping Timer
    ES_Timer_InitTimer(RAMP_TIMER, INCREMENT_TIME);

		//start Wall Timer
    ES_Timer_InitTimer(WALL_TIMER, WALL_TIME);

		//query location of miner
    ES_Event_t OtherEvent;
    OtherEvent.EventType = INFO_QUERY;
    OtherEvent.EventParam = MINER_LOC_OURS;
    PostSPUDHSM(OtherEvent);



  }
  else if (Event.EventType == ES_EXIT)
  {
    // stop RAMP timer just in case
    ES_Timer_StopTimer(RAMP_TIMER);

		//stop wall timer
    ES_Timer_StopTimer(WALL_TIMER);



  }
  else
  // do the 'during' function for this state
  {
    // no during functionality
  }
  // return either Event, if you don't want to allow the lower level machine
  // to remap the current event, or ReturnEvent if you do want to allow it.
  return ReturnEvent;
}

static ES_Event_t DuringMovingFromMiner(ES_Event_t Event)
{
  ES_Event_t ReturnEvent = Event;   // assume no re-mapping or consumption

  // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
  if ((Event.EventType == ES_ENTRY) ||
      (Event.EventType == ES_ENTRY_HISTORY))
  {
    // release the miner
    MinerLib_Release();

    // Turn off Electromagnet
    MinerLib_DisableEM();

    //Drive Backwards a Specified number of ticks
    ES_Event_t ThisEvent;
    ThisEvent.EventType = EM_DRIVE_R_POS;
    ThisEvent.EventParam = BACK_AWAY_FROM_MINER_TICKS;
    PostDriveService(ThisEvent);

    // clear miner of interest
    SetMinerOfInterest(None);

    // Disable tap detection
    AcclFSM_DisableTap();

    // Update the current color the tractor is in
    Color_t CurrentColor = Query_Color();
    if ((CurrentColor == White) || (CurrentColor == Black) ||
      (CurrentColor == Unknown))
    {
      CurrentColor = Query_LastColor();
    }

  }
  else if (Event.EventType == ES_EXIT)
  {
    // no exit functionality
  }
  else
  // do the 'during' function for this state
  {
    // no during function in this state
  }
  return ReturnEvent;
}

/****************************************************************************
Function
    LookForClosestPermit

Parameters
    void

Returns
    Color_t (ID of Permit Location closest to the tractor)

Description
    Check current position of the tractor and compare with the position 
	of the Permits. Decides which one is the closest one.

Notes

Author
    G.Bravo, D.Petrakis 2/27/20
****************************************************************************/
static Color_t LookForClosestPermit(void)
{
  //by default place in our exclusive location
  Color_t ReturnVal = LocationsArray[0];

  //Determine position of tractor in terms of row/columns
  uint8_t tractor_col;
  uint8_t tractor_row;
  NavLib_GetRowColumn(TractorColor, &tractor_row, &tractor_col);

  //Determine position of the permits in terms of row columns
  uint8_t PERMIT1_col;
  uint8_t PERMIT1_row;
  NavLib_GetRowColumn(LocationsArray[0], &PERMIT1_row, &PERMIT1_col);

  uint8_t PERMIT2_col;
  uint8_t PERMIT2_row;
  NavLib_GetRowColumn(LocationsArray[1], &PERMIT2_row, &PERMIT2_col);

  uint8_t PERMIT3_col;
  uint8_t PERMIT3_row;
  NavLib_GetRowColumn(LocationsArray[2], &PERMIT3_row, &PERMIT3_col);

  uint8_t Distance[3];

  //Compute distance
  Distance[0] = (PERMIT1_row - tractor_row) * (PERMIT1_row - tractor_row) + 
  (PERMIT1_col - tractor_col) * (PERMIT1_col - tractor_col);
  Distance[1] = (PERMIT2_row - tractor_row) * (PERMIT2_row - tractor_row) + 
  (PERMIT2_col - tractor_col) * (PERMIT2_col - tractor_col);
  Distance[2] = (PERMIT3_row - tractor_row) * (PERMIT3_row - tractor_row) + 
  (PERMIT3_col - tractor_col) * (PERMIT3_col - tractor_col);

  //check my other miner color
  Color_t MyOtherMiner_Color;
  Miner_t MinerOfInterest = QueryMinerOfInterest();
  if (MinerOfInterest == OurMiner1ID())//if the miner of interest if miner1
  {
    //save the color of miner 2 as my other miner_color
	MyOtherMiner_Color = QueryMINERLocation(OurMiner2ID()); 
  }
  else  //else, my miner of interest is miner2
  {
    //save color of miner 1 as other miner color
	MyOtherMiner_Color = QueryMINERLocation(OurMiner1ID()); 
  }

  // Function to get the min of the distance values
  uint8_t MinDistance = 100;
  for (int i = 0; i < 3; i++)
  {
    //save target location if it is the smallest distance 
	//and if not the color of my other miner.
    if ((Distance[i] < MinDistance) && 
	  (MyOtherMiner_Color != LocationsArray[i]))
    {
      MinDistance = Distance[i];
      ReturnVal = LocationsArray[i];
    }
  }

  return ReturnVal;
}

/*------------------------------- Footnotes -------------------------------*/
// REMEMBER TO ADD THE NAVIGATION TIMER TO ES CONFIGURE
/*------------------------------ End of file ------------------------------*/
