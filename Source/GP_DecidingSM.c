/****************************************************************************
 Module
   GP_DecidingSM.c

 Revision
   1

 Description
   Decide next movement based on position of MINERS and the locations with permits.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 02/23/20				gab				first code
****************************************************************************/
#pragma diag_suppress 550
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"

#include "GP_DecidingSM.h"
#include "SPUDHSM.h"
#include "GP_MasterGameHSM.h"
#include "TypesDefinition.h"
#include "NavigationLibrary.h"
#include "Color.h"



/*----------------------------- Module Defines ----------------------------*/
// define constants for the states for this machine
// and any other local defines


/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine, things like during
   functions, entry & exit functions. They should be functions relevant to the
   behavior of this state machine
*/
static ES_Event_t DuringCheckMinerPosition( ES_Event_t Event);
static ES_Event_t DuringCheckPermitLocation( ES_Event_t Event);
static Miner_t LookForClosestMiner(void);


/*---------------------------- Module Variables ---------------------------*/

// everybody needs a state variable, you may need others as well
static DecidingState_t CurrentState;


//color location miners
static Color_t OurMINER1_Color;
static Color_t OurMINER2_Color;

//permits locations
static Color_t MyPermits[3];

//keep tract if miners are in unknow location
static bool MyMINER1_UNK;
static bool MyMINER2_UNK;


//keep track if miners where located
static bool MyMINER1_LOCATED;
static bool MyMINER2_LOCATED;
static bool OUR_MINERS_DIF_LOC;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
    RunDecidingSM

 Parameters
   ES_Event CurrentEvent: the event to process

 Returns
   ES_Event

 Description
   Implements the deciding machine for the MasterGameHSM

 Author
   G. Bravo, 2/23/20
****************************************************************************/
ES_Event_t RunDecidingSM( ES_Event_t CurrentEvent )
{
   bool MakeTransition = false;/* are we making a state transition? */
   DecidingState_t NextState = CurrentState;
   ES_Event_t ReturnEvent = CurrentEvent; // assume we are not consuming event

   switch ( CurrentState )
   {
		 case CheckMinerPosition :
		 {
         // Call during function of the state
         DuringCheckMinerPosition( CurrentEvent);

         if ( CurrentEvent.EventType != ES_NO_EVENT ) //If an event is active
         {
           if(CurrentEvent.EventType == MINER_LOC_UPDATE_OURS ){//Miner location of my team was updated
            //query the color of our miners
             OurMINER1_Color=QueryMINERLocation(OurMiner1ID());
						 OurMINER2_Color=QueryMINERLocation(OurMiner2ID());

						//check if miners are in a unknow location
						 MyMINER1_UNK=(OurMINER1_Color==White || OurMINER1_Color==Black || OurMINER1_Color==Unknown);
						 MyMINER2_UNK=(OurMINER2_Color==White || OurMINER2_Color==Black || OurMINER2_Color==Unknown);

						 // if we know where either miner is, go to CheckPermitLocation
						 if ((!MyMINER1_UNK) || (!MyMINER2_UNK))
						 {
							 	NextState = CheckPermitLocation;//Decide what the next state will be
                MakeTransition = true; //mark that we are taking a transition
                ReturnEvent.EventType = ES_NO_EVENT; // consume this event
						 }
						 else //if we don't know the location of both miners, always go for miner 1
						 {
							 NewEvent.EventParam=OurMiner1ID();
							 //post to GameState machine
							 PostMasterGameHSM(NewEvent);
							 ReturnEvent.EventType = ES_NO_EVENT; // consume this event
						 }
					 }
	 }
 }

		          break;

       case CheckPermitLocation :{
			// Call during function of the state
         DuringCheckPermitLocation(CurrentEvent);

		     //new event to post
		 			ES_Event_t NewEvent;
				  NewEvent.EventType=GO_TO_MINER;
         if ( CurrentEvent.EventType != ES_NO_EVENT ) //If an event is active
         {
           if(CurrentEvent.EventType==ALL_OUR_PERMITS_UPDATE){//if permits location of our team was updated
					ReturnEvent.EventType = ES_NO_EVENT; // consume this event
					//save permits locations in array
					QueryOursLocations(MyPermits);

					//check if MINERS are in a permitted zone
					MyMINER1_LOCATED=(OurMINER1_Color==MyPermits[0] || OurMINER1_Color==MyPermits[1] || OurMINER1_Color==MyPermits[2]);
					MyMINER2_LOCATED=(OurMINER2_Color==MyPermits[0] || OurMINER2_Color==MyPermits[1] || OurMINER2_Color==MyPermits[2]);

					//check if both of my miners in different location
				  OUR_MINERS_DIF_LOC=(OurMINER1_Color!=OurMINER2_Color);

					//if both miners in correct position
					if(MyMINER1_LOCATED && MyMINER2_LOCATED){
						//if they are in different locations return to check miner position
						if(OUR_MINERS_DIF_LOC){
							 NewEvent.EventType=ES_NO_EVENT;//no post
							NextState = CheckMinerPosition;//Decide what the next state will be
              MakeTransition = true; //mark that we are taking a transition
						}else{//else implies they are in same location, move miner 1 to another location
						  NewEvent.EventParam=OurMiner1ID();
						}
						 }else if(MyMINER1_LOCATED){//if only 1 is located, relocate 2
								NewEvent.EventParam=OurMiner2ID();
						 }else if(MyMINER2_LOCATED){//if only 2 is located, relocate 1
								NewEvent.EventParam=OurMiner1ID();
						 }else
						 {//else both MINERS in known location, but bad located, decide which one to go based on proximity

						 NewEvent.EventParam=LookForClosestMiner();

						 }

						//post event
						 //post to GameState machine if there is an new event
						 if(NewEvent.EventType!=ES_NO_EVENT){
						PostMasterGameHSM(NewEvent);

						 }


					 }
					 }


			 }
			     break;
		 }
	 //   If we are making a state transition
    if (MakeTransition == true)
    {
       //   Execute exit function for current state
       CurrentEvent.EventType = ES_EXIT;
       RunDecidingSM(CurrentEvent);

       CurrentState = NextState; //Modify state variable

       //   Execute entry function for new state
       CurrentEvent.EventType = ES_ENTRY;
       RunDecidingSM(CurrentEvent);
     }
     return(ReturnEvent);
	 }
	 /****************************************************************************
 Function
     StartDecidingSM

 Parameters
     ES_Event CurrentEvent

 Returns
     None

 Description
     Does any required initialization for this state machine
 Notes

 Author
     G.Bravo, 2/23/20
****************************************************************************/
void StartDecidingSM  ( ES_Event_t CurrentEvent )
{
   CurrentState = CheckMinerPosition;  // always start in CheckMinerPosition

   // call the entry function (if any) for the ENTRY_STATE
   RunDecidingSM(CurrentEvent);
}

/***************************************************************************
 private functions
 ***************************************************************************/

static ES_Event_t DuringCheckMinerPosition( ES_Event_t Event)
{

    // process ES_ENTRY & ES_EXIT events
    if ( (Event.EventType == ES_ENTRY) ||
         (Event.EventType == ES_ENTRY_HISTORY) )
    {
			// implement any entry actions required for this state machine

			//Query miners location
			//create event to query SPUD
			ES_Event_t NewEvent;
			NewEvent.EventType=INFO_QUERY;
			NewEvent.EventParam=MINER_LOC_OURS;
			//post event
			PostSPUDHSM(NewEvent);

    }else if ( Event.EventType == ES_EXIT)
    {
        // no exist functionality

    }else
    // do the 'during' function for this state
    {
        // no during function for this machine
    }
    return ( Event ); // Don't remap this event
}


static ES_Event_t DuringCheckPermitLocation( ES_Event_t Event)
{
    // process ES_ENTRY & ES_EXIT events
    if ((Event.EventType == ES_ENTRY)|| (Event.EventType == ES_ENTRY_HISTORY))
    {
			//Query permitted locations
			ES_Event_t NewEvent;
			NewEvent.EventType=INFO_QUERY;
			NewEvent.EventParam=ALL_OUR_PERMITS;
			//post event
			PostSPUDHSM(NewEvent);

    }else if ( Event.EventType == ES_EXIT)
    {
        // no exist functionality

    }else
    // do the 'during' function for this state
    {
        // no during function for this machine
    }
    return ( Event ); // Don't remap this event
}


 /****************************************************************************
 Function
     LookForClosestMiner

 Parameters
     void

 Returns
     Miner_t (ID of miner closest to the tractor)

 Description
     Check current position of the tractor and compare with the position of the Miners
Decide which one is the closest one.

 Notes

 Author
     G.Bravo, 2/23/20
****************************************************************************/
static Miner_t LookForClosestMiner(void){
	Miner_t ReturnVal;
//check in which region we are based on the color sensor
	 Color_t TractorColor;
	 //Use a query function to know tractor color (from color SM)
	TractorColor=Query_Color();

	 //Determine position of tractor in terms of row/columns
	 uint8_t tractor_col;
	 uint8_t tractor_row;
	 NavLib_GetRowColumn (TractorColor, &tractor_row, &tractor_col);

	 //Determine position of the Miner in terms of row columns
	 uint8_t MINER1_col;
	 uint8_t MINER1_row;
	 NavLib_GetRowColumn (OurMINER1_Color, &MINER1_row, &MINER1_col);

	 uint8_t MINER2_col;
	 uint8_t MINER2_row;
	 NavLib_GetRowColumn (OurMINER2_Color, &MINER2_row, &MINER2_col);


	//Compute distance
	uint8_t Distance1=(MINER1_row-tractor_row)*(MINER1_row-tractor_row)+(MINER1_col-tractor_col)*(MINER1_col-tractor_col);
	uint8_t Distance2=(MINER2_row-tractor_row)*(MINER2_row-tractor_row)+(MINER2_col-tractor_col)*(MINER2_col-tractor_col);

	//decide which one is closest
	if(Distance1 < Distance2){
	ReturnVal=OurMiner1ID();

	}else{
	ReturnVal=OurMiner2ID();
	}

	return ReturnVal;

}
