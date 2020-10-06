/****************************************************************************
 Template header file for Hierarchical Sate Machines AKA StateCharts
 02/08/12 adjsutments for use with the Events and Services Framework Gen2
 3/17/09  Fixed prototpyes to use Event_t
 ****************************************************************************/
#pragma diag_suppress 1
#ifndef SPUDHSM_H
#define SPUDHSM_H

#include "ES_Framework.h"
#include "TypesDefinition.h"


// typedefs for the states
// State definitions for use with the query function
typedef enum { Waiting4InfoQuery, QueryingStatus, QueryingOther } SPUDHSMState_t ;

// SPUD status definitions
typedef enum { W4P, PI, SD, PE } SPUDStatus_t ;

// info_query param definitions
typedef enum { STATUS, NEUTRAL_PERMITS,
							MINER_LOC_OURS, MINER_LOC_THEIRS, 
							RES_OURS, RES_THEIRS, 
							EX_PERMIT_OURS, EX_PERMIT_THEIRS, 
							ALL_OUR_PERMITS, ALL_THEIR_PERMITS } InfoParam_t ;


// Public Function Prototypes 
bool InitSPUDHSM(uint8_t Priority);
ES_Event_t RunSPUDHSM( ES_Event_t CurrentEvent );
void StartSPUDHSM ( ES_Event_t CurrentEvent );
bool PostSPUDHSM(ES_Event_t ThisEvent);
SPUDHSMState_t QuerySPUDHSM( void );
void QueryOursLocations(Color_t * permits);
void QueryTheirsLocations(Color_t * permits);
Color_t QueryMINERLocation(Miner_t MinerID);


#endif /*SPUDHSM_H */
              