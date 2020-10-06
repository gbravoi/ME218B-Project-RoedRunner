/****************************************************************************
 Template header file for MasterGameHSM

 ****************************************************************************/

#ifndef MasterGameHSM_H
#define MasterGameHSM_H

#include "TypesDefinition.h"
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// State definitions for use with the query function
typedef enum { Waiting4Permits, PlayingGame, HandlingCollision } MasterState_t ;

// Public Function Prototypes

ES_Event_t RunMasterGameHSM( ES_Event_t CurrentEvent );
void StartMasterGameHSM ( ES_Event_t CurrentEvent );
bool PostMasterGameHSM( ES_Event_t ThisEvent );
bool InitMasterGameHSM( uint8_t Priority );
Team_t QueryTeam(void);
Miner_t QueryMinerOfInterest(void);
void SetMinerOfInterest(Miner_t MinerID);
Miner_t OurMiner1ID(void);
Miner_t OurMiner2ID(void);
Miner_t TheirMiner1ID(void);
Miner_t TheirMiner2ID(void);


#endif /*MasterGameHSMe_H */
