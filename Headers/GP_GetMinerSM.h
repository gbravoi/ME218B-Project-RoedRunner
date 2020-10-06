/****************************************************************************
Header file for GetMinerSM
 ****************************************************************************/

#ifndef GetMiner_H
#define GetMiner_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */
#include "Miner.h"

// typedefs for the states
// State definitions for use with the query function
typedef enum { AligningWithMiner, Repositioning, RampingUp, Moving2Miner, GrabbingMiner, CheckGrabbedMiner } GetMinerState_t ;


// Public Function Prototypes

ES_Event_t RunGetMinerSM( ES_Event_t CurrentEvent );
void StartGetMinerSM ( ES_Event_t CurrentEvent );
GetMinerState_t QueryGetMinerSM ( void );
void SetMinerOfInterest(Miner_t SomeMiner);

#endif /*GetMiner_H */

