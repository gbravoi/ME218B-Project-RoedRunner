/****************************************************************************
Header file for GamePlayStrategySM
 ****************************************************************************/

#ifndef GamePlayStrategy_H
#define GamePlayStrategy_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum { Deciding, GettingMiner, Navigating } GamePlayStrategyState_t ;


// Public Function Prototypes

ES_Event_t RunGamePlayStrategySM( ES_Event_t CurrentEvent );
void StartGamePlayStrategySM ( ES_Event_t CurrentEvent );
GamePlayStrategyState_t QueryGamePlayStrategySM ( void );

#endif /*GamePlayStrategy_H */
