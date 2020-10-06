/****************************************************************************

  Header file for template Flat Sate Machine
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef BumperFSM_H
#define BumperFSM_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum{
  InitBumperState, 
  Waiting4Press, 
  DebouncingFront,
  DebouncingRear
}BumperState_t;

// Public Function Prototypes

bool InitBumperFSM(uint8_t Priority);
bool PostBumperFSM(ES_Event_t ThisEvent);
ES_Event_t RunBumperFSM(ES_Event_t ThisEvent);
BumperState_t QueryBumperSM(void);

bool Check4FrontBump(void);
bool Check4RearBump(void);

#endif /* BumperFSM_H */
