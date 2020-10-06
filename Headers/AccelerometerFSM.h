/****************************************************************************

  Header file for template Flat Sate Machine
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef AccelerometerFSM_H
#define AccelerometerFSM_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum
{
  InitAcclState, 
  InitializingMeter, 
  SendingQueryBytes,
  SendingCmdBytes,
  Waiting4Action
}AccelerometerState_t;

// Public Function Prototypes

bool InitAccelerometerFSM(uint8_t Priority);
bool PostAccelerometerFSM(ES_Event_t ThisEvent);
ES_Event_t RunAccelerometerFSM(ES_Event_t ThisEvent);
AccelerometerState_t QueryAccelerometerSM(void);

void AcclFSM_QueryVector(int16_t* vector);

void AcclFSM_EnableTap(void);
void AcclFSM_DisableTap(void);

bool Check4Data(void);
bool Check4Tap(void);

//ISRs
void AcclFSM_EOTISR(void);

#endif /* AccelerometerFSM_H */
