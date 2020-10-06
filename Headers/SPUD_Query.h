/****************************************************************************
 Template header file for Hierarchical Sate Machines AKA StateCharts

 ****************************************************************************/

#ifndef SPUD_Query_H
#define SPUD_Query_H

#include <stdint.h>
#include <stdbool.h>

#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */
#include "ES_Events.h"


// State definitions for use with the query function
typedef enum { Waiting4Query, SendingBytes, Wait2SendNextByte} SPUDQueryState_t ;


// Public Function Prototypes
void StartSPUDQuerySM ( ES_Event_t CurrentEvent );
ES_Event_t  RunSPUDQuerySM ( ES_Event_t CurrentEvent );
void SPUD_EOTISR(void);





#endif /*SPUD_Query */
