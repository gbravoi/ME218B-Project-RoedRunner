/****************************************************************************
 Template header file for Hierarchical Sate Machines AKA StateCharts

 ****************************************************************************/

#ifndef GP_Deciding_H
#define GP_Deciding_H


// State definitions for use with the query function
typedef enum { CheckMinerPosition, CheckPermitLocation } DecidingState_t ;


// Public Function Prototypes

ES_Event_t RunDecidingSM( ES_Event_t CurrentEvent );
void StartDecidingSM ( ES_Event_t CurrentEvent );

#endif /*MW_Timer_H */
