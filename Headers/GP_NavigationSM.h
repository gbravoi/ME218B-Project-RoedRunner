/****************************************************************************
 Template header file for Hierarchical Sate Machines AKA StateCharts
 02/08/12 adjsutments for use with the Events and Services Framework Gen2
 3/17/09  Fixed prototpyes to use Event_t
 ****************************************************************************/

#ifndef GP_NavigationSM_H
#define GP_NavigationSM_H


// typedefs for the states
// State definitions for use with the query function
typedef enum { 
  CheckLocations,
  Wait2Stabilize, 
  Orient, 
  QueryMiner, 
  MovingAwayFromMiner, 
  Move2Color 
} NavState_t ;


// Public Function Prototypes

ES_Event_t RunNavigationSM( ES_Event_t CurrentEvent );
void StartNavigationSM ( ES_Event_t CurrentEvent );
NavState_t QueryNavSM ( void );

#endif /*GP_NavigationSM_H */

