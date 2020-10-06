/****************************************************************************
 Template header file for Hierarchical Sate Machines AKA StateCharts

 ****************************************************************************/

#ifndef GP_Collision_H
#define GP_Collision_H


// State definitions for use with the query function
typedef enum { WaitingCollision, MoveAway1,Rotate,MoveAway2} CollisionState_t ;


// Public Function Prototypes

ES_Event_t RunCollisionSM( ES_Event_t CurrentEvent );
void StartCollisionSM ( ES_Event_t CurrentEvent );

#endif /*GP_Collision_H */

