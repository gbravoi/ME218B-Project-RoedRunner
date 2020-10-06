#ifndef NavigationLibrary_H
#define NavigationLibrary_H

#include "TypesDefinition.h"

bool NavLib_GetRowColumn (Color_t Region, uint8_t *ptrRow, uint8_t *ptrCol);
ES_Event_t NavLib_GetOrientEvent(Color_t TargetRegion, 
               Color_t CurrentRegion, int16_t* pCurVec);
               
bool NavLib_CheckPath(Color_t Region);
void Query_DesiredVector(int16_t *pNewDesiredVec);

#endif /*NavigationLibrary_H*/

