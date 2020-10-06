#ifndef ColorLib_H
#define ColorLib_H

#include "TypesDefinition.h"

//Public Functions
Color_t Query_Color(void);
Color_t DecodeHSV(void);

// Event Checker
bool ColorChanged(void);

// Enable/Disable ColorChanged Event Checker
void EnableColorEventChecker(void);
void DisableColorEventChecker(void);

Color_t Query_LastColor(void);

#endif /*ColorLib_H*/
