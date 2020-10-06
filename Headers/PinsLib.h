/****************************************************************************
Header File for Pins Library
****************************************************************************/

#ifndef PinsLib_H
#define PinsLib_H

#include "TypesDefinition.h"

//public function prototypes
void InitPins(void);
void TurnOnMiningStatus(void);
void TurnOffMiningStatus(void);
Team_t ReadTeam(void);
#endif /*PinsLib_H*/
