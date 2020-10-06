/****************************************************************************
Header File for Miner Library
****************************************************************************/

#ifndef Miner_H
#define Miner_H

#include "TypesDefinition.h"

//public function prototypes
void Miner_HWInit(void);
void StartFindingMiner(Miner_t MinerName);
void StopFindingMiner(void);

// functions for electomagnet
void MinerLib_EnableEM(void);
void MinerLib_DisableEM(void);

// functions for grabbing
void MinerLib_Grab(void);
void MinerLib_Release(void);
bool MinerLib_QueryGrab(void);

//Event checkers
bool Check4Beacon(void);
bool Check4MinerRing(void);
 
//ISRs
void MinerLib_BeaconISR(void);

//Queries miner number
Miner_t Query_Miner_Number(void);

#endif /*Miner_H*/

