/****************************************************************************
 Common definitions used by different SM
 ****************************************************************************/

#ifndef TYPESDEF_H
#define TYPESDEF_H

#include "ES_Framework.h"

// team definitions 
typedef enum {CHK = 0, GHI = 1} Team_t ;

//miners definition
typedef enum{
	CHK_Miner1 = 0,
	CHK_Miner2 = 1,
	GHI_Miner1 = 2,
	GHI_Miner2 = 3,
	None = 5
} Miner_t; 

//Regions defined
// enumeration for regions
typedef enum
{
	Region0 = 0, Region1 = 1, Region2 = 2, Region3 = 3,
	Region4 = 4, Region5 = 5, Region6 = 6, Region7 = 7,
	Region8 = 8, Region9 = 9, Region10 = 10, Region11 = 11,
	Region12 = 12, Region13 = 13, Region14 = 14, Region15 = 15,
	White = 16, Black = 17, Unknown=18
}Color_t; //Each declared to be safe!

//types of collitions
typedef enum
{
	Coll_Rear, Coll_Front
}Collision_t; //Each declared to be safe!

//enum for color indicies for standardization
typedef enum
{
	R_INDEX = 0, G_INDEX = 1, 
	B_INDEX = 2, C_INDEX = 3
}ColorIndex_t;


#endif /*TYPESDEF_H */

