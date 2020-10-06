/****************************************************************************
 Module
   Navigation Library.c

 Revision
   1.0.3

 Description
   Functions for purpose of orienting the TRACTOR. Includes low level
	 functions to determine region, and calculate needed heading. 
 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
02/15/2020      ram      initial release
02/23/2020      ram     Over haul of function organization to make lib more
                        user friendly. Reduction of public functions.
02/26/2020      ram     Removed all function associated with color. Said
                        functions will live in the color library instead
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
// the common headers for C99 types
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// the headers to access the GPIO subsystem
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"

#include "BITDEFS.H"
#include "gpio.h"
#include "termio.h"


// framework
#include "ES_Port.h"
#include "ES_Framework.h"

#include "NavigationLibrary.h"

/*----------------------------- Module Defines ----------------------------*/
// For angle lookup table
#define SIN 1
#define COS 2
#define ANGLE_RES 1000

// for acceleration vectors
#define X_INDEX 0
#define Y_INDEX 1

// for calculating needed orientation
#define PI 3.14159265f
#define TURN_R_MM 118
#define WHEEL_C_MM 236
#define TICKS_PER_REV 150

//#define DEBUG // enable/disable printing

typedef struct Path
{
  int8_t dRow;
  int8_t dCol;
  uint8_t tRow;
  uint8_t tCol; 
} Path_t;
  
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this library.They should be functions
   relevant to the behavior of this library.
*/
static bool NavLib_CalcOrientation(Color_t TargetRegion, Color_t CurrentRegion);
static float Vector2Angle(int16_t *pVec);
static int16_t NavLib_GetTicks(int16_t* pTarVec, int16_t* pCurVec);

static int16_t DesiredVector[2];

/*---------------------------- Module Variables ---------------------------*/

static const uint16_t AngleTable[9][3] = 
{
  {0,0,1000}, 
  {333, 316, 949},
  {500, 447, 894},
  {666, 555, 832},
  {1000, 707, 707},
  {1500, 832, 555},
  {2000, 894, 447},
  {3000, 949, 316},
  {0xFF, 1000, 0}
};

static uint8_t maxAccl = 10; // calibrate this value!

Path_t PathData;

/*------------------------------ Module Code ------------------------------*/

/****************************************************************************
 Function
    GetRowColumn

 Parameters
   color, pointer to row value, pointer to col value

 Returns
   status

 Description
   Takes the color region value and calculated the row and column value for
   said region. To facilitate singular return, the function requires a 
   pointer for the row and column value to be provided to the function
   as an argument.

 Notes
   When getting the row and column, make sure the POINTER for the value is
   passed to the function, NOT the value, otherwise a segmentation fault 
   will occur. Use the "&" operator
   
   Example call: status = NavLib_GetRowColumn(Region, &Row, &Column);
   function will return values to Row and Column, they must be declared
   before calling the function!
   
 Author
   R. Merchant
****************************************************************************/

bool NavLib_GetRowColumn (Color_t Region, uint8_t *ptrRow, uint8_t *ptrCol)
{
   // set status flag false
   bool status = false;
   //if valid region
   if((uint16_t)Region < 16)
   {
      // calculate column by getting the modulus of 4
      *ptrCol = (uint16_t)Region % 4;
      // calculate row by dividing by 4 and flooring
      *ptrRow = (uint16_t)Region / 4;
      // Set status falg true
      status = true;
   }

   return (status);
}
/****************************************************************************
 Function
    NavLib_GetOrientEvent

 Parameters
   Target Region (where you want to go), CurrentRegion (where you are),
   Current orientation

 Returns
   Event to post to drive service

 Description
   Generates the needed event to post to drive service to get the tractor
   into the orientation needed to drive to the target region.
 Notes

 Author
   R. Merchant
****************************************************************************/

ES_Event_t NavLib_GetOrientEvent(Color_t TargetRegion, 
               Color_t CurrentRegion, int16_t* pCurVec)
{
  // allocate variables
  ES_Event_t OrientEvent;
  // Get the needed target accleration vector
  NavLib_CalcOrientation(TargetRegion, CurrentRegion);

  #ifdef DEBUG 
  printf("Target: %i, %i\r\n", DesiredVector[0], DesiredVector[1]);
  #endif

  // Get the needed ticks to orient to the target vector
  int16_t Ticks = NavLib_GetTicks(DesiredVector, pCurVec);
  // set event parameter to the absolute value of ticks
  OrientEvent.EventParam = abs(Ticks);

  #ifdef DEBUG
  printf("Tickss: %i\r\n", Ticks);
  #endif
  
  // if the tick count if positive, turn CCW
  if(Ticks >= 0)
  {
    OrientEvent.EventType = EM_ROT_CCW_POS;
  }
  // else turn CW
  else
  {
    OrientEvent.EventType = EM_ROT_CW_POS;
  }
  // endif
  // return event
  return OrientEvent;  
}

/****************************************************************************
 Function
    NavLib_CheckPath

 Parameters
   Current region tractor is in

 Returns
   Path good bool

 Description
   fucntion sees if the current region the tractor is in is okay given the
  path it should be on. If the tractor overshoots the row and/or column of
  the target, the function returns false, saying the tractor is off course.
 Notes

 Author
   R. Merchant
****************************************************************************/
bool NavLib_CheckPath(Color_t Region)
{
  // allocate column and row flag
  bool ColFlag = true;
  bool RowFlag = true;
  // get the row and column of the region
  uint8_t row, col;
  NavLib_GetRowColumn(Region, &row, &col);
  
  // start with the columns
  // If dCol < 0 and  col < target col
  if((PathData.dCol < 0) && (col <= PathData.tCol))
  {
    //set column flag false
    ColFlag = false;
  }
  // If dCol > 0 and target col > target col
  else if((PathData.dCol > 0) && (col >= PathData.tCol))
  {
    // set column flag false
    ColFlag = false;
  }
  // If dCol = 0 and col =/= target col
  else if((PathData.dCol == 0) && (col != PathData.tCol))
  {
    ColFlag = false;
  }
  
  //Now the rows
  // If dRow < 0 and  row < target row
  if((PathData.dRow < 0) && (row <= PathData.tRow))
  {
    //set row flag false
    RowFlag = false;
  }
  // If dRow > 0 and  row > target row
  else if((PathData.dRow > 0) && (row >= PathData.tRow))
  {
    // set row flag false
    RowFlag = false;
  }
  // If dRow = 0 and row =/= target row
  else if((PathData.dRow == 0) && (row != PathData.tRow))
  {
    RowFlag = false;
  }
  
  // make sure both row and column are good
  return (ColFlag & RowFlag);
}
  

/****************************************************************************
 Function
    Query_DesiredVector

 Parameters
   int16_t *NewDesiredVec

 Returns
	 None

 Description
	query function for desired vector

 Author
   rpt
****************************************************************************/
void Query_DesiredVector(int16_t *pNewDesiredVec)
{
  //Populate to given vector
  *(pNewDesiredVec + X_INDEX) = DesiredVector[0];
  *(pNewDesiredVec + Y_INDEX) = DesiredVector[1];
}

 
 /***************************************************************************
 private functions
 ***************************************************************************/
 /****************************************************************************
 Function
    NavLib_CalcOrientation

 Parameters
   Target Region (where you want to go), Current Region (where are you now)
   pointer to acceleration vector

 Returns
   Status of the caluclation

 Description
   Function determines the geometric properties of the straight line path 
   between the target and current reagions and generates the acceleration 
   vector the tractor needs to check against to orient itself toward the 
   target.
 Notes
   To get the acceleration vector, allocate the vector in the function
   that call this one
 Author
   R. Merchant
****************************************************************************/

static bool NavLib_CalcOrientation(Color_t TargetRegion, Color_t CurrentRegion)
{
   // set status false
   bool funcStatus = false;
  // Allocate
   int16_t dCol, dRow;
   uint8_t tableIndex = 10;
   // Get the row column indices for the target
   uint8_t tRow, tCol;
   bool tStatus = NavLib_GetRowColumn(TargetRegion, &tRow, &tCol);
  // store to path data
  PathData.tRow = tRow;
  PathData.tCol = tCol;
   // Get the row column indices for the current
   uint8_t cRow, cCol;
   bool cStatus = NavLib_GetRowColumn(CurrentRegion, &cRow, &cCol);
   // if the regions are valid
   if(tStatus && cStatus)
   {
      // Calculate dRow and dCol
      dRow = tRow - cRow;
      dCol = tCol - cCol;
     // store info in object
     PathData.dCol = dCol;
     PathData.dRow = dRow;
     
      #ifdef DEBUG
      printf("dCol: %i\tdRow: %i\n", dCol, dRow);
      #endif
     
      // if delta column = 0
      if(dCol == 0)
         {
         // Set the lookup index to 8
         tableIndex = 8;
         }
      // else
      else
      {
         // Get the ratio of |row|:|col| out of 1000
         uint16_t ratio = (1000*abs(dRow))/abs(dCol);
        printf("Ratio: %u\r\n", ratio);

         // Determine the index from the lookup table
         switch(ratio)
         {
            case 0:
               tableIndex = 0;
            break;
            
            case 333:
               tableIndex = 1;
            break;
            
            case 500:
               tableIndex = 2;
            break;
            
            case 666:
               tableIndex = 3;
            break;
            
            case 1000:
               tableIndex = 4;
            break;
            
            case 1500:
               tableIndex = 5;
            break;
            
            case 2000:
               tableIndex = 6;
            break;
            
            case 3000:
               tableIndex = 7;
            break;
         }
      }
      // use table values to calculate ax and ay desired
      uint8_t magax = (maxAccl*AngleTable[tableIndex][COS])/ANGLE_RES;
      uint8_t magay = (maxAccl*AngleTable[tableIndex][SIN])/ANGLE_RES;
      //printf("magax: %i\t magay: %i\r\n", magax, magay);
      // apply sign based on sign of col and row diff
      int16_t a_x = (dCol < 0) ? magax: (-1)*magax;
      int16_t a_y = (dRow < 0) ? magay: (-1)*magay;
      //printf("ax: %i\t ay: %i\r\n\n", a_x, a_y);
      // Populate acceleration vector accordingly
      DesiredVector[X_INDEX] = a_x;
      DesiredVector[Y_INDEX] = a_y;
      
      // set function status true
      funcStatus = true;
   }
   // endif
   // return status
   return funcStatus;
}

/****************************************************************************
 Function
    Vector2Angle

 Parameters
   Vector array (must have 2 values, x and y comp)

 Returns
   The angle the vector makes with the East axis

 Description
   Uses atan to calculate the angle the input vector makes from the east
   axis.Quadtrant definitions -> I: NE, II:NW, III: SW, IV: SE
 Notes

 Author
   R. Merchant
****************************************************************************/	 
static float Vector2Angle(int16_t *pVec)
{
   // dereference component from vector
   int16_t x_comp = *(pVec + X_INDEX);
   int16_t y_comp = *(pVec + Y_INDEX);
  
  #ifdef DEBUG
   printf("x-comp: %i\ty-comp: %i\r\n", x_comp, y_comp);
  #endif
   
   // sign adjust to match mathematical quadrants
   // calculate arctan
   float angle = atan2(y_comp, (-1)*(x_comp)); 
   return angle;
}
/****************************************************************************
 Function
    NavLib_GetTicks

 Parameters
   Target Vector, Current Vector

 Returns
   tick count

 Description
   Determines the angle between the 2 orientation vecotrs and determines
   the best direction of travel to reach the target orientation. Returns
   the tick amount for the motors to turn.
 Notes
   Direction is determined by direction! Positive tick values mean turing
   counter-clockwise, while negative is clockwise!
 Author
   R. Merchant
****************************************************************************/	
static int16_t NavLib_GetTicks(int16_t* pTarVec, int16_t* pCurVec)
{
  // calculate dtheta
  float CurTheta = Vector2Angle(pCurVec);
  float TarTheta = Vector2Angle(pTarVec);
  float dTheta = TarTheta - CurTheta;
  
  #ifdef DEBUG
  printf("TarTheta: %0.2f\tCurTheta: %0.2f\r\n", TarTheta, CurTheta);
  printf("dtheta :%0.2f\r\n", dTheta);
  #endif

  // Optamize the turning directiona and distance
  
  // If dtheta < 0 and is < a half-circle
  if ((dTheta < 0) && (fabs(dTheta) < PI))
  {
    // Rotate CW be |dTheta|
    dTheta = (-1)*fabs(dTheta);
  }
  // If dtheta < 0 and is > a half-circle
  else if ((dTheta < 0) && (fabs(dTheta) > PI))
  {
    // Rotate CCW be 2pi - |dTheta|, the complement
    dTheta = (2*PI) - fabs(dTheta);
  }
  // If dtheta > 0 and is < half-circle
  else if ((dTheta > 0) && (fabs(dTheta) < PI))
  {
    // Rotate CCW be |dTheta|
    dTheta = fabs(dTheta);
  }
  // If dtheta > 0 and is > half-circle
  else if ((dTheta > 0) && (fabs(dTheta) > PI))
  {
    // Rotate CW be 2pi - |dTheta|, the complement
    dTheta = (-1)*(2*PI - fabs(dTheta));
  }
  // calculate dS from r and dtheta
  int16_t dS = TURN_R_MM*dTheta;
  // calculate the number of ticks for dS
  int16_t rotateTicks = (dS*TICKS_PER_REV)/WHEEL_C_MM;
  return rotateTicks;
}
  
 
 /*------------------------------- Footnotes -------------------------------*/
/* Designed for drive motor encoders.
   Make sure to add ISR labels to startup file for vector placement
*/
/*------------------------------ End of file ------------------------------*/
