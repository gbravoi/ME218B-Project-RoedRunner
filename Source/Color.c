/****************************************************************************
 Module
   Color.c

 Revision
   1.0.1

 Description
   Functions for purpose of decoding the color the tractor is in
 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
02/15/2020      ram     initial release
02/26/2020		dgp		branched off old navlib to create separate color lib.
            Created HSV decoding method.
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

#include "Color.h"
#include "I2CService.h"
#include "GP_MasterGameHSM.h"

/*----------------------------- Module Defines ----------------------------*/
#define NUM_COLORS 18 //total number of valid colors on the field
#define RGB_SIZE 3
#define H_TOLERANCE 5   // Tolerance for Hue
#define SV_TOLERANCE 3  // Tolerance for Saturation and value
#define TOLERANCE 3     // tolerance on closeness to color element
#define MAX_VAL 255     // max color value for rgb standard

#define H_INDEX 0
#define S_INDEX 1
#define V_INDEX 2

//#define DEBUG // enable/disable printing
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this library.They should be functions
   relevant to the behavior of this library.
*/
static void SensorRGB_2_RGB255(uint16_t *ptrRGBCvals, uint16_t *ptrRGB255);
static void RGB2HSV(uint16_t *ptrRGB255, uint16_t *ptrHSV);

/*---------------------------- Module Variables ---------------------------*/

// Variable used for public query function
Color_t         ReturnColor;
// Statically allocates last color and current color for event checker
static Color_t  CurrentColor;
static Color_t  LastColor;
static bool     ColorChangedActive = true;

// Values Calibrated on the field
// 2D array of HSV values for each color square
// Values determined empirically
static const uint16_t ColorMapHSV [NUM_COLORS][RGB_SIZE] =
{
  { 345, 8, 36 },           // Region  0 Lavender
  { 5, 47, 49 },            // Region  1 Maroon
  { 205, 41, 42 },          // Region  2 Blue
  { 88, 23, 38 },           // Region  3 Mint
  { 41, 37, 40 },           // Region  4 Olive
  { 47, 29, 38 },           // Region  5 Beige
  { 10, 64, 56 },           // Region  6 Orange
  { 345, 43, 46 },          // Region  7 Magenta
  { 26, 40, 42 },           // Region  8 Apricot
  { 2, 67, 62 },            // Region  9 Red
  { 322, 27, 40 },          // Region 10 Purple
  { 170, 31, 38 },          // Region 11 Cyan
  { 271, 13, 36 },          // Region 12 Navy
  { 54, 49, 40 },           // Region 13 Lime
  { 17, 51, 48 },           // Region 14 Brown
  { 35, 59, 46 },           // Region 15 Yellow
  { 10000, 10000, 10000 },  // White
  { 0, 0, 0 },              // Black
};

/*------------------------------ Module Code ------------------------------*/

/****************************************************************************
 Function
    DecodeHSV

 Parameters
   None

 Returns
   Region where the color is

 Description
   Takes in the RBGC array from the color sensor, converts them to 0-255 RGB
   values, then converts those to HSV values.  Compares the Measured HSV value
   to a predetermined list of HSV values to return the correct color.
 Notes

 Author
   Dimitri Petrakis
****************************************************************************/
Color_t DecodeHSV(void)
{
  uint16_t RGBCvals[4]; // Sets Array of size 4 for R, G, B, and Clear Values

  // Uses the Getter functions from the I2C service to populate the current colors into the array
  RGBCvals[R_INDEX] = I2C_GetRedValue();
  RGBCvals[G_INDEX] = I2C_GetGreenValue();
  RGBCvals[B_INDEX] = I2C_GetBlueValue();
  RGBCvals[C_INDEX] = I2C_GetClearValue();

  // start a decoding counter to loop through the array
  uint8_t decodeCnt = 0;

  // set return value to uknown color
  Color_t ReturnColor = Unknown;

  // set up an error values
  int16_t hError; // red error
  int16_t sError; // green error
  int16_t vError; // blue error

  // Sets up an array for the next function to populate
  uint16_t RGB255[RGB_SIZE];

  // Converts the sensor RGB values to a 0-255 scale
  SensorRGB_2_RGB255(RGBCvals, RGB255);

  // Sets up an array for the next function to populate
  uint16_t HSV[RGB_SIZE];

  // Converts the 0-255 RGB values to HSV
  RGB2HSV(RGB255, HSV);

  #ifdef DEBUG
  printf("\t\t\t%i \t\t%i \t\t%i      \n\r",
      HSV[H_INDEX], HSV[S_INDEX], HSV[V_INDEX]);
  #endif

  //start loop to decode
  for (decodeCnt = 0; decodeCnt < NUM_COLORS; decodeCnt++)
  {
    // Get the red error
    hError = HSV[H_INDEX] - ColorMapHSV[decodeCnt][H_INDEX];
    #ifdef DEBUG
    printf("hue err: %i\n\r", hError);
    #endif
    // if the red value matches
    if (abs(hError) < H_TOLERANCE)
    {
      // Get the green error
      sError = HSV[S_INDEX] - ColorMapHSV[decodeCnt][S_INDEX];
      #ifdef DEBUG
      printf("saturation err: %i\n\r", sError);
      #endif
      //If green error within tolerance
      if (abs(sError) < SV_TOLERANCE)
      {
        // Get the blue error
        vError = HSV[V_INDEX] - ColorMapHSV[decodeCnt][V_INDEX];
        #ifdef DEBUG
        printf("value err: %i\n\r", vError);
        #endif
        // If blue error within tolerance
        if (abs(vError) < SV_TOLERANCE)
        {
          // set return value to the decoder counter
          ReturnColor = (Color_t)decodeCnt;
          // set decodeCnt to the limit to breakout
          decodeCnt = NUM_COLORS;
        }
        //endif
      }
      //endif
    }
    //endif
    #ifdef DEBUG
    printf("\n\n");
    #endif
  }
  //end loop
  // return
  return ReturnColor;
}

/*----------------------------- Public Functions ---------------------------*/

// Public Query function that returns the region that the robot is in
Color_t Query_Color(void)
{
  ReturnColor = DecodeHSV();

  return ReturnColor;
}

// Event Checker to determine if a color has changed
//    Ignores White, Black, and Unknown
bool ColorChanged(void)
{
  if (ColorChangedActive == true)
  {
    // Gets Current Color
    CurrentColor = DecodeHSV();

    // Check if Color isn't White, isn't Black, isn't Unkown
    // and the color changed from the last value
    if (((CurrentColor < White) && (LastColor < White)) &&
        (CurrentColor != LastColor))
    {
      LastColor = CurrentColor;
      // post color changed event
      ES_Event_t ColorEvent;
      ColorEvent.EventType = EV_NEW_COLOR;
      ColorEvent.EventParam = LastColor;
      PostMasterGameHSM(ColorEvent);
      return true;
    }
  }
  return false;
}

void EnableColorEventChecker(void)
{
  ColorChangedActive = true;

  return;
}

void DisableColorEventChecker(void)
{
  ColorChangedActive = false;

  return;
}

Color_t Query_LastColor(void)
{
  return LastColor;
}

/***************************************************************************
private functions
***************************************************************************/
/****************************************************************************
Function
   SensorRGB_2_RGB255

Parameters
  Populated array of RGBC vals, Empty array of RGB255 values that will be
  populated by the function

Returns
  None

Description
  Takes the RGB values from the color sensor ranging 0-65535, normalizes
  by the clear value(total of RGB values) and multiplies by 255 to get
  a 0-255 RGB value
Notes

Author
  Dimitri Petrakis
****************************************************************************/
static void SensorRGB_2_RGB255(uint16_t *ptrRGBCvals, uint16_t *ptrRGB255)
{
  // Dereference the clear value
  uint16_t  ClearVal = *(ptrRGBCvals + C_INDEX);
  // Start loop to go throgh each color element
  uint8_t   i = 0;
  for (i = 0; i < RGB_SIZE; i++)
  {
    // Convert the ith element to a standard RGB value 0-255
    // Assumes Max_Val = 255
    *(ptrRGB255 + i) = (uint16_t)((*(ptrRGBCvals + i) * MAX_VAL) / ClearVal);
    //printf("%i\n\r", RGB255[i]);
  } // stop when all elements converted
  return;
}

/****************************************************************************
 Function
    RGB2HSV

 Parameters
   Populated array of 0-255 RGB vals, Empty array of HSV values that will
   be populated by the function

 Returns
   None

 Description
   Takes the 0-255 RGB values and converts them to HSV values
 Notes

 Author
   Dimitri Petrakis
****************************************************************************/
static void RGB2HSV(uint16_t *ptrRGB255, uint16_t *ptrHSV)
{  // h, s, v = hue, saturation, value
  // Dereferences the values and
  float R = *(ptrRGB255 + R_INDEX);
  float G = *(ptrRGB255 + G_INDEX);
  float B = *(ptrRGB255 + B_INDEX);

  // normalizes from 0-255 to 0-1
  R = R / MAX_VAL;
  G = G / MAX_VAL;
  B = B / MAX_VAL;

  float RGBNorm[3] = { R, G, B };

  // Function to get the max of the RGB values
  float Cmax = 0;
  for (int i = 0; i < RGB_SIZE; i++)
  {
    // Check current array element against established max
    Cmax = (RGBNorm[i] > Cmax) ? RGBNorm[i] : Cmax;
  }

  // Function to get the min of the RGB values
  float Cmin = 10000;
  for (int i = 0; i < RGB_SIZE; i++)
  {
    // Check current array element against established min
    Cmin = (RGBNorm[i] < Cmin) ? RGBNorm[i] : Cmin;
  }

  // diff of cmax and cmin.
  float Diff = Cmax - Cmin;

  // if cmax and cmax are equal then h = 0
  if (Cmax == Cmin)
  {
    *(ptrHSV + H_INDEX) = 0;
  }
  // if cmax equal r then compute h
  else if (Cmax == R)
  {
    *(ptrHSV + H_INDEX) = (uint16_t)(60 * ((G - B) / Diff) + 360) % 360;
  }
  // if cmax equal g then compute h
  else if (Cmax == G)
  {
    *(ptrHSV + H_INDEX) = (uint16_t)(60 * ((B - R) / Diff) + 120) % 360;
  }
  // if cmax equal b then compute h
  else if (Cmax == B)
  {
    *(ptrHSV + H_INDEX) = (uint16_t)(60 * ((R - G) / Diff) + 240) % 360;
  }
  else
  {
    #ifdef DEBUG
    printf("Bro\n\r");
  #endif
  }

  // if cmax equal zero
  if (Cmax == 0)
  {
    *(ptrHSV + S_INDEX) = 0;
  }
  else
  {
    *(ptrHSV + S_INDEX) = (Diff / Cmax) * 100;
  }

  // compute v
  *(ptrHSV + V_INDEX) = Cmax * 100;

  return;
}

/*------------------------------- Footnotes -------------------------------*/

/*------------------------------ End of file ------------------------------*/
