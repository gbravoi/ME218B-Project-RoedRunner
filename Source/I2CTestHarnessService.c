/****************************************************************************
 Module
   I2CTestHarnessService.c

 Revision
   1.0.1

 Description
   This is the first service for the Test Harness under the
   Gen2 Events and Services Framework.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 10/26/17 18:26 jec     moves definition of ALL_BITS to ES_Port.h
 10/19/17 21:28 jec     meaningless change to test updating
 10/19/17 18:42 jec     removed referennces to driverlib and programmed the
                        ports directly
 08/21/17 21:44 jec     modified LED blink routine to only modify bit 3 so that
                        I can test the new new framework debugging lines on PF1-2
 08/16/17 14:13 jec      corrected ONE_SEC constant to match Tiva tick rate
 11/02/13 17:21 jec      added exercise of the event deferral/recall module
 08/05/13 20:33 jec      converted to test harness service
 01/16/12 09:58 jec      began conversion from TemplateFSM.c
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/

// Hardware
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"

// Event & Services Framework
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_DeferRecall.h"
#include "ES_ShortTimer.h"
#include "ES_Port.h"

// Our application
#include "I2CService.h"
#include "Color.h"

// This module
#include "TestHarnessService0.h"

/*----------------------------- Module Defines ----------------------------*/
// these times assume a 1.000mS/tick timing
#define ONE_SEC 1000
#define HALF_SEC (ONE_SEC / 2)
#define TWO_SEC (ONE_SEC * 2)
#define FIVE_SEC (ONE_SEC * 5)

#define NUM_COLORS 18 //total number of valid colors on the field
#define RGB_SIZE 3
#define TOLERANCE 150 // tolerance on closeness to color element
#define MAX_VAL 255 // max color value for rgb standard


#define H_INDEX 0
#define S_INDEX 1
#define V_INDEX 2

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/
Color_t NavLib_ColorDecodeRGB(uint16_t *ptrRGBCvals);

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
uint16_t RGBCvals[4];
uint16_t RGB255[3];
uint16_t HSV[3];
	// set return value to uknown color
	Color_t ReturnCurrentColor = Unknown;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitTestHarnessI2C

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and does any
     other required initialization for this service
 Notes

 Author
     J. Edward Carryer, 02/03/19, 10:46
****************************************************************************/
bool InitTestHarnessI2C(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  // post the initial transition event
  ThisEvent.EventType = ES_INIT;
  if (ES_PostToService(MyPriority, ThisEvent) == true)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/****************************************************************************
 Function
     PostTestHarnessI2C

 Parameters
     ES_Event ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
J. Edward Carryer, 02/03/19, 10:48
****************************************************************************/
bool PostTestHarnessI2C(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunTestHarnessI2C

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes

 Author
J. Edward Carryer, 02/03/19, 10:49
****************************************************************************/
ES_Event_t RunTestHarnessI2C(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no error 
  
  switch (ThisEvent.EventType)
  {
    case ES_INIT:
    {
      ES_Timer_InitTimer(I2C_TEST_TIMER, (ONE_SEC));
      puts("I2C TestHarness:");
      printf("\rES_INIT received in Service %d\r\n", MyPriority);
    }
    break;
    case ES_TIMEOUT:   // re-start timer & announce
    {
      
      ES_Timer_InitTimer(I2C_TEST_TIMER, HALF_SEC);
      RGBCvals[C_INDEX] = I2C_GetClearValue();
      RGBCvals[R_INDEX]   = I2C_GetRedValue();
      RGBCvals[G_INDEX] = I2C_GetGreenValue();
      RGBCvals[B_INDEX]  = I2C_GetBlueValue();
      
      
      #if 0
      RGBCvals[R_INDEX] = 698;
      RGBCvals[G_INDEX] = 744;
      RGBCvals[B_INDEX] = 661;
      RGBCvals[C_INDEX] = 2162;
      #endif
      
      #if 0 // Tests Norm Values
      Color_t ColorNorm;
      ColorNorm = ColorNorm_Decode();
      printf("Color \t\t\t\t\t%i     \n\r", ColorNorm);
      #endif
      
      #if 0 // Tests HSV Values
      SensorRGB_2_RGB255(RGBCvals, RGB255);
      //printf("%i \t%i \t%i      \n\r", RGB255[R_INDEX],RGB255[G_INDEX],RGB255[B_INDEX]);
      RGB2HSV(RGB255, HSV);
      printf("\t\t\t%i \t\t%i \t\t%i      \n\r", HSV[H_INDEX],HSV[S_INDEX],HSV[V_INDEX]);
      #endif
      
      #if 0 // Test full HSV Color Decode
          Color_t ColorHSV;
          ColorHSV = Query_Color();
          printf("Color \t\t\t\t\t%i     \n\r", ColorHSV);
          ES_Timer_InitTimer(I2C_TEST_TIMER, (ONE_SEC));
      #endif
      
      //ReturnCurrentColor = NavLib_ColorDecode(RGBCvals);
      //printf("%d\n\r", ReturnCurrentColor);
      
//      printf("Clr: %d, Red: %d, Grn: %d, Blu: %d, R%%: %.2f, G%% %.2f, B%% %.2f \r\n",
//          ClearValue, RedValue, GreenValue, BlueValue, 
//          ((float)RedValue*100/ClearValue),
//          ((float)GreenValue*100/ClearValue),
//          ((float)BlueValue*100/ClearValue));
//      printf("\t%.2f\t \t%.2f\t \t%.2f\t \r\n", 
//          ((float)RedValue*255/ClearValue),
//          ((float)GreenValue*255/ClearValue),
//          ((float)BlueValue*255/ClearValue));
    }
    break;
    case ES_NEW_KEY:   // announce
    {
      printf("ES_NEW_KEY received with -> %c <- in Service 0\r\n",
          (char)ThisEvent.EventParam);
    }
    break;
    default:
    {}
     break;
  }

  return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/


/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

