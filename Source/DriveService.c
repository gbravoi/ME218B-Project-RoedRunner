/****************************************************************************
 Module
  MotorService.c

 Revision
   1.0.2

 Description


 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 01/31/20 11:30 gab     firstcode
 02/04/20 08:55 ram		Modified for use in the project. Added PI control ISRs
                      and initialization functions for WTIMER5.
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this service
*/
#include "DriveService.h"
#include "EncoderLib.h"
#include "GeneralPWM.h"

/* include header files for hardware access
*/
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_timer.h"
#include "inc/hw_nvic.h" \

#include "gpio.h"

#include "bitdefs.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* include header files for the framework
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_DeferRecall.h"
#include "ES_ShortTimer.h"

/*----------------------------- Module Defines ----------------------------*/
#define ONE_SEC 1000 // this time assume a 1.000mS/tick timing
#define NUM_DRIVE_MOTORS 2

#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1

#define FORWARD 1
#define REVERSE -1
#define CLOCKWISE 1
#define CCLOCKWISE (-1)

#define POS_ROT_SPEED 20    // fixed turning speed
#define POS_DRIVE_SPEED 25  // fixed position speed
#define POS_MOTOR 1         // motor to track position on

//For controller
#define KP0 0.91f
#define KI0 0.015f

#define KP1 1.0f
#define KI1 0.015f

#define CNTRL_TIME_MS 2
#define TICKS_MS 40000

#define TOGGLE_ON() HWREG(GPIO_PORTE_BASE + GPIO_O_DATA + (BIT2HI << 2)) = BIT2HI
#define TOGGLE_OFF() HWREG(GPIO_PORTE_BASE + GPIO_O_DATA + (BIT2HI << 2)) = BIT2LO

/*---------------------------- Module Functions ---------------------------*/
static void DriveService_HWInit(void);
static void DriveService_EnableTimer(void);

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t        MyPriority;
static int16_t        TargetRPM[NUM_DRIVE_MOTORS];
static volatile float Kp[NUM_DRIVE_MOTORS] = { KP0, KP1 };
static volatile float Ki[NUM_DRIVE_MOTORS] = { KI0, KI1 };

static bool           PositionMode = false;
static uint32_t       PositionTarget;
static uint32_t       PositionStart;

static int16_t        PosRotSpeed = 20;
static int16_t        PosDriveSpeed = 25;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitDriveService

 Parameters
  Service priority

 Returns


 Description
     Saves away the priority, and does any
     other required initialization for this service
 Notes

 Author
    Gabriela Bravo, 01/31/20
****************************************************************************/
bool InitDriveService(uint8_t Priority)
{
  ES_Event_t ThisEvent;
  //bool InitStatus = true;
  //Initialize the MyPriority variable with the passed in parameter.
  MyPriority = Priority;
  // Set PE2 as the debug pin
  GPIO_ConfigPin("PE2", Output);
  //intialize timer for control law
  DriveService_HWInit();

  //Start with motors stopped
  DCMotorPWM_SetDutyCycle(MOTOR_LEFT,   0);
  DCMotorPWM_SetDutyCycle(MOTOR_RIGHT,  0);
  DCMotorPWM_EnableOutputs();

  // Init encoder start timers
  EncLib_StartHWTimers();

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
     PostMotorService

 Parameters
     EF_Event ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     Gabriela B
****************************************************************************/
bool PostDriveService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunDriveService

 Parameters

 Returns
   ES_NO_EVENT

 Description


 Notes

 Authors
   Gabriela Bravo, Riyaz Merchant
****************************************************************************/
ES_Event_t RunDriveService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT;

  // Cast event param to signed int
  int16_t ParamVal = (int16_t)ThisEvent.EventParam;

  switch (ThisEvent.EventType)
  {
    case ES_INIT:
    {
      // Enable timer for control law ISR
      DriveService_EnableTimer();
    }
    break;
    case EM_STOP:   // Want tractor to stop
    {
      // Set RPM for both motors to 0
      TargetRPM[MOTOR_LEFT] = 0;
      TargetRPM[MOTOR_RIGHT] = 0;
      // Get out of poistion mode
      PositionMode = false;
    }
    break;
    case EM_AT_POS:   // Want tractor to stop at position
    {
      //Set target RPM to 0 for both motors
      TargetRPM[MOTOR_LEFT] = 0;
      TargetRPM[MOTOR_RIGHT] = 0;
    }
    break;
    case EM_DRIVE_F:   //Drive in a straight line forward
    {
      //Set target RPM to param for both motors
      TargetRPM[MOTOR_LEFT] = FORWARD * ParamVal;
      TargetRPM[MOTOR_RIGHT] = FORWARD * ParamVal;
      // Get out of poistion mode
      PositionMode = false;
    }
    break;

    case EM_DRIVE_R:   //Drive in a straight line backwards
    {
      //Set target RPM to param for both motors
      TargetRPM[MOTOR_LEFT] = REVERSE * ParamVal;
      TargetRPM[MOTOR_RIGHT] = REVERSE * ParamVal;
      // Get out of poistion mode
      PositionMode = false;
    }
    break;
    case EM_ROT_CCW:   // rotate counter clockwise around motor center
    {
      TargetRPM[MOTOR_LEFT] = CCLOCKWISE * ParamVal;
      TargetRPM[MOTOR_RIGHT] = -1 * CCLOCKWISE * ParamVal;
      // Get out of poistion mode
      PositionMode = false;
    }
    break;
    case EM_ROT_CW:   //rotate clockwise about motor center
    {
      TargetRPM[MOTOR_LEFT] = CLOCKWISE * ParamVal;
      TargetRPM[MOTOR_RIGHT] = -1 * CLOCKWISE * ParamVal;
      // Get out of poistion mode
      PositionMode = false;
    }
    break;
    case EM_SET_L:   // Only adjust target on left motor
    {
      TargetRPM[MOTOR_LEFT] = ParamVal;
      // Get out of poistion mode
      PositionMode = false;
    }
    break;
    case EM_SET_R:   // Only adjust target on right motor
    {
      TargetRPM[MOTOR_RIGHT] = ParamVal;
      // Get out of poistion mode
      PositionMode = false;
    }
    break;
    case EM_ROT_CCW_POS:   // Rotate a fixed amount and stop
    {
      TargetRPM[MOTOR_LEFT] = CCLOCKWISE * PosRotSpeed;
      TargetRPM[MOTOR_RIGHT] = -1 * CCLOCKWISE * PosRotSpeed;
      PositionTarget = ParamVal;
      // Get starting count from encoder
      EncData_t ThisData;
      EncLib_GetData(POS_MOTOR, &ThisData);
      PositionStart = ThisData.Count;
      // Set service into position mode
      PositionMode = true;
    }
    break;
    case EM_ROT_CW_POS:   // Rotate a fixed amount and stop
    {
      TargetRPM[MOTOR_LEFT] = CLOCKWISE * PosRotSpeed;
      TargetRPM[MOTOR_RIGHT] = -1 * CLOCKWISE * PosRotSpeed;
      PositionTarget = ParamVal;
      // Get starting count from encoder
      EncData_t ThisData;
      EncLib_GetData(POS_MOTOR, &ThisData);
      PositionStart = ThisData.Count;
      // Set service into position mode
      PositionMode = true;
    }
    break;
    case EM_DRIVE_F_POS:
    {
      TargetRPM[MOTOR_LEFT] = FORWARD * PosDriveSpeed;
      TargetRPM[MOTOR_RIGHT] = FORWARD * PosDriveSpeed;
      PositionTarget = ParamVal;
      // Get starting count from encoder
      EncData_t ThisData;
      EncLib_GetData(POS_MOTOR, &ThisData);
      PositionStart = ThisData.Count;
      // Set service into position mode
      PositionMode = true;
    }
    break;

    case EM_DRIVE_R_POS:
    {
      TargetRPM[MOTOR_LEFT] = REVERSE * PosDriveSpeed;
      TargetRPM[MOTOR_RIGHT] = REVERSE * PosDriveSpeed;
      PositionTarget = ParamVal;
      // Get starting count from encoder
      EncData_t ThisData;
      EncLib_GetData(POS_MOTOR, &ThisData);
      PositionStart = ThisData.Count;
      // Set service into position mode
      PositionMode = true;
    }
    break;
    default:
    {
      ;
    }
  }

  //Return ES_NO_EVENT
  return ReturnEvent;
}

/****************************************************************************
 Function
     Drive_SetDrivePosSpeed/Drive_SetRotPosSpeed

 Parameters
  Desired RPM

 Returns
  Nothing

 Description
    Sets speed value for position control when driving/rotating
 Notes

 Author
    R. Merchant
****************************************************************************/
void Drive_SetDrivePosSpeed(int16_t DesiredRPM)
{
  PosDriveSpeed = DesiredRPM;
}

void Drive_SetRotPosSpeed(int16_t DesiredRPM)
{
  PosRotSpeed = DesiredRPM;
}

/***************************************************************************
 Service ISR functions
 ***************************************************************************/
/****************************************************************************
 ISR Name
     DriveService_PIctrlISR

 Description
   Performs the needed calculations to control the RPM of the motor using a
   linear PI controller. Each motor has its own Kp and Ki which can be
   adjusted at the top of the file in the definition statements. ISR also
   handles position control when the service is in position mode.
 Notes

 Author
    R. Merchant, 02/04/20
****************************************************************************/
void DriveService_PIctrlISR(void)
{
  // Remember the summed error
  static int16_t SummedError[NUM_DRIVE_MOTORS] = { 0, 0 };

  //static for speed
  static int16_t    RPMerror;
  static EncData_t  ThisData;
  static int16_t    SetPWM;

  static uint32_t   dCount;
  static ES_Event_t PosEvent = { EM_AT_POS, 0 };

   #ifdef DEBUG
  TOGGLE_ON();
   #endif

  // Clear interrupt flag
  HWREG(WTIMER5_BASE + TIMER_O_ICR) = TIMER_ICR_TATOCINT;
  // Loop through both motors
  uint8_t index;
  for (index = 0; index < NUM_DRIVE_MOTORS; index++)
  {
    // Query encoder data
    EncLib_GetData(index, &ThisData);
    // If RPM is not 0
    if (TargetRPM[index] != 0)
    {
      // Find RPR error (target - actual)
      RPMerror = TargetRPM[index] - ThisData.RPM;
      // Track total error (total += current)
      SummedError[index] += RPMerror;
      // Get PWM value (ki*(error+ki*summmederror))
      SetPWM = (int16_t)(Kp[index] * (RPMerror + Ki[index] * SummedError[index]));
    }
    // If RPM is 0
    else
    {
      // Simply lower the pins
      SetPWM = 0;
      // squash the accumulated error
      SummedError[index] = 0;
    }
    // endif
    // if PWM value > 100
    if (SetPWM > 100)
    {
      // Set to 100
      SetPWM = 100;
      // Subract off error from summed error
      SummedError[index] -= RPMerror;
    }
    // else if PWM value < -100
    else if (SetPWM < -100)
    {
      // Set to -100
      SetPWM = -100;
      // Subtract off error from summed error
      SummedError[index] -= RPMerror;
    }
    //end if

    // Set duty cycle
    DCMotorPWM_SetDutyCycle(index, SetPWM);
  }
  // If service is in position mode
  if (PositionMode)
  {
    // Get the current delta count
    dCount = ThisData.Count - PositionStart;
    if (dCount >= PositionTarget)
    {
      // Post motor at position
      PositionMode = false;
      //Post to GP and drive service
      ES_PostList00(PosEvent);
    }
  }
   #ifdef DEBUG
  TOGGLE_OFF();
   #endif
}

/***************************************************************************
 private functions
 ***************************************************************************/
/****************************************************************************
 Function
     InitMotorService

 Parameters
  none

 Returns
  Nothing

 Description
    Sets up WTIMER5 for periodic interrupt to execute the PI control law for
  speed control of the drive motors.
 Notes

 Author
    R. Merchant, 02/04/20
****************************************************************************/
static void DriveService_HWInit(void)
{
  // Start clock to WTIMER5
  HWREG(SYSCTL_RCGCWTIMER) |= SYSCTL_RCGCWTIMER_R5;
  while ((HWREG(SYSCTL_PRWTIMER) & SYSCTL_PRWTIMER_R5) != SYSCTL_PRWTIMER_R5)
  {}

  // disable tmr
  HWREG(WTIMER5_BASE + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
  // Set TAIRL to poll period;
  HWREG(WTIMER5_BASE + TIMER_O_TAILR) = (CNTRL_TIME_MS * TICKS_MS) - 1;
  // Set up timer to count down (TACDIR = 0); periodic mode (TAMR = 0x2)
  HWREG(WTIMER5_BASE + TIMER_O_TAMR) =
      (HWREG(WTIMER5_BASE + TIMER_O_TAMR) &
      (~TIMER_TAMR_TAAMS & ~TIMER_TAMR_TACDIR)) |
      TIMER_TAMR_TAMR_PERIOD;

  // Clear pending timeout interrupt
  HWREG(WTIMER5_BASE + TIMER_O_ICR) = TIMER_ICR_TAMCINT;
  // Enable timout event interrrupt mask
  HWREG(WTIMER5_BASE + TIMER_O_IMR) |= TIMER_IMR_TATOIM;

  // Set priority for tmr interrupt to 1 (don't miss edges!)
  HWREG(NVIC_PRI26) = (HWREG(NVIC_PRI26) & ~NVIC_PRI26_INTA_M) |
      (1 << NVIC_PRI26_INTA_S);
  // Enable NVIC_EN3 bit 8
  HWREG(NVIC_EN3) = BIT8HI;
  // Enable interrupts globally
  __enable_irq();
  return;
}

static void DriveService_EnableTimer(void)
{
  //Set TAEN high
  HWREG(WTIMER5_BASE + TIMER_O_CTL) |= TIMER_CTL_TAEN;
  return;
}
