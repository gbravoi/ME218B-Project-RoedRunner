/****************************************************************************
 Module
   DCMotorPWM.c

 Revision
   1.0.1

 Description
   The following module holds functions associated with intializing
   and confuiguring the TiVa M0PWM0 and M0PWM1 module to control the speed of
   the DC drive motors and the positions of the gripper servos

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 01/16/12       ram     intial release
 02/13/20				ram			Added code for the servos as well
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
// the common headers for C99 types
#include <stdint.h>
#include <stdbool.h>

// the headers to access the GPIO subsystem
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"

//header for PWM operations
#include "inc/hw_pwm.h"

// the headers to access the TivaWare Library
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "termio.h"

#include "BITDEFS.H"

/*----------------------------- Module Defines ----------------------------*/
#define NUM_MOTORS 2
#define NUM_SERVOS 2

#define TICKS_US 5
#define ALL_BITS (0xff << 2)
#define BYTE_3_M 0xff000000
#define SHIFT_6_NIBBLES_L(x) (x << 6)

#define LOAD_M HWREG(PWM0_BASE + PWM_O_0_LOAD)  // for motor period
#define LOAD_S HWREG(PWM0_BASE + PWM_O_1_LOAD)  // for servo period

#define GEN_A_NORM (PWM_0_GENA_ACTCMPAU_ONE | PWM_0_GENA_ACTCMPAD_ZERO)
#define GEN_B_NORM (PWM_0_GENB_ACTCMPBU_ONE | PWM_0_GENB_ACTCMPBD_ZERO)
#define INVERSION HWREG(PWM0_BASE + PWM_O_INVERT)
#define DIR_PORT HWREG(GPIO_PORTA_BASE + (GPIO_O_DATA + ALL_BITS))

//Adjust these values to get the range desired; count values, use tick_us conversion!
#define SERVO_MAX 12000 // Pseudo 180 deg
#define SERVO_MIN 3200  // Pseudo 0 deg
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this module.They should be functions
   relevant to the behavior of this module
*/
//none
/*---------------------------- Module Variables ---------------------------*/
static const uint16_t Period_us = 100;
static const uint16_t ServoPeriod_us = 20000;

//Array of constants holding normal behavior calls
static const uint32_t Gen_Normals[NUM_MOTORS] = { GEN_A_NORM, GEN_B_NORM };

// Array of constant holding address for each motor gen reg
static const uint32_t GenRegs[NUM_MOTORS] = { PWM_O_0_GENA, PWM_O_0_GENB };

// Array of constant holding address for each motor compare reg
static const uint32_t CompareRegs[NUM_MOTORS] = { PWM_O_0_CMPA, PWM_O_0_CMPB };

// Array of constant holding address for each servo compare reg
static const uint32_t ServoCompareRegs[NUM_SERVOS] =
{ PWM_O_1_CMPA, PWM_O_1_CMPB };

// Arrays holing generator behavoir constants for 0% and 100% DC
static const uint32_t DC_100_Select[NUM_MOTORS] =
{ PWM_0_GENA_ACTZERO_ONE, PWM_0_GENB_ACTZERO_ONE };
static const uint32_t DC_0_Select[NUM_MOTORS] =
{ PWM_0_GENA_ACTZERO_ZERO, PWM_0_GENB_ACTZERO_ZERO };

// Array of constants holding address for each generation inversion reg
static const uint32_t Invert[NUM_MOTORS] =
{ PWM_INVERT_PWM0INV, PWM_INVERT_PWM1INV };

// Array of constants holding direction info
static const uint32_t Direction[NUM_MOTORS] = { BIT7HI, BIT6HI };

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
    GeneralPWM_HWInit

 Parameters
   None

 Returns
   None

 Description
  Starts up the M0PWM0module and configures for center-aligned
  pulses. Sets up with a /8 for 200ns ticks (5ticks/us)
 Notes

 Author
   R. Merchant
****************************************************************************/
void GeneralPWM_HWInit(void)
{
  // Bring the M0PWM0 clock up
  HWREG(SYSCTL_RCGCPWM) |= SYSCTL_RCGCPWM_R0;
  // Set clock to 8 divider (200ns/tick)
  HWREG(SYSCTL_RCC) = (HWREG(SYSCTL_RCC) & ~SYSCTL_RCC_PWMDIV_M) |
      (SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_8);
  // Wait for the clock to be ready
  while ((HWREG(SYSCTL_PRPWM) & SYSCTL_PRPWM_R0) != SYSCTL_PRPWM_R0)
  {}

  // Start PORTA clock and wait
  HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R0;
  while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R0) != SYSCTL_PRGPIO_R0)
  {}

  // Start PORTB clock and wait
  HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R1;
  while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R1) != SYSCTL_PRGPIO_R1)
  {}

  // Disable PWM before configuring
  HWREG(PWM0_BASE + PWM_O_0_CTL) = 0;
  HWREG(PWM0_BASE + PWM_O_1_CTL) = 0;
  // Set rise to 1 and fall to 0 for cmp vals
  HWREG(PWM0_BASE + PWM_O_0_GENA) = GEN_A_NORM;
  HWREG(PWM0_BASE + PWM_O_0_GENB) = GEN_B_NORM;
  HWREG(PWM0_BASE + PWM_O_1_GENA) = GEN_A_NORM;
  HWREG(PWM0_BASE + PWM_O_1_GENB) = GEN_B_NORM;

  // Load in drive period (from module var)
  HWREG(PWM0_BASE + PWM_O_0_LOAD) = (Period_us / 2) * TICKS_US;
  // Load in servo period
  HWREG(PWM0_BASE + PWM_O_1_LOAD) = (ServoPeriod_us / 2) * TICKS_US;

  // Load in dummy 50% duty cycle
  HWREG(PWM0_BASE + PWM_O_0_CMPA) = LOAD_M >> 1;
  HWREG(PWM0_BASE + PWM_O_0_CMPB) = LOAD_M >> 1; //just to be safe
  // Load in servo min position
  HWREG(PWM0_BASE + PWM_O_1_CMPA) = LOAD_S - (SERVO_MIN / 2);
  HWREG(PWM0_BASE + PWM_O_1_CMPB) = LOAD_S - (SERVO_MIN / 2); //just to be safe

  // Configure PB4 PB5 PB6 amd PB7 to be M0PWM outputs
  // Start by setting bit 4 5 6 and  7 of AFSEL high
  HWREG(GPIO_PORTB_BASE + GPIO_O_AFSEL) |= (BIT4HI | BIT5HI | BIT6HI | BIT7HI);
  // Set 4th 5th 6th and 7th nibble of PCTL to 4 (per datasheet)
  HWREG(GPIO_PORTB_BASE + GPIO_O_PCTL) =
      ((HWREG(GPIO_PORTB_BASE + GPIO_O_PCTL) & 0x0000ffff) | (0x44440000));

  // Enable DIO mode on PB4 PB5 PB6 and PB7
  HWREG(GPIO_PORTB_BASE + GPIO_O_DEN) |= (BIT4HI | BIT5HI | BIT6HI | BIT7HI);
  ;
  // Set PB4 PB5 PB6 and PB7 to output dir
  HWREG(GPIO_PORTB_BASE + GPIO_O_DIR) |= (BIT4HI | BIT5HI | BIT6HI | BIT7HI);
  ;

  // Enable DIO mode on PA6 and PA7
  HWREG(GPIO_PORTA_BASE + GPIO_O_DEN) |= (BIT6HI | BIT7HI);
  // Set PA6 and PA7 to output dir
  HWREG(GPIO_PORTA_BASE + GPIO_O_DIR) |= (BIT6HI | BIT7HI);
  //Set PA7 and PA6 low
  HWREG(GPIO_PORTA_BASE + GPIO_O_DATA + ALL_BITS) &= (BIT6LO & BIT7LO);

  // Set up/down cnt mode, locally sync to 0cnt
  HWREG(PWM0_BASE + PWM_O_0_CTL) = (PWM_0_CTL_MODE | PWM_0_CTL_ENABLE |
      PWM_0_CTL_GENAUPD_LS | PWM_0_CTL_GENBUPD_LS);
  HWREG(PWM0_BASE + PWM_O_1_CTL) = (PWM_1_CTL_MODE | PWM_1_CTL_ENABLE |
      PWM_1_CTL_GENAUPD_LS | PWM_1_CTL_GENBUPD_LS);
}

/****************************************************************************
 Function
    DCMotorPWM_EnableOutputs/ServoPWM_EnableOutputs

 Parameters
   None

 Returns
   None

 Description
  Connects generators to pins

 Notes

 Author
   R. Merchant
****************************************************************************/
void DCMotorPWM_EnableOutputs(void)
{
  //Set enable bit high
  HWREG(PWM0_BASE + PWM_O_ENABLE) |= (PWM_ENABLE_PWM0EN | PWM_ENABLE_PWM1EN);
  return;
}

void ServoPWM_EnableOutputs(void)
{
  //Set enable bit high
  HWREG(PWM0_BASE + PWM_O_ENABLE) |= (PWM_ENABLE_PWM2EN | PWM_ENABLE_PWM3EN);
  return;
}

/****************************************************************************
 Function
    DCMotorPWM_SetDutyCycle

 Parameters
   MotorSel, DutyCycle

 Returns
   None

 Description
  Sets the duty cycle for the desired motor. Motors are numbered 0 and 1.

 Notes

 Author
   R. Merchant
****************************************************************************/
void DCMotorPWM_SetDutyCycle(uint8_t MotorSel, int16_t DutyCycle)
{
  // If dc = 0%
  if (DutyCycle == 0)
  {
    // set to 0 when cnt = 0
    HWREG(PWM0_BASE + GenRegs[MotorSel]) = DC_0_Select[MotorSel];
    // Don't invert
    INVERSION &= ~(Invert[MotorSel]);
    DIR_PORT &= ~(Direction[MotorSel]);
  }
  //If dc = 100%
  else if (DutyCycle > 99)
  {
    // Set to 1 when cnt = 0 (100% duty cycle)
    HWREG(PWM0_BASE + GenRegs[MotorSel]) = DC_100_Select[MotorSel];
    // Don't invert
    INVERSION &= ~(Invert[MotorSel]);
    DIR_PORT &= ~(Direction[MotorSel]);
  }
  //If dc = -100%
  else if (DutyCycle < -99)
  {
    // Set to 1 when cmp = 0 (100% duty cycle)
    HWREG(PWM0_BASE + GenRegs[MotorSel]) = DC_100_Select[MotorSel];
    // Write 0 into cmp
    HWREG(PWM0_BASE + CompareRegs[MotorSel]) = 0;
    // Invert
    INVERSION |= Invert[MotorSel];
    DIR_PORT |= Direction[MotorSel];
  }
  //end if
  // If cnt valid positive
  else if (DutyCycle > 0)
  {
    // Return to normal generator behavior
    HWREG(PWM0_BASE + GenRegs[MotorSel]) = Gen_Normals[MotorSel];
    // Convert duty cycle to cmp value
    uint16_t CMPval = (LOAD_M * (100 - DutyCycle)) / 100;
    //printf("\rCMPA: %u\r\n", CMPval);
    // write new compare value to register
    HWREG(PWM0_BASE + CompareRegs[MotorSel]) = CMPval;
    // Don't invert
    INVERSION &= ~(Invert[MotorSel]);
    DIR_PORT &= ~(Direction[MotorSel]);
  }
  // endif
  // If cnt valid negative
  else if (DutyCycle < 0)
  {
    // Return to normal generator behavior
    HWREG(PWM0_BASE + GenRegs[MotorSel]) = Gen_Normals[MotorSel];
    // Convert duty cycle to cmp value
    uint16_t CMPval = (LOAD_M * (100 + DutyCycle)) / 100;
    // write new compare value to register
    HWREG(PWM0_BASE + CompareRegs[MotorSel]) = CMPval;
    //Invert
    INVERSION |= Invert[MotorSel];
    DIR_PORT |= Direction[MotorSel];
  }
  //end if
}

/****************************************************************************
 Function
    ServoPWM_SetPosition

 Parameters
   ServoSel, Postion

 Returns
   None

 Description
  Sets the pulse width for the desired position. Position is an "angle" from
  0 to 180.

 Notes
  The min and max pulse values will need to be adjusted to get the
  desired range of motion.
 Author
   R. Merchant
****************************************************************************/
bool ServoPWM_SetPosition(uint8_t ServoSel, uint8_t Position)
{
  // Set return flag to false
  bool ReturnFlag = false;
  // If angle <=180 go ahead and proceed
  if (Position <= 180)
  {
    // convert angle to a pulse width value
    uint16_t PulseWidth = (((SERVO_MAX - SERVO_MIN) * Position) / 180)
        + SERVO_MIN;
    // calculate needed compare value to get pulse width
    uint16_t CMPval = LOAD_S - (PulseWidth / 2);
    // Load value into compare reg
    HWREG(PWM0_BASE + ServoCompareRegs[ServoSel]) = CMPval;
    // Set return flag to true
    ReturnFlag = true;
  }
  // endif
  return ReturnFlag;
}

/***************************************************************************
 private functions
 ***************************************************************************/
// none
/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/
