/****************************************************************************
Header File for GeneralPWM Library
****************************************************************************/

#ifndef GeneralPWM_H
#define GeneralPWM_H

//Public Function Prototypes
void GeneralPWM_HWInit(void);
void DCMotorPWM_EnableOutputs(void);
void ServoPWM_EnableOutputs(void);

void DCMotorPWM_SetDutyCycle(uint8_t MotorSel, int16_t DutyCycle);
bool ServoPWM_SetPosition(uint8_t ServoSel, uint8_t Position);

#endif /*GeneralPWM_H*/

