/****************************************************************************
Header File for Encoder Library
****************************************************************************/
#ifndef EncoderLib_H
#define EncoderLib_H

// Pulblic Definitions
typedef struct EncData
{
	int32_t RPM;
	uint16_t Count;
} EncData_t;

// Pulblic Function Prototypes
void EncLib_InitCapture(void);
void EncLib_Init1ShotTimers(void);

void EncLib_StartHWTimers(void);

void EncLib_GetData(uint8_t Which, EncData_t* pData);

// ISRs
void EncLib_Capture0ISR(void);
void EncLib_Capture1ISR(void); 
void EncLib_StallISR0(void);
void EncLib_StallISR1(void);

#endif /*EncoderLib_H*/
