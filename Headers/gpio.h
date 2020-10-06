#ifndef gpio_H
#define gpio_H

#include <stdint.h>
#include <stdbool.h>



typedef enum
{
  High = 0xff,
  Low = 0x00,
  Input,
  Output
}PinState_t;

bool GPIO_ConfigPin(char *PinName, PinState_t Dir);
bool GPIO_SetPin(char *PinName, PinState_t State);
uint8_t GPIO_ReadPin(char *PinName);

#endif //gpio_H
