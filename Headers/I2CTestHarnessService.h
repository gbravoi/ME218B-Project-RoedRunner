/****************************************************************************

  Header file for Test Harness I2C Service
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef I2CTestHarnessService_H
#define I2CTestHarnessService_H

#include <stdint.h>
#include <stdbool.h>

#include "ES_Events.h"

// Public Function Prototypes

bool InitTestHarnessI2C(uint8_t Priority);
bool PostTestHarnessI2C(ES_Event_t ThisEvent);
ES_Event_t RunTestHarnessI2C(ES_Event_t ThisEvent);

#endif /* I2CTestHarnessService_H */

