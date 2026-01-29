/****************************************************************************

  Header file for MotorService
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef MotorService_H
#define MotorService_H

#include "ES_Types.h"

// Public Function Prototypes

bool InitMotorService(uint8_t Priority);
bool PostMotorService(ES_Event_t ThisEvent);
ES_Event_t RunMotorService(ES_Event_t ThisEvent);

#endif /* MotorService_H */

