/****************************************************************************

  Header file for TimeConstantService
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef TimeConstantService_H
#define TimeConstantService_H

#include "ES_Types.h"

// Public Function Prototypes

bool InitTimeConstantService(uint8_t Priority);
bool PostTimeConstantService(ES_Event_t ThisEvent);
ES_Event_t RunTimeConstantService(ES_Event_t ThisEvent);

#endif /* TimeConstantService_H */

