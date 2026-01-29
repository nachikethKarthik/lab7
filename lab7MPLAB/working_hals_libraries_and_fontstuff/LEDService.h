/****************************************************************************

  Header file for template service
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef LEDService_H
#define LEDService_H

#include "ES_Types.h"

// Public Function Prototypes

bool InitLEDService(uint8_t Priority);
bool PostLEDService(ES_Event_t ThisEvent);
ES_Event_t RunLEDService(ES_Event_t ThisEvent);

#endif /* LEDService_H */

