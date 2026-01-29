/****************************************************************************
 Module
   TemplateService.c

 Revision
   1.0.1

 Description
   This is a template file for implementing a simple service under the
   Gen2 Events and Services Framework.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 01/16/12 09:58 jec      began conversion from TemplateFSM.c
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
//#include "TemplateService.h"
#include "LEDService.h"
#include "dbprintf.h"
#include "DM_Display.h"
#include "PIC32_SPI_HAL.h"

/*----------------------------- Module Defines ----------------------------*/

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;

typedef enum{
  INITIALIZING_DISPLAY = 0,
  WAITING_FOR_INPUT,
  UPDATING_DISPLAY,
} State_t;
State_t CurrentLEDState = INITIALIZING_DISPLAY;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitTemplateService

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and does any
     other required initialization for this service
 Notes

 Author
     J. Edward Carryer, 01/16/12, 10:00
****************************************************************************/
bool InitLEDService(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  /********************************************
   in here you write your initialization code
   *******************************************/
  // Initialize PIC32 SPI (100 kHz)
  SPISetup_BasicConfig(SPI_SPI1);
  SPISetup_SetLeader(SPI_SPI1, SPI_SMP_END);         // MSTEN = 1, SMP = end
  SPISetup_SetBitTime(SPI_SPI1, 10000);              // Freq = 100 kHz
  SPISetup_MapSSOutput(SPI_SPI1, SPI_RPA0);          // Set RA0 to SS Output
  SPISetup_MapSDOutput(SPI_SPI1, SPI_RPA1);          // Set RA1 to SDO Output
  SPISetup_SetClockIdleState(SPI_SPI1, SPI_CLK_HI);  // Clock active high
  SPISetup_SetActiveEdge(SPI_SPI1, SPI_SECOND_EDGE); // Clock rising edge
  SPISetup_SetXferWidth(SPI_SPI1, SPI_16BIT);        // 16-bit messages
  SPISetEnhancedBuffer(SPI_SPI1, true);              // Enable enhanced buffer
  SPISetup_EnableSPI(SPI_SPI1);                      // ON = 1, begin operation
  
  CurrentLEDState = INITIALIZING_DISPLAY; // Initialize state
  
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
     PostTemplateService

 Parameters
     EF_Event_t ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostLEDService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunTemplateService

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes

 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event_t RunLEDService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  /********************************************
   in here you write your service code
   *******************************************/
  
  if (ThisEvent.EventType == ES_INIT)
  {    
    if ( DM_TakeInitDisplayStep() == true )
    {
      CurrentLEDState = WAITING_FOR_INPUT; // Move on to next state
    }
    else
    {
      ES_Event_t NewEvent;
      NewEvent.EventType = ES_INIT;
      PostLEDService( NewEvent ); // Post same event
    }
  }
  
  switch (CurrentLEDState)
  {
    case WAITING_FOR_INPUT: // State 0 = Checking for inputs
    {
      switch (ThisEvent.EventType)
      {
        case ES_WRITE_CHARACTER: // State 0 Event (character input)
        {
          // Check if valid character
          if ( (uint16_t) ThisEvent.EventParam > 255)
          {
            ThisEvent.EventParam = '?'; // Replace non-ASCII symbols with ?
            // Alternatively, just break;
          }
          
          // Scroll LED display 4 columns to left
          DM_ScrollDisplayBuffer( 4 );
                  
          // Write character to display
          DM_AddChar2DisplayBuffer( (uint8_t) ThisEvent.EventParam );
          
          // Change state
          CurrentLEDState = UPDATING_DISPLAY;
          ES_Event_t NewEvent;
          NewEvent.EventType = ES_UPDATE_DISPLAY;
          PostLEDService(NewEvent);
        }
        break;
        default:
        break;
      }
    }
    case UPDATING_DISPLAY: // State 1 = Updating Display
    {
      switch (ThisEvent.EventType)
      {
        case ES_UPDATE_DISPLAY:
        {
          if ( DM_TakeDisplayUpdateStep() == true )
          {
            CurrentLEDState = WAITING_FOR_INPUT; // Change state
          }
          else
          {
            ES_Event_t NewEvent;
            NewEvent.EventType = ES_UPDATE_DISPLAY;
            PostLEDService(NewEvent);
          }
        }
        break;
        default:
        break;
      }
    }
    break;
    default:
    break;
  }
  
  return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

