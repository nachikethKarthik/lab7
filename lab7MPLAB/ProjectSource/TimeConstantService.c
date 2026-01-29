/****************************************************************************
 Module
   TimeConstantService.c

 Revision
   1.0.2

 Description
    This service generates a square wave on RA2 (pin 10) to help measure
   the motor's electrical time constant.

 Notes
   - Controls:
        't' - Start toggling
        's' - Stop toggling
 History
 When           Who     What/Why
 -------------- ---     --------
 01/28/2026     karthi24    Started coding service to find time constant
 01/16/12 09:58 jec      began conversion from TemplateFSM.c
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "TimeConstantService.h"
#include "dbprintf.h"

/*----------------------------- Module Defines ----------------------------*/
// Pin definitions for RA2 (pin 10)
#define SQUARE_WAVE_TRIS    TRISAbits.TRISA2
#define SQUARE_WAVE_LAT     LATAbits.LATA2

// Timer settings
#define INITIAL_HALF_PERIOD_MS  500       // Start with 100ms half-period (5 Hz square wave)


/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
// States for the state machine
typedef enum {
    IDLE,
    TOGGLING
} ServiceState_t;

static ServiceState_t CurrentState;
static uint8_t PinState;
static uint16_t HalfPeriodMS;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitTimeConstantService

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
bool InitTimeConstantService(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  /********************************************
   in here you write your initialization code
   *******************************************/
    CurrentState = IDLE;
    PinState = 0;
    HalfPeriodMS = INITIAL_HALF_PERIOD_MS;
  
  // Configure RA2 as digital output
    SQUARE_WAVE_TRIS = 0;   // Set as output
    SQUARE_WAVE_LAT = 0;    // Initialize low
    
    DB_printf("\r\n=== Time Constant Measurement Service ===\r\n");
    DB_printf("Commands:\r\n");
    DB_printf("  't' - Start square wave\r\n");
    DB_printf("  's' - Stop square wave\r\n");
    DB_printf("Current half-period: %d ms (%.1f Hz)\r\n", 
              HalfPeriodMS, 1000.0 / (2.0 * HalfPeriodMS));
    DB_printf("=========================================\r\n\n");
  
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
     PostTimeConstantService

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
bool PostTimeConstantService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunTimeConstantService

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   Implements the state machine for square wave generation
 Notes

 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event_t RunTimeConstantService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  /********************************************
   in here you write your service code
   *******************************************/
  switch (CurrentState){
        case IDLE:
              if (ThisEvent.EventType == ES_NEW_KEY)
              {
                  char key = (char)ThisEvent.EventParam;

                  if (key == 't' || key == 'T')
                  {
                      // Start toggling
                      DB_printf("Starting square wave...\r\n");

                      // Set pin high and start timer
                      PinState = 1;
                      SQUARE_WAVE_LAT = 1;
                      ES_Timer_InitTimer(TIME_CONST_TIMER, HalfPeriodMS);

                      CurrentState = TOGGLING;
                  }
              }
              break;
        
        case TOGGLING:
            if (ThisEvent.EventType == ES_TIMEOUT && 
                ThisEvent.EventParam == TIME_CONST_TIMER)
            {
                // Toggle the pin
                if (PinState == 1)
                {
                    PinState = 0;
                    SQUARE_WAVE_LAT = 0;
                }
                else
                {
                    PinState = 1;
                    SQUARE_WAVE_LAT = 1;
                }
                
                // Restart timer for next toggle
                ES_Timer_InitTimer(TIME_CONST_TIMER, HalfPeriodMS);
            }
            else if (ThisEvent.EventType == ES_NEW_KEY)
            {
                char key = (char)ThisEvent.EventParam;
                
                if (key == 's' || key == 'S')
                {
                    // Stop toggling
                    DB_printf("Stopping square wave.\r\n");
                    
                    // Set pin low and stop timer
                    SQUARE_WAVE_LAT = 0;
                    PinState = 0;
                    ES_Timer_StopTimer(TIME_CONST_TIMER);
                    
                    CurrentState = IDLE;
                }
            }
            break;
  }
  return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

