/****************************************************************************
 Module
   TemplateService.c

 Revision
   1.0.2

 Description
   This service implements PWM motor control with encoder feedback for Lab 7
   Part 1.

 Notes
 * 
 * Drive-Brake Mode:
   - Forward: IN 1A = 0, IN 2A = PWM
   - Reverse: IN 1A = 1, IN 2A = inverted PWM (use 100% - duty cycle)

 History
 When           Who     What/Why
 -------------- ---     --------
 01/28/2026     karthi24    started coding up part 1 of lab 7
 01/16/12 09:58 jec      began conversion from TemplateFSM.c
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "MotorService.h"
#include "PIC32_AD_Lib.h"
#include "dbprintf.h"
#include <xc.h>
#include <sys/attribs.h>
/*----------------------------- Module Defines ----------------------------*/
#define PWM_PRESCALE        0b010       // 1:4 prescaler
#define PWM_PERIOD          1249        // For 4000 Hz PWM

#define IC_TIMER_PRESCALE   0b011       // 1:8 prescaler

#define RPM_NUMERATOR       49656

#define ADC_UPDATE_TIME_MS      100     // 10 Hz ADC reading
#define PRINT_UPDATE_TIME_MS    500     // 2 Hz terminal printing

// Direction Control Pin to the motor driver (RB2, pin 6) - IN 1A
#define DIR_1A_TRIS         TRISBbits.TRISB2
#define DIR_1A_ANSEL        ANSELBbits.ANSB2
#define DIR_1A_LAT          LATBbits.LATB2

// Direction Input Pin changed by user (RA1, pin 3)
#define DIR_INPUT_TRIS      TRISAbits.TRISA1
#define DIR_INPUT_ANSEL     ANSELAbits.ANSA1
#define DIR_INPUT_PORT      PORTAbits.RA1

// PWM Output Pin (RB13, pin 24) - IN 2A via OC4
#define PWM_PIN_TRIS        TRISBbits.TRISB13
#define PWM_PIN_ANSEL       ANSELBbits.ANSB13
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/
static void InitPWM(void);
static void InitDirectionPins(void);
static void InitInputCapture(void);
static void SetDutyCycle(uint8_t dutyPercent);

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;

// Shared with IC ISR (volatile)
static volatile uint16_t CurrentPeriod = 0;
static volatile uint16_t LastCapture = 0;

static uint8_t CurrentDutyCyclePercent = 0;


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
bool InitMotorService(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  /********************************************
   in here you write your initialization code
   *******************************************/
  InitDirectionPins();
  
  ADC_ConfigAutoScan(BIT0HI);
  
  InitPWM();
  
  InitInputCapture();
  
  // Print startup message
    DB_printf("\r\n=== Motor Control Service (For part 1) ===\r\n");
    DB_printf("RA1 input controls direction\r\n");
    DB_printf("======================================\r\n");
    DB_printf("\r\nDuty%%, RPM\r\n");  // CSV header for data collection
    
    ES_Timer_InitTimer(MOTOR_ADC_TIMER, ADC_UPDATE_TIME_MS);
    
    // Start periodic timer for terminal printing (2 Hz)
    ES_Timer_InitTimer(MOTOR_PRINT_TIMER, PRINT_UPDATE_TIME_MS);
    
    
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
     PostMotorService

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
bool PostMotorService(ES_Event_t ThisEvent)
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
   Handles ADC reading, direction control, and RPM reporting
 Notes

 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event_t RunMotorService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  /********************************************
   in here you write your service code
   *******************************************/
  switch(ThisEvent.EventType){
      case ES_TIMEOUT:
            if (ThisEvent.EventParam == MOTOR_ADC_TIMER)
            {
                // --- Read Potentiometer ---
                uint32_t adcResult[1];
                ADC_MultiRead(adcResult);
                
                // Calculate duty cycle percentage (0-100)
                CurrentDutyCyclePercent = (adcResult[0] * 100) / 1023;
                
                // --- Read Direction Input ---
                uint8_t directionInput = DIR_INPUT_PORT;
                
                // --- Apply Direction and Duty Cycle ---
                if (directionInput == 0)    // Forward
                {
                    DIR_1A_LAT = 0;
                    SetDutyCycle(CurrentDutyCyclePercent);
                }
                else                        // Reverse
                {
                    DIR_1A_LAT = 1;
                    // Invert duty cycle for Drive-Brake with IN1A = 1
                    SetDutyCycle(100 - CurrentDutyCyclePercent);
                }
                
                // Restart ADC timer
                ES_Timer_InitTimer(MOTOR_ADC_TIMER, ADC_UPDATE_TIME_MS);
            }
            else if (ThisEvent.EventParam == MOTOR_PRINT_TIMER)
            {
                // --- Calculate and Print RPM ---
                // Safely read volatile period
                __builtin_disable_interrupts();
                uint16_t periodSnapshot = CurrentPeriod;
                __builtin_enable_interrupts();
                
                if (periodSnapshot > 0)
                {
                    uint32_t rpm = RPM_NUMERATOR / periodSnapshot;
                    
                    // Print in CSV format for easy data collection
                    DB_printf("%d, %d\r\n", CurrentDutyCyclePercent, rpm);
                }
                else
                {
                    // Motor stopped or no encoder pulses
                    DB_printf("%d, 0\r\n", CurrentDutyCyclePercent);
                }
                
                // Restart print timer
                ES_Timer_InitTimer(MOTOR_PRINT_TIMER, PRINT_UPDATE_TIME_MS);
            }
            break;
  }
  return ReturnEvent;
}

/***************************************************************************
 ISR: Input Capture 2
 ***************************************************************************/
void __ISR(_INPUT_CAPTURE_2_VECTOR, IPL6AUTO) IC2_ISR(void)
{
    uint16_t ThisCapture;
    
    // Drain the FIFO - read all buffered captures, keep only the last one
    while (IC2CONbits.ICBNE)
    {
        ThisCapture = IC2BUF;
    }
    
    // Calculate period (unsigned subtraction handles 16-bit rollover)
    CurrentPeriod = ThisCapture - LastCapture;
    
    // Save for next edge
    LastCapture = ThisCapture;
    
    // Clear interrupt flag
    IFS0CLR = _IFS0_IC2IF_MASK;
}

/***************************************************************************
 private functions
 ***************************************************************************/
/****************************************************************************
 Function
     InitPWM

 Description
     Initializes Timer2 and OC4 for PWM output on RB13 at 4000 Hz
****************************************************************************/
static void InitPWM(void)
{
    // ===== Timer2 Setup =====
    T2CONbits.ON = 0;           // Disable Timer2
    T2CONbits.TCS = 0;          // Select internal PBCLK
    T2CONbits.TCKPS = PWM_PRESCALE;  // 1:4 prescaler
    TMR2 = 0;                   // Clear timer register
    PR2 = PWM_PERIOD;           // Set period for 4000 Hz
    // No need to clear interrupt flags since we are not using them
    T2CONbits.ON = 1;           // Enable Timer2
    
    // ===== OC4 Setup =====
    OC4CONbits.ON = 0;          // Disable OC4 during setup
    
    // Set initial duty cycle to 0%
    OC4R = 0;
    OC4RS = 0;
    
    // Disable analog on PWM pin
    PWM_PIN_ANSEL = 0;
    
    // Map OC4 to RB13 (pin 24)
    RPB13R = 0b0101;
    
    // Configure OC4 for PWM mode
    OC4CONbits.OCTSEL = 0;      // Use Timer2 as clock source
    OC4CONbits.OCM = 0b110;     // PWM mode, fault pin disabled
    
    // Enable OC4
    OC4CONbits.ON = 1;
}

/****************************************************************************
 Function
     InitDirectionPins

 Description
     Initializes the direction control output (RB2) and direction input (RA1)
****************************************************************************/
static void InitDirectionPins(void)
{
    // --- Direction Control Output (RB2, IN 1A) ---
    DIR_1A_ANSEL = 0;       // Disable analog
    DIR_1A_TRIS = 0;        // Set as output
    DIR_1A_LAT = 0;         // Initialize low (forward direction)
    
    // --- Direction Input (RA1) ---
    DIR_INPUT_ANSEL = 0;    // Disable analog
    DIR_INPUT_TRIS = 1;     // Set as input
}

/****************************************************************************
 Function
     InitInputCapture

 Description
     Initializes Timer3 (maximum period) and IC2 for encoder period measurement
****************************************************************************/
static void InitInputCapture(void)
{
    // ===== Timer3 Setup (free-running for Input Capture) =====
    T3CONbits.ON = 0;           // Disable Timer3
    T3CONbits.TCS = 0;          // Select internal PBCLK
    T3CONbits.TCKPS = IC_TIMER_PRESCALE;  // 1:8 prescaler
    TMR3 = 0;                   // Clear timer register
    PR3 = 0xFFFF;               // Max period 
    IFS0CLR = _IFS0_T3IF_MASK;  // Clear Timer3 interrupt flag
    T3CONbits.ON = 1;           // Enable Timer3
    
    // ===== Input Capture 2 Setup =====
    // Map IC2 input to RB9 (pin 18)
    IC2R = 0b0100;
    
    // Disable IC2 during setup
    IC2CONbits.ON = 0;
    
    // Configure IC2
    IC2CONbits.ICTMR = 0;       // Use Timer3 (ICTMR=0 means Timer3)
    IC2CONbits.ICI = 0b00;      // Interrupt on every capture event
    IC2CONbits.ICM = 0b011;     // Capture on every rising edge
    
    // Configure IC2 interrupt
    IFS0CLR = _IFS0_IC2IF_MASK; // Clear interrupt flag
    IPC2bits.IC2IP = 6;         // Priority 6
    IPC2bits.IC2IS = 0;         // Subpriority 0
    IEC0SET = _IEC0_IC2IE_MASK; // Enable IC2 interrupt
    
    // Enable Input Capture module
    IC2CONbits.ON = 1;
}

/****************************************************************************
 Function
     SetDutyCycle

 Parameters
     uint8_t dutyPercent - duty cycle as percentage (0-100)

 Description
     Converts percentage to PWM register value and updates OC4RS
****************************************************************************/
static void SetDutyCycle(uint8_t dutyPercent)
{
    // Clamp to valid range
    if (dutyPercent > 100)
    {
        dutyPercent = 100;
    }
    
    // Convert percentage to PWM register value
    // dutyValue = (dutyPercent * PWM_PERIOD) / 100
    uint32_t dutyValue = ((uint32_t)dutyPercent * PWM_PERIOD) / 100;
    
    // Write to OC4RS (double-buffered, updates on next period match)
    OC4RS = dutyValue;
}

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

