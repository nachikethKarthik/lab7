/****************************************************************************
 Module
   TemplateService.c

 Revision
   1.0.2

 Description
   This service implements closed-loop PI speed control of a DC motor.
   The potentiometer sets the commanded RPM, and a PI controller running
   at 2ms intervals adjusts the duty cycle to maintain the target speed.

 Notes
 No more direction control via input pin. 

 History
 When           Who     What/Why
 -------------- ---     --------
 01/29/2026     karthi24    started coding for part 2
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

// Control Loop Timer Configuration
// Timer4 with 1:4 prescaler -> 5 MHz tick rate
#define CONTROL_TIMER_PRESCALE  0b010   // 1:4 prescaler
#define CONTROL_TIMER_PERIOD    9999    // For 2ms control loop

#define MAX_COMMANDED_RPM   50 // Used in antiwindup


#define PER2RPM       49656

#define ADC_UPDATE_TIME_MS      100     // 10 Hz ADC reading
#define PRINT_UPDATE_TIME_MS    700     // 1.42 Hz terminal printing

// (RB2, pin 6) - IN 1A - previously used for direction control. only set once for unidirectional PI control
#define DIR_1A_TRIS         TRISBbits.TRISB2
#define DIR_1A_ANSEL        ANSELBbits.ANSB2
#define DIR_1A_LAT          LATBbits.LATB2

// PWM Output Pin (RB13, pin 24) - IN 2A via OC4
#define PWM_PIN_TRIS        TRISBbits.TRISB13
#define PWM_PIN_ANSEL       ANSELBbits.ANSB13

// Timing Pin (RA2, pin 10) - for ISR timing measurement
#define TIMING_PIN_TRIS     TRISAbits.TRISA2
#define TIMING_PIN_LAT      LATAbits.LATA2
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/
static void InitPWM(void);
static void InitDirectionPins(void);
static void InitInputCapture(void);
static void InitControlTimer(void);
static void SetDuty(float dutyPercent);
/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;

// Shared with IC ISR (volatile)
static volatile uint16_t CurrentPeriod = 0;
static volatile uint16_t LastCapture = 0;

// PI Controller Gains (adjustable via keyboard)
static float pGain = 0.5;               // Proportional gain - start conservative
static float iGain = 0.01;              // Integral gain - start small

// PI Controller State (used by Control ISR)
static volatile float SumError = 0.0;            // Integral accumulator
static volatile float TargetRPM = 0.0;           // Commanded speed from ADC

// For display (updated by Control ISR)
static volatile float CurrentRPM = 0.0;
static volatile float LastRPMError = 0.0;
static volatile float RequestedDuty = 0.0;


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
  
  // Initialize the Timing pin
  // RA2 does not have an ANSEL register (not analog-capable)
    TIMING_PIN_TRIS = 0;            // Set as output
    TIMING_PIN_LAT = 0;             // Initialize low
    
    
  InitDirectionPins();
  
  ADC_ConfigAutoScan(BIT0HI);
  
  InitPWM();
  
  InitInputCapture();
  
    // Initialize control loop timer (Timer4, 2ms)
  InitControlTimer();
  
  // Print startup message
    DB_printf("\r\n=== Motor Control Service (PI controller with anti-windup)===\r\n");
    DB_printf("Potentiometer sets commanded RPM (0-%d)\r\n", MAX_COMMANDED_RPM);
    DB_printf("Initial gains: pGain = %d, iGain = %d\r\n", (int)(100 * pGain), (int)(100 * iGain));
    DB_printf("\r\nKeyboard Controls:\r\n");
    DB_printf("  'P'/'p' - increase/decrease pGain\r\n");
    DB_printf("  'I'/'i' - increase/decrease iGain\r\n");
//    DB_printf("  'r'     - reset integral term\r\n");
    DB_printf("  '?'     - show current gains\r\n");
    DB_printf("==============================================\r\n");
    DB_printf("\r\nTargetRPM, ActualRPM, Error, Duty%%\r\n");
    
    ES_Timer_InitTimer(MOTOR_ADC_TIMER, ADC_UPDATE_TIME_MS);
    
    // Start periodic timer for terminal printing
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
    RunMotorService

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   Handles ADC reading for speed command, terminal printing, and gain tuning
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
                // --- Read Potentiometer for Speed Command ---
                uint32_t adcResult[1];
                ADC_MultiRead(adcResult);
                
                // Map ADC (0-1023) to commanded RPM (0-MAX_COMMANDED_RPM)
                TargetRPM = ((float)adcResult[0] * MAX_COMMANDED_RPM) / 1023.0;
                
                // Restart ADC timer
                ES_Timer_InitTimer(MOTOR_ADC_TIMER, ADC_UPDATE_TIME_MS);
            }
            else if (ThisEvent.EventParam == MOTOR_PRINT_TIMER)
            {
                // --- Print current state for monitoring ---
                // Read volatile variables safely
                __builtin_disable_interrupts();
                float displayRPM = CurrentRPM;
                float displayError = LastRPMError;
                float displayDuty = RequestedDuty;
                __builtin_enable_interrupts();
                
                // Print in CSV format
                
                DB_printf("%d, %d, %d, %d\r\n", 
                          (int)(TargetRPM), (int)(displayRPM), (int)(displayError), (int)(displayDuty));
                
                // Restart print timer
                ES_Timer_InitTimer(MOTOR_PRINT_TIMER, PRINT_UPDATE_TIME_MS);
            }
            break;
            
        case ES_NEW_KEY:
        {
            char key = (char)ThisEvent.EventParam;
            
            switch (key)
            {
                case 'P':   // Increase pGain
                    pGain += 0.1;
                    DB_printf("pGain = %d\r\n", (int)(100 * pGain));
                    break;
                    
                case 'p':   // Decrease pGain
                    pGain -= 0.1;
                    if (pGain < 0) pGain = 0;
                    DB_printf("pGain = %d\r\n", (int)(100 * pGain));
                    break;
                    
                case 'I':   // Increase iGain
                    iGain += 0.005;
                    DB_printf("iGain = %d\r\n", (int)(100 * iGain));
                    break;
                    
                case 'i':   // Decrease iGain
                    iGain -= 0.005;
                    if (iGain < 0) iGain = 0;
                    DB_printf("iGain = %d\r\n", (int)(100 * iGain));
                    break;
                    
//                case 'r':   // Reset integral term
//                case 'R':
//                    SumError = 0;
//                    DB_printf("Integral term reset (SumError = 0)\r\n");
//                    break;
                    
                case '?':   // Show current gains
                    DB_printf("Current gains: pGain = %d, iGain = %d\r\n", 
                              (int)(100 * pGain), (int)(100 * iGain));
                    DB_printf("SumError = %d\r\n", (int)(SumError));
                    DB_printf("\r\nTargetRPM, ActualRPM, Error, Duty%%\r\n");
                    break;
                    
                default:
                    break;
            }
        }break;
            
  }
  return ReturnEvent;
}

/***************************************************************************
 ISR: Control Loop (Timer4) - Priority 5
 Runs every 2ms to calculate and apply PI control law
 ***************************************************************************/
void __ISR(_TIMER_4_VECTOR, IPL5AUTO) Timer4_ISR(void)
{
    // Clear interrupt flag first
    IFS0CLR = _IFS0_T4IF_MASK;
    
    // Raise timing pin on entry
    TIMING_PIN_LAT = 1;
    
    // --- Local variables (static for speed) ---
    static float RPMError;
    static uint32_t ThisPeriod;
    static float Rpm;
    
    // --- Get Current Motor Speed ---
    ThisPeriod = CurrentPeriod;
    
    if (ThisPeriod > 0)
    {
        Rpm = (float)PER2RPM / (float)ThisPeriod;
    }
    else
    {
        Rpm = 0;
    }
    
    // --- Calculate Error ---
    RPMError = TargetRPM - Rpm;
    SumError += RPMError;
    
    // --- Calculate Requested Duty Cycle (PI Control Law) ---
    RequestedDuty = (pGain * RPMError) + (iGain * SumError);
    
    // --- Clamp Output and Apply Anti-Windup ---
    if (RequestedDuty > 100)
    {
        RequestedDuty = 100;
        SumError -= RPMError;   // Anti-windup
    }
    else if (RequestedDuty < 0)
    {
        RequestedDuty = 0;
        SumError -= RPMError;   // Anti-windup
    }
    
    // --- Update values for display ---
    LastRPMError = RPMError;
    CurrentRPM = Rpm;
    
    // --- Apply Duty Cycle to PWM ---
    SetDuty(RequestedDuty);
    
    // Lower timing pin before exit
    TIMING_PIN_LAT = 0;
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
     InitControlTimer

 Description
     Initializes Timer4 for 2ms control loop interrupt
****************************************************************************/
static void InitControlTimer(void)
{
    // Disable Timer4
    T4CONbits.ON = 0;
    
    // Select internal PBCLK
    T4CONbits.TCS = 0;
    
    // Set prescaler (1:4)
    T4CONbits.TCKPS = CONTROL_TIMER_PRESCALE;
    
    // Clear timer register
    TMR4 = 0;
    
    // Set period for 2ms
    // PBCLK = 20MHz, Prescaler = 1:4 -> 5MHz tick rate
    // 2ms = 0.002s * 5,000,000 = 10,000 ticks
    // PR4 = 10000 - 1 = 9999
    PR4 = CONTROL_TIMER_PERIOD;
    
    // Configure Timer4 interrupt - Priority 5 (LOWER than Input Capture)
    IFS0CLR = _IFS0_T4IF_MASK;      // Clear interrupt flag
    IPC4bits.T4IP = 5;              // Priority 5 lower than IC interrupt priority
    IPC4bits.T4IS = 0;              // Subpriority 0
    IEC0SET = _IEC0_T4IE_MASK;      // Enable Timer4 interrupt
    
    // Enable Timer4
    T4CONbits.ON = 1;
}
/****************************************************************************
 Function
     SetDutyCycle

 Parameters
     uint8_t dutyPercent - duty cycle as percentage (0-100)

 Description
     Converts percentage to PWM register value and updates OC4RS
****************************************************************************/
static void SetDuty(float dutyPercent)
{
    // Clamp to valid range
    if (dutyPercent > 100)
    {
        dutyPercent = 100;
    }
    else if (dutyPercent < 0)
    {
        dutyPercent = 0;
    }
    
    // Convert percentage to PWM register value
    uint32_t dutyValue = (uint32_t)((dutyPercent * PWM_PERIOD) / 100.0);
    
    // Write to OC4RS (double-buffered, updates on next period match)
    OC4RS = dutyValue;
}

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

