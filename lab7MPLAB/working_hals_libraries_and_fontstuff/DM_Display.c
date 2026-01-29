/****************************************************************************
 Module
     DM_Display.c
 Description
     Source file for the Dot Matrix LED Hardware Abstraction Layer 
     used in ME218
 Notes
     This is the prototype. Students will re-create this functionality
 History
 When           Who     What/Why
 -------------- ---     --------
  10/03/21 12:32 jec    started coding
  2023-10-18    klg     Aligned DM_AddChar2DisplayBuffer
*****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
#include <xc.h>
#include <stdbool.h>
#include "PIC32_SPI_HAL.h"
#include "DM_Display.h"
#include "FontStuff.h"

/*----------------------------- Module Defines ----------------------------*/
#define NumModules 4
#define NUM_ROWS   8
#define NUM_ROWS_IN_FONT 5
#define DM_START_SHUTDOWN 0x0C00
#define DM_END_SHUTDOWN   0x0C01
#define DM_DISABLE_CODEB  0x0900
#define DM_ENABLE_SCAN    0x0B07
#define DM_SET_BRIGHT     0x0A00

/*------------------------------ Module Types -----------------------------*/
// this union definition assumes that the display is made up of 4 modules
// 4 modules x 8 bits/module = 32 bits total
// this union allows us to easily scroll the whole buffer, using the uint32_t
// while picking out the individual bytes to send them to the controllers
typedef union{
    uint32_t FullRow;
    uint8_t ByBytes[NumModules];
}DM_Row_t;

typedef enum { DM_StepStartShutdown = 0, DM_StepFillBufferZeros, 
               DM_StepDisableCodeB, DM_StepEnableScanAll, DM_StepSetBrighness,
               DM_StepCopyBuffer2Display, DM_StepEndShutdown
}InitStep_t;

/*---------------------------- Module Functions ---------------------------*/
static void sendCmd( uint16_t Cmd2Send );
static void sendRow( uint8_t RowNum, DM_Row_t RowData );

/*---------------------------- Module Variables ---------------------------*/
// We make the display buffer from an array of these unions, one for each 
// row in the display
static DM_Row_t DM_Display[NUM_ROWS];

// this is the state variable for tracking init steps
static InitStep_t CurrentInitStep =  DM_StepStartShutdown;

// In order to keep up with the display at 10MHz, the bit reverse operation
// must be as fast as possible, hence the look-up table approach is the only
// solution that will work with the SPI at 10MHz
// This bit reversal table comes from
// https://stackoverflow.com/questions/746171/efficient-algorithm-for-bit-reversal-from-msb-lsb-to-lsb-msb-in-c
// usage: BitReversedValue = BitReverseTable256[OriginalByteValue];
//
static const uint8_t BitReverseTable256[] = 
{
  0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0, 
  0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8, 
  0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4, 
  0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC, 
  0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2, 
  0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
  0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6, 
  0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
  0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
  0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9, 
  0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
  0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
  0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3, 
  0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
  0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7, 
  0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF
};


/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
  DM_TakeInitDisplayStep

  Description
  Initializes the MAX7219 4-module display performing 1 step for each call:
    First, bring put it in shutdown to disable all displays, return false
    Next fill the display RAM with Zeros to insure blanked, return false
    Then Disable Code B decoding for all digits, return false
    Then, enable scanning for all digits, return false
    The next setup step is to set the brightness to minimum, return false
    Copy our display buffer to the display, return false
    Finally, bring it out of shutdown and return true
****************************************************************************/
bool DM_TakeInitDisplayStep( void )
{
    static uint8_t rowIndex = 0;
    bool ReturnVal = false;
    
    switch (CurrentInitStep)
    {
    case DM_StepStartShutdown:
      sendCmd( 0x0C00 ); // First, bring put it in shutdown to disable all displays
      CurrentInitStep++; // move on to next step
      
      break;
      
    case DM_StepFillBufferZeros:
      DM_ClearDisplayBuffer(); // fill the buffer with Zeros
      CurrentInitStep++; // move on to next step
      break;
      
    case DM_StepDisableCodeB:
      sendCmd( 0x0900 ); // Next Disable Code B decoding for all digits
      CurrentInitStep++; // move on to next step
      break;

    case DM_StepEnableScanAll:
      sendCmd( 0x0B07 ); // Then, enable scanning for all digits
      CurrentInitStep++; // move on to next step
      break;

    case DM_StepSetBrighness:
      sendCmd( 0x0A00 ); // The next setup step is to set the brightness to minimum
      CurrentInitStep++; // move on to next step
      break;

    case DM_StepCopyBuffer2Display: // copy our display buffer to the display
      if (true == DM_TakeDisplayUpdateStep())
      {
        CurrentInitStep++; // move on to next step
      }
      else
      {}
      break;
      
    case DM_StepEndShutdown:  
      sendCmd( 0x0C0F ); // Finally, bring it out of shutdown
      CurrentInitStep =  DM_StepStartShutdown; // prepare for a re-init
      ReturnVal = true; // let the caller know that we are done
      break;
      
    default:
      break;
    }
    
    return ReturnVal;
}

/****************************************************************************
 Function
  DM_TakeDisplayUpdateStep

 Description
  Copies the contents of the display buffer to the MAX7219 controllers 1 row
  per call.
****************************************************************************/
bool DM_TakeDisplayUpdateStep( void )
{
    bool ReturnVal = false;
    static int8_t WhichRow = 0;
    
    sendRow(WhichRow, DM_Display[WhichRow]);
    
    // your code  to check when we are done sending rows goes here 
    WhichRow++; // Increment WhichRow for the next loop
    if (WhichRow == NUM_ROWS) // After filling the eighth row, we're done
      
    {
      ReturnVal = true; // show we are done
      WhichRow = 0; // set up for next update
    }
    return ReturnVal;
}

/****************************************************************************
 Function
  DM_ScrollDisplayBuffer
 Description
  Scrolls the contents of the display buffer by the indicated number of 
  columns. Makes use of the FullRow member to scroll all modules at once
****************************************************************************/
void DM_ScrollDisplayBuffer( uint8_t NumCols2Scroll)
{
    uint8_t WhichRow;
    
    // your code goes here
    for (WhichRow = 0; WhichRow < NUM_ROWS; WhichRow++)
    {
      DM_Display[WhichRow].FullRow = DM_Display[WhichRow].FullRow << NumCols2Scroll;
    }
}

/****************************************************************************
 Function
  DM_AddChar2DisplayBuffer

 Description
  Copies the bitmap data from the font file into the rows of the frame buffer
  at the right-most character position in the buffer  
****************************************************************************/
void DM_AddChar2DisplayBuffer( unsigned char Char2Display)
{
    uint8_t WhichRow;
    
    // Your code to loop for every row in the character font
    for (WhichRow = 0; WhichRow < NUM_ROWS_IN_FONT; WhichRow++)
    {
        DM_Display[WhichRow].ByBytes[0]  |= getFontLine(Char2Display, WhichRow);
    } 
}

/****************************************************************************
 Function
  DM_ClearDisplayBuffer

 Description
  Clears the contents of the display buffer by filling it with zeros.
****************************************************************************/
void DM_ClearDisplayBuffer( void )
{
  uint8_t rowIndex;
  
  // Now fill the display RAM with Zeros to insure blanked
  for (rowIndex = 0; rowIndex < NUM_ROWS; rowIndex++)
  {
    DM_Display[rowIndex].FullRow = 0x00000000;
  }
}

/****************************************************************************
 Function
  DM_PutDataIntoBufferRow

 Description
  Copies the raw data from the Data2Insert parameter into the specified row 
  of the frame buffer 
****************************************************************************/
bool DM_PutDataIntoBufferRow( uint32_t Data2Insert, uint8_t WhichRow)
{
  bool ReturnVal = true;
  
  if (WhichRow < 0 || WhichRow >= NUM_ROWS) // test for legal row
  {
    ReturnVal = false;
  }
  else // legal row, so stuff the data into the buffer
  {
    DM_Display[WhichRow].FullRow = Data2Insert;
  }
  return ReturnVal;
}

/****************************************************************************
 Function
  DM_QueryRowData

 Description
  copies the contents of the specified row of the frame buffer into the
 location pointed to by pReturnValue
****************************************************************************/
bool DM_QueryRowData( uint8_t RowToQuery, uint32_t * pReturnValue)
{
  bool ReturnVal = true;
  
  if (RowToQuery < 0 || RowToQuery >= NUM_ROWS) // test for legal row
  {
    ReturnVal = false;
  }
  else // legal row, so grab the data from the buffer
  {
    *pReturnValue = DM_Display[RowToQuery].FullRow;
  }
  return ReturnVal;
}

//*********************************
// private functions
//*********************************

/****************************************************************************
 Function
 sendCmd

 Description
  Send a single command to all 4 modules and waits for the SS to rise to
  indicate completion.
****************************************************************************/
static void sendCmd( uint16_t Cmd2Send )
{
    uint8_t index;
    for (index = 0; index <= (NumModules-2); index++)
    {
        SPIOperate_SPI1_Send16( Cmd2Send );
    }
    SPIOperate_SPI1_Send16Wait(  Cmd2Send );
}

/****************************************************************************
 Function
 sendRow

 Description
  Sends a row of data to the 4-module cluster. Translates from the logical
 row number to the MAX7219 row numbers (mirrors)
****************************************************************************/
static void sendRow( uint8_t RowNum, DM_Row_t RowData )
{
     uint8_t index;
    // The rows on the display are mirrored relative to the rows in the memory
    RowNum = NUM_ROWS - (RowNum+1); // this will swap them top to bottom
    // loop through, sending the first 3 values as fast as possible
    for (index = 0; index <= (NumModules-2); index++)
    {
        SPIOperate_SPI1_Send16( ((((uint16_t)RowNum+1)<<8) | 
                              BitReverseTable256[(RowData.ByBytes[index])]));
    }
    // then send the final byte and wait for the SS line to rise
    SPIOperate_SPI1_Send16Wait( 
                          ((((uint16_t)RowNum+1)<<8) | 
                              BitReverseTable256[(RowData.ByBytes[index])]));
}



