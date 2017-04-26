/********************************************************************
 *
 * SoutheastCon 2017 Arena control - Controller.h
 *
 * This is the header file for the lcd/button controller. 
 *
 * The controller is a handheld box that contains the start and 
 * stop buttons, and LCD status/score display.
 *
 ********************************************************************/

#ifndef Controller_h
#define Controller_h

#include "Arduino.h"
#include "Sainsmart_I2CLCD.h"
#include "ArenaControl.h"

#define BTN_START   (1 << 0)
#define BTN_STOP    (1 << 3)

class Controller
{
   public:
      Controller(uint8_t lcd_Addr=0x27, uint8_t lcd_cols=40, uint8_t lcd_rows=4);
      
      void start();
      void stop(uint32_t timestamp);
      void step(uint32_t timestamp);
      void report(uint32_t timestamp, int score);

      boolean attached();
      int buttons();
      Sainsmart_I2CLCD *lcdp();
};

#endif

