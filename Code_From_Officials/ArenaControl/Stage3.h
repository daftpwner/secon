/********************************************************************
 *
 * SoutheastCon 2017 Arena control - Stage3.h
 *
 * This is the header file for the stage #3 control elements. 
 *
 * Stage3 control is responsible for rotary encoder motion and
 * encoder LED control.
 *
 ********************************************************************/

#ifndef Stage3_h
#define Stage3_h

#include "Arduino.h"
#include "ArenaControl.h"

class Stage3 
{
   public:
      Stage3();

      void start(void);
      void stop(uint32_t timestamp);
      void step(uint32_t timestamp);
      void report(void);
      int  score(void);
};

#endif
