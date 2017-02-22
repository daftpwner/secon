/********************************************************************
 *
 * SoutheastCon 2017 Arena control - Stage2.h
 *
 * This is the header file for the stage #2 control elements. 
 *
 * Stage2 control is responsible for activating the electro
 * magnet, counting hits on the light saber, and controlling
 * the lightsaber LEDs.
 *
 ********************************************************************/

#ifndef Stage2_h
#define Stage2_h

#include "Arduino.h"
#include "ArenaControl.h"

class Stage2 
{
   public:
      Stage2();

      void start(void);
      void stop(uint32_t timestamp);
      void step(uint32_t timestamp);
      void report(void);
      int  score(void);
};

#endif
