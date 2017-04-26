/********************************************************************
 *
 * SoutheastCon 2017 Arena control - Stage1.cpp
 *
 * This is the code file for the stage #1 control elements. 
 *
 * Stage 1 is a 6 pad arrangement of copper pads (one in the center
 *    and five places around the center in a pentagon arrangement).
 *    Connected to these pads is an inductor, resistor, capacitor,
 *    diode and straight wire, with each component having one wire
 *    connected to the center and one to each of the surrounding
 *    pads. Each component type is present, but the arrangement is
 *    random.
 *
 * The code for this section picks the relay settings that control
 *    the random arrangment of the components at the beginning of
 *    the contest, and has no further interaction with the robot.
 *
 ********************************************************************/

#include <Wire.h>

#include "Arduino.h"
#include <avr/pgmspace.h>

#include "Stage1.h"
#include "relayTable.h"

#include "Controller.h"
extern Controller controller;

#define I2C_ADDR_RELAY   0x20

void setRelays(uint16_t relayPattern);


Stage1::Stage1() 
{
}


void Stage1::start() 
{
  /* Since we don't have enough I/Os on the Arduino UNO to control
    *    the 16 relays, we use an I2C port expander. An alternative
    *    idea would have been to use a larger Arduino, but this 
    *    solution was a little cheaper and did not require routing
    *    a large number of wires between the Arduino and stage 1.
    * Choose a relayTable index to control the placement of components 
    */
   relayIndex   = random(RELAY_TABLE_LENGTH);
   relayPattern = pgm_read_word_near((relayTable[relayIndex]) + 0);
   turnPattern  = pgm_read_word_near((relayTable[relayIndex]) + 1);
}


/* Stop any stage 1 processing */
void Stage1::stop(uint32_t timestamp) 
{
   /* turn off the relays at the end of the competition */
   setRelays(0);
}


/* Step - cooperative multi-tasker between the stages */
void Stage1::step(uint32_t timestamp) 
{
   static int firstTime = true;
   
   /* Set the relays once the contest starts - nothing to do otherwise */
   if (firstTime) {
       setRelays(relayPattern);
       firstTime = false;
   }
}


/* End of run report on points, and stage 1 specifics (relay pattern 
 *    chosen to select the component ordering)
 */
void Stage1::report(void) 
{
   Serial.print(F("------ Stage 1 report ------\n"));
   Serial.print(F("RELAY INDEX: "));
   Serial.print(relayIndex);
   Serial.print(F("\nRELAY PATTERN: "));
   Serial.print(String(relayTable[relayIndex][0], 16));
   Serial.print(F("\nSTAGE SCORE: N/A"));
   Serial.print(F("\n\n")); 
   
   if (controller.attached()) {
      controller.lcdp()->setCursor(0,1);
      controller.lcdp()->print("1: #");
      controller.lcdp()->print(relayIndex);
      controller.lcdp()->print(" ");
      controller.lcdp()->print(String(relayPattern, 16));
   }
}


/* Calculates the score for stage 1 (which is always zero). THe score is 
 *    is based on digits entered at stage 3 (or proof of valid decoding
 *    by showing a readout on the robot's external LCD/LED display)
 */
int Stage1::score(void) 
{
  return 0;
}


/* Set the 16 relays to the state in the 16-bit relay parameter. The relays
 *    are attached to the Arduino via an I2C 16-bit port expander.
 * This code can still run without the I2C port expander attached - the
 *    write commands fail. This will allow testing of the code without a
 *    full setup, and the components can be hard-wired to the desired pads.
 */
void setRelays(uint16_t value)
{  
   Wire.beginTransmission(I2C_ADDR_RELAY);
   Wire.write(~(value & 0xFF));
   Wire.write(~((value >> 8) & 0xFF));
   Wire.endTransmission();
}
