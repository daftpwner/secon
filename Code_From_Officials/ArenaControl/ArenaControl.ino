/**
 *
 * SoutheastCon 2017 Arena control - ArenaControl.cpp
 *
 * This is the code file for the main init/control program. 
 *
 * Each stage is confined within it's own class, with a start,
 *    step, and stop method. Each of the stages is declared 
 *    withi this file. In the Arduino setup() phase, the match
 *    start and end time is calculated, and the start() method
 *    for each stage class is invoked. Each stage is responsible
 *    for initializing the I/O pins, interrupts, initial state,
 *    etc needed for that stage.
 * In the Arduino loop() phase, the step() method for each
 *    stage is invoked. In this method, each stage checks the
 *    state of any I/O, timers, interrupts, or internal state
 *    to determine what action should be taken at that time. 
 *    Most of the stages use a finite state machine (FSM) to
 *    control the sequence of events within the stage.
 * Once the competition match time has expired, the stop()
 *    method for each stage is invoked, ending the competition
 *    and updating the score for that stage.
 *
 * The score and state of the competition is shown in two
 *    different ways - via the Arduino serial port (which can
 *    be monitored via the serial monitor in the Arduino IDE),
 *    and via a 4x20 LCD display. The use of the LCD display is
 *    to make the job of starting, stopping, and scoring of the
 *    competition easier by the judges. It is an optional item
 *    that is not needed for the student practice arenas, and
 *    the rest of the code will function fine without it.
 *
 * In the advent of no serial LCD, the competition countdown
 *    will start as soon as the Arduino is powered on (or reset)
 *    and will stop when the match timer expires. However, if an
 *    LCD (and associated control buttons) is available, the 
 *    Arduino will poll A0 to look for a start button, and will
 *    stop whenever the match timer expires or the A3 stop button
 *    is pressed. The arena can be reset and ready for the next
 *    match by either pressing reset, or pressing and holding
 *    both start and stop at the same time, then releasing for
 *    at least one second prior to pressing start for the next
 *    match.
 *    
 * Pin assignments, as well as a challenge to the code reviwers
 *    to find bugs, and the list of those who helped, are listed
 *    in the ArenaControl.h file
 */
 
#include <Adafruit_NeoPixel.h>
#include <LiquidCrystal_I2C.h>
#include <MsTimer2.h>
#include <Wire.h>

#include "Arduino.h"
#include "ArenaControl.h"
#include "Stage1.h"
#include "Stage2.h"
#include "Stage3.h"
#include "Controller.h"

Stage1 stage1;
Stage2 stage2;
Stage3 stage3;
Controller controller;
uint32_t startTimestamp = 0;

extern unsigned int __bss_end;
extern unsigned int __heap_start;
extern void *__brkval;

uint16_t getFreeSram() {
  uint8_t newVariable;
  // heap is empty, use bss as start memory address
  if ((uint16_t)__brkval == 0)
    return (((uint16_t)&newVariable) - ((uint16_t)&__bss_end));
  // use heap end as the start of the memory address
  else
    return (((uint16_t)&newVariable) - ((uint16_t)__brkval));
};

int randomSeedValue = 0;

void setup() 
{
   Serial.begin(9600);
   Wire.begin();

   Serial.print(F("FreeSram = "));
   Serial.println(getFreeSram());
   
   // Randomize the random number generator by reading the A1 voltage
   //    and using that as a seed. Since A1 is floating, it's value is
   //    bouncing, it's value is constantly changing (by a small 
   //    amount, but enough). This is a known 'hack' in the Arduino
   //    world to get around the fact that the random number generator
   //    *always* starts with the same value if not randomized first.
   randomSeedValue = analogRead(1);
   randomSeed(randomSeedValue);

   // Initialize processing for each stage
   stage1.start();
   stage2.start();
   stage3.start();

   // Wait here until the START button is pressed, or return
   //   immediately if there is no LCD
   controller.start();
   startTimestamp = millis();
}

void loop() 
{  
   uint32_t now = millis() - startTimestamp;
   int score = 0;

   // If the competition is still running, invoke each stage step (poor man's cooperative tasker)   
   if ((now < MATCH_RUNTIME) && (BTN_STOP != (controller.buttons() & BTN_STOP))) {
      controller.step(now);
      stage1.step(now);
      stage2.step(now);
      stage3.step(now);
      
   // Else the competition is over, so stop everything and report the results
   } else {
   
      // Stop all the stage functions
      controller.stop(now);
      stage1.stop(now);
      stage2.stop(now);
      stage3.stop(now);
 
      // Add up and print the total score (not counting stage 4, which is manual)     
      score = stage1.score() + stage2.score() + stage3.score();
      Serial.print(F("------ RESULTS ------\n"));
      Serial.print(F("FINAL SCORE: "));
      Serial.print(score);
      Serial.print(F("\nRANDOM SEED: "));
      Serial.print(randomSeedValue);      
      Serial.print(F("\n\n"));
      
      // Print out more detail on each stage
      controller.report(now, score);
      stage1.report();
      stage2.report();
      stage3.report();
      
      // Wait here forever
      for(;;);
   }
 }
 
