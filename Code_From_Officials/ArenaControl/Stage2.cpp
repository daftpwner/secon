#include <LiquidCrystal_I2C.h>

/********************************************************************
 *
 * SoutheastCon 2017 Arena control - Stage2.cpp
 *
 * This is the code file for the stage #2 control elements. 
 *
 ********************************************************************/

#include <Adafruit_NeoPixel.h>

#include "Arduino.h"
#include "Stage2.h"

#include "Controller.h"
extern Controller controller;

/*
 * Defines used by this stage
 */
#define HALF_SECOND         500     //  500ms
#define ONE_SECOND          MSECS   // 1000ms

#define VIBRATE_PIN         2       // IRQ0 on UNO (only one of two)

#define NEOPIXEL_PIN        6       // Connects to data-in of neopixel stick
#define NEOPIXEL_LED_COUNT  8       // 8-pixel Adafruit neopixel stick (RGB)

#define FIELD_PIN           13      // UNO has an LED on this pin - nice for visual reference

#define FLASH_TIMEOUT       50      // # of msecs red/blue flash after 


/*
 * Internal types for this stage
 */
enum states {
   INITIAL,
   COUNTDOWN_1,
   COUNTDOWN_2,
   COUNTDOWN_3,
   START,
   WAITING,
   FIELD_OFF_NEUTRAL,
   FIELD_OFF,
   FIELD_ON,
   STOPPED
}; 


/*
 * Internal variables for this stage
 */
static uint32_t white, black, red, lt_red, green, lt_green, blue, amber;
static volatile uint16_t hit = 0;
int ignore_hits = true;
enum states curState  = INITIAL;
enum states nextState = COUNTDOWN_1;
uint32_t nextStateTimestamp = 0;
uint32_t flashOffTimestamp  = 0;
uint8_t  *patternPtr = NULL;
uint8_t  patternIndex = 0;
uint32_t hitTimeout = 0;
int      enableField = false;
char     hitReport[10], *hitReportPtr = hitReport;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NEOPIXEL_LED_COUNT, NEOPIXEL_PIN, NEO_GRB+NEO_KHZ800);


/*
 * Internal functions for this stage
 */
static void vibrate();
static int hit_detected(void);
static void singleColor(uint32_t c);
static void activateField(boolean state);


/* This array is the "fighting pattern" of ON/OFF/ON for the magnetic field. Since the
 *    field is always on for 2 seconds, the ON time is not listed and the table only
 *    includes the OFF times.
 * Since the lightsaber duel lasts 30 seconds, and there are 4 ON times, that means the
 *    total off time is 22 seconds. Since there is a 0.5 second grace period after each ON
 *    time, that means the OFF times for a duel must equal 20 seconds in each row below.
 * An entry is 'randomly' chosen using the LSB digit of the micros() time output at the
 *    time of the first hit. A zero sentinel value is at the end of each row to indicate
 *    the lightsaber duel is over.
 */
uint8_t fightingPatterns[10][5] = {
    { 2, 5, 7, 6, 0 }, // 0
    { 4, 3, 5, 8, 0 }, // 1
    { 2, 4, 7, 7, 0 }, // 2
    { 7, 1, 7, 5, 0 }, // 3
    { 5, 5, 5, 5, 0 }, // 4
    { 4, 6, 7, 3, 0 }, // 5
    { 3, 7, 9, 1, 0 }, // 6
    { 1, 3, 7, 9, 0 }, // 7
    { 6, 2, 4, 8, 0 }, // 8
    { 6, 8, 2, 4, 0 }  // 9
};


Stage2::Stage2() 
{
}


void Stage2::start() 
{
   /* Set the output bit for the magnet force field and deactivate it */
   pinMode(FIELD_PIN, OUTPUT);
   activateField(false);  

   /* Initialize the interrupt routine for vibration sensor */
   pinMode(VIBRATE_PIN, INPUT_PULLUP);
   attachInterrupt(0, vibrate, CHANGE);

   /* Predefine the colors for convenience */
   black    = strip.Color(  0,  0,  0);
   white    = strip.Color(255,255,255);
   red      = strip.Color(255,  0,  0);
   lt_red   = strip.Color( 10,  0,  0);
   green    = strip.Color(  0,255,  0);
   lt_green = strip.Color(  0, 10,  0);
   blue     = strip.Color(  0,  0,255);
   amber    = strip.Color(255,191,  0);

   /* initial state of the lightsaber until contest begins */
   strip.begin();
   singleColor(black);
   strip.setPixelColor(7, blue);
   strip.show();
   
   /* Initialize the hit report log to empty */
   memset(hitReport, '.', sizeof(hitReport));
   hitReport[sizeof(hitReport)-1] = '\0';
   hitReportPtr = hitReport;
}


void Stage2::stop(uint32_t timestamp) 
{    
   /* Final state of the lightsaber (red - stop) and field deactivated */
   singleColor(red);
   activateField(false);
   detachInterrupt(0);
}


void Stage2::step(uint32_t timestamp) 
{
   /* If the hit timer is on, then the lightsaber is either red or blue, so
    *    check if it is time to turn the lightsaber back off 
    */
   if ((0 != hitTimeout) && (millis() > hitTimeout)) {
      hitTimeout = 0;
      singleColor(black);
   }
   
   /* If the next timestamp has not yet occurred, then not time to advance
    *    to the next state, so nothing more to do
    */
   if ((INITIAL != curState) && (millis() < nextStateTimestamp)) {
      return;
   }
   
   /* If we are switching state, then update next state variable and zero out
    *    the hit flag to avoid hits in the previous state being counted in the
    *    next state
    */
   if (curState != nextState) {
     curState = nextState;
     hit = 0;
   }
   
   /* State table - actions and next state based on the current state, timeout
    *    and vibration hit flag 
    */
   switch (curState) {
     
     /* We have three countdown states. The plan is that when the robot is ready,
      *    the team is warned they have 3 seconds to start the robot. The arena
      *    controller is then turned on and the arena judge counts down 3,2,1 
      *    with each of the countdown states.
      * The first state is COUNTDOWN state 1 and occurs as soon as the board is
      *    powered on. When the lightsaber turns red, the arena judge counts '3'
      *    to the team indicating 3 seconds before the match begins.
      * Next state: COUNTDOWN_2 after 1 second
      */
     case COUNTDOWN_1: 
          singleColor(red);
          strip.setPixelColor(7, green);
          strip.setPixelColor(6, green);
          strip.show();

          nextState = COUNTDOWN_2;
          nextStateTimestamp = millis() + ONE_SECOND;
          break;

      /* This is the continuation of the countdown. The top half of the lightsaber
       *    changes from red to amber, and the judge counts '2' to the team,
       *    indicating 2 seconds before the match begins.
       * Next state: COUNTDOWN_3 after 1 second
       */      
      case COUNTDOWN_2:
          singleColor(red);
          strip.setPixelColor(4, green);
          strip.setPixelColor(3, green);
          strip.show();
          
          nextState = COUNTDOWN_3;
          nextStateTimestamp = millis() + ONE_SECOND;
          break;

      /* This is the next and final of the countdown states. The entire lightsaber
       *    is now amber and the judge counts '1' indicating 1 more second before
       *    the match begins. The team should be prepared to press their start
       *    button and get out of the arena.
       * Next state: START after 1 second
       */
      case COUNTDOWN_3:
          singleColor(red);
          strip.setPixelColor(1, green);
          strip.setPixelColor(0, green);
          strip.show();

          nextState = START;
          nextStateTimestamp = millis() + ONE_SECOND;
          break;

      /* The entire lightsaber lights green to indicate the match has begun and
       *    they should press their start button and get out of the arena.
       * While the match has begun and stages 1 and 3 are active, there is a 5
       *    second delay before stage 2 begins. This allows the team to back out,
       *    stepping over/around stage 2 without triggering the vibration
       *    sensor. 
       * Next state: WAITING after 5 seconds
       */
      case START:
          singleColor(green);
          nextState = WAITING;
          nextStateTimestamp = millis() + (5 * ONE_SECOND);
          break;
          
      /* In this stage, we are waiting for the first hit of the 
       *    lightsaber to start the lightsaber duel. The vibration 
       *    sensor is active.
       * Next state: FIELD_OFF_NEUTRAL after vibration detected
       */
      case WAITING:
          /* Activate the field and interrupt and wait for first hit */
          ignore_hits = false;
          activateField(true);
          
          /* the stage 2 lightsaber battle begins when the first hit is detected */
          if (hit_detected()) {
             singleColor(blue);
             hitTimeout = millis();
             nextState = FIELD_OFF_NEUTRAL;
             nextStateTimestamp = millis();
             
             /* Choose one of the 10 patterns using LSB of micros() function */
             patternIndex = micros() % 10;
             patternPtr = &(fightingPatterns[patternIndex][0]);
             *hitReportPtr++ = '+';
             
          }
          break;
      
      /* This state is entered immediately after the field is deactivated
       *    yet it is during the neutral (or no penalty) phase so points 
       *    are not deducted during this time, nor are hits shown on the
       *    lightsaber as flashes of light. The vibration sensor is not
       *     active and hits are not recognized.
       * Next state: FIELD_OFF after a 0.5 second delay
       */
      case FIELD_OFF_NEUTRAL:
          activateField(false);
          nextState = FIELD_OFF;
          nextStateTimestamp = millis() + HALF_SECOND;
          break;
         
      /* In this state, the field is off, but the vibration sensor is active
       *    and any hits are counted as deductions and are shown as red 
       *    flashes on the light saber.
       * Next state: FIELD_ON when the next field on cycle occurs
       */ 
      case FIELD_OFF:
          if (hit_detected()) {
             *hitReportPtr = '-';
             singleColor(red);
             hitTimeout = millis() + FLASH_TIMEOUT;
          }
          if ((millis()-nextStateTimestamp) > (*patternPtr * ONE_SECOND)) {
             nextState = FIELD_ON;
             enableField = true;
             nextStateTimestamp = millis();
             hitReportPtr++;
          }
          break;
        
      /* In this state, the field and vibration sensor is on and any hits 
       *    are counted as positive points and are shown as blue flashes 
       *    on the lightsaber. The on period always lasts 2 seconds or 
       *    until the lightsaber is hit.
       * Next state: FIELD_OFF, unless this is the last on, then STOPPED
       */ 
      case FIELD_ON:
          if (enableField) {
             activateField(true);
             enableField = false;
          }
          if (hit_detected()) {
             *hitReportPtr = '+';
             singleColor(blue);
             hitTimeout = millis() + FLASH_TIMEOUT;
             activateField(false);
          }
          if ((millis()-nextStateTimestamp) > (2*ONE_SECOND)) {
             patternPtr++;
             nextState = (0 == *patternPtr) ? STOPPED : FIELD_OFF_NEUTRAL;
             nextStateTimestamp = millis();
             hitReportPtr++;
          }          
          break;
      
      /* The 30 second lightsaber duel timer is up and stage 2 is disabled. 
       *    This is signaled by the lightsaber being lit as all green again 
       *    until the match is over, and then it turns to all red. In 
       *    addition, turn off the magnetic field and detach the interrupt.
       * Next state: (none)
       */
      case STOPPED:
          singleColor(green);
          activateField(false);
          detachInterrupt(0);
          break;
        
      /* Should never get here */
      default:
          break;
    }
}


/* End of run report on points, and stage 2 specifics (pattern chosen for
 *    the lightsaber, and log record of good and bad hits
 */
void Stage2::report(void) {
   Serial.print("------ Stage 2 report ------\n");
   Serial.print("SABER INDEX: ");
   Serial.print(patternIndex);
   Serial.print("\nHIT REPORT : [");
   Serial.print(hitReport);
   Serial.print("]\nSTAGE SCORE: ");
   Serial.print(score());
   Serial.print("\n\n");   
   
   if (controller.attached()) {
      controller.lcdp()->setCursor(0,2);
      controller.lcdp()->print("2: ");
      controller.lcdp()->print(String(score()));
      controller.lcdp()->print(" #");
      controller.lcdp()->print(patternIndex);
      controller.lcdp()->print(" ");
      controller.lcdp()->print(String(hitReport));
   }
}


/* Calculates the score for stage 2 based on the number of hits during the
 *    activated and deactivated force fields. When deactivated, the penalty
 *    is always 50 points, but when activated, the number of points awarded
 *    increases with each hit.
 */
int Stage2::score(void) {
  int goodHitPoints[] = { 0, 40, 105, 150, 210, 290 };
  int badHits = 0;
  int goodHits = 0;
  int score = 0;
  
  for (char *cptr=hitReport; *cptr; cptr++) {
      if ('-' == *cptr) {
         badHits++;
      } else if ('+' == *cptr) {
         goodHits++;
      }
  }

  score = goodHitPoints[goodHits] - (badHits * 50);
  if (0 > score) {
     score = 0;
  }

  return score;
}


/* Interrupt routine that is triggered on every vibration hit. It just
 *    increments a global variable that is checked within the state machine.
 */
static void vibrate() {
  hit++;
}


/* This returns TRUE if a hit has been detected since the last time it was
 *    invoked (and resets the hit counter), else returns FALSE
 */
static int hit_detected(void) {
  int detected = 0;
  
  if (hit && !ignore_hits) {
     detected = 1;
     hit = 0;
  }
  
  return detected;
}


/* Lights up the lightsaber all one color 
 */
static void singleColor(uint32_t c) {
  for (uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
  }
  strip.show();
}


/* Activates (or deactivates) the magnetic force depending on the state parameter
 */
static void activateField(boolean state) {
   digitalWrite(FIELD_PIN, state ? HIGH : LOW); 
}

