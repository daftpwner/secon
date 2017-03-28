/********************************************************************
 *
 * SoutheastCon 2017 Arena control - Stage3.cpp
 *
 * This is the code file for the stage #3 control elements. 
 *
 ********************************************************************/

#include <MsTimer2.h>

#include "Arduino.h"
#include "Stage3.h"

#include "SimplePinChange.h"

#include "Stage1.h"
extern Stage1 stage1;
#include "Controller.h"
extern Controller controller;

#define ENCODER_A_PIN     3     // This one is an interrupt pin
#define ENCODER_B_PIN     4     // This is a standard (not interrupt) pin

#define ENABLE_LED_PIN    7
#define RED_LED_PIN       8
#define GREEN_LED_PIN     9
#define BLUE_LED_PIN     10

#define ONE_REVOLUTION  96
#define ONE_CLICK        4
#define TWO_CLICKS       8
#define PLUS_MINUS       6

#define CENTER_WHITE  0x0
#define RIGHT_RED     0x1
#define LEFT_BLUE     0x2

static volatile long encoderValue = 0;          // updated by the encoder interrupt
static int oldState = 0;                        // previous quadrature interrupt pin state
static int blink = 0;                           // on/off value of blink led - updated by MsTimer2 irq
static int blinkEnabled = true;                 // true if blink enabled (off after motion)

#define NOT_MOVED (4269)                        // 6*9 = 42, in base 13, Hitchiker's Guide to the Galaxy
static int prevCenter = 1;                      // were we in the center on the last time we checked?
static int prevTurns = 0;                       // previous number of turns (at last digit entered)
static int prevPosition = NOT_MOVED;            // previous position (last time digit entered)
static boolean enteringClockwise = 1;           // did we enter center in a clockwise direction?
static boolean exitingClockwise = 0;            // did we exit center in a clockwise direction?
static boolean lastDigitClockwise = true;       // was last digit entered in clockwise direction?

static uint16_t turnPattern = 0;                // turns pattern as chosen by stage 1 relays

static int digitCounter = 0;                    // number of digits stored
static char digitString[10] = { '\0' };         // Printable version of the digits stored
static int stageScore = 0;                      // Stage score

#define MAX_DIGITS_STORED 32                       // Maximum number of digits (only last 5 count)
static uint8_t digits[MAX_DIGITS_STORED] = { 0 };  // digits stored on each cw/ccw or ccw/cw transition

static void addDigit(void);
static boolean inCenter(long);
static boolean movementDetected(long *encoder, int *center);
static void blinkQuadratureLEDs(void);
static void updateEncoder(void);
static void calculateScore(void);


Stage3::Stage3() 
{
}

void Stage3::start() 
{
  
  /* Set both quadrature channels to input and turn on pullup resistors */
  pinMode(ENCODER_A_PIN, INPUT_PULLUP); 
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);

  /* Get the initial state of the two quadrature pins */
  oldState = 0;
  if (digitalRead(ENCODER_A_PIN)) oldState |= 1;
  if (digitalRead(ENCODER_B_PIN)) oldState |= 2;

  /* invoke interrupt routine on each edge of ENCODER_A_PIN */
  SimplePinChange.attach(ENCODER_A_PIN, updateEncoder);
  SimplePinChange.attach(ENCODER_B_PIN, updateEncoder);  
  
  /* Setup the quadrature encoder pin modes */
  pinMode(ENABLE_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN,    OUTPUT);
  pinMode(GREEN_LED_PIN,  OUTPUT);
  pinMode(BLUE_LED_PIN,   OUTPUT);

  /* Start the blink at 5hz rate (200ms period, 100ms irq rate */
  MsTimer2::set(100, blinkQuadratureLEDs);
  MsTimer2::start();

  /* Initial state of the LEDs is blinking white */
  digitalWrite(RED_LED_PIN,    LOW);
  digitalWrite(GREEN_LED_PIN,  LOW);
  digitalWrite(BLUE_LED_PIN,   LOW);
  digitalWrite(ENABLE_LED_PIN, HIGH);

  turnPattern = stage1.turnPattern;
  
  Serial.print(F("pattern="));
  Serial.println(turnPattern);
}


void Stage3::stop(uint32_t timestamp) 
{
  /* Stop the blink LEDs */
  MsTimer2::stop();
  detachInterrupt(1);
  delay(1);

  /* Turn off the leds and the blink so the knob is off at contest end */
  digitalWrite(RED_LED_PIN,    HIGH);
  digitalWrite(GREEN_LED_PIN,  HIGH);
  digitalWrite(BLUE_LED_PIN,   HIGH);
  digitalWrite(ENABLE_LED_PIN, LOW);
  
  calculateScore();
}


void Stage3::step(uint32_t timestamp) 
{
   long encoder;
   int center;
   int turns;

   /* Check if motion was detected, and control the quadrature red/white/blue
    *    LEDs. and to return the current encoder value.  We can't use a 'live'
    *    encoder value below as we could have a race condition if the encoder
    *    changed while in the code below
    */
   if (!movementDetected(&encoder, &center)) {
      return;
   }

#if 0
   if (!blinkEnabled) {
      Serial.print(F("encoder="));
      Serial.println(encoder);
   }
#endif

   /* General logic of the code below...
    *  
    *  We really only care if we are transitioning into, or out of, a center - no
    *  other motion counts. This allows the robot to move the quadrature back and 
    *  forth and all that motion is filtered out - until we cross a center.
    *  
    *  Once we cross into a center we need to store the the number of turns and
    *  a flag indicating if we entered it clockwise or counterclockwise
    *  
    *  Once we exit a center, if we exit it in the same direction we entered it,
    *  we have nothing to do (just continuing motion in the same direction). However,
    *  if we exit it in a different direction than we entered, we are entering a
    *  digit. The only extra issue here is to verify we are entereing digits in a
    *  clockwise, counterclockwise sequence.
    *  
    *  Note that if we start off moving in a counterclockwize direction the first
    *  time, we are lost, and the system will not recover - this is an error state
    *  as defined in the rules (must start clockwise).
    */
    
    /* Are we currently in a center region? */
    if (center) {
      
      /* did we just transition into center on this step? */
      if (!prevCenter) {
         turns = ((encoder > 0) ? (encoder + (ONE_REVOLUTION/2))
                                : (encoder - (ONE_REVOLUTION/2))) / ONE_REVOLUTION;
         enteringClockwise = (encoder < (turns*ONE_REVOLUTION));
#if 1 
         Serial.print(F("Entering center @ "));
         Serial.print(encoder);
         Serial.print(F(", turns="));
         Serial.print(turns);
         Serial.print(F(", enteringClockwise="));
         Serial.println(enteringClockwise);
#endif

         prevTurns = turns;
      }

   /* we are not in the center */
   } else {

      /* did we just transition out of the center on this step? */
      if (prevCenter) {

         exitingClockwise = (encoder > (prevTurns*ONE_REVOLUTION));
#if 1
         Serial.print(F("Exiting center @ "));
         Serial.print(encoder);
         Serial.print(F(", exitingClockwise="));
         Serial.println(exitingClockwise);
#endif

         /* If we entered, then exited the center in opposite directions
          * then we have just dialed a digit
          */
         if (enteringClockwise != exitingClockwise) {

#if 1
            Serial.print(F("lastDigitClockwise "));
            Serial.println(lastDigitClockwise);
            Serial.print(F("prevturns "));
            Serial.println(prevTurns);
#endif

            /* However, we only count digits entered in alternating
             * clockwise and counterclockwise directions
             */
            if (lastDigitClockwise != exitingClockwise) {
               addDigit();
            }
         }
      }
   }  

   /* Save our previous center state for our next time */
   prevCenter = center;
}


void Stage3::report(void) 
{
   Serial.print(F("------ Stage 3 report ------\n"));
   Serial.print(F("STAGE SCORE: "));
   Serial.println(stageScore);
   Serial.print(F("Digits entered: "));
   Serial.println(digitString);
   
   if (controller.attached()) {
      controller.lcdp()->setCursor(0,3);
      controller.lcdp()->print("3: ");
      controller.lcdp()->print(String(stageScore));
      controller.lcdp()->print(" ");
      controller.lcdp()->print(String(digitString));
   }
 
}


int Stage3::score(void) 
{
  /* Already calculated by the report function */
  return stageScore;
}


/* This handles the logic of adding a digit, calculating the digit value
 *  from the current and previous positions. This was broken out of the 
 *  rest of the code so it can be called for the last digit from the report
 *  function
 */
static void addDigit(void)
{
#if 1
   Serial.print(F("prevPosition="));
   Serial.println(prevPosition);
#endif

   /* If this is the first digit, the digit is just the number of turns */
   if (prevPosition == NOT_MOVED) {
       digits[digitCounter] = prevTurns;

    /* Else we need to subtract the last position to get the delta number of
     *  turns. We have tow cases here - traveling clockwise or counterclockwise
     */
    } else if (enteringClockwise) {
       digits[digitCounter] = prevTurns - prevPosition;
    } else {
       digits[digitCounter] = prevPosition - prevTurns;
    }

#if 1
    Serial.print(F("Adding digit: ")); 
    Serial.println(digits[digitCounter]);
#endif

    /* Update our global variables for the next digit */
    lastDigitClockwise = exitingClockwise;
    prevPosition = prevTurns;
    digitCounter++;
}


static boolean inCenter(long enc) 
{
   int relativeEncoder;
   
   /* Calculate the relative value +/- from center position */
   relativeEncoder = (int) (abs(enc) % ONE_REVOLUTION);
   if (relativeEncoder > (ONE_REVOLUTION / 2)) {
      relativeEncoder -= ONE_REVOLUTION;
   }
   

   /* Center is true if we are +/- a small margin of the center */
   return abs(relativeEncoder) <= PLUS_MINUS;   
}


/* Process the digits array to convert it to a printable string and 
 * calculate the stage score 
 */
static void calculateScore(void) {

   int loop;
   int numDigits;
   int numCorrect = 0;
   uint16_t pattern;
   int goodDigitPoints[] = { 0, 45, 95, 155, 230, 325 };
   int powersOfTen[] = { 10000, 1000, 100, 10, 1 };

   /* If we are in the center and moved (no longer blinking), then
    * don't forget to add in the last digit before calculating the score
    */
   if (!blinkEnabled && inCenter(encoderValue)) {
      Serial.println(F("Adding last digit"));
      addDigit();
   }
   
   /* Handle the trivial case of no digits entered */   
   if (digitCounter < 0) {
      strcpy(digitString, "N/A");
      stageScore = 0;
      return;
   }

   /* Else loop through all digits in the wrap-around buffer, 
    * converting to printable digits
    */
   numDigits = (digitCounter > 5) ? 5 : digitCounter;
   for (loop=numDigits-1; loop >= 0; loop--) {
      digitCounter--;
      if (digitCounter < 0) {
         digitCounter += MAX_DIGITS_STORED;
      }

      /* Handle values outside of [0..9] so not converted to non-printable ASCII */
      if ((digits[digitCounter] >= 0) && (digits[digitCounter] <= 9)) {
         digitString[loop] = digits[digitCounter] + '0';
      } else {
         digitString[loop] = '?';
      }
   }
   
#if 1
   Serial.print(F("pattern="));
   Serial.println(turnPattern);
#endif

   /* Now loop through the digits and count how many are correct */
   pattern = turnPattern;
   for (loop=0; loop < numDigits; loop++) {
      if ((((int)(pattern / powersOfTen[loop])%10)) == (digitString[loop]-'0')) {
         numCorrect++;
      }
   }

#if 1
   Serial.print(F("Num digits correct "));
   Serial.println(numCorrect);
#endif
   
   /* Map the number of correct digits to the stage 3 score */
   stageScore = goodDigitPoints[numCorrect];
}


uint32_t movementHistory = 0;
long prevEncoderValue = 0;

#define MOVEMENT_HISTORY_SIZE  4
#define MOVEMENT_MASK_WIDTH    2
#define MOVEMENT_HISTORY_MASK  ((1 << MOVEMENT_MASK_WIDTH) - 1)

/*
 * This function is called to turn the encoder value into a true/false
 * is in center flag and to track the last 'MOVEMENT_HISTORY_SIZE' 
 * number of motions to determine if we are moving left or right. This
 * is needed to filter out small motions left/right causing the LEDs 
 * in teh quadrathre to light up red/blue - this is only for visual use
 * and nothing more.
 */
static boolean movementDetected(long *encoder, int *center) 
{
   int loop; 
   int curDirection;
   int rightMotion  = 0;
   int leftMotion   = 0;
     
   /* If no motion, then return with nothing else to do */
   *encoder = encoderValue;
   if (prevEncoderValue == *encoder) {
       return false;
    }
   
   /* if we are center, then set the quadrature LEDs to white and wipe
    * out all past history. This is needed as if we are moving right to 
    * white, then reverse to left, we have more right motion than left 
    * and can temporarily light the LEDs the wrong color
    */
   *center = inCenter(*encoder);   
   if (*center) {
      curDirection = CENTER_WHITE;
      movementHistory = 0;
   
   /* Else if not int the center, we keep a running history of the last 'N'
      movement directions to average out small +/- movements setting the color
    */   
   } else {
     
      /* convert the current position to either center, or left/right of center */
      if (prevEncoderValue < *encoder) {
         curDirection = RIGHT_RED;
      } else {
         curDirection = LEFT_BLUE;
      } 
 
      /* movement history contains the history of last 'n' movements */
      movementHistory <<= MOVEMENT_MASK_WIDTH;
      movementHistory |= curDirection;
      movementHistory &= ((1 << (MOVEMENT_MASK_WIDTH*MOVEMENT_HISTORY_SIZE)) - 1);
     
      /* count the left and right movements to determine the major move direction */
      uint32_t history = movementHistory; 
     
      /* count number of 1 (right) or 0 (left) bits to determine the average direction */
      for (loop=0; loop < MOVEMENT_HISTORY_SIZE; loop++) {
         if ((history & MOVEMENT_HISTORY_MASK) == RIGHT_RED) {
            rightMotion++;
         } else if ((history & MOVEMENT_HISTORY_MASK) == LEFT_BLUE) {
            leftMotion++; 
         }
         history >>= MOVEMENT_MASK_WIDTH;
      }

      /* if we are currently at center, light the LED white, else red or 
       *   blue based on the motion counts above
       */
      if (leftMotion > rightMotion) {
         curDirection = LEFT_BLUE;
      } else {
         curDirection = RIGHT_RED;
      }
   }
   
   /* Tricky code to handle the 3x conditions of red, white and blue for
    *   the quadrature encoding.  Green is only on if we are in the center
    *   position (not on for left or right), while red is off only when
    *   not heading left, and blue is only on when not heading right.
    */
   digitalWrite(GREEN_LED_PIN, !(CENTER_WHITE == curDirection));
   digitalWrite(RED_LED_PIN,   !(LEFT_BLUE    != curDirection));
   digitalWrite(BLUE_LED_PIN,  !(RIGHT_RED    != curDirection));

   /* If we moved out of the center area, disable blinking of the LEDs */
   if ((CENTER_WHITE != curDirection) && (blinkEnabled)) {
      blinkEnabled = false;
   }

   /* update previous encoder value for the next iteration */
   prevEncoderValue = *encoder;

   /* return true indicating we did have motion */
   return true;
}  


/* Toggle the quadrature LEDs on and off This function is invoked via
 *    the a timeout counter callback at a rate of 10Hz (100ms) to generate
 *    a 5Hz blink rate.
 */
static void blinkQuadratureLEDs() {
  static boolean onOff = HIGH;
  
  /* if blink is enabled, then toggle the enable line */
  if (blinkEnabled) {
     digitalWrite(ENABLE_LED_PIN, onOff);
     onOff = !onOff;
     
  /* else turn off the blink timer and set the enable pin to always on */
  } else {
     MsTimer2::stop();
     digitalWrite(ENABLE_LED_PIN, HIGH);
  }
}


/* This quadrature encoder interrupt handler is based heavily
 *   on an the implementation in the Encoder library at:
 *
 *     https://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * First an overview of the two lines in a quadrature encoder:
 *               _____       _____
 *   Pin1:  ____|     |_____|     |____
 *            _____       _____       _  
 *   Pin2:  _|     |_____|     |_____|
 *
 *        <--- negative     positive --->
 *
 * By examining the current two pins and the two previous
 *    pins, we can determine the position and amount of
 *    motion (assume we can respond fast enough to only 
 *    lose at most 1 transition).
 *
 *   new    new    old    old
 *   pin2   pin1   pin2   pin1   Result
 *   ----   ----   ----   ----   ------
 *    0      0      0      0     no movement
 *    0      0      0      1     +1
 *    0      0      1      0     -1
 *    0      0      1      1     +2
 *    0      1      0      0     -1
 *    0      1      0      1     no movement
 *    0      1      1      0     -2
 *    0      1      1      1     +1
 *    1      0      0      0     +1
 *    1      0      0      1     -2
 *    1      0      1      0     no movement
 *    1      0      1      1     -1
 *    1      1      0      0     +2
 *    1      1      0      1     -1
 *    1      1      1      0     +1
 *    1      1      1      1     no movement
 *   ----   ----   ----   ----   ------
 */

/* state change based on table above */
static const int stateChange[16] = 
{  
   0, +1, -1, +2, -1,  0, -2, +1, 
  +1, -2,  0, -1, +2, -1, +1,  0 
};

static void updateEncoder(void) 
{
  /* Build up the 4bit state of current and past pin values */
  uint8_t state = oldState & 3;
  if (digitalRead(ENCODER_A_PIN)) state |= 4;
  if (digitalRead(ENCODER_B_PIN)) state |= 8;

  /* Based on 4-bit state value, update encoder value */
  encoderValue += stateChange[state];  

  /* Save our current two pins for the next interrupt */
  oldState = (state >> 2);
}


