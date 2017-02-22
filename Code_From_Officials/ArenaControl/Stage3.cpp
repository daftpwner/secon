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

#include "Controller.h"
extern Controller controller;

#define ENCODER_A_PIN     3     // This one is an interrupt pin
#define ENCODER_B_PIN     4     // This is a standard (not interrupt) pin

#define ENABLE_LED_PIN    7
#define RED_LED_PIN       8
#define GREEN_LED_PIN     9
#define BLUE_LED_PIN     10

#define TICKS_PER_REVOLUTION  96
#define PLUS_MINUS             5

#define CENTER_WHITE  0x0
#define RIGHT_RED     0x1
#define LEFT_BLUE     0x2

volatile long encoderValue = 0;
int oldState = 0;
int blink = 0;
int blinkEnabled = true;
int digits[10] = { 0 };
int digitCounter = 0;


boolean movementDetected(int *movement, int *turns);
void blinkQuadratureLEDs(void);
void updateEncoder(void);


Stage3::Stage3() 
{
}

void Stage3::start() 
{
  /* Set both quadrature channels to input and turn on pullup resistors */
  pinMode(ENCODER_A_PIN, INPUT); 
  pinMode(ENCODER_B_PIN, INPUT);
  digitalWrite(ENCODER_A_PIN, HIGH);
  digitalWrite(ENCODER_B_PIN, HIGH);

  /* Setup the quadrature encoder pin modes */
  pinMode(ENABLE_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN,    OUTPUT);
  pinMode(GREEN_LED_PIN,  OUTPUT);
  pinMode(BLUE_LED_PIN,   OUTPUT);

  /* Start the blink at 5hz rate (200ms period, 100ms irq rate */
  MsTimer2::set(100, blinkQuadratureLEDs);
  MsTimer2::start();

  /* Get the initial state of the two quadrature pins */
  oldState = 0;
  if (digitalRead(ENCODER_A_PIN)) oldState |= 1;
  if (digitalRead(ENCODER_B_PIN)) oldState |= 2;

  /* invoke interrupt routine on each edge of ENCODER_A_PIN */
  attachInterrupt(1, updateEncoder, CHANGE);

  /* Initial state of the LEDs is blinking white */
  digitalWrite(RED_LED_PIN,    LOW);
  digitalWrite(GREEN_LED_PIN,  LOW);
  digitalWrite(BLUE_LED_PIN,   LOW);
  digitalWrite(ENABLE_LED_PIN, HIGH);
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
}


void Stage3::step(uint32_t timestamp) 
{
   int turns, direction;
  
   if (!movementDetected(&turns, &direction)) {
      return;
   }
   
   if (!blinkEnabled) {
      Serial.print("encoder=");
      Serial.print(encoderValue);
      Serial.print(", turns=");
      Serial.print(turns);
      Serial.print(", direction=");
      Serial.println(direction);
   }
   
   digitalWrite(RED_LED_PIN,   !(LEFT_BLUE != direction));
   digitalWrite(BLUE_LED_PIN,  !(RIGHT_RED != direction));
   digitalWrite(GREEN_LED_PIN, !(CENTER_WHITE == direction));

   if ((CENTER_WHITE != direction) && (blinkEnabled)) {
      blinkEnabled = false;
   }

   if ((CENTER_WHITE == direction) && (digits[digitCounter] != turns)) {
      digits[digitCounter] = turns;
   }
}


void Stage3::report(void) 
{
  Serial.print("------ Stage 3 report ------\n");
  Serial.print("STAGE SCORE: ");
  Serial.print(score());
  Serial.print("\n\n");   
 
   if (controller.attached()) {
      controller.lcdp()->setCursor(0,3);
      controller.lcdp()->print("3: ");
      controller.lcdp()->print(String(score()));
      controller.lcdp()->print(" ");
      controller.lcdp()->print(String("32154"));
   }
 
}


int Stage3::score(void) 
{
  return 0;
}

uint32_t movementHistory = 0;
long prevEncoderValue = 0;

#define MOVEMENT_HISTORY_SIZE  7
#define MOVEMENT_MASK          0x3

boolean movementDetected(int *turns, int *movement) 
{
  int offset;
  int saveEncoderValue;
  int loop; 
     
  /* If no motion, then return with nothing else to do */
  saveEncoderValue = encoderValue;
  if (prevEncoderValue == saveEncoderValue) {
     return false;
  }

  if (saveEncoderValue >= 0) {
     *turns = (saveEncoderValue + PLUS_MINUS) / TICKS_PER_REVOLUTION;
     offset = saveEncoderValue % TICKS_PER_REVOLUTION;
  } else {
     *turns = -((PLUS_MINUS - saveEncoderValue) / TICKS_PER_REVOLUTION);
     offset = TICKS_PER_REVOLUTION - ((-saveEncoderValue) % TICKS_PER_REVOLUTION);
  }
  
  Serial.print("*turns="); Serial.print(*turns);
  Serial.print(", offset="); Serial.println(offset);
  
  /* convert the position to either center CENTER_WHITE, or left/right of center */
  if ((offset < PLUS_MINUS) || (offset > (TICKS_PER_REVOLUTION-PLUS_MINUS))) {
     *movement = CENTER_WHITE;
  } else if (prevEncoderValue < encoderValue) {
     *movement = RIGHT_RED;
  } else {
     *movement = LEFT_BLUE;
  } 
 
  /* if not center CENTER_WHITE, filter motion to avoid quick red/blue changes */
  if (CENTER_WHITE != *movement) {
     int rightMotion = 0;
     int leftMotion  = 0;

     /* movement history contains the history of last 'n' movements */
     movementHistory <<= 2;
     movementHistory |= *movement;
     movementHistory &= ((1 << (2*MOVEMENT_HISTORY_SIZE)) - 1);
     
     /* count the left and right movements to determine the major move position */
     uint32_t history = movementHistory; 
     
     for (loop=0; loop < MOVEMENT_HISTORY_SIZE; loop++) {
        if ((history & MOVEMENT_MASK) == RIGHT_RED) {
           rightMotion++;
        } else if ((history & MOVEMENT_MASK) == LEFT_BLUE) {
           leftMotion++;
        }
        history >>= 2;
     }

     /* movement is the result of the left/right filter above */     
     *movement = (rightMotion > leftMotion) ? RIGHT_RED : LEFT_BLUE;
  }
  
     
  /* update previous encoder value for the next iteration */
  prevEncoderValue = saveEncoderValue;

  /* return true indicating we did have motion */
  return true;
}  


/* Toggle the quadrature LEDs on and off This function is invoked via
 *    the a timeout counter callback at a rate of 10Hz (100ms) to generate
 *    a 5Hz blink rate.
 */
void blinkQuadratureLEDs() {
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
const int stateChange[16] = 
{  
   0, +1, -1, +2, -1,  0, -2, +1, 
  +1, -2,  0, -1, +2, -1, +1,  0 
};

void updateEncoder(void) 
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


