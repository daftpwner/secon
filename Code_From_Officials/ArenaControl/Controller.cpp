/********************************************************************
 *
 * SoutheastCon 2017 Arena control - Controller.cpp
 *
 * This is the header file for the lcd/button controller. 
 *
 * The controller is a handheld box that contains the start and 
 * stop buttons, and LCD status/score display.
 *
 ********************************************************************/

#include "Arduino.h"
#include "Controller.h"

#define LCD_ADDRESS   0x27

boolean lcdAttached = false;
boolean initialDisplay = true;


Sainsmart_I2CLCD lcd(LCD_ADDRESS,20,4);

Controller::Controller(uint8_t lcd_Addr, uint8_t lcd_cols, uint8_t lcd_rows) 
{
}


void Controller::start() 
{
  // Check if the LCD is attached
  Wire.beginTransmission (LCD_ADDRESS);
  lcdAttached = (0 == Wire.endTransmission ());

  // If not, start the competition immediately
  if (false == lcdAttached) {
     Serial.println(F("LCD controller not found, starting competition immediately\n"));
     return;
  }
   
  // Else we will use the LCD and buttons, so initialize the buttons as digital input
  int b;
  for (b=A0; b <= A3; b++) {  
     pinMode(b, INPUT_PULLUP);
  }

  // Initialize the display and print the splash screen  
  lcd.init();
  lcd.backlight();
  lcd.home ();
  lcd.print("SoutheastCon 2017");
  lcd.setCursor(0,2);
  lcd.print("[     ] to begin");

  // Now wait here until START button is pressed, updating the display
  int sequence = 0;
  while (BTN_START != (buttons() & BTN_START)) {

    lcd.setCursor(1,2);
    lcd.print((sequence % 3) ? "START" : "     ");

    lcd.setCursor(0,3);
    lcd.print("      ");
    lcd.setCursor((sequence%9)/3,3);
    lcd.print("vvv");
    
    sequence = (sequence + 1) % 9;
    delay(100);
  }

  lcd.clear();
  lcd.print("Match starting in...");
  initialDisplay = true;
}

void Controller::stop(uint32_t timestamp)
{
   if (false == lcdAttached) {
      return;
   }
}


void Controller::step(uint32_t timestamp)
{
   if (false == lcdAttached) {
      return;
   }
   
   lcd.setCursor(0,2);
   
   if (MSECS > timestamp) {
      lcd.print("  THREE seconds...");
      
   } else if ((2*MSECS) > timestamp) {
      lcd.print("  TWO seconds...  ");
      
   } else if ((3*MSECS) > timestamp) {
      lcd.print("  ONE second...   ");
      
   } else if (initialDisplay) {
      initialDisplay = false;
      
      lcd.clear();
      lcd.home();
      lcd.print("Running time: ");
      lcd.setCursor(0,2);
      lcd.print("press here to [    ]");
      
   } else {
      lcd.setCursor(14,0);
      lcd.print((timestamp / MSECS) - COUNTDOWN_TIME);
      lcd.print(".");
      lcd.print((timestamp/100) % 10);

      lcd.setCursor(15,2);
      lcd.print((timestamp % 300) ? "STOP" : "    ");
    
      lcd.setCursor(13,3);
      lcd.print("      ");
      lcd.setCursor(14+(timestamp%900)/300,3);
      lcd.print("vvv");
   }
}


void Controller::report(uint32_t timestamp, int score)
{
   if (false == lcdAttached) {
      return;
   }
   
   uint32_t runTime = 0;

   /* If we are still in the countdown, set the run time to 0 */   
   if (timestamp < COUNTDOWN_TIME * MSECS) {
      runTime = 0;
      
   /* Else deduct the countdown time and round up to nearest second */
   } else {
      runTime = timestamp - (COUNTDOWN_TIME * MSECS) + (MSECS / 2);
   }
   
   lcd.clear();
   lcd.print("SCORE:");
   lcd.print(score);
   lcd.setCursor(11,0);
   lcd.print("TIME:");
   lcd.print(runTime / MSECS);
}



boolean Controller::attached()
{
  return lcdAttached;
}


int Controller::buttons()
{
   // If no controller, always assume buttons are not active
   if (false == lcdAttached) {
       return 0;
    }
     
    int bit;
    int bits = 0;
   
    // Create a bit vector of the button bits at [A0..A3]
    for (bit=0; bit <= 3; bit++) {
       if (0 == digitalRead(A0 + bit)) {
           bits |= (1 << bit);
       }
    }
    
    return bits;
}


Sainsmart_I2CLCD *Controller::lcdp() 
{
   if (false == lcdAttached) {
      Serial.println(F("ERROR: LCD NOT ATTACHED"));
   }
   
   return &lcd;
}
