// FAILED

/*
 * This code is uploaded on the Arduino UNO for Pinky functionality.
 * 
 * Authors: Brett Farris and Bryan LaGrone
 */
 
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

#include <AFMotor.h>
#include <Servo.h> 

// Variable Tuning Values
#define delta_field 12 // This value is the minimum change to detect a field activation
#define field_fall_time 300 // This value is the wait time after first detecting the field to fall

// Wheels
Servo servo1; // Right wheel
Servo servo2; // Left wheel

// Hit Stick
Servo servo3;

// Bump Switch 
#define BUMP_SWITCH 2
#define E_STOP 3 // Starting push button

// Assign and ID to the magnetic field sensor
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

enum state{
  Waiting,
  Starting,
  Driving,
  Field_OFF,
  Field_ON,
  Finished,
  Stopped
};

// State control
state current_state = Starting;

// Keeps track of four subsequent strikes over 30 seconds
int strike_count = 0; 

// reference coordinates
double ref_vect = 0;

// updating coordinates
double new_vect = 0;

// E_Stop check
int stop_check = 0; // 1: E_STOP pressed

void setup() {
  
  Serial.begin(115200);  // set up Serial library at 115200 bps

  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
  delay(500);
  
  // attach servos
  servo3.attach(7);
  servo2.attach(9);
  servo1.attach(10);

  // initialize bump switch
  pinMode(BUMP_SWITCH, INPUT_PULLUP);
  pinMode(E_STOP, INPUT_PULLUP); // Emergency stop 

  // attach emergency stop interrupt
  attachInterrupt(digitalPinToInterrupt(E_STOP),STOP, FALLING);

  // Initialize hit stick
  servo3.write(180);

  // Stop wheels
  servo2.write(90);
  servo1.write(90);

}


// Test the servo 
void loop() {
  switch (current_state) {
    case Starting: {
      // Assigning next state of system
      current_state = Driving;

      Serial.print("Begining Driving\n");

      // Writing drive values to continuous rotation servos
      servo2.write(97);
      servo1.write(83);
      break;
    }
    case Driving: {
      if (digitalRead(BUMP_SWITCH) == LOW){
        // Assigning next state of system
        current_state = Field_OFF;

        Serial.print("\tStage 2 Reached\n");
        
        // Stage 2 reached, stop driving
        servo2.write(90);
        servo1.write(90);

        // Removing control of continuous rotation motors to prevent accidential motion
        servo2.detach();
        servo1.detach();

        // First of five strikes
        servo3.write(90);
        Serial.println("First Hit");
        delay(300); // Time for hit to contact and then wait
        servo3.write(130); // come back
        delay(500); // Used to wait till the field falls

        // Get reference coordinates
        sensors_event_t event; 
        mag.getEvent(&event);
        ref_vect = sqrt(event.magnetic.x * event.magnetic.x + event.magnetic.y * event.magnetic.y +event.magnetic.z * event.magnetic.z);
    
        // Printing out reference field
        Serial.print("\tReference Field: ");
        Serial.println(ref_vect);
        break;
      }
    }
    case Field_ON: {
      // Get new reference magnetic vector
      sensors_event_t event; 
      mag.getEvent(&event);
      ref_vect = sqrt(event.magnetic.x * event.magnetic.x + event.magnetic.y * event.magnetic.y +event.magnetic.z * event.magnetic.z);
      
      // Waiting to assign new field value until magnetic field has been turned off
      if (new_vect - ref_vect > delta_field || new_vect - ref_vect < -delta_field){
        current_state = Field_OFF;
        delay(field_fall_time); // Used to prevent sudden hits when the magnetic field is still dropping, will probably need finer tuning on timing.

        // Gets final reference value
        sensors_event_t event; 
        mag.getEvent(&event);
        ref_vect = sqrt(event.magnetic.x * event.magnetic.x + event.magnetic.y * event.magnetic.y +event.magnetic.z * event.magnetic.z);
        break;
      }
    }
    case Field_OFF: {
      // Seeing if the strike limit has been reached
      if (strike_count < 4) {
        // Get new magnetic value
        sensors_event_t event; 
        mag.getEvent(&event);
        new_vect = sqrt(event.magnetic.x * event.magnetic.x + event.magnetic.y * event.magnetic.y +event.magnetic.z * event.magnetic.z);
  
        // Checking magnetic vector magnitude
        if (new_vect - ref_vect > delta_field || new_vect - ref_vect < -delta_field){
          current_state = Field_ON;
          servo3.write(90); // strike
          delay(300); // Time for hit to contact and then wait
          servo3.write(130); // pull back
          delay(500);
  
          strike_count = strike_count + 1;
          Serial.print(strike_count, DEC);
          Serial.println(" Strike Occurred");
          
          Serial.print("\tField Change: ");
          Serial.println(new_vect);
        }
      }
      else {
        current_state = Finished; 
        break;
      }
    }
    case Finished:
    case Stopped: {
      servo3.write(180);
    }
  }
}

void STOP(){

   // Emergency Stop Pressed
   current_state = Stopped;
   // Press reset to restart
}

