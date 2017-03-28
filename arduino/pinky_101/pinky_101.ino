

/*
 * This code is uploaded on the Arduino UNO for Pinky functionality.
 * 
 * Authors: Brett Farris, Garrett Porter, and Bryan LaGrone
 */
 
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

#include <AFMotor.h>
#include <Servo.h> 

// TWEAK this parameter to the field strength
#define delta_field 5

// Wheels
Servo servo1; // Right wheel
Servo servo2; // Left wheel

// Hit Stick
Servo servo3;

// Bump Switch 
#define BUMP_SWITCH 2
#define E_STOP 3 // Starting push button
#define START 4

// Assign and ID to the magnetic field sensor
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

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
  pinMode(START, INPUT_PULLUP); // start

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
  
  // Used to control when Pinky starts via the serial monitor
  // To start pinky, simply type in "s" and send
  while (digitalRead(START) == 1){ 

    /*
    // Beginning course run
    Serial.print("Begin Driving\n");
    */
  
    // Drive to Stage 2
    while ((digitalRead(BUMP_SWITCH) == HIGH) && (stop_check == 0)){
      
      // Drive to Stage 2
      servo2.write(97);
      servo1.write(80);
      
    }
    
    // Stage 2 reached, stop driving
    servo2.write(90);
    servo1.write(90);

    servo2.detach();
    servo1.detach();

    // check for E_STOP
    if (stop_check == 1){
        break;
    }

    /*
    // Telling user that stage 2 has been reached
    Serial.print("\tStage 2 Reached\n");
    Serial.println("Swinging once");
    */
    
    // First of five strikes
    servo3.write(80);
    delay(400); // Time for hit to contact and then wait
    servo3.write(130); // come back
    delay(400);

    // check for E_STOP
    if (stop_check == 1){
      break;
    }
    
    // Get reference coordinates
    sensors_event_t event; 
    mag.getEvent(&event);
    ref_vect = sqrt(event.magnetic.x * event.magnetic.x + event.magnetic.y * event.magnetic.y +event.magnetic.z * event.magnetic.z);

    /*
    // Printing out reference field
    Serial.print("\tReference Field: ");
    Serial.println(ref_vect);
    */
  
    // Four more strikes possible
    while (strike_count < 4){
      
      // check for E_STOP
      if (stop_check == 1){
        break;
      }
      // Get new magnetic value
      sensors_event_t event; 
      mag.getEvent(&event);
      new_vect = sqrt(event.magnetic.x * event.magnetic.x + event.magnetic.y * event.magnetic.y +event.magnetic.z * event.magnetic.z);
      
      // Checking magnetic vector magnitude
      if (new_vect - ref_vect > delta_field || new_vect - ref_vect < -delta_field){
        servo3.write(80); // strike
        delay(400); // Time for hit to contact and then wait
        servo3.write(130); // pull back
        delay(400);

        strike_count = strike_count + 1;

        /*
        Serial.print(strike_count, DEC);
        Serial.println(" Strike Occurred");
        Serial.print("\tField Change: ");
        Serial.println(new_vect);
        */

        sensors_event_t event; 
        mag.getEvent(&event);
        ref_vect = sqrt(event.magnetic.x * event.magnetic.x + event.magnetic.y * event.magnetic.y +event.magnetic.z * event.magnetic.z);

        /*
        Serial.print("\tNew Reference: ");
        Serial.println(ref_vect);
        */
      }
    }
    stop_check = 1;
    servo3.write(180);
  }
}


void STOP(){

   // Emergency Stop Pressed
   stop_check = 1;
   servo2.detach();
   servo1.detach();
   servo3.detach();
}

