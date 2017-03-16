

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

// Wheels
//Servo servo1; // Right wheel
//Servo servo2; // Left wheel

// Hit Stick
//Servo servo3;

// Bump Switch 
#define BUMP_SWITCH 2

// Assign and ID to the magnetic field sensor
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// Keeps track of four subsequent strikes over 30 seconds
int strike_count = 0; 

// reference coordinates
int ref_x = 0;
int ref_y = 0;
int ref_z = 0;

// updating coordinates
int new_x = 0;
int new_y = 0;
int new_z = 0;

void setup() {
  
  Serial.begin(115200);  // set up Serial library at 9600 bps

  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
  delay(500);
  
  // attach servos
  //servo3.attach(7);
  //servo2.attach(9);
  //servo1.attach(10);

  // initialize bump switch
  pinMode(BUMP_SWITCH, INPUT_PULLUP);

  // Initialize hit stick
  //servo3.write(180);

  // Stop wheels
  //servo2.write(90);
  //servo1.write(90);

}


// Test the servo 
void loop() {

  Serial.print("Begin Driving");
  Serial.print("\n");

  // Drive to Stage 2
  while (digitalRead(BUMP_SWITCH) == HIGH){
    // Drive to Stage 2
    //servo2.write(97);
    //servo1.write(83);
  }

  // Stage 2 reached, stop driving
  //servo2.write(90);
  //servo1.write(90);

  Serial.print("Stage 2 Reached");
  Serial.print("\n");
  
  // First of five strikes
  //servo3.write(102);
  Serial.print("Swinging once");
  Serial.print("\n");
  delay(250);
  //servo3.write(130); // come back
  //delay(250);
  
  // Get reference coordinates
  sensors_event_t event; 
  mag.getEvent(&event);
  ref_x = event.magnetic.x;
  ref_y = event.magnetic.y;
  ref_z = event.magnetic.z;

  Serial.println("Reference Field: ");
  Serial.print("X: ");
  Serial.println(ref_x);
  Serial.print("Y: ");
  Serial.println(ref_y);
  Serial.print("Z: ");
  Serial.println(ref_z);

  // Four more strikes possible
  while (strike_count < 4){

    // Get new reference coordinates
    sensors_event_t event; 
    mag.getEvent(&event);
    new_x = event.magnetic.x;
    new_y = event.magnetic.y;
    new_z = event.magnetic.z; 
    
    // check coordinates to see if there is a variation of at least 3; 3 was chosen arbitrary
    if ((ref_x < new_x - 3) || (ref_x > new_x + 3) || (ref_y < new_y - 3) || (ref_y > new_y + 3) || (ref_z < new_z - 3) || (ref_z > new_z + 3)){
      //servo3.write(102); // strike
      //delay(250);
      //servo3.write(130); // pull back
      Serial.println("Strike Occurred");
      //delay(250);
      
      Serial.println("Field Change: ");
      Serial.print("X: ");
      Serial.println(new_x);
      Serial.print("Y: ");
      Serial.println(new_y);
      Serial.print("Z: ");
      Serial.println(new_z);
      
      strike_count = strike_count + 1;
      
      /* 
       *  Field turns off when hit is detected; minimal field "off" time is 1 second.
       *  Thus, need to get a new reference and be waiting sooner than 1 second.
      */
      
      // Get new reference coordinates
      sensors_event_t event; 
      mag.getEvent(&event);
      ref_x = event.magnetic.x;
      ref_y = event.magnetic.y;
      ref_z = event.magnetic.z; 
      
      Serial.println("New Reference: ");
      Serial.print("X: ");
      Serial.println(ref_x);
      Serial.print("Y: ");
      Serial.println(ref_y);
      Serial.print("Z: ");
      Serial.println(ref_z);
    }
  }
}
