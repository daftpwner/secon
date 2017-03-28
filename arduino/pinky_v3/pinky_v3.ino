

/*
 * This code is uploaded on the Arduino UNO for Pinky functionality.
 * 
 * Authors: Brett Farris and Bryan LaGrone and Garrett Porter
 */
 
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

#include <AFMotor.h>
#include <Servo.h> 

#define delta_field 13
#define field_change_time 300

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

// Keeps track of four subsequent strikes over 30 seconds
int strike_count = 0; 

// reference coordinates
double off_vect = 0;
double on_vect = 0;
double mid_vect = 0;

// testing coordinates
double current_vect = 0;

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


void loop() {
  while(stop_check);
  Serial.print("Begin Driving\n");
  while (digitalRead(BUMP_SWITCH) == HIGH){
    if (stop_check) break;

    // setting motors to drive forward
    servo2.write(97);
    servo1.write(83);
  }

  // stop motors
  servo2.write(90);
  servo1.write(90);

  // detach motors to prevent erronous motion
  servo2.detach();
  servo1.detach();

  while(stop_check);

  // Getting the first on_vect value
  sensors_event_t event;
  mag.getEvent(&event);
  on_vect = sqrt(event.magnetic.x * event.magnetic.x + event.magnetic.y * event.magnetic.y +event.magnetic.z * event.magnetic.z);

  Serial.print("Reached Stage 2\nStarting Fight\n\tOn value: ");
  Serial.println(on_vect);

  servo3.write(90);
  delay(250); // waiting time for hit
  servo3.write(130);

  // Waiting for intial field to turn off
  delay(300);

  // Getting off value
   
  mag.getEvent(&event);
  off_vect = sqrt(event.magnetic.x * event.magnetic.x + event.magnetic.y * event.magnetic.y +event.magnetic.z * event.magnetic.z);

  // making mid point value
  mid_vect = (on_vect - off_vect)/2 + off_vect; // This method was checked for accuracy
  
  if (on_vect > mid_vect){
    while (strike_count < 4){
      if (stop_check) break;
      
      // Getting current value
      mag.getEvent(&event);
      current_vect = sqrt(event.magnetic.x * event.magnetic.x + event.magnetic.y * event.magnetic.y +event.magnetic.z * event.magnetic.z);

      // off_loop
      while(current_vect < mid_vect){
        if (stop_check) break;
        
        // Getting current value
        mag.getEvent(&event);
        current_vect = sqrt(event.magnetic.x * event.magnetic.x + event.magnetic.y * event.magnetic.y +event.magnetic.z * event.magnetic.z);
      }
      
      // wait for field to finish rising
      delay(field_change_time);
      
      // Getting on value
      mag.getEvent(&event);
      on_vect = sqrt(event.magnetic.x * event.magnetic.x + event.magnetic.y * event.magnetic.y +event.magnetic.z * event.magnetic.z);

      // making new mid point value
      mid_vect = (on_vect - off_vect)/2 + off_vect; // This method was checked for accuracy
      
      // increasing strike_count
      strike_count++;
      
      Serial.print(strike_count, DEC);
      Serial.print(" Strike Occurred\n\tON_value: ");
      Serial.println(on_vect);

      if (stop_check) break;
      
      // Hit the light saber
      servo3.write(90);
      delay(250); // waiting time for hit
      servo3.write(130);
      
      // Getting current value
      mag.getEvent(&event);
      current_vect = sqrt(event.magnetic.x * event.magnetic.x + event.magnetic.y * event.magnetic.y +event.magnetic.z * event.magnetic.z);

      // off_loop
      while(current_vect > mid_vect){
        if (stop_check) break;
        
        // Getting current value
        mag.getEvent(&event);
        current_vect = sqrt(event.magnetic.x * event.magnetic.x + event.magnetic.y * event.magnetic.y +event.magnetic.z * event.magnetic.z);
      }
      
      // wait for field to finish rising
      delay(field_change_time);
      
      // Getting on value
       
      mag.getEvent(&event);
      off_vect = sqrt(event.magnetic.x * event.magnetic.x + event.magnetic.y * event.magnetic.y +event.magnetic.z * event.magnetic.z);

      // making new mid point value
      mid_vect = (on_vect - off_vect)/2 + off_vect; // This method was checked for accuracy
      
      // printing out off value for us
      Serial.print("\tOFF_value: ");
      Serial.println(off_vect);
    }
  }
  else{
    while (strike_count < 4){
      if (stop_check) break;
      
      // Getting current value
      mag.getEvent(&event);
      current_vect = sqrt(event.magnetic.x * event.magnetic.x + event.magnetic.y * event.magnetic.y +event.magnetic.z * event.magnetic.z);

      // off_loop
      while(current_vect > mid_vect){
        if (stop_check) break;
        
        // Getting current value
        mag.getEvent(&event);
        current_vect = sqrt(event.magnetic.x * event.magnetic.x + event.magnetic.y * event.magnetic.y +event.magnetic.z * event.magnetic.z);
      }
      
      // wait for field to finish rising
      delay(field_change_time);
      
      // Getting on value
      mag.getEvent(&event);
      on_vect = sqrt(event.magnetic.x * event.magnetic.x + event.magnetic.y * event.magnetic.y +event.magnetic.z * event.magnetic.z);

      // making new mid point value
      mid_vect = (on_vect - off_vect)/2 + off_vect; // This method was checked for accuracy
      
      // increasing strike_count
      strike_count++;
      
      Serial.print(strike_count, DEC);
      Serial.print(" Strike Occurred\n\tON_value: ");
      Serial.println(on_vect);
      
      if (stop_check) break;
      
      // Hit the light saber
      servo3.write(90);
      delay(250); // waiting time for hit
      servo3.write(130);
      
      // Getting current value
      mag.getEvent(&event);
      current_vect = sqrt(event.magnetic.x * event.magnetic.x + event.magnetic.y * event.magnetic.y +event.magnetic.z * event.magnetic.z);

      // off_loop
      while(current_vect < mid_vect){
        if (stop_check) break;
        
        // Getting current value
        mag.getEvent(&event);
        current_vect = sqrt(event.magnetic.x * event.magnetic.x + event.magnetic.y * event.magnetic.y +event.magnetic.z * event.magnetic.z);
      }
      
      // wait for field to finish rising
      delay(field_change_time);
      
      // Getting on value
      mag.getEvent(&event);
      off_vect = sqrt(event.magnetic.x * event.magnetic.x + event.magnetic.y * event.magnetic.y +event.magnetic.z * event.magnetic.z);

      // making new mid point value
      mid_vect = (on_vect - off_vect)/2 + off_vect; // This method was checked for accuracy
      
      // printing out off value for us
      Serial.print("\tOFF_value: ");
      Serial.println(off_vect);
    }
  }

  servo3.write(180); // putting the arm back in default position
}


void STOP(){

   // Emergency Stop Pressed
   stop_check = 1;
   // Press reset to restart
}

