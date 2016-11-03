/*
 * This code is uploaded on the Arduino Mega 2560 for Brain functionality.
 * 
 * The microcontroller receives commands from the Aaeon UP state machine.
 * Depending on the state, the Arduino will execute different functions.
 * 
 * Authors: Jonathan Watts, Brett Farris, Lucas Cagle
 */

#include <Wire.h>
// External libraries
#include <Adafruit_MotorShield.h>
//#include "utility/Adafruit_PWMServoDriver.h"
#include <PID_v1.h>

// Pin definitions
// NOTE: pins 0 and 1 reserved for USB TX/RX
// Encoder pins
#define FL_ENC_A 2
#define FL_ENC_B 52
#define FR_ENC_A 3
#define FR_ENC_B 51
#define BL_ENC_A 19
#define BL_ENC_B 15
#define BR_ENC_A 18
#define BR_ENC_B 12

// PWM pins
#define STG1_PWM 44

// Analog pins
#define ADC_TOP A0
#define ADC_TL A1
#define ADC_BL A2
#define ADC_BR A3
#define ADC_TR A4

// Hardware parameters
// These are parameters endemic to the hardware and
// thus do not need dynamic reassignment
const int WHEEL_RAD = 30; // in mm! not m!
const int ENC_PER_REV = 810; // encoder pulses per motor output revolution

// Variable initialization
// NOTE: use volatile for non-constant data lest the compiler hardcode it!

// Motor variables
// time for encoder velocity calculation
volatile int prev_time = 0;
volatile int cur_time = 0;

// encoder counts
volatile int fl_enc = 0;
volatile int fr_enc = 0;
volatile int bl_enc = 0;
volatile int br_enc = 0;

// wheel velocities ( mm/s! not m/s! )
double fl_vel = 0;  // mm/s! not m/s! 
double fr_vel = 0;  // mm/s! not m/s! 
double bl_vel = 0;  // mm/s! not m/s! 
double br_vel = 0;  // mm/s! not m/s! 

// commanded wheel velocities ( mm/s! not m/s! )
double cmd_fl_vel = 0;  // mm/s! not m/s! 
double cmd_fr_vel = 0;  // mm/s! not m/s! 
double cmd_bl_vel = 0;  // mm/s! not m/s! 
double cmd_br_vel = 0;  // mm/s! not m/s!

// PWM command
double fl_pwm = 0;
double fr_pwm = 0;
double bl_pwm = 0;
double br_pwm = 0;

// Stage variables
int target_an_pin = 0;  // Case expression for sampling
int target_value = 0;  // Sampling assignment
int pad[5] = {0}; // Five copper pads
int sample_count = 0; // Number of samples taken
char seq[] = "00000";  // decoded sequence
char cmd_rot[] = "00000";  // commanded rotation sequence
char res_rot[] = "00000";  // resultant rotation sequence

// Command variables
String cmd_str;
int STG_trigger = 0b00;

// Parameter variables
// These are parameters that can be changed at runtime via ROS

// Front Left wheel
double fl_Kp = 1; // Proportional gain
double fl_Ki = 0; // Integral gain
double fl_Kd = 0; // Derivative gain

// Front Right wheel
double fr_Kp = 1; // Proportional gain
double fr_Ki = 0; // Integral gain
double fr_Kd = 0; // Derivative gain

// Back Left wheel
double bl_Kp = 1; // Proportional gain
double bl_Ki = 0; // Integral gain
double bl_Kd = 0; // Derivative gain

// Back Right wheel
double br_Kp = 1; // Proportional gain
double br_Ki = 0; // Integral gain
double br_Kd = 0; // Derivative gain

// Motor shield
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *FL_mot = AFMS.getMotor(1);
Adafruit_DCMotor *FR_mot = AFMS.getMotor(2);
Adafruit_DCMotor *BL_mot = AFMS.getMotor(3);
Adafruit_DCMotor *BR_mot = AFMS.getMotor(4);

// PIDs
PID FL_PID(&fl_vel, &fl_pwm, &cmd_fl_vel, fl_Kp, fl_Ki, fl_Kd, DIRECT);
PID FR_PID(&fr_vel, &fr_pwm, &cmd_fr_vel, fr_Kp, fr_Ki, fr_Kd, DIRECT);
PID BL_PID(&bl_vel, &bl_pwm, &cmd_bl_vel, bl_Kp, bl_Ki, bl_Kd, DIRECT);
PID BR_PID(&br_vel, &br_pwm, &cmd_br_vel, br_Kp, br_Ki, br_Kd, DIRECT);

void setup() {
    
    // USB port setup
    Serial.begin(9600);
    
    // initilize pins
    pinMode(FL_ENC_A,INPUT_PULLUP);
    pinMode(FL_ENC_B,INPUT_PULLUP);
    pinMode(FR_ENC_A,INPUT_PULLUP);
    pinMode(FR_ENC_B,INPUT_PULLUP);
    pinMode(BL_ENC_A,INPUT_PULLUP);
    pinMode(BL_ENC_B,INPUT_PULLUP);
    pinMode(BR_ENC_A,INPUT_PULLUP);
    pinMode(BR_ENC_B,INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(FL_ENC_A),FL_A,CHANGE);
    attachInterrupt(digitalPinToInterrupt(FR_ENC_A),FR_A,CHANGE);
    attachInterrupt(digitalPinToInterrupt(BL_ENC_A),BL_A,CHANGE);
    attachInterrupt(digitalPinToInterrupt(BR_ENC_A),BR_A,CHANGE);
    pinMode(STG1_PWM, OUTPUT);
    pinMode(ADC_TOP, INPUT);
    pinMode(ADC_TL, INPUT);
    pinMode(ADC_BL, INPUT);
    pinMode(ADC_BR, INPUT);
    pinMode(ADC_TR, INPUT);

    // Start motor shield
    AFMS.begin();
  
    // Initialize motors to zero movement
    FL_mot->setSpeed(0);
    FR_mot->setSpeed(0);
    BL_mot->setSpeed(0);
    BR_mot->setSpeed(0);
    FL_mot->run(RELEASE);
    FR_mot->run(RELEASE);
    BL_mot->run(RELEASE);
    BR_mot->run(RELEASE);
  
    // PID range set
    FL_PID.SetOutputLimits(-255,255);
    FR_PID.SetOutputLimits(-255,255);
    BL_PID.SetOutputLimits(-255,255);
    BR_PID.SetOutputLimits(-255,255);
  
    // PID start
    FL_PID.SetMode(AUTOMATIC);
    FR_PID.SetMode(AUTOMATIC);
    BL_PID.SetMode(AUTOMATIC);
    BR_PID.SetMode(AUTOMATIC);

}

// Handles Front Left motor interrupt
void FL_A() {
    
    cli();  //stop interrupts during routine
    static volatile int enc;
    enc = ((PINE & (1<<PE4))>>4) ^ ((PINB & (1<<PB1))>>1);
    //enc = digitalRead(FL_ENC_A) ^ digitalRead(FL_ENC_B);
    switch(enc){
        
        case (0b1):  // CW
            fl_enc ++;
            break;
        
        case (0b0):  // CCW
            fl_enc --;
            break;
    sei();
    }
}

// Handles Front Right motor interrupt
void FR_A() {
  
    cli();  //stop interrupts during routine
    static int enc;
    //enc = (PORTE & 0b1000) ^ (PORTB & 0b0010);
    //enc = digitalRead(FR_ENC_A) ^ digitalRead(FR_ENC_B);
  
    switch(enc){
  
        case (0b1):  // CW
            fr_enc ++;
            break;
      
        case (0b0):  // CCW
            fr_enc --;
            break;
    sei();
    }
}

// Handles Back Left motor interrupt
void BL_A() {
  
    cli();  //stop interrupts during routine
    static int enc;
    enc = digitalRead(BL_ENC_A) ^ digitalRead(BL_ENC_B);
  
    switch(enc){
  
        case (0b1):  // CW
            bl_enc ++;
            break;
  
        case (0b0):  // CCW
            bl_enc --;
            break;
    sei();
    }
}

// Handles Back Right motor interrupt
void BR_A() {
  
    cli();  //stop interrupts during routine
    static int enc;
    enc = digitalRead(BR_ENC_A) ^ digitalRead(BR_ENC_B);
  
    switch(enc){
    
        case (0b1):  // CW
            br_enc ++;
            break;
  
        case (0b0):  // CCW
            br_enc --;
            break;
    sei();
    }
}

void loop() {
  
    receive_str();
    cmd_motors();
  
    switch(STG_trigger){
        
        case (0b00):  // Do nothing
            break;
    
        case (0b01):  // Stage 1 trigger
            STG1();
            break;
    
        case (0b10):  // Stage 3 trigger
            STG3();
            break;
    
        case (0b11):  // ERROR STATE!
            // handle error here
            break;
            
    }
    update_status();
    delay(50);
    cmd_motors();
    delay(50);
    // sleep for some amount of time
    // mainly to keep PID loops updated at
    // reasonable rate
}

// receive and process command/parameter-reassignment strings
// updates relevant command/parameter variables
void receive_str(){
    
    // Get next string
    if (Serial.available() > 0) {
        cmd_str = Serial.readStringUntil('\n');
    }else{
      return;
    }
    // command string
    if (cmd_str.substring(0,1) == "C"){
        cmd_fl_vel = cmd_str.substring(1,6).toFloat();
        cmd_fr_vel = cmd_str.substring(7,12).toFloat();
        cmd_bl_vel = cmd_str.substring(13,18).toFloat();
        cmd_br_vel = cmd_str.substring(19,24).toFloat();
        STG_trigger = (int)(cmd_str.substring(27).toInt()<<1) | cmd_str.substring(25).toInt();
    // parameter string
    }else{
        fl_Kp = cmd_str.substring(1,6).toFloat();
        fl_Ki = cmd_str.substring(7,12).toFloat();
        fl_Kd = cmd_str.substring(13,18).toFloat();
        
        fr_Kp = cmd_str.substring(19,24).toFloat();
        fr_Ki = cmd_str.substring(25,30).toFloat();
        fr_Kd = cmd_str.substring(31,36).toFloat();
        
        bl_Kp = cmd_str.substring(37,42).toFloat();
        bl_Ki = cmd_str.substring(43,48).toFloat();
        bl_Kd = cmd_str.substring(49,54).toFloat();
        
        br_Kp = cmd_str.substring(55,60).toFloat();
        br_Ki = cmd_str.substring(61,66).toFloat();
        br_Kd = cmd_str.substring(72,77).toFloat();
    }
}

// updates motor PID's and sends velocity commands
void cmd_motors() {
  
    update_motor_vel();
    FL_PID.Compute();
    FR_PID.Compute();
    BL_PID.Compute();
    BR_PID.Compute();
    // Front Left motor command
    FL_mot->setSpeed((uint8_t) abs((int) fl_pwm));
    if (fl_pwm < 0){
        FL_mot->run(BACKWARD);
    }
    else{
        FL_mot->run(FORWARD);
    }
    
    // Front Right motor command
    FR_mot->setSpeed((uint8_t) abs((int) fr_pwm));
    if (fr_pwm < 0){
        FR_mot->run(BACKWARD);
    }
    else{
        FR_mot->run(FORWARD);
    }
  
    // Back Left motor command
    BL_mot->setSpeed((uint8_t) abs((int)bl_pwm));
    if (bl_pwm < 0){
        BL_mot->run(BACKWARD);
    }
    else{
        BL_mot->run(FORWARD);
    }
  
    // Back Right motor command
    BR_mot->setSpeed((uint8_t) abs((int) br_pwm));
    if (br_pwm < 0){
        BR_mot->run(BACKWARD);
    }
    else{
        BR_mot->run(FORWARD);
    }
}

// reads encoder values and updates motor velocity ( in mm/s! )
void update_motor_vel() {
  
    // CW is forward for right side, CCW for left
    // Time interval
    cur_time = millis();
    int del_time = cur_time - prev_time;
    // Front Left
    // ( pulses ) / ( milliseconds / 1000000 ) * pulses per revolution * wheel radius
    fl_vel = (fl_enc)*1000000/((float) del_time)/ENC_PER_REV*WHEEL_RAD;
    // Front Right
    // ( pulses ) / ( milliseconds / 1000 ) * pulses per revolution * wheel radius
    fr_vel = (fr_enc)*1000000/((float) del_time)/ENC_PER_REV*WHEEL_RAD;
  
    // Front Left
    // ( pulses ) / ( milliseconds / 1000 ) * pulses per revolution * wheel radius
    bl_vel = (bl_enc)*1000000/((float) del_time)/ENC_PER_REV*WHEEL_RAD;
  
    // Front Left
    // ( pulses ) / ( milliseconds / 1000 ) * pulses per revolution * wheel radius
    br_vel = (br_enc)*1000000/((float) del_time)/ENC_PER_REV*WHEEL_RAD;
    // time set
    prev_time = cur_time;
    fl_enc = 0;
    fr_enc = 0;
    bl_enc = 0;
    br_enc = 0;
}

// Performs Stage 1
void STG1() {
  
    // DC Setup
    analogWrite(STG1_PWM, 255);  // analogWrite value 255 = DC
  
    // Timer interrupt setup
    //cli(); // Stop interrupts
    //TCCR2A = 0;  // set register to 0
    //TCCR2B = 0;  // set register to 0
    //TCNT2 = 0;   // set counter to 0
  
    // compare match register set for 8kHz increments
    //OCR2A = 249; // = (16e10) / (8000*8) - 1
    //TCCR2A |= (1 << WGM21);  // CTC mode
    //TCCR2B |= (1 << CS21);  // CS21 bit for 8 prescaler
    //TIMSK2 |= (1 << OCIE2A); // enable timer compare interrupt
    //sei(); // resume interrupts

    // Samples DC peak values
    while (target_an_pin < 5){
        if ((target_an_pin == 2) && (pad[0] >= 37) && (pad[1] >= 37) && (pad[0] <= 40) && (pad[1] <= 40)){
          target_an_pin = 0;
        }
        switch(target_an_pin) { 
          case 0:
            target_value = analogRead(ADC_TOP);
            if (target_value >= pad[0]){
              pad[0] = target_value;
              sample_count = sample_count + 1;
            }
            break;   
          case 1:
            target_value = analogRead(ADC_TL);
            if (target_value >= pad[1]){
              pad[1] = target_value;
              sample_count = sample_count + 1;
            }
            break;  
          case 2:
            target_value = analogRead(ADC_BL);
            if (target_value >= pad[2]){
              pad[2] = target_value;
              sample_count = sample_count + 1;
            }
            break;      
          case 3:
            target_value = analogRead(ADC_BR);
            if (target_value >= pad[3]){
              pad[3] = target_value;
              sample_count = sample_count + 1;
            }
            break;        
          case 4:
            target_value = analogRead(ADC_TR);
            if (target_value >= pad[4]){
              pad[4] = target_value;
              sample_count = sample_count + 1;
            }
            break;
      }
      // set number of samples wanted for each pad
      if (sample_count >= 250){
        sample_count = 0;
        target_an_pin = target_an_pin + 1;
        // reset input voltage to get initial transient for next pad
        analogWrite(STG1_PWM, 0);  
        delay(2000);
        analogWrite(STG1_PWM, 255);
      }
    }

    // Final pad assigments
    for (int m = 0; m < 5; m = m + 1){
      if ((pad[m] >= 0) && (pad[m] <= 10)){
        pad[m] = 1; //wire
      }
      else if ((pad[m] >= 34) && (pad[m] <= 36)){
        pad[m] = 2; //resistor
      }
      else if ((pad[m] >= 75) && (pad[m] <= 95)){
        pad[m] = 3; //capacitor
      }
      else if ((pad[m] >= 20) && (pad[m] <= 26)){
        pad[m] = 4; //inductor
      }
      else if ((pad[m] >= 37) && (pad[m] <= 40)){
        pad[m] = 5; //diode
      }
    }

    // disable timer interrupt
    //cli();
    //TIMSK2 |= (0 << OCIE2A);
    //sei();
    
    analogWrite(STG1_PWM, 0); //done    
}

/* // Stage 1 pin sampler
ISR (TIMER2_COMPA_vect) {
  
    // sample the pin and add to data range
    switch(target_an_pin) { 
    
        case 0:
            target_value = analogRead(ADC_TOP);
            break;
            
        case 1:
            target_value = analogRead(ADC_TL);
            break;
            
        case 2:
            target_value = analogRead(ADC_BL);
            open_check = 0;
            break;
               
        case 3:
            target_value = analogRead(ADC_BR);
            break; 
                 
        case 4:
            target_value = analogRead(ADC_TR);
            break;
    }
}*/

// Performs Stage 3
void STG3(){
  
}

// Check and send statuses including Stage 1 connection validation
void update_status(){
    Serial.print("B:sw:");
    // Print switch states with no separation characters
    Serial.print("wv:");
    Serial.print(fl_vel);
    Serial.print(";");
    Serial.print(fr_vel);
    Serial.print(";");
    Serial.print(bl_vel);
    Serial.print(";");
    Serial.print(br_vel);
    Serial.print("sq:");
    Serial.print(seq);
    Serial.print("rt:");
    Serial.print(res_rot);
    Serial.print('\n');
}

