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
#include <Servo.h>
#include <Stepper.h>

// Pin definitions
// NOTE: pins 0 and 1 reserved for USB TX/RX
// NOTE: pins 9 and 10 reserved for Stage 3 Servos
Servo servo1; // gripper
Servo servo2; // rotation

// Encoder pins
#define FL_ENC_A 2
#define FL_ENC_B 4
#define FR_ENC_A 3
#define FR_ENC_B 5
#define BL_ENC_A 19
#define BL_ENC_B 17
#define BR_ENC_A 18
#define BR_ENC_B 16

// Bump switch pins
#define STG1_WALL_SWITCH 22
#define STG1_ALIGN_SWITCH 24
#define STG3_WALL_SWITCH 23
#define STG3_ALIGN_SWITCH 25

// Start/Stop switch pins
#define START_SWITCH 26
#define STOP_SWITCH 27

// Analog pins
#define STG1_PWM 9
#define ADC_TOP A0
#define ADC_TR A1
#define ADC_BR A2
#define ADC_BL A3
#define ADC_TL A7

// STG1
const int SAMPLES = 10000;
int seq[5] = {0};

// STG4
const int STG4_LAUNCHER = 14;
const int STG4_FEEDER = 15;

// Hardware parameters
// These are parameters endemic to the hardware and
// thus do not need dynamic reassignment
const int WHEEL_RAD = 30; // in mm! not m!
const int ENC_PER_REV = 810; // encoder pulses per motor output revolution

// Variable initialization
// NOTE: use volatile for non-constant data lest the compiler hardcode it!


// Command variables
String cmd_str;
int STG_trigger = 0b000;

// Stage 1 variables
int pad[5] = {0}; // Final copper pad assignment
// Stage 3 Variables
const int STEP_PER_REV = 200;
Stepper STG3_rotation_stepper(STEP_PER_REV, A11, A12, A13, A14); // Initialize stepper library
int rot_seq[5] = {0};

// Stage 4 Variables
int stg4_fin = 0;


/*******************
 * Motor variables *
 *******************/
volatile int prev_time = 0; //  Time  //
volatile int cur_time = 0;  // Stamps //
volatile int fl_enc = 0; /////////////
volatile int fr_enc = 0; // Encoder //
volatile int bl_enc = 0; //  Counts //
volatile int br_enc = 0; /////////////
double fl_enc_vels[5] = { 0, 0, 0, 0, 0 }; //////////////
double fr_enc_vels[5] = { 0, 0, 0, 0, 0 }; // Velocity //
double bl_enc_vels[5] = { 0, 0, 0, 0, 0 }; // Averages //
double br_enc_vels[5] = { 0, 0, 0, 0, 0 }; //////////////
uint8_t fl_index = 0; ////////////////////
uint8_t fr_index = 0; // Velocity Array //
uint8_t bl_index = 0; // Index Pointers //
uint8_t br_index = 0; ////////////////////
double fl_vel = 0;  ////////////////
double fr_vel = 0;  // Velocities //
double bl_vel = 0;  //  (mm/sec)  //
double br_vel = 0;  ////////////////
double cmd_fl_vel = 0;  ////////////////////////
double cmd_fr_vel = 0;  // Command Velocities //
double cmd_bl_vel = 0;  //      (mm/sec)      //
double cmd_br_vel = 0;  ////////////////////////
double fl_pwm = 0; //////////////
double fr_pwm = 0; //   PWM    //
double bl_pwm = 0; // Commands //
double br_pwm = 0; //////////////
double fl_Kp = 1.4; ////////////////////
double fl_Ki = 6.0; // Front Left PID //
double fl_Kd = 0.1; ////////////////////
// Front Right wheel
double fr_Kp = 1.4; /////////////////////
double fr_Ki = 6.0; // Front Right PID //
double fr_Kd = 0.1; /////////////////////
// Back Left wheel
double bl_Kp = 1.4; ///////////////////
double bl_Ki = 6.0; // Back Left PID //
double bl_Kd = 0.1; ///////////////////
// Back Right wheel
double br_Kp = 1.4; ////////////////////
double br_Ki = 6.0; // Back Right PID //
double br_Kd = 0.1; ////////////////////
Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); // Drive Motor Shield
Adafruit_DCMotor *FL_mot = AFMS.getMotor(1); ////////////
Adafruit_DCMotor *FR_mot = AFMS.getMotor(2); //  Drive //
Adafruit_DCMotor *BL_mot = AFMS.getMotor(3); // Motors //
Adafruit_DCMotor *BR_mot = AFMS.getMotor(4); ////////////
PID FL_PID(&fl_vel, &fl_pwm, &cmd_fl_vel, fl_Kp, fl_Ki, fl_Kd, DIRECT); ///////////
PID FR_PID(&fr_vel, &fr_pwm, &cmd_fr_vel, fr_Kp, fr_Ki, fr_Kd, DIRECT); // Motor //
PID BL_PID(&bl_vel, &bl_pwm, &cmd_bl_vel, bl_Kp, bl_Ki, bl_Kd, DIRECT); //  PID  //
PID BR_PID(&br_vel, &br_pwm, &cmd_br_vel, br_Kp, br_Ki, br_Kd, DIRECT); ///////////
Adafruit_MotorShield STG_MS = Adafruit_MotorShield(0x60); // Stage Carriage Stepper Shield
Adafruit_StepperMotor *STG1_motor = STG_MS.getStepper(200, 1); //   Stage  //
Adafruit_StepperMotor *STG3_motor = STG_MS.getStepper(200, 2); // Steppers //

/*********
 * Setup *
 *********/
void setup() {
    // USB port setup
    Serial.begin(115200);
    
    // initilize encoder pins
    pinMode(FL_ENC_A,INPUT_PULLUP);
    pinMode(FL_ENC_B,INPUT_PULLUP);
    pinMode(FR_ENC_A,INPUT_PULLUP);
    pinMode(FR_ENC_B,INPUT_PULLUP);
    pinMode(BL_ENC_A,INPUT_PULLUP);
    pinMode(BL_ENC_B,INPUT_PULLUP);
    pinMode(BR_ENC_A,INPUT_PULLUP);
    pinMode(BR_ENC_B,INPUT_PULLUP);

    // Set interrupts for the four motor A encoders
    attachInterrupt(digitalPinToInterrupt(FL_ENC_A),FL_A,CHANGE);
    attachInterrupt(digitalPinToInterrupt(FR_ENC_A),FR_A,CHANGE);
    attachInterrupt(digitalPinToInterrupt(BL_ENC_A),BL_A,CHANGE);
    attachInterrupt(digitalPinToInterrupt(BR_ENC_A),BR_A,CHANGE);

    // Initialize bump switch pins
    pinMode(STG1_WALL_SWITCH,INPUT_PULLUP);
    pinMode(STG1_ALIGN_SWITCH,INPUT_PULLUP);
    pinMode(STG3_WALL_SWITCH,INPUT_PULLUP);
    pinMode(STG3_ALIGN_SWITCH,INPUT_PULLUP);

    // Initialize Start/Stop switch pins
    pinMode(START_SWITCH,INPUT_PULLUP);
    pinMode(STOP_SWITCH,INPUT_PULLUP);

    // Initialize stage 1 pins
    pinMode(STG1_PWM, OUTPUT);
    pinMode(ADC_TOP, INPUT);
    pinMode(ADC_TR, INPUT);
    pinMode(ADC_BR, INPUT);
    pinMode(ADC_BL, INPUT);
    pinMode(ADC_TL, INPUT);
    
    pinMode(STG4_LAUNCHER, OUTPUT);
    pinMode(STG4_FEEDER, OUTPUT);

    // Start motor shield
    AFMS.begin();
    STG_MS.begin();

    // Initialize stage 3 servos
    servo1.attach(9);
    servo2.attach(10);
    
    // Stop the motors
    servo2.write(90);
    servo1.write(0);
    
    // Initialize motors to zero movement
    FL_mot->setSpeed(0);
    FR_mot->setSpeed(0);
    BL_mot->setSpeed(0);
    BR_mot->setSpeed(0);
    FL_mot->run(FORWARD);
    FR_mot->run(FORWARD);
    BL_mot->run(FORWARD);
    BR_mot->run(FORWARD);
  
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

/*************
 * Main Loop *
 *************/
void loop() {
    receive_str();    
    switch(STG_trigger){
        
        case (0b000):  // Do nothing
            break;
    
        case (0b100):  // Stage 1 trigger
            STG1();
            break;
    
        case (0b010):  // Stage 3 trigger
            STG3();
            break;
        
        case (0b001):  // Stage 4 trigger
            STG4();
            break;
            
    }
    update_status(); // Send update string at about 10 Hz
    delay(10);
    cmd_motors();
    delay(20);
    cmd_motors();
    delay(20);
    cmd_motors();
    delay(20);
    cmd_motors();
    delay(20);
    

}
/***************
 *  Serial In  *
 ***************/
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
        STG_trigger = (int)(cmd_str.substring(25).toInt()<<2) | (cmd_str.substring(27).toInt()<<1) | cmd_str.substring(29).toInt();
    // parameter string
    }else{
        fl_Kp = cmd_str.substring(1,7).toFloat();
        fl_Ki = cmd_str.substring(7,13).toFloat();
        fl_Kd = cmd_str.substring(13,19).toFloat();
        
        fr_Kp = cmd_str.substring(19,25).toFloat();
        fr_Ki = cmd_str.substring(25,31).toFloat();
        fr_Kd = cmd_str.substring(31,37).toFloat();
        
        bl_Kp = cmd_str.substring(37,43).toFloat();
        bl_Ki = cmd_str.substring(43,49).toFloat();
        bl_Kd = cmd_str.substring(49,55).toFloat();
        
        br_Kp = cmd_str.substring(55,61).toFloat();
        br_Ki = cmd_str.substring(61,67).toFloat();
        br_Kd = cmd_str.substring(67,73).toFloat();
        FL_PID.SetTunings(fl_Kp, fl_Ki, fl_Kd);
        FR_PID.SetTunings(fr_Kp, fr_Ki, fr_Kd);
        BL_PID.SetTunings(bl_Kp, bl_Ki, bl_Kd);
        BR_PID.SetTunings(br_Kp, br_Ki, br_Kd);
    }
}

/*************
 *  Stage 1  *
 *************/
// Stage 1
void STG1(){
  deploy_STG1();

  perform_STG1(seq);

  retract_STG1();
  
  STG_trigger = 0;
}
// Deploy Stage 1 arm
void deploy_STG1() {

    STG1_motor->step(195,BACKWARD,MICROSTEP);
}
// Retract Stage 1 arm
void retract_STG1() {

  STG1_motor->step(195,FORWARD,MICROSTEP);
  STG1_motor->release();
}
// Sample analog pin
void sample_pin(int an_pin, double* max_val, double* avg) {
  uint16_t count = 0;
  *avg = 0;
  *max_val = 0;
  analogRead(an_pin);
  while(count++ < SAMPLES) {
    double val = analogRead(an_pin);
    *avg += val;
    *max_val = (*max_val>=val) ? *max_val : val;
  }
  *avg = *avg/((double) SAMPLES);
}
// Sample and determine sequence
void perform_STG1(int seq[]) {
  // Start PWM
  // allocate variables
  double max_val[5] = {0};
  double avg[5] = {0};
  delay(10);
  // Sample pads
  analogWrite(STG1_PWM,127);
  sample_pin(ADC_TOP, &max_val[0], &avg[0]);
  analogWrite(STG1_PWM,0);
  delay(100);
  analogWrite(STG1_PWM,127);
  sample_pin(ADC_TR, &max_val[1], &avg[1]);
  analogWrite(STG1_PWM,0);
  delay(100);
  analogWrite(STG1_PWM,127);
  sample_pin(ADC_BR, &max_val[2], &avg[2]);
  analogWrite(STG1_PWM,0);
  delay(100);
  analogWrite(STG1_PWM,127);
  sample_pin(ADC_BL, &max_val[3], &avg[3]);
  analogWrite(STG1_PWM,0);
  delay(100);
  analogWrite(STG1_PWM,127);
  sample_pin(ADC_TL, &max_val[4], &avg[4]);
  analogWrite(STG1_PWM,0);

  // Evaluate
  int wire = -1;
  int res = -1;
  int cap = -1;
  int ind = -1;
  int diode = -1;

  // Find wire
  for(int i = 0;i<5;i++){
    if ((avg[i]<10)&&(max_val[i]<10)){
      wire = i;
      seq[i] = 1;
    }
  }
  // Find cap
  for(int i=0;i<5;i++){
    if ((avg[i]<30)&&(avg[i]>10) && (max_val[i]<75)){
      cap = i;
      seq[i] = 3;
    }
  }
  // Find FB-diode
  for(int i=0;i<5;i++){
    if ((avg[i]>50)&&(avg[i]<70)&&(max_val[i]<150)&&(max_val[i]>75)){
      diode = i;
      seq[i] = 5;
    }
  }
  // Find Resistor
  for(int i=0;i<5;i++){
    if ((avg[i]>65)&&(avg[i]<80)&&(max_val[i]<250)&&(max_val[i]>190)){
      res = i;
      seq[i] = 2;
    }
  }
  // Find Inductor
  for(int i=0;i<5;i++){
    if ((avg[i]>70)&&(avg[i]<80)&&(max_val[i]>250)){
      ind = i;
      seq[i] = 4;
    }
  }
  // Find RB-diode
  for( int i=0;i<5;i++){
    if (seq[i]==0){
      diode = i;
      seq[i] = 5;
    }
  }
}

/*************
 *  Stage 1  *
 *************/
// Stage 3
void STG3(){
  deploy_STG3();
  Serial.println("Begin Stage 3");
  perform_STG3();
  Serial.println("Complete Stage 3");
  retract_STG3();
  STG_trigger = 0;
}
// Deploy Stage 3 arm
void deploy_STG3() {

    STG3_motor->step(140,FORWARD,MICROSTEP);
}
// Retract Stage 3 arm
void retract_STG3() {

  STG3_motor->step(140,BACKWARD,MICROSTEP);
  STG3_motor->release();
}
// Perform Stage 3
void perform_STG3(){
  int rot_dir = 0;

  for (int m = 0; m < 5; m = m + 1){
    Serial.print("Rotation Sequence: ");
    Serial.println(m+1); // 1-5 instead of 0-4
    (rot_dir == 1) ? Serial.print("Clockwise: ") : Serial.print("Counter-Clockwise: ");
    Serial.println(seq[m]);
    
    STG3_rotation_stepper.step((-1**rot_dir)*stepsPerRevolution*seq[m]); // rotate seq[m] revolutions c/cw
    rot_dir = (rot_dir == 0); // toggle between 1 and 0
  }
}

/*************
 *  Stage 1  *
 *************/
// Stage 4
void STG4() {
    spinup_STG4();
    delay(250);
    fire_STG4();
    spindown_STG4();
    stg4_fin = 1;
    STG_trigger = 0;
}
void spinup_STG4() {
    digitalWrite(STG4_LAUNCHER, HIGH);
}
void fire_STG4() {
    digitalWrite(STG4_FEEDER, HIGH);
    delay(3000);
    digitalWrite(STG4_FEEDER, LOW);
}
void spindown_STG4() {
    digitalWrite(STG4_LAUNCHER, LOW);
}


/****************
 *  Serial Out  *
 ****************/
// Check and send statuses including Stage 1 connection validation
void update_status(){
    Serial.print("B:sw:");
    Serial.print(1^digitalRead(START_SWITCH));
    Serial.print(1^digitalRead(STG1_WALL_SWITCH));
    Serial.print(1^digitalRead(STG1_ALIGN_SWITCH));
    Serial.print(1^digitalRead(STG3_WALL_SWITCH));
    Serial.print(1^digitalRead(STG3_ALIGN_SWITCH));
    Serial.print(1^digitalRead(STOP_SWITCH));
    Serial.print("wv:");
    Serial.print(fl_vel);
    Serial.print(";");
    Serial.print(fr_vel);
    Serial.print(";");
    Serial.print(bl_vel);
    Serial.print(";");
    Serial.print(br_vel);
    Serial.print("sq:");
    Serial.print(seq[0]); Serial.print(seq[1]); Serial.print(seq[2]); Serial.print(seq[3]); Serial.print(seq[4]);
    Serial.print("rt:");
    Serial.print(rot_seq);
    Serial.print("stg4:");
    Serial.print(1&stg4_fin);
    Serial.print('\n');
}

/*******************
 *  Drive Control  *
 *******************/
// Set drive PWM commands
void cmd_motors() {
  
    update_motor_vel();
    FL_PID.Compute();
    FR_PID.Compute();
    BL_PID.Compute();
    BR_PID.Compute();
    
    // Front Left motor command
    FL_mot->setSpeed((uint8_t) abs((int) round(fl_pwm)));
    if (fl_pwm < 0){
        FL_mot->run(FORWARD);
    }
    else{
        FL_mot->run(BACKWARD);
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
        BL_mot->run(FORWARD);
    }
    else{
        BL_mot->run(BACKWARD);
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
// update motor velocity measurements
void update_motor_vel() {
  
    // CW is forward for right side, CCW for left
    // Time interval
    static double del_time;
    static int fl_,fr_,bl_,br_;
    static double enc_per_sec_to_mm_per_sec = 1000*WHEEL_RAD*2.0*3.14/ENC_PER_REV;
    cur_time = millis();
    fl_ = fl_enc;
    fr_ = fr_enc;
    bl_ = bl_enc;
    br_ = br_enc;
    del_time = cur_time - prev_time;
    
    // time set
    prev_time = cur_time;
    fl_enc = 0;
    fr_enc = 0;
    bl_enc = 0;
    br_enc = 0;
    
    // Front Left
    // ( pulses ) / ( milliseconds / 1000000 ) * pulses per revolution * wheel radius
    fl_enc_vels[fl_index] = fl_ / del_time * enc_per_sec_to_mm_per_sec;
    fl_index = (fl_index+1) % 5;
    fl_index = (fl_index+1) % 5;
    fl_vel = 0;
    for (int i = 0; i<5 ; i++){
        fl_vel += fl_enc_vels[i]/5.0;
    }
    // Front Right
    // ( pulses ) / ( milliseconds / 1000 ) * pulses per revolution * wheel radius
    fr_enc_vels[fr_index] = fr_ / del_time * enc_per_sec_to_mm_per_sec;
    fr_index = (fr_index+1) % 5;
    fr_vel = 0;
    for (int i = 0; i<5 ; i++){
        fr_vel += fr_enc_vels[i]/5.0;
    }
    // Back Left
    // ( pulses ) / ( milliseconds / 1000 ) * pulses per revolution * wheel radius
    bl_enc_vels[bl_index] = bl_ / del_time * enc_per_sec_to_mm_per_sec;
    bl_index = (bl_index+1) % 5;
    bl_vel = 0;
    for (int i = 0; i<5 ; i++){
        bl_vel += bl_enc_vels[i]/5.0;
    }
    // Front Left
    // ( pulses ) / ( milliseconds / 1000 ) * pulses per revolution * wheel radius * 2pi
    br_enc_vels[br_index] = br_ / del_time * enc_per_sec_to_mm_per_sec;
    br_index = (br_index+1) % 5;
    br_vel = 0;
    for (int i = 0; i<5 ; i++){
        br_vel += br_enc_vels[i]/5.0;
    }
}
// Handles Front Left motor interrupt
void FL_A() {
    
    cli();  //stop interrupts during routine
    static volatile int enc;
    // Reads the two pins and xors them
    enc = ((PINE & (1<<PE4))>>4) ^ ((PINB & (1<<PB1))>>1);
    //enc = digitalRead(FL_ENC_A) ^ digitalRead(FL_ENC_B);
    switch(enc){
        
        case (0b1):  // CW Forward
            fl_enc ++;
            break;
        
        case (0b0):  // CCW Backwards
            fl_enc --;
            break;
    sei();
    }
}
// Handles Front Right motor interrupt
void FR_A() {
  
    cli();  //stop interrupts during routine
    static int enc;
    // Reads the two pins and xors them
    enc = ((PINE & (1<<PE5))>>5) ^ ((PINB & (1<<PB2))>>2);
    //enc = digitalRead(FR_ENC_A) ^ digitalRead(FR_ENC_B);
  
    switch(enc){
  
        case (0b1):  // CW Backwards
            fr_enc --;
            break;
      
        case (0b0):  // CCW Forwards
            fr_enc ++;
            break;
    sei();
    }
}
// Handles Back Left motor interrupt
void BL_A() {
  
    cli();  //stop interrupts during routine
    static int enc;
    // Reads the two pins and xors them
    enc = ((PIND & (1<<PD2))>>2) ^ ((PINJ & (1<<PJ0)) >> 0);
    //enc = digitalRead(BL_ENC_A) ^ digitalRead(BL_ENC_B);
  
    switch(enc){
  
        case (0b1):  // CW Forwards
            bl_enc ++;
            break;
  
        case (0b0):  // CCW Backwards
            bl_enc --;
            break;
    sei();
    }
}
// Handles Back Right motor interrupt
void BR_A() {
  
    cli();  //stop interrupts during routine
    static int enc;
    // Reads the two pins and xors them
    enc = ((PIND & (1<<PD3))>>3) ^ ((PING & (1<<PG5))>>5);
    //enc = digitalRead(BR_ENC_A) ^ digitalRead(BR_ENC_B);
  
    switch(enc){
    
        case (0b1):  // CW Backwards
            br_enc --;
            break;
  
        case (0b0):  // CCW Forwards
            br_enc ++;
            break;
    sei();
    }
}
