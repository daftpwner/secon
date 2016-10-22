/*
 * This code is uploaded on the Arduino Mega 2560 for Brain functionality.
 * 
 * The microcontroller receives commands from the Aaeon UP state machine.
 * Depending on the state, the Arduino will execute different functions.
 * 
 * Authors: Jonathan Watts, Brett Farris, Lucas Cagle
 */
// Pin definitions
// NOTE: pins 0 and 1 reserved for USB TX/RX
// Encoder pins
#define FL_ENC_A 21
#define FL_ENC_B 52
#define FR_ENC_A 20
#define FR_ENC_B 51
#define BL_ENC_A 19
#define BL_ENC_B 15
#define BR_ENC_A 18
#define BR_ENC_B 12

// PWM pins
#define STG1_PWM 44;

// Analog pins
#define ADC_TOP A0;
#define ADC_TL A1;
#define ADC_BL A2;
#define ADC_BR A3;
#define ADC_TR A4;

// Hardware parameters
// These are parameters endemic to the hardware and
// thus do not need dynamic reassignment
const int WHEEL_RAD = 30; // in mm! not m!
const int ENC_PER_REV = 810; // encoder pulses per motor output revolution
// Variable initialization
// NOTE: use volatile for non-constant data lest the compiler hardcode it!
// time for encoder velocity calculation
volatile int prev_time = 0;
volatile int cur_time = 0;
// encoder counts
volatile int fl_enc = 0;
volatile int fr_enc = 0;
volatile int bl_enc = 0;
volatile int br_enc = 0;
// wheel velocities ( integers in mm/sec! not m/s! )
volatile int fl_vel = 0;  // integers in mm/sec! not m/s! 
volatile int fr_vel = 0;  // integers in mm/sec! not m/s! 
volatile int bl_vel = 0;  // integers in mm/sec! not m/s! 
volatile int br_vel = 0;  // integers in mm/sec! not m/s! 
// commanded wheel velocities ( integers in mm/s! not m/s! )
volatile int cmd_fl_vel = 0;  // integers in mm/sec! not m/s! 
volatile int cmd_fr_vel = 0;  // integers in mm/sec! not m/s! 
volatile int cmd_bl_vel = 0;  // integers in mm/sec! not m/s! 
volatile int cmd_br_vel = 0;  // integers in mm/sec! not m/s! 
// decoded sequence
volatile char seq[] = "00000";
// commanded rotation sequence
volatile char cmd_rot[] = "00000";
// resultant rotation sequence
volatile char res_rot[] = "00000";
// Command variables
int STG_trigger = 0b00;
// Parameter variables
// These are parameters that can be changed at runtime via ROS


void setup() {

  // initilize pins
  attachInterrupt(0,FL_A,CHANGE);
  attachInterrupt(1,FR_A,CHANGE);
  attachInterrupt(2,BL_A,CHANGE);
  attachInterrupt(3,BR_A,CHANGE);
}
// Handles Front Left motor interrupt
void FL_A(){
  cli();  //stop interrupts during routine
  static int enc = digitalRead(FL_ENC_A) & digitalRead(FL_ENC_B);
  switch(enc){
  case (0b0):
    // CW
    fl_enc ++;
    break;
  case (0b1):
    // CCW
    fl_enc --;
    break;
  }
}
// Handles Front Right motor interrupt
void FR_A(){
  cli();  //stop interrupts during routine
  static int enc = digitalRead(FR_ENC_A) & digitalRead(FR_ENC_B);
  switch(enc){
  case (0b0):
    // CW
    fr_enc ++;
    break;
  case (0b1):
    // CCW
    fr_enc --;
    break;
  }
}
// Handles Back Left motor interrupt
void BL_A(){
  cli();  //stop interrupts during routine
  static int enc = digitalRead(BL_ENC_A) & digitalRead(BL_ENC_B);
  switch(enc){
  case (0b0):
    // CW
    bl_enc ++;
    break;
  case (0b1):
    // CCW
    bl_enc --;
    break;
  }
}
// Handles Back Right motor interrupt
void BR_A(){
  cli();  //stop interrupts during routine
  static int enc = digitalRead(BR_ENC_A) & digitalRead(BR_ENC_B);
  switch(enc){
  case (0b0):
    // CW
    br_enc ++;
    break;
  case (0b1):
    // CCW
    br_enc --;
    break;
  }
}

void loop() {
  receive_str();
  cmd_motors();
  switch(STG_trigger){
    case (0b00):
      // Do nothing
      break;
    case (0b01):
      // Stage 1 trigger
      STG1();
      break;
    case (0b10):
      // Stage 3 trigger
      STG3();
      break;
    case (0b11):
      // AAAAAAAAAAHHHHHHHHHHHHH!!!!
      // ERROR STATE!
      // handle error here
      break;
  update_status();
  }
  // sleep for some amount of time
  // mainly to keep PID loops updated at
  // reasonable rate
}

// receive and process command/parameter-reassignment strings
// updates relevant command/parameter variables
void receive_str(){
    
}

// updates motor PID's and sends velocity commands
void cmd_motors(){
  update_motor_vel();
}

// reads encoder values and updates motor velocity ( in mm/s! )
void update_motor_vel(){
  // CW is forward for right side, CCW for left
  // Time interval
  cur_time = millis();
  
  // Front Left
  // ( pulses ) / ( milliseconds / 1000 ) * pulses per revolution * wheel radius
  fl_vel = (fl_enc)*1000/(cur_time-prev_time)/ENC_PER_REV*WHEEL_RAD;
  // Front Right
  // ( pulses ) / ( milliseconds / 1000 ) * pulses per revolution * wheel radius
  fr_vel = (fr_enc)*1000/(cur_time-prev_time)/ENC_PER_REV*WHEEL_RAD;
  // Front Left
  // ( pulses ) / ( milliseconds / 1000 ) * pulses per revolution * wheel radius
  bl_vel = (bl_enc)*1000/(cur_time-prev_time)/ENC_PER_REV*WHEEL_RAD;
  // Front Left
  // ( pulses ) / ( milliseconds / 1000 ) * pulses per revolution * wheel radius
  br_vel = (br_enc)*1000/(cur_time-prev_time)/ENC_PER_REV*WHEEL_RAD;

  // time set
  prev_time = cur_time;
  fl_enc = 0;
  fr_enc = 0;
  bl_enc = 0;
  br_enc = 0;
}

// Performs Stage 1
void STG1(){
  // PWM setup
  
  // Timer interrupt setup
  cli(); // Stop interrupts
  TCCR2A = 0;  // set register to 0
  TCCR2B = 0;  // set register to 0
  TCNT2 = 0;   // set counter to 0
  // compare match register set for 8kHz increments
  OCR2A = 249; // = (16e10) / (8000*8) - 1
  TCCR2A |= (1 << WGM21);  // CTC mode
  TCCR2B |= (1 << CS21);  // CS21 bit for 8 prescaler
  TIMSK2 |= (1 << OCIE2A); // enable timer compare interrupt
  sei(); // resume interrupts



  // disable timer interrupt
  cli();
  TIMSK2 |= (0 << OCIE2A);
  sei();
}

// Stage 1 pin sampler
ISR(TIMER2_COMPA_vect){
  // sample the pin and add to data range
}

// Performs Stage 3
void STG3(){
  
}

// Check and send statuses including Stage 1 connection validation
void update_status(){
  
}

