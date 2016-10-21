/*
 * This code is uploaded on the Arduino Mega 2560 for Brain functionality.
 * 
 * The microcontroller receives commands from the Aaeon UP state machine.
 * Depending on the state, the Arduino will execute different functions.
 * 
 * Authors: Jonathan Watts, Brett Farris
 */

void setup() {
  
  // define and initilize pins
  
}

void loop() {
  
  // receive state from Aaeon UP
  state = NULL;

  // call functions based on state
  switch(state) {

      case "start":
          start();
          break;

      case "nav_to_STG1_wall":
          nav_to_STG1_wall();
          break;

      case "nav_to_STG1":
          nav_to_STG1();
          break;

      case "align_to_STG1":
          align_to_STG1();
          break;

      case "perform_STG1":
          perform_STG1();
          break;

      case "nav_to_STG3_wall":
          nav_to_STG3_wall();
          break;

      case "nav_to_STG3":
          nav_to_STG3();
          break;

      case "perform_STG3":
          perform_STG3();
          break;

      default:
          break;  // wait for command
  }
  
  // function definitions for each state
  
  // left = towards Stage 4
  // right = towards Stage 2
  // forward = towards Stage 1
  // backwards = towards Stage 3
  
  void start()
  {
      // disconnect Pinky
  }

  void nav_to_STG1_wall()
  {
      // operate motors to move a hardcoded distance to the left (approx 1 foot)
      // operate motors to move forward until reaching the wall
  }

  void nav_to_STG1()
  {
      // operate motors to move right until reaching the Stage 1 frame
  }

  void align_to_STG1()
  {
      // minute movements to ensure proper connection with all probes
  }

  void perform_STG1()
  {
      // send PWM signal to probes and determine components
  }

  void nav_to_STG3_wall()
  {
      // operate motors to move backwards across field towards Stage 3
      // operate motors to move a hardcoded distance to the right
      // operate motors to move backwards until reaching the wall
  }

  void nav_to_STG3()
  {
      // operate motors to move left until reaching the Stage 3 frame
  }

  void align_to_STG3()
  {
      // minute movements to ensure proper grip on knob
  }

  void perform_STG3()
  {
      // operate gripper motors to turn knob the correct number of revolutions
  }
  
}
