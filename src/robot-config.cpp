#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor LeftMotorA = motor(PORT3, ratio18_1, true);
motor LeftMotorB = motor(PORT11, ratio18_1, true);
motor_group Left = motor_group(LeftMotorA, LeftMotorB);
motor RightMotorA = motor(PORT20, ratio18_1, false);
motor RightMotorB = motor(PORT9, ratio18_1, false);
motor_group Right = motor_group(RightMotorA, RightMotorB);
motor Arm = motor(PORT10, ratio18_1, false);
motor Intake = motor(PORT4, ratio6_1, true);
inertial Gyro = inertial(PORT7);
digital_out Claw = digital_out(Brain.ThreeWirePort.A);
digital_out Back_Left = digital_out(Brain.ThreeWirePort.B);
digital_out Back_Right = digital_out(Brain.ThreeWirePort.C);
controller Controller1 = controller(primary);
motor LeftBack = motor(PORT1, ratio18_1, true);
motor RightBack = motor(PORT8, ratio18_1, false);
digital_out yeet = digital_out(Brain.ThreeWirePort.D);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool Controller1LeftShoulderControlMotorsStopped = true;
bool Controller1RightShoulderControlMotorsStopped = true;
bool Controller1XBButtonsControlMotorsStopped = true;

// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {
    if(RemoteControlCodeEnabled) {
      // check the ButtonL1/ButtonL2 status to control Left
      if (Controller1.ButtonL1.pressing()) {
        Left.spin(forward);
        Controller1LeftShoulderControlMotorsStopped = false;
      } else if (Controller1.ButtonL2.pressing()) {
        Left.spin(reverse);
        Controller1LeftShoulderControlMotorsStopped = false;
      } else if (!Controller1LeftShoulderControlMotorsStopped) {
        Left.stop();
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        Controller1LeftShoulderControlMotorsStopped = true;
      }
      // check the ButtonR1/ButtonR2 status to control Right
      if (Controller1.ButtonR1.pressing()) {
        Right.spin(forward);
        Controller1RightShoulderControlMotorsStopped = false;
      } else if (Controller1.ButtonR2.pressing()) {
        Right.spin(reverse);
        Controller1RightShoulderControlMotorsStopped = false;
      } else if (!Controller1RightShoulderControlMotorsStopped) {
        Right.stop();
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        Controller1RightShoulderControlMotorsStopped = true;
      }
      // check the ButtonX/ButtonB status to control Arm
      if (Controller1.ButtonX.pressing()) {
        Arm.spin(forward);
        Controller1XBButtonsControlMotorsStopped = false;
      } else if (Controller1.ButtonB.pressing()) {
        Arm.spin(reverse);
        Controller1XBButtonsControlMotorsStopped = false;
      } else if (!Controller1XBButtonsControlMotorsStopped) {
        Arm.stop();
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        Controller1XBButtonsControlMotorsStopped = true;
      }
    }
    // wait before repeating the process
    wait(20, msec);
  }
  return 0;
}

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);
}