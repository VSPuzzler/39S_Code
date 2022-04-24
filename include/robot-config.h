using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor_group Left;
extern motor_group Right;
extern motor Arm;
extern motor Intake;
extern inertial Gyro;
extern digital_out Claw;
extern digital_out Back_Left;
extern digital_out Back_Right;
extern controller Controller1;
extern motor LeftBack;
extern motor RightBack;
extern digital_out yeet;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );