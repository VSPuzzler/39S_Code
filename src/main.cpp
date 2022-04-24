/*----------------------------------------------------------------------------*/
  /*                                                                            */
  /*    Module:       main.cpp                                                  */
  /*    Author:       C:\Users\Vineet Saravanan                                 */
  /*    Created:      Sun Oct 17 2021                                           */
  /*    Description:  V5 project                                                */
  /*                                                                            */
  /*----------------------------------------------------------------------------*/



  #include "vex.h"
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Left                 motor_group   3, 11           
// Right                motor_group   20, 9           
// Arm                  motor         10              
// Intake               motor         4               
// Gyro                 inertial      7               
// Claw                 digital_out   A               
// Back_Left            digital_out   B               
// Back_Right           digital_out   C               
// Controller1          controller                    
// LeftBack             motor         1               
// RightBack            motor         8               
// yeet                 digital_out   D               
// ---- END VEXCODE CONFIGURED DEVICES ----

//Creates a competition object that allows access to Competition methods.


vex::competition    Competition;
// storage for our auton selection
int   autonomousSelection = -1;

/* 
        James Pearman autoselect functions and definitions. These are modified for Walsh
*/
// collect data for on screen button and include off and on color feedback for button
// prc - instead of radio approach with one button on or off at a time, each button has
//          a state.  ie shootPreload may be low yellow and high yellow when on.
typedef struct _button {
    int    xpos;
    int    ypos;
    int    width;
    int    height;
    bool   state;
    vex::color offColor;
    vex::color onColor;
    const char *label;
} button;

// Button array definitions for each software button. The purpose of each button data structure
// is defined above.  The array size can be extended, so you can have as many buttons as you 
// wish as long as it fits.
button buttons[] = {
    {   30,  30, 60, 60,  false, 0xE00000, 0x0000E0, "Skills" },
    {  150,  30, 60, 60,  false, 0x303030, 0xD0D0D0, "Left" },
    {  270,  30, 60, 60,  false, 0x303030, 0xF700FF, "Right" },
    {  390,  30, 60, 60,  false, 0x303030, 0xDDDD00, "Middle" },
    {   30, 150, 60, 60,  false, 0x404040, 0xC0C0C0, "Hyper" },
    {  150, 150, 60, 60,  false, 0x404040, 0xC0C0C0, "FillGoal" },
    {  270, 150, 60, 60,  false, 0x404040, 0xC0C0C0, "RightMiddle" },
    {  390, 150, 60, 60,  false, 0x404040, 0xC0C0C0, "7-" }
};

// forward ref
void displayButtonControls( int index, bool pressed );

/*-----------------------------------------------------------------------------*/
/** @brief      Check if touch is inside button                                */
/*-----------------------------------------------------------------------------*/
int
findButton(  int16_t xpos, int16_t ypos ) {
    int nButtons = sizeof(buttons) / sizeof(button);

    for( int index=0;index < nButtons;index++) {
      button *pButton = &buttons[ index ];
      if( xpos < pButton->xpos || xpos > (pButton->xpos + pButton->width) )
        continue;

      if( ypos < pButton->ypos || ypos > (pButton->ypos + pButton->height) )
        continue;

      return(index);
    }
    return (-1);
}

/*-----------------------------------------------------------------------------*/
/** @brief      Init button states                                             */
/*-----------------------------------------------------------------------------*/
void
initButtons() {
    int nButtons = sizeof(buttons) / sizeof(button);

    for( int index=0;index < nButtons;index++) {
      buttons[index].state = false;
    }
}

/*-----------------------------------------------------------------------------*/
/** @brief      Screen has been touched                                        */
/*-----------------------------------------------------------------------------*/
void
userTouchCallbackPressed() {
    int index;
    int xpos = Brain.Screen.xPosition();
    int ypos = Brain.Screen.yPosition();

    if( (index = findButton( xpos, ypos )) >= 0 ) {
      displayButtonControls( index, true );
    }

}

/*-----------------------------------------------------------------------------*/
/** @brief      Screen has been (un)touched                                    */
/*-----------------------------------------------------------------------------*/
void
userTouchCallbackReleased() {
    int index;
    int xpos = Brain.Screen.xPosition();
    int ypos = Brain.Screen.yPosition();

    if( (index = findButton( xpos, ypos )) >= 0 ) {
      // clear all buttons to false, ie. unselected
      //      initButtons(); 

      // now set this one as true
      if( buttons[index].state == true) {
      buttons[index].state = false; }
      else    {
      buttons[index].state = true;}

      // save as auton selection
      autonomousSelection = index;

      displayButtonControls( index, false );
    }
}

/*-----------------------------------------------------------------------------*/
/** @brief      Draw all buttons                                               */
/*-----------------------------------------------------------------------------*/
void displayButtonControls( int index, bool pressed ) {
    vex::color c;
    Brain.Screen.setPenColor( vex::color(0xe0e0e0) );

    for(int i=0;i<sizeof(buttons)/sizeof(button);i++) {

      if( buttons[i].state )
        c = buttons[i].onColor;
      else
        c = buttons[i].offColor;

      Brain.Screen.setFillColor( c );

      // button fill
      if( i == index && pressed == true ) {
        Brain.Screen.drawRectangle( buttons[i].xpos, buttons[i].ypos, buttons[i].width, buttons[i].height, c );
      }
      else
        Brain.Screen.drawRectangle( buttons[i].xpos, buttons[i].ypos, buttons[i].width, buttons[i].height );

      // outline
      Brain.Screen.drawRectangle( buttons[i].xpos, buttons[i].ypos, buttons[i].width, buttons[i].height, vex::color::transparent );

// draw label
      if(  buttons[i].label != NULL )
        Brain.Screen.printAt( buttons[i].xpos + 8, buttons[i].ypos + buttons[i].height - 8, buttons[i].label );
    }
}

void move_arm (float rotations){
  Arm.rotateFor(rotations,rotationUnits::deg,100,velocityUnits::pct,false);
}
void move (float rotations, float speed){
    Right.rotateFor(rotations,rotationUnits::rev,speed,velocityUnits::pct,false);
    RightBack.rotateFor(rotations,rotationUnits::rev,speed,velocityUnits::pct,false);
    LeftBack.rotateFor(rotations,rotationUnits::rev,speed,velocityUnits::pct,false);
    Left.rotateFor(rotations,rotationUnits::rev,speed,velocityUnits::pct,true);
}
void move_time (float seconds){
    Left.spin(forward);
    Right.spin(forward);
    LeftBack.spin(forward);
    RightBack.spin(forward);
    vex::task::sleep(seconds*1000);
    Left.stop(vex::brakeType::brake);
    Right.stop(vex::brakeType::brake);
    LeftBack.stop(vex::brakeType::brake);
    RightBack.stop(vex::brakeType::brake);

}
void move_time_back (float seconds){
    Left.spin(reverse);
    Right.spin(reverse);
    LeftBack.spin(reverse);
    RightBack.spin(reverse);
    vex::task::sleep( seconds*1000 );
    Left.stop(vex::brakeType::brake);
    Right.stop(vex::brakeType::brake);
    LeftBack.stop(vex::brakeType::brake);
    RightBack.stop(vex::brakeType::brake);

}
void turnLeft (float degrees, float speed, bool flag){
    Left.rotateFor(-degrees,rotationUnits::deg,speed,velocityUnits::pct,false);
    LeftBack.rotateFor(-degrees,rotationUnits::deg,speed,velocityUnits::pct,false);
    RightBack.rotateFor(degrees,rotationUnits::deg,speed,velocityUnits::pct,false);
    Right.rotateFor(degrees,rotationUnits::deg,speed,velocityUnits::pct,true);
}
void turnRight (float degree, float speed, bool flag){
    Left.rotateFor(degree,rotationUnits::deg,speed,velocityUnits::pct,false);
    LeftBack.rotateFor(degree,rotationUnits::deg,speed,velocityUnits::pct,false);
    RightBack.rotateFor(-degree,rotationUnits::deg,speed,velocityUnits::pct,true);
    Right.rotateFor(-degree,rotationUnits::deg,speed,velocityUnits::pct,true);
}
void turnArm (float degrees, float speed, bool flag){
    Arm.rotateFor(degrees,rotationUnits::deg,speed,velocityUnits::pct,true);
}
void turnClaw (bool value){
  Claw.set(value);
}
void turnBackClaw (bool value){
  Back_Left.set(value);
  Back_Right.set(value);
}
void goal_rush (){
  Left.spin( fwd, 12.0, voltageUnits::volt );
  Right.spin( fwd, 12.0, voltageUnits::volt );
  LeftBack.spin( fwd, 12.0, voltageUnits::volt );
  RightBack.spin( fwd, 12.0, voltageUnits::volt );


  while (true){
  }
  
  Left.stop(vex::brakeType::brake);
  Right.stop(vex::brakeType::brake);
  LeftBack.stop(vex::brakeType::brake);
  RightBack.stop(vex::brakeType::brake);
}
void intake(bool spin){
  if (spin == true){
    Intake.spin( fwd, 12.0, voltageUnits::volt );
  }
  else if (spin == false){
    Intake.stop(vex::brakeType::brake);
  }
}
//PID Settings

double kP = .45;//.45
double kI = 0.0001; //0.00013
double kD = 0; //1

//PID gains for InertialDrive (driving)
#define K_P1 .5
#define K_I1 .001
#define K_D1 .7

//Also PID gains for InertialDrive (turning)
#define K_P2 .3
#define K_I2 0
#define K_D2 0

#define H 1.0
 
int error;
int prevError = 0;
int derivative;
int totalError = 0; 

void gyroPIDClock(float desiredValue){
  error=0;
  prevError = 0;
  derivative = 0;
  totalError = 0;
  float intialError;
  intialError = Gyro.heading(degrees) - desiredValue; 
  while(true){
    float position = Gyro.heading(degrees);
    //Propotional
    error = position - desiredValue;
    //Integral
    totalError += error;
    if (error<intialError/3)
      totalError += error;
    //Derivative
    derivative = error - prevError;

    if( abs(error) < .5)  // we will stop within .5 deg from target
    {
       break;
    }

    double motorPower = error * kP  + totalError * kI+ derivative * kD;

    task::sleep(5);
    Left.spin(vex::directionType::rev, motorPower, vex::velocityUnits::pct);
    LeftBack.spin(vex::directionType::rev, motorPower, vex::velocityUnits::pct);
    Right.spin(vex::directionType::fwd, motorPower, vex::velocityUnits::pct);
    RightBack.spin(vex::directionType::fwd, motorPower, vex::velocityUnits::pct);

    prevError = error;
  }
  Right.stop(vex::brakeType::brake);
  RightBack.stop(vex::brakeType::brake);
  LeftBack.stop(vex::brakeType::brake);
  Left.stop(vex::brakeType::brake);
}


void gyroPIDCounter(float desiredValue){
  error=0;
  prevError = 0;
  derivative = 0;
  totalError = 0; 
  float intialError;
  intialError = Gyro.heading(degrees) - desiredValue;
  while(true){
    int position = Gyro.heading(degrees);
    //Propotional
    error = position - desiredValue;
    //Integral
    if (error<intialError/3)
      totalError += error;
    //Derivative
    derivative = error - prevError;

    if( abs(error) < .5)  // we will stop within 1 deg from target
    {
       break;
    }

    double motorPower = error * kP  + totalError * kI+ derivative * kD;

    task::sleep(5);

    Left.spin(vex::directionType::rev, motorPower, vex::velocityUnits::pct);
    LeftBack.spin(vex::directionType::rev, motorPower, vex::velocityUnits::pct);
    RightBack.spin(vex::directionType::fwd, motorPower, vex::velocityUnits::pct);
    Right.spin(vex::directionType::fwd, motorPower, vex::velocityUnits::pct); 
    prevError = error;
  }
  Right.stop(vex::brakeType::brake);
  RightBack.stop(vex::brakeType::brake);
  LeftBack.stop(vex::brakeType::brake);
  Left.stop(vex::brakeType::brake);
}

void gyroPID(float desiredValue){
  if (desiredValue>Gyro.rotation(degrees)){
    gyroPIDClock(desiredValue);
  }
}
void brake_unchecked(){
  Right.stop(vex::brakeType::brake);
  RightBack.stop(vex::brakeType::brake);
  LeftBack.stop(vex::brakeType::brake);
  Left.stop(vex::brakeType::brake);
}
void allBaseVoltage(bool Dir, double v){
  if(Dir){
    Left.spin(fwd, v, volt);
    Right.spin(fwd, v, volt);
    RightBack.spin(fwd, v, volt);
    LeftBack.spin(fwd, v, volt);
  }
  else {
    Left.spin(reverse, v, volt);
    Right.spin(reverse, v, volt);
    RightBack.spin(reverse, v, volt);
    LeftBack.spin(reverse, v, volt);
  }
}
void rush(float tim=3) { // tim is backup time (maybe set high if you dont need other goals in auotn so u dont lose goal)
  float rot=2.5;
  double fwd=1300;
  //yeet.set(true); //ring deploy
  //allBaseVoltage(true, 12);
  Left.setVelocity(100, percent);
  Right.setVelocity(100, percent);
  LeftBack.setVelocity(100, percent);
  RightBack.setVelocity(100, percent);
  Left.setPosition(0, degrees);
  Right.setPosition(0, degrees);
  RightBack.setPosition(0, degrees);
  LeftBack.setPosition(0, degrees);
  while(Left.position(degrees)<fwd && Right.position(degrees)<fwd){
    Left.spin(forward);
    Right.spin(forward);
    RightBack.spin(forward);
    LeftBack.spin(forward);
  }
  Left.stop();
  Right.stop();
  LeftBack.stop();
  RightBack.stop();

  /*
  while(float((Left.position(turns)+Right.position(turns))/2) < rot){ //rot is the # of rotations
    vex::task::sleep(3);
  }
  */
  turnClaw(false);
  allBaseVoltage(false, 12);
  vex::task::sleep(tim*1000);
  brake_unchecked(); //stop all motors
}

  
void autonomous( void ) {

    /* initialize capabilities from buttons */
    bool Skills = buttons[0].state;
    bool LeftRush = buttons[1].state;
    bool RightRush = buttons[2].state;
    bool Middle = buttons[3].state;
    bool Hyper = buttons[4].state;
    bool FillGoal = buttons[5].state;
    bool RightMiddle = buttons[6].state;


    if(Skills){     
      //Grab Blue Mogo     
      turnBackClaw(true);
      turnClaw(true);
      move(.1,100);
      turnRight(15,70,0);
      gyroPID(90);
      move(3.65,90);
      //Grab Yellow Mogo
      turnClaw(false);
      //Stack Yellow Mogoxx
      turnRight(15,60,0);
      gyroPID(110);
      turnArm(1400,100,0);
      move(5,100);
      turnArm(-100,100,0);
      turnClaw(true);
      turnArm(100,100,0);
      //Stack Middle Yellow Mogo
      move(-1,70);
      turnArm(-1400,100,0);
      turnRight(15,50,0);
      gyroPID(270);
      move(1.5,70);
      //Grab Middle Yellow Mogo
      turnClaw(false);
      //Stack Middle Yellow Mogo
      turnArm(1500,100,0);
      turnLeft(20,70,0);
      gyroPID(90);
      move(1.5,70);
      turnArm(-150,100,0);
      turnClaw(true);
      turnArm(150,100,0);
      //Grab Farthest Middle Yellow Mogo
      move(-1,70);
      turnRight(15,70,0);
      gyroPID(200);
      move(3,70);
            
    }
    else if(LeftRush){
      yeet.set(true);
      turnBackClaw(true);
      turnClaw(true);
      move(2.25,100);
      turnClaw(false);
      move(.15,100);
      move(-1.65,100);
      turnRight(30,50,0);
      gyroPID(45);
      move(-1,50);
      turnLeft(30,50,0);
      gyroPID(315);
      yeet.set(false);
      move(-1.8,50);
      turnBackClaw(false);
      move(2,50);
      turnLeft(15,50,0);
      gyroPID(45);
      turnArm(500,100,0);
      intake(true);
      move(2,50);
    }
    else if(RightRush){
      turnClaw(true);
      turnBackClaw(true);
      move(2.25,100);
      turnClaw(false);
      move(.15,100);
      move(-1.65,100);
      turnLeft(20,50,0);
      gyroPID(270);
      move(-.2,60);
      move_time_back(1);
      turnBackClaw(false);
      turnArm(400,100,0);
      intake(true);
      move(.5,100);      
      turnRight(50,50,0);
      gyroPID(2);
      move(1.9,30);
      move(-2.5,60);
    }
    else if(Middle){  
      turnClaw(true);  
      turnBackClaw(true);
      move(3.3,100);
      turnClaw(false);
      move(-1.5,100);
      turnLeft(25,80,0);
      gyroPID(270);
      move(-3,70);
      move_time_back(1);
      turnBackClaw(false);
      intake(true);
      turnArm(700,100,0);
      move(1.5,50);
      
    }
    else if(Hyper){

    }
    else if(FillGoal){
      turnBackClaw(true);
      turnArm(800,100,0);
      move(-1,50);
      turnBackClaw(false);
      intake(true);
      move(1.5,20);
      move(-1.5,50);
      move(1.5,20);
      move(-1.5,50);
      move(1.5,20);

    }
    else if(RightMiddle){
      turnClaw(true);
      turnBackClaw(true);
      move(2.25,100);
      turnClaw(false);
      wait(50,msec);
      move(.15,100);
      move(-1.65,100);
      turnRight(100,100,0);
      turnClaw(true);
      turnLeft(20,50,0);
      gyroPID(303);
      move(2.8,90);
      turnClaw(false);
      move(-3,100);
      turnLeft(20,50,0);
      gyroPID(270);
      move(-3,50);
      move_time_back(1);
      turnBackClaw(false);
      intake(true);
      turnArm(700,100,0);
      move(1.5,50);

    }
}



void usercontrol( void ) {
  Arm.setVelocity(100,percent);
  Intake.setVelocity(100,percent);
  Right.setStopping(hold);
  Left.setStopping(hold);
  RightBack.setStopping(hold);
  LeftBack.setStopping(hold);
  LeftBack.stop(brake);
  Arm.setStopping(hold);
  Left.stop(brake);
  RightBack.stop(brake);
  LeftBack.stop(brake);
  while(1){
    //Drivetrain Code
    double left_speed=Controller1.Axis3.position();
    double right_speed=Controller1.Axis2.position();
    Left.setVelocity(left_speed,percent);
    LeftBack.setVelocity(left_speed,percent);
    RightBack.setVelocity(right_speed,percent);
    Right.setVelocity(right_speed,percent);
    Left.spin(forward);
    LeftBack.spin(forward);
    RightBack.spin(forward);
    Right.spin(forward);
    //Arm code
    if (Controller1.ButtonL1.pressing()){
      Arm.spin(forward);
    }
    else if(Controller1.ButtonL2.pressing()){
      Arm.spin(reverse);
    }
    else{
      Arm.stop(brake);
    }
    //Claw turning
    if (Controller1.ButtonR1.pressing()){
      Claw.set(true);
    }
    else if(Controller1.ButtonR2.pressing()){
      Claw.set(false);
    }
    //BackClaw turning
    if (Controller1.ButtonUp.pressing()){
      Back_Left.set(true);
      Back_Right.set(true);
    }
    else if(Controller1.ButtonDown.pressing()){
      Back_Left.set(false);
      Back_Right.set(false);
    }
    if (Controller1.ButtonX.pressing()){
      Intake.spin(forward);
    }
    else if(Controller1.ButtonB.pressing()){
      Intake.stop(brake);
    }
    else if (Controller1.ButtonA.pressing()){
      Intake.spin(reverse);
    }
    if (Controller1.ButtonRight.pressing()){
      yeet.set(true);
    }
    else if(Controller1.ButtonLeft.pressing()){
      yeet.set(false);
    }
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
    Gyro.calibrate();
    while(Gyro.isCalibrating()){
        wait(100,msec);
    }

    //Set up callbacks for autonomous and driver control periods.
    Competition.autonomous( autonomous );
    Competition.drivercontrol( usercontrol );

    // register events for button selection
    Brain.Screen.pressed( userTouchCallbackPressed );
    Brain.Screen.released( userTouchCallbackReleased );

    // make nice background
    Brain.Screen.setFillColor( vex::color(0x404040) );
    Brain.Screen.setPenColor( vex::color(0x404040) );
    Brain.Screen.drawRectangle( 0, 0, 480, 120 );
    Brain.Screen.setFillColor( vex::color(0x808080) );
    Brain.Screen.setPenColor( vex::color(0x808080) );
    Brain.Screen.drawRectangle( 0, 120, 480, 120 );

    // initial display
    displayButtonControls( 0, false );

    while(1) {
        // Allow other tasks to run
        if( !Competition.isEnabled() )
            Brain.Screen.setFont(fontType::mono40);
        Brain.Screen.setFillColor( vex::color(0xFFFFFF) );

        Brain.Screen.setPenColor( vex::color(0xc11f27));
        Brain.Screen.printAt( 0,  135, "  39S - MSK - V2 - Dubs  " );
        this_thread::sleep_for(10);
    }
}







