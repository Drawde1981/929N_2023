/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>
#include <algorithm>
using namespace std;
using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

//Definition of left motors for drivetrain (PORTS subject to change)
motor lf = motor(PORT1,ratio18_1,true);
motor lm = motor(PORT2,ratio18_1,true);
motor lb = motor(PORT3,ratio18_1,true);

//Definition of right motors for drivetrain (PORTS subject to change)
motor rf = motor(PORT4,ratio18_1,false);
motor rm = motor(PORT5,ratio18_1,false);
motor rb = motor(PORT6,ratio18_1,false);

//Definition of motor groups
motor_group leftDrive = motor_group(lf,lm,lb);
motor_group rightDrive = motor_group(rf,rm,rb);

//Definition of drivetrain
drivetrain driver = drivetrain(leftDrive,rightDrive,3.25*M_PI,13/*Subject to change*/,10.5/*Subject to change*/,inches,1.666);

//Definition of Encoder sesnor and Inertial Gyrometer (PORTS subject to change)
encoder trackWheel = encoder(Brain.ThreeWirePort.A);
inertial inertialSensor = inertial(PORT7);
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  inertialSensor.calibrate(0);

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                             Autonomous Helpers                            */
/*                                                                           */
/*  These helper function help improve the autonomous by increasing          */
/*  efficiency when performing basic motions.                                */
/*---------------------------------------------------------------------------*/

double clamp(double d, double lower, double upper) {
  if(d<lower) {d=lower;}
  if(d>upper) {d=upper;}
  return d;
}

void robotDrive(double dist, int velocity, directionType dir) {
  velocity = clamp(velocity,0,100);
  dist = fabs(dist);
  trackWheel.setPosition(0,turns);
  dist -= 0.075; //Compensation for slight error
  if(dir==vex::reverse) {
    dist = -dist;
  }
  double oldProportion=0;
  int precision = 25;
  double newPercent=10;
  while(1) {
    double distTravel = trackWheel.position(turns)*3.25*M_PI;
    double proportion = distTravel/dist;
    if(proportion>=oldProportion+100/precision) {
      if(proportion<=0.25) {
        newPercent = proportion*360+10;
      } else if(proportion<=0.75) {
        newPercent = 100;
      } else if (proportion<=1) {
        newPercent = -400*(proportion-0.75)+100;
      } else {
        break;
      }
    }
    wait(5,msec);
    driver.stop();
    driver.setDriveVelocity(newPercent,percent);
    driver.drive(dir);
    oldProportion = proportion;
  }
}

void robotTurn(int deg, int velocity, turnType dir) {
  inertialSensor.setRotation(0,degrees);
  velocity = clamp(velocity,0,100);
  while(1) {
    if(deg<0) {
        deg+=360;
        continue;
    }
    break;
  }
  double oldProportion=0;
  int precision = 25;
  double newPercent=10;
  while(1) {
    double degTravel = inertialSensor.rotation(degrees);
    double proportion = degTravel/deg;
    if(proportion>=oldProportion+100/precision) {
      if(proportion<=0.25) {
        newPercent = proportion*360+10;
      } else if(proportion<=0.75) {
        newPercent = 100;
      } else if(proportion<=1) {
        newPercent = -400*(proportion-0.75)+100;
      } else {
        break;
      }
    }
    wait(5,msec);
    driver.stop();
    driver.setTurnVelocity(newPercent,percent);
    driver.turn(dir);
    oldProportion = proportion;
  }
}
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}