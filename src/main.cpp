/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       david                                                     */
/*    Created:      9/3/2024, 11:22:36 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain       Brain;

// define your global instances of motors and other devices here

vex::controller Controller1 = controller(primary);
vex::motor TR = motor(PORT1, ratio6_1, false);
vex::motor TL = motor(PORT2, ratio6_1, false);
vex::motor BL = motor(PORT3, ratio6_1, false);
vex::motor BR = motor(PORT4, ratio6_1, false);
vex::inertial Inertial5 = inertial(PORT5);


//this could potentialy output error to do calculations
//these are fuctions as not to clutter main 
void drive_robot(float X_direction, float Y_direction, float angular_direction); 

void robot_auto();

//these are constants of the robot so if somthing changes we just have to change it here instead of the entire code 

float diameter_of_wheels = 2.75; 

int angle_of_wheels = 45;

//these next functions are for automatic robot movement mainly for auto but can be used for driver control if we want 

void translate_robot(float X_pos_inches, float Y_pos_inches); 

void rotate_robot(float theta);

//these functions are for computing valves to make code less confusing 

float distance_to_wheel_rotations(float distance_);

int main() {

  while(true) {

    // put driver control stuff here
    float X_input = Controller1.Axis4.position(percent);

    float Y_input = Controller1.Axis3.position(percent); // =make these into array at some point 

    float theta_input = Controller1.Axis1.position(percent);

    drive_robot(X_input, Y_input, theta_input);

    // eventually we'll use the other fancy robot call functions

    if(Controller1.ButtonA.pressing() == true) {
      
      Inertial5.calibrate();

      wait(3, seconds); 
      
      robot_auto();

    }

  }
  
}

void drive_robot(float X_direction, float Y_direction, float angular_direction) {

  TR.setVelocity(X_direction - Y_direction + angular_direction ,percent);
  TL.setVelocity(X_direction + Y_direction + angular_direction ,percent);
  BL.setVelocity(-X_direction + Y_direction + angular_direction ,percent);
  BR.setVelocity(-X_direction - Y_direction + angular_direction ,percent);

  TR.spin(forward);
  TL.spin(forward);
  BL.spin(forward);
  BR.spin(forward);

}

void robot_auto() {

  //rotate_robot(45);

  TR.spinToPosition( 1000, degrees, true);

 // translate_robot(0, 30);

}

void rotate_robot(float theta) {

  float P_tuning_para = .25;
  
  int error = (theta - Inertial5.heading(degrees)) / 365;

  while (true) {

    TR.setVelocity(-P_tuning_para * error ,percent);
    TL.setVelocity(-P_tuning_para * error ,percent);
    BL.setVelocity(-P_tuning_para * error ,percent);
    BR.setVelocity(-P_tuning_para * error ,percent); 

    TR.spin(forward);
    TL.spin(forward);
    BL.spin(forward);
    BR.spin(forward); 

    error = theta - Inertial5.heading(degrees);

    if(abs(error) < 1) {

      break;

    }

  }

  TR.stop();
  TL.stop();
  BL.stop();
  BR.stop();

}

void translate_robot(float X_pos_inches, float Y_pos_inches) {

  TR.spinToPosition(- distance_to_wheel_rotations(Y_pos_inches) * .707 , degrees, false);
  TL.spinToPosition(distance_to_wheel_rotations(Y_pos_inches) * .707 , degrees, false);
  BL.spinToPosition(distance_to_wheel_rotations(Y_pos_inches) * .707 , degrees, false);
  TR.spinToPosition(- distance_to_wheel_rotations(Y_pos_inches) * .707 , degrees, true);

  



}

float distance_to_wheel_rotations(float distance_) {

  int rotations = (distance_ / (diameter_of_wheels * 3.14)) * 365;

  return rotations;

}