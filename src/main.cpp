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

double distance_to_wheel_rotations(float distance_);

int main() {

  translate_robot(0,30);

  while(true) {

    // put driver control stuff here
    float X_input = Controller1.Axis4.position(percent);

    float Y_input = Controller1.Axis3.position(percent); // =make these into array at some point 

    float theta_input = Controller1.Axis1.position(percent);

    drive_robot(X_input, Y_input, theta_input);

    // eventually we'll use the other fancy robot call functions

    //this is cause i have no controller change it to == if there is a controler or need to test driver control 

    if(Controller1.ButtonA.pressing() == true) { 
      
      Inertial5.calibrate();

      wait(5, seconds); 
      
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

  bool record_data = false; 

  if( Brain.SDcard.isInserted() == true ) {

    record_data = true; 

  }

  rotate_robot(45);

  //translate_robot(0, 30);

}

void rotate_robot(float theta) {

  float P_tuning_para = .15;

  int direction = 1; //the robot will go couterclockwise if neg and clockwise if pos 
  
  int error = theta - Inertial5.heading(degrees);

  while (true) {

    TR.setVelocity(direction * P_tuning_para * error ,percent);
    TL.setVelocity(direction * P_tuning_para * error ,percent);
    BL.setVelocity(direction * P_tuning_para * error ,percent);
    BR.setVelocity(direction * P_tuning_para * error ,percent); 

    TR.spin(forward);
    TL.spin(forward);
    BL.spin(forward);
    BR.spin(forward); 

    error = theta - Inertial5.heading(degrees);

    Brain.Screen.print(error);

    if(abs(error) < 1) {

      TR.stop(hold);
      TL.stop(hold);
      BL.stop(hold);
      BR.stop(hold);

      break;

    }

  }

  TR.setBrake(brake);
  TL.setBrake(brake);
  BL.setBrake(brake);
  BR.setBrake(brake);

}

void translate_robot(float X_pos_inches, float Y_pos_inches) {

  //this block moves it in the Y direction 

  TR.setPosition(- distance_to_wheel_rotations(Y_pos_inches) * .707 , degrees); 
  TL.setPosition(distance_to_wheel_rotations(Y_pos_inches) * .707 , degrees);
  BL.setPosition(distance_to_wheel_rotations(Y_pos_inches) * .707 , degrees);
  TR.setPosition(- distance_to_wheel_rotations(Y_pos_inches) * .707 , degrees);


  TR.spinToPosition(- distance_to_wheel_rotations(Y_pos_inches) * .707 , degrees, false); 
  TL.spinToPosition(distance_to_wheel_rotations(Y_pos_inches) * .707 , degrees, false);
  BL.spinToPosition(distance_to_wheel_rotations(Y_pos_inches) * .707 , degrees, false);
  TR.spinToPosition(- distance_to_wheel_rotations(Y_pos_inches) * .707 , degrees, true);

  

  //this block moves it in the X direction

  TR.spinToPosition(distance_to_wheel_rotations(X_pos_inches) * .707 , degrees, false);
  TL.spinToPosition( - distance_to_wheel_rotations(X_pos_inches) * .707 , degrees, false);
  BL.spinToPosition( - distance_to_wheel_rotations(X_pos_inches) * .707 , degrees, false);
  TR.spinToPosition(distance_to_wheel_rotations(X_pos_inches) * .707 , degrees, true);

}

double distance_to_wheel_rotations(float distance_) {

  double degrees = (distance_ / (diameter_of_wheels * 3.14)) * 360;

  return degrees;

}