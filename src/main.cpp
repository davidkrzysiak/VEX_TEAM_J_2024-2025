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
void drive_robot(); 

void robot_auto();

//these next functions are for automatic robot movement mainly for auto but can be used for driver control if we want 

void translate_robot(float X_pos_inches, float Y_pos_inches); 

void rotate_robot(float theta);

//these are constants of the robot so if somthing changes we just have to change it here instead of the entire code 

float diameter_of_wheels = 2.75; 

int angle_of_wheels = 45;

//these functions are for computing valves to make code less confusing 

double distance_to_wheel_rotations(float distance_);

int main() {

  competition Competition;

  Competition.drivercontrol(drive_robot);

  Competition.autonomous(robot_auto);

  while (true) {
    wait(100, msec);
  }

}

void drive_robot() {

  while(true) {

    float direction_coord[3] = {Controller1.Axis4.position(percent), Controller1.Axis3.position(percent), Controller1.Axis1.position(percent)};

    // the direction_coord stores the contollers position in (X, Y, ϴ) the first position would be X second y and so on 

    TR.setVelocity( direction_coord[1] - direction_coord[2] + direction_coord[3] ,percent);
    TL.setVelocity( direction_coord[1] + direction_coord[2] + direction_coord[3] ,percent);
    BL.setVelocity(-direction_coord[1] + direction_coord[2] + direction_coord[3] ,percent);
    BR.setVelocity(-direction_coord[1] - direction_coord[2] + direction_coord[3] ,percent);

    TR.spin(forward);
    TL.spin(forward);
    BL.spin(forward);
    BR.spin(forward);


  }


}

void robot_auto() {

  bool record_data = false; 

  if( Brain.SDcard.isInserted() == true ) {

    record_data = true; 

  }

  rotate_robot(45);

  translate_robot(30, 30);

}

void rotate_robot(float theta) {

  float P_tuning_para = .3;

  int direction = 1; //the robot will go couterclockwise if neg and clockwise if pos 
  
  float error = theta - Inertial5.heading(degrees);

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

    if(error < 0.25 && error > -0.25) {

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

  Brain.Screen.print("help");

  //this block moves it in the Y direction 

  TR.spinToPosition(- distance_to_wheel_rotations(Y_pos_inches) * .707 , degrees, false); 
  TL.spinToPosition(distance_to_wheel_rotations(Y_pos_inches) * .707 , degrees, false);
  BL.spinToPosition(distance_to_wheel_rotations(Y_pos_inches) * .707 , degrees, false);
  BR.spinToPosition(- distance_to_wheel_rotations(Y_pos_inches) * .707 , degrees, true);

  TR.setPosition( 0, degrees);
  TL.setPosition( 0, degrees);
  BL.setPosition( 0, degrees);
  BR.setPosition( 0, degrees);

  //this block moves it in the X direction

  TR.spinToPosition(distance_to_wheel_rotations(X_pos_inches) * .707 , degrees, false);
  TL.spinToPosition(distance_to_wheel_rotations(X_pos_inches) * .707 , degrees, false);
  BL.spinToPosition( - distance_to_wheel_rotations(X_pos_inches) * .707 , degrees, false);
  BR.spinToPosition( - distance_to_wheel_rotations(X_pos_inches) * .707 , degrees, true);

  TR.setPosition( 0, degrees); 
  TL.setPosition( 0, degrees);
  BL.setPosition( 0, degrees);
  BR.setPosition( 0, degrees);

}

double distance_to_wheel_rotations(float distance_) {

  double degrees = (distance_ / (diameter_of_wheels * 3.14)) * 360;

  return degrees;

}