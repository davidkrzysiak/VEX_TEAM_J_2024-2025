/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       david                                                     */
/*    Created:      9/3/2024, 11:22:36 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <fstream>

#include "v5.h"
#include "v5_vcs.h"


//#include "vex.h"

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
vex::motor Clamp_motor = motor(PORT6, ratio36_1, false);
vex::motor intake_motor = motor(PORT12, ratio6_1, false);
vex::motor spinny_screw = motor(PORT13, ratio18_1, false);

//this could potentialy output error to do calculations
//these are fuctions as not to clutter main 
void drive_robot(); 

void robot_auto();

//these next functions are for automatic robot movement mainly for auto but can be used for driver control if we want 

void translate_robot(float X_pos_inches, float Y_pos_inches); 

void rotate_robot(float theta);

void move_rightward();

void move_leftward();

// these are suppose to be in the tranlates function but appently vex is dumb and olny does callbacks in threads
// these are meant to be the distance the robot wants in each converted axis they will change based on the translate function 

double distancce_rightward = 1000;

double distancce_leftward = 890;

//these are constants of the robot so if somthing changes we just have to change it here instead of the entire code 

float diameter_of_wheels = 2.75; 

int angle_of_wheels = 45;

float max_motor_voltage = 11.5;

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

    long direction_coord[3] = {Controller1.Axis4.position(percent), Controller1.Axis3.position(percent), Controller1.Axis1.position(percent)};

    // the direction_coord stores the contollers position in (X, Y, ϴ) the first position would be X second y and so on 

    TR.setVelocity( direction_coord[0] - direction_coord[1] + direction_coord[2] ,percent);
    TL.setVelocity( direction_coord[0] + direction_coord[1] + direction_coord[2] ,percent);
    BL.setVelocity(-direction_coord[0] + direction_coord[1] + direction_coord[2] ,percent);
    BR.setVelocity(-direction_coord[0] - direction_coord[1] + direction_coord[2] ,percent);

    TR.spin(forward);
    TL.spin(forward);
    BL.spin(forward);
    BR.spin(forward);

    Clamp_motor.setBrake(hold);

    if( Controller1.ButtonR1.pressing() == true ) {

      Clamp_motor.spinToPosition(Clamp_motor.position(degrees) + 6, degrees);

    }
    else { Clamp_motor.stop() ;}

    if( Controller1.ButtonL1.pressing() == true ) {

      Clamp_motor.spinToPosition(Clamp_motor.position(degrees) - 6, degrees);

    }

    if( Controller1.ButtonR2.pressing() == true ) {

      intake_motor.spin(forward, 12, volt);

    } 
    else if( Controller1.ButtonL2.pressing() == true ) {

      intake_motor.spin(forward, -12, volt);

    }
    else {

      intake_motor.stop();

    }

    if ( Controller1.ButtonA.pressing() == true) {

      spinny_screw.spin(forward, 12, volt);

    }
    else {

      spinny_screw.stop();

    }

  }

}

void robot_auto() {

  bool record_data = false; 

  if( Brain.SDcard.isInserted() == true ) {

    record_data = true; 

  }

  //rotate_robot(300);

  translate_robot(10,0);

  Brain.Screen.print("done");

}

void rotate_robot(float theta) {

  float P_tuning_para = .45;

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

    if(error < 0.4 && error > -0.4) {

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

  distancce_rightward = distance_to_wheel_rotations((Y_pos_inches + X_pos_inches)/sqrt(2));
  distancce_leftward = distance_to_wheel_rotations((Y_pos_inches - X_pos_inches)/sqrt(2));    // this is the mathamatics to transform X,Y coords to 45 degree perp lines

  thread thread1 = thread(move_rightward);
  
  thread thread2 = thread(move_leftward);

  wait(.5, seconds);

  while (TR.voltage() != 0 && TL.voltage() != 0 && BR.voltage() != 0 && BL.voltage() != 0) {

    wait(.1, seconds);

  }
  
}

double distance_to_wheel_rotations(float distance_) {

  double degrees = (distance_ / (diameter_of_wheels * 3.14)) * 360;        // this might explode and we'll nmeed to change a bunch of stuff to doubles

  return degrees;

}

void move_rightward() {

  TL.setPosition(0, degrees);
  BR.setPosition(0, degrees);

  float error = distancce_rightward; 

  Brain.Screen.print(distancce_rightward);
  Brain.Screen.newLine();

  float P_param = 0.025;
  float I_param = 0.000000001;
  float D_param = 0.00015;

  float voltage_to_motor = 0;

  float error_accumalate = 0;

  float error_previous = error;

  while (true) {

    float wheel_pos = (TL.position(degrees) + -(BR.position(degrees))) / 2;

    //float wheel_pos = TL.position(degrees);
    
    error = distancce_rightward - wheel_pos; 

    voltage_to_motor = D_param * (error - error_previous);

    error_previous = error; 

    voltage_to_motor = voltage_to_motor  + (P_param * (error)) + (I_param * (error_accumalate));

    error_accumalate = error_accumalate + error;

    if ( voltage_to_motor > max_motor_voltage ) {

      voltage_to_motor = max_motor_voltage;

    }

    TL.spin(forward, voltage_to_motor, volt);
    BR.spin(reverse, voltage_to_motor, volt);

    if (error < 4 && error > -4) {
      break;
    }
    

  }

  TL.spin(forward, 0, volt);
  BR.spin(reverse, 0, volt);

}

void move_leftward() {

  TR.setPosition(0, degrees);
  BL.setPosition(0, degrees);

  float error = distancce_leftward; 

  Brain.Screen.print(distancce_leftward);
  Brain.Screen.newLine();

  float P_param = 0.025;
  float I_param = 0.000000001;
  float D_param = 0.00015;

  float voltage_to_motor = 0;

  float error_accumalate = error;

  float error_previous = error;

  while (true) {


    float wheel_pos = (-(TR.position(degrees)) + BL.position(degrees)) / 2;

    //float wheel_pos = -TR.position(degrees);
    
    error = distancce_leftward - wheel_pos; 

    voltage_to_motor = D_param * (error - error_previous);

    error_previous = error; 

    voltage_to_motor = voltage_to_motor  + (P_param * (error)) + (I_param * (error_accumalate));

    error_accumalate = error_accumalate + error;

    if ( voltage_to_motor > max_motor_voltage ) {

      voltage_to_motor = max_motor_voltage;

    }

    TR.spin(reverse, voltage_to_motor, volt);
    BL.spin(forward, voltage_to_motor, volt);

    if (error < 4 && error > -4) {
      break;
    }

  }

  TR.stop();
  BL.stop();

}