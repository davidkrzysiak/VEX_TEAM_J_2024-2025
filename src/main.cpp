/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       david                                                     */
/*    Created:      9/3/2024, 11:22:36 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
//#include <math.h>
//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>
#include <iostream>
#include <fstream>

//#include "v5.h"
//#include "v5_vcs.h"

//expiremal branch init commit

#include "vex.h"

using namespace vex;

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain       Brain; 

// define your global instances of motors and other devices here

vex::controller Controller1 = controller(primary);
vex::motor TR = motor(PORT1, ratio18_1, false);
vex::motor TL = motor(PORT2, ratio18_1, false);
vex::motor BL = motor(PORT3, ratio18_1, false);
vex::motor BR = motor(PORT4, ratio18_1, false);
vex::inertial Inertial5 = inertial(PORT5);
vex::motor intake_motor = motor(PORT19, ratio6_1, false);
vex::motor intake_arm_half_motor = motor(PORT18, ratio6_1, false);
digital_out clamp_piston1 = digital_out(Brain.ThreeWirePort.A);
digital_out clamp_piston2 = digital_out(Brain.ThreeWirePort.B);

class PIDController
{
private:
     float m_kP = 0;
     float m_kI = 0;
     float m_kD = 0;

     float m_minOutput = 0;
     float m_maxOutput = 0;

     float m_setpoint;
     float m_iAccumulator = 0;
     float m_lastError = 0;

     bool m_isEnabled = false;

     unsigned long m_previousComputeTime = 0;

public:
     void SetGains(float _kP, float _kI, float _kD)
     {
         m_kP = _kP;
         m_kI = _kI;
         m_kD = _kD;
     }

     void SetSetpoint(float _setpoint)
     {
         m_setpoint = _setpoint;
     }

     float ComputePID(float _input)
     {
         static unsigned long previousComputeTime = 0;
         static unsigned long currentComputeTime;
         currentComputeTime = timer();
         float deltaTime = (currentComputeTime - previousComputeTime) / 1e6;
         float error = m_setpoint - _input;
         float pTerm = m_kP * error;
         float iTerm = m_kI * m_iAccumulator;
         float dTerm = m_kD * (error - m_lastError);

         m_iAccumulator += error * deltaTime;
         m_lastError = error;
         float output = pTerm + iTerm - dTerm;

         previousComputeTime = currentComputeTime;

         if (output > m_maxOutput)
         {
             return m_maxOutput;
         }
         else if (output < m_minOutput)
         {
             return m_minOutput;
         }

         // Serial.println("iAccumulator:\t" + (String) m_iAccumulator +"\terror:\t" + (String) m_lastError);
         return output;
     }

     float GetkP()
     {
         return m_kP;
     }

     float GetkI()
     {
         return m_kI;
     }

     float GetkD()
     {
         return m_kD;
     }

     void SetkP(float _kP)
     {
         m_kP = _kP;
     }
     void SetkI(float _kI)
     {
         m_kI = _kI;
     }
     void SetkD(float _kD)
     {
         m_kD = _kD;
     }

     void SetOutputLimits(float min, float max)
     {
         m_minOutput = min;
         m_maxOutput = max;
     }

     float GetSetpoint()
     {
         return m_setpoint;
     }

     float GetLastError()
     {
         return m_lastError;
     }

     float GetIAccumulator()
     {
         return m_iAccumulator;
     }

     /**
      * @brief Resets the IAccumulator amount as well as the previous
compute time
      *
      */
     void ResetIAccumulator()
     {
         m_iAccumulator = 0;
         m_previousComputeTime = timer();
     }

     bool IsEnabled()
     {
         return m_isEnabled;
     }

     void SetEnabled(bool _isEnabled)
     {
         if (_isEnabled != m_isEnabled)
         {
             m_isEnabled = _isEnabled;
             if (m_isEnabled)
             {
                 ResetIAccumulator();
             }
         }
     }
};

//this could potentialy output error to do calculations
//these are fuctions as not to clutter main 
void drive_robot(); 

void robot_auto();

//these next functions are for automatic robot movement mainly for auto but can be used for driver control if we want 

void translate_robot(float X_pos_inches, float Y_pos_inches); 

void rotate_robot(float theta, int rotdirection);

void move_rightward();

void move_leftward();

// these are suppose to be in the tranlates function but appently vex is dumb and olny does callbacks in threads
// these are meant to be the distance the robot wants in each converted axis they will change based on the translate function 

double distancce_rightward = 0;

double distancce_leftward = 0;

//these are constants of the robot so if somthing changes we just have to change it here instead of the entire code 

float diameter_of_wheels = 2.75; 

int angle_of_wheels = 45;

float max_motor_voltage = 11.7;

float acceleration_constant = .5; 

float driver_contol_voltage_cap = 0.60;

//these functions are for computing valves to make code less confusing 

double distance_to_wheel_rotations(float distance_);

//float Robot_position_absolute();

int main() {

  competition Competition;

  Competition.drivercontrol(drive_robot);

  Competition.autonomous(robot_auto);

  clamp_piston1.set(false); 

  while (true) {
    wait(100, msec);
  }

}

void drive_robot() {

  while(true) {

    float volt_cap = driver_contol_voltage_cap;

    long direction_coord[3] = {Controller1.Axis4.position(percent), Controller1.Axis3.position(percent), Controller1.Axis1.position(percent)};

    // the direction_coord stores the contollers position in (X, Y, ϴ) the first position would be X second y and so on 

    if (Controller1.ButtonB.pressing() == false) {

      volt_cap = 1; 

    }

    TR.setVelocity( volt_cap * (direction_coord[0] - direction_coord[1] + direction_coord[2]) ,percent);
    TL.setVelocity( volt_cap * (direction_coord[0] + direction_coord[1] + direction_coord[2]) ,percent);
    BL.setVelocity( volt_cap * (-direction_coord[0] + direction_coord[1] + direction_coord[2]) ,percent);
    BR.setVelocity( volt_cap * (-direction_coord[0] - direction_coord[1] + direction_coord[2]) ,percent);

    TR.spin(forward);
    TL.spin(forward);
    BL.spin(forward);
    BR.spin(forward);

    //clamp motor code 

    if(Controller1.ButtonR1.pressing() == true) {

     clamp_piston1.set(false);
     clamp_piston2.set(false);

    } else if(Controller1.ButtonL1.pressing() == true) {

     clamp_piston1.set(true);
     clamp_piston2.set(true);

    }

    // intake code

    if( Controller1.ButtonR2.pressing() == true ) {

      intake_motor.spin(forward, 12, volt);
      intake_arm_half_motor.spin(forward, 12, volt);

    } 
    else if( Controller1.ButtonL2.pressing() == true ) {

      intake_motor.spin(forward, -12, volt);
      intake_arm_half_motor.spin(forward, -12, volt);

    }
    else { intake_motor.stop(); intake_arm_half_motor.stop();}

  }

}

void robot_auto() {

  bool record_data = false; 

  if( Brain.SDcard.isInserted() == true ) {

    record_data = true; 

  }

  clamp_piston1.set(false);

  wait(1, seconds );

  translate_robot(0,-12);

  clamp_piston1.set(true);

  wait(1,seconds);

  rotate_robot(0.18, -1);

  intake_arm_half_motor.spin(reverse, 12, volt);
  intake_motor.spin(reverse, 12, volt);

  translate_robot(0,24);

  rotate_robot(2, -1);

  translate_robot(0,6);

  rotate_robot(1, -1);

  translate_robot(0,6);

  rotate_robot(2, 1);

  translate_robot(0,12);

  clamp_piston1.set(false);

}

void rotate_robot(float theta, int rotdirection) {

  if (rotdirection == 1) {

    TR.spin(forward, 7, volt);
    TL.spin(forward, 7, volt);
    BR.spin(forward, 7, volt);
    BL.spin(forward, 7, volt);

    float timery = 0;

    while(theta > timery) {

      wait(.005, seconds);

      timery = timery + .005;

    }

    TR.stop();
    TL.stop();
    BR.stop();
    BL.stop();

  }

  if (rotdirection == -1) {

    TR.spin(reverse, 7, volt);
    TL.spin(reverse, 7, volt);
    BR.spin(reverse, 7, volt);
    BL.spin(reverse, 7, volt);

    float timery = 0;

    while(theta > timery) {

      wait(.005, seconds);

      timery = timery + .005;

    }

    TR.stop();
    TL.stop();
    BR.stop();
    BL.stop();


  }

  // float P_tuning_para = .15;

  // Inertial5.resetHeading();

  // if ( rotdirection == 1 ) {

  //   float error = theta - Inertial5.heading(degrees);

  //   if ( error < 0 ) {

  //     error = theta;

  //   }

  //   while(true){

  //     TR.spin(forward, error * P_tuning_para, volt );
  //     TL.spin(forward, error * P_tuning_para, volt ); 
  //     BR.spin(forward, error * P_tuning_para, volt );
  //     BL.spin(forward, error * P_tuning_para, volt );

  //     error = theta - Inertial5.heading();

  //     if (error < 0.5 && error > -0.5) {

  //       TR.stop(hold);
  //       TL.stop(hold);
  //       BR.stop(hold);
  //       BL.stop(hold);

  //       break;

  //     }

  //   }
    
  // }
  // else {

  //   theta = 360 - theta; 

  //   float error = -(theta - Inertial5.heading(degrees)); 

  //   while(true){

  //     TR.spin(forward, error * P_tuning_para, volt );
  //     TL.spin(forward, error * P_tuning_para, volt ); 
  //     BR.spin(forward, error * P_tuning_para, volt );
  //     BL.spin(forward, error * P_tuning_para, volt );

  //     error = -(theta - Inertial5.heading());

  //     if (error < 0.5 && error > -0.5) {

  //       TR.stop(hold);
  //       TL.stop(hold);
  //       BR.stop(hold);
  //       BL.stop(hold);

  //       break;

  //     }

  //   }

  // }

}

void translate_robot(float X_pos_inches, float Y_pos_inches) {

  distancce_rightward = distance_to_wheel_rotations((Y_pos_inches + X_pos_inches)/sqrt(2));
  distancce_leftward = distance_to_wheel_rotations((Y_pos_inches - X_pos_inches)/sqrt(2));    // this is the mathamatics to transform X,Y coords to 45 degree perp lines

  Brain.Screen.print(distancce_leftward);
  Brain.Screen.newLine();  
  Brain.Screen.print(distancce_rightward);
  Brain.Screen.newLine();
  Brain.Screen.print("help");

  Brain.setTimer(0, seconds); 

  thread thread1 = thread(move_rightward);
  
  thread thread2 = thread(move_leftward);

  wait(1, seconds);

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

  // the rotation reaeding and values set in terms of roatition are just to temporarly control abgle of thew robot while it's moving to prevent drift
  Inertial5.setHeading(180, degrees);

  while( TL.voltage(volt)  <  11.5 ) {

    if (distancce_rightward - TL.position(degrees) < 700) {

      break; 

    }

    TL.spin(forward, (acceleration_constant * Brain.timer(seconds)) + (.2 * (180 - Inertial5.heading(degrees))), volt);
    BR.spin(forward, (-acceleration_constant * Brain.timer(seconds)) + (.2 * (180 - Inertial5.heading(degrees))), volt);

  }

  while (true) {

    if (distancce_rightward - TL.position(degrees) < 700) {

      break; 

    }

    TL.spin(forward, 11.6 + (.2 * (180 - Inertial5.heading(degrees))), volt);
    BR.spin(forward, -11.6 + (.2 * (180 - Inertial5.heading(degrees))), volt);  

  }

  float error = 0; 

  Brain.Screen.print(distancce_rightward);
  Brain.Screen.newLine();

  float P_param = 0.09;
  float I_param = 0.0000001;
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

    TL.spin(forward, voltage_to_motor + (.2 * (180 - Inertial5.heading(degrees))), volt);
    BR.spin(forward, (- voltage_to_motor) + (.2 * (180 - Inertial5.heading(degrees))), volt);

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

  while( TR.voltage(volt)  <  11.5 ) {

    if (distancce_leftward + TR.position(degrees) < 700) {

      break; 

    }

    TR.spin(forward, -acceleration_constant * Brain.timer(seconds), volt);
    BL.spin(forward, acceleration_constant * Brain.timer(seconds), volt);

  }

  while (true) {

    if(distancce_leftward + TR.position(degrees) < 700) {

      break;

    }

    TR.spin(forward, -11.6, volt);
    BL.spin(forward, 11.6, volt);  

  }

  float error = 0; 

  Brain.Screen.print(distancce_leftward);
  Brain.Screen.newLine();

  float P_param = 0.09;
  float I_param = 0.0000001;
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

//float Robot_position_absolute() {


 

 
//}