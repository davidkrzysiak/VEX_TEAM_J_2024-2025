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

struct state {
   float pos_x;
   float pos_y;
   float theta;
};

struct control {
   float velocity_x;
   float velocity_y;
   float omega;
};

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

class Robot 
{

private:

    float diameter_of_wheels = 2.75; 

    int angle_of_wheels = 45;

    float max_motor_voltage = 11.7;

    PIDController TR_motor;
    PIDController TL_motor;
    PIDController BR_motor;
    PIDController BL_motor;

    void spin_the_wheels() {
        while(true) {
         TR.spin(forward, TR_motor.ComputePID(TR.velocity(rpm)), volt);
         TL.spin(forward, TL_motor.ComputePID(TL.velocity(rpm)), volt);
         BR.spin(forward, BR_motor.ComputePID(BR.velocity(rpm)), volt);
         BL.spin(forward, BL_motor.ComputePID(BL.velocity(rpm)), volt);
        }
    }

public: 

    void drive_with_velocity( control velocity )
    {
        
        TR_motor.SetSetpoint(velocity.velocity_x - velocity.velocity_y + velocity.omega);
        TL_motor.SetSetpoint(velocity.velocity_x + velocity.velocity_y + velocity.omega);
        BR_motor.SetSetpoint(- velocity.velocity_x - velocity.velocity_y + velocity.omega);
        BL_motor.SetSetpoint(- velocity.velocity_x + velocity.velocity_y + velocity.omega);


    }

};

int main()
{

    Robot robot;

    control move;
    move.velocity_x = 1;
    move.velocity_y = 2;
    move.omega = 0.5; 

    robot.drive_with_velocity(move);

}