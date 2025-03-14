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
#include <list>

#include "v5.h"
#include "v5_vcs.h"

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
vex::motor lady_brown = motor(PORT11, ratio36_1, false);
vex::gps GPS = gps(PORT17);
digital_out clamp_piston1 = digital_out(Brain.ThreeWirePort.A);
digital_out clamp_piston2 = digital_out(Brain.ThreeWirePort.B);
vex::rotation X_encoder = rotation(PORT16);
vex::rotation Y_encoder = rotation(PORT15);
//enables functions i may need when programming when holding R1 on startup 
bool debug_mode = true;

//TODO 
//fix intake func(rn its backwards)
//improve odom precision 

// these are some structs to control and store the robots states in a compact form

struct state {
   float pos_x = 0;
   float pos_y = 0;
   float theta = 0;
   bool piston_clamped = false;
   float intake_moving = 0;

   state() = default;

   state(int pos_x, int pos_y, int theta, bool piston_clamped, int intake_moving)
        :pos_x(pos_x), pos_y(pos_y), theta(theta), piston_clamped(piston_clamped), intake_moving(intake_moving) {}
};

struct control {
   float velocity_x;
   float velocity_y;
   float omega;
};

// this is the number that will be updated constantly

static state Robot_state;

// self explanatory

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
         currentComputeTime = vex::timer::system();
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
         m_previousComputeTime = vex::timer::system();
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
   
static PIDController TR_motor;
static PIDController TL_motor;
static PIDController BR_motor;
static PIDController BL_motor;

//this below is the list that controls auto its a sequence that tells the code where the robot should be 
//currently its based on position so it will stop at every point(kinda theres nuiance)
std::list<state> Automonus_tragectory = {
    //state[x-pos(mm), y-pos(mm), theta(degrees), piston - up = false - down = true, Intake toggle - 1 = forward - 0 = off - -1 = backward] 
    state{-1400,-750,30,false,0},
    state{-1151,-672,30,true,0},
    state{-1123,-582,180,true,-1},
    state{-740,-545,180,true,-1},
    state{-468,-812,90,true,-1},
    state{-422,-1224,90,true,-1},
    state{-756,-1493,21,true,-1},
    state{-943,-1606,22,true,-1},
    state{-917,-1658,283,true,-1},
    state{-883,-1323,270,true,-1},
    state{-1025,-1416,357,true,-1},
    state{-1437,-1313,3,true,-1},
    state{-1745,-1341,246,false,-1},
    state{-1150,240,90,false,0},
    state{-1150,568,84,true,0}
};

//robot class!!!! yay this define the things the robot can do and its parameters

class Robot
{
private:

    bool piston_pos = false;  

    static float get_average_angle(float angle1, float angle2) {

            // Convert degrees to radians
        double rad1 = angle1 * 3.14159 / 180.0;
        double rad2 = angle2 * 3.14159 / 180.0;

        // Compute the average using vector components
        double x = (cos(rad1) + cos(rad2)) / 2.0;
        double y = (sin(rad1) + sin(rad2)) / 2.0;

        // Calculate the resultant angle
        double meanRad = atan2(y, x);

        // Convert back to degrees
        double meanDeg = meanRad * 180.0 / 3.14159;

        // Ensure the angle is within [0, 360)
        if (meanDeg < 0) {
            meanDeg += 360.0;
        }

        return meanDeg;
    }

    static float wieghted_average_of_2_values(float mean1, float wieght1, float mean2, float wieght2) {

        return (((mean1*wieght1) + (mean2 * wieght2)) / (wieght1 + wieght2));

    }

    static float deg_to_rad(float degree) {
        return degree * (M_PI/180);
    }

    static float normalize_angle_deg(float input_angle ) {

        while (input_angle > 180) {
            input_angle -= 360;
        }
        while (input_angle < -180) {
            input_angle += 360;
        }

        return input_angle;
    }

    static float heading_to_angle(float heading) {

        float angle = 90 - heading; 

        return normalize_angle_deg(angle);

    }

    static float compute_mean(float data_list[], int size) {

        float sum = 0;

        for(int i = 0; i < size;) {

            sum = sum + data_list[i];

            i = i + 1;

        }

        return sum / size;

    }

    static float compute_fake_stdev(float data_list[], float mean, int size) {
    
        float sum = 0;

        for(int i = 0; i < size;) {

            sum = sum + abs(mean - data_list[i]);

            i = i + 1;

        }

        return sum / size;

    }

    
public:

    static void state_updater() {

        //todo 
        // figure out how to make state estimation less jittery; i maybe fized this ?!?!?!!?? maybe add time delay so it has enough time to change
        // figure out the inital state of the robot 
        // learn how to use odom decay (evaluate if its even useful with the jittery gps)

        //calibration sequence

        Inertial5.calibrate();
        GPS.calibrate();
        while(GPS.isCalibrating()) { wait(0.1, seconds); }
        Inertial5.setHeading(GPS.heading(),degrees);

        // Configuration constants
        const float WHEEL_DIAMETER = 50.8; // mm
        const float ENCODER_TPI = (360.0 / (WHEEL_DIAMETER * M_PI)); // Ticks per mm
        const float GPS_TRUST_THRESHOLD = 0.85; // Minimum GPS quality to trust
        const float odometry_wieght = 100; // Odometry error decay factor
        const int number_of_samples = 5;
        const float stall_threshold = .001;

        // State variables
        float previous_degree_x = -X_encoder.position(degrees);     
        float previous_degree_y = Y_encoder.position(degrees);
        float odometry_x = GPS.xPosition();
        float odometry_y = GPS.yPosition();

        while(true) {

            // sample the The gps positions
            float gps_sample_x[number_of_samples];
            float gps_sample_y[number_of_samples];

            for (int i = 0; i < number_of_samples;) {
                
                gps_sample_x[i] = GPS.xPosition(); 
                gps_sample_y[i] = GPS.yPosition();

                i = i + 1;

                wait(.04, seconds);
            }

            float gps_x = compute_mean(gps_sample_x, number_of_samples);
            float gps_y = compute_mean(gps_sample_y, number_of_samples);
            float gps_heading = GPS.heading();
            float gps_quality = GPS.quality();
            float inertial_heading = Inertial5.heading();

            // Calculate encoder deltas in inches
            float dx_enc = (-X_encoder.position(degrees) - previous_degree_x) / ENCODER_TPI;
            float dy_enc = (Y_encoder.position(degrees) - previous_degree_y) / ENCODER_TPI;
            previous_degree_x = -X_encoder.position(degrees);
            previous_degree_y = Y_encoder.position(degrees);

            // gets the average angle between gps and gyro when gps quality is high 

            if(gps_quality > GPS_TRUST_THRESHOLD) {
            
                Robot_state.theta = get_average_angle(heading_to_angle(gps_heading), heading_to_angle(inertial_heading));

            }else {

                Robot_state.theta = heading_to_angle(inertial_heading);

            }

            // Convert encoder deltas to global coordinates
            float theta_rad = deg_to_rad(- Robot_state.theta);
            float cos_theta = cosf(theta_rad);
            float sin_theta = sinf(theta_rad);
        
            odometry_x = dx_enc * cos_theta - dy_enc * sin_theta;
            odometry_y = dx_enc * sin_theta + dy_enc * cos_theta;

            // Fuse GPS and odometry using adaptive weighting
            if(gps_quality > GPS_TRUST_THRESHOLD) {
            
                Robot_state.pos_x = wieghted_average_of_2_values(gps_x, gps_quality, Robot_state.pos_x + odometry_x, odometry_wieght);
                Robot_state.pos_y = wieghted_average_of_2_values(gps_y, gps_quality, Robot_state.pos_y + odometry_y, odometry_wieght);

            } else {
                // Use pure odometry when GPS is unreliable
                Robot_state.pos_x = Robot_state.pos_x + odometry_x;
                Robot_state.pos_y = Robot_state.pos_y + odometry_y;
            }

            // Apply odometry error decay
            //odometry_x += (Robot_state.pos_x - odometry_x) * (1 - ODOMETRY_DECAY);
            //odometry_y += (Robot_state.pos_y - odometry_y) * (1 - ODOMETRY_DECAY);

            wait(1, msec); // Consistent update interval
        }
    }

    static void spin() {
        while(true) {
        TR.spin(forward, TR_motor.ComputePID(TR.velocity(rpm)), volt);
        TL.spin(forward, TL_motor.ComputePID(TL.velocity(rpm)), volt);
        BR.spin(forward, BR_motor.ComputePID(BR.velocity(rpm)), volt);
        BL.spin(forward, BL_motor.ComputePID(BL.velocity(rpm)), volt);
        }
        
    }

    void drive_with_velocity( control velocity )
    {

        TR_motor.SetSetpoint(velocity.velocity_x - velocity.velocity_y + velocity.omega);
        TL_motor.SetSetpoint(velocity.velocity_x + velocity.velocity_y + velocity.omega);
        BR_motor.SetSetpoint(- velocity.velocity_x - velocity.velocity_y + velocity.omega);
        BL_motor.SetSetpoint(- velocity.velocity_x + velocity.velocity_y + velocity.omega);

    }

    void run_intake(int direction) {
        // the direction parameter defines which way you want it to go -1 for backwards 1 for forwards and 0 for stop 
        //ig you put any number but it is already maxed out 
        intake_motor.spin(forward, direction *10, volt);   
        intake_arm_half_motor.spin(forward, direction*10, volt);

    }

    void set_piston_to(bool state){

        clamp_piston1.set(state);

        piston_pos = state;

    }

    void close_piston() {

        clamp_piston1.set(false);

        piston_pos = false;

    }

    void open_piston() {

        clamp_piston1.set(true);

        piston_pos = true;
       
    }

    // this will eventually turn into everything i want to display on the UI 
    static void robot_heat() {

      while(true) {
        
        Controller1.Screen.clearScreen();

      // Print all motor temperatures in one line
        //Controller1.Screen.setCursor(1, 1); // Set cursor to the first line
     
        //Controller1.Screen.print("T%.1f T%.1f B%.1f B%.1f",TR.temperature(temperatureUnits::celsius), TL.temperature(temperatureUnits::celsius), BR.temperature(temperatureUnits::celsius), BL.temperature(temperatureUnits::celsius));
        //Controller1.Screen.newLine();
        //Controller1.Screen.print(Robot_state.pos_x);
        //Controller1.Screen.newLine();
        //Controller1.Screen.print(Robot_state.pos_y);

        Controller1.Screen.setCursor(1, 1); // Set cursor to the first line
     
            Controller1.Screen.print(Robot_state.pos_x/10);
            Controller1.Screen.newLine();
            Controller1.Screen.print(Robot_state.pos_y/10);
            Controller1.Screen.newLine();
            Controller1.Screen.print(Robot_state.theta);

        wait(0.1, seconds);
      }
    }
};

Robot robot;

class auto_control_funcs {
    
    private:

    float normalize_angle_deg(float angle) {
    angle = fmodf(angle, 360.0f);
    if(angle > 180.0f) angle -= 360.0f;
    if(angle < -180.0f) angle += 360.0f;
    return angle;
    }

    float deg_to_rad(float degrees) {
        return degrees * (M_PI / 180.0f);
    }

    float rad_to_deg(float radians) {
        return radians * (180.0f / M_PI);
    }

    public:

    control inverseKinematics(state startState, state endState) {
        control output;
    
        // Tunable gains and limits:
        const float maxVelocity = 150;   // Maximum translational command (units match your system)
        const float maxOmega = 100;       // Maximum rotational command (degrees or rpm, as appropriate)
        const float kP_position = 1;   // Proportional gain for translation
        const float kP_rotation = 1.2;   // Proportional gain for rotation

        // Compute field coordinate error (assumes field X = right, field Y = up)
        float deltaX = endState.pos_x - startState.pos_x; // Error to the right
        float deltaY = endState.pos_y - startState.pos_y; // Error upward

        // Compute angular error (desired heading minus current heading)
        float deltaTheta = endState.theta - startState.theta;
        // Normalize to the range [-180, 180]
        normalize_angle_deg(deltaTheta);
        
        // Compute desired field velocities using proportional control:
        // We want a forward command proportional to field Y error and strafe to field X error.
        float v_field_Y = kP_position * deltaY;  // “Forward” in field space (up)
        float v_field_X = kP_position * deltaX;  // “Right” in field space

        // Convert from field coordinates to robot coordinates.
        // Our assumption: when robot heading = 0, its forward (velocity_x) aligns with field +Y and its strafe (velocity_y) aligns with field +X.
        // To convert, we rotate the vector by the robot's heading angle (in radians).
        float currentThetaRad = deg_to_rad(startState.theta);  // convert degrees to radians

        // Using the rotation matrix for a rotation by +theta:
        // [ v_forward ]   [ cos(theta)    sin(theta) ] [ v_field_Y ]
        // [ v_strafe  ] = [ -sin(theta)   cos(theta) ] [ v_field_X ]
        float v_forward = cos(currentThetaRad) * v_field_Y - sin(currentThetaRad) * v_field_X;
        float v_strafe  = sin(currentThetaRad) * v_field_Y + cos(currentThetaRad) * v_field_X;

        output.velocity_x = v_forward;  // robot forward command
        output.velocity_y = -v_strafe;   // robot strafe command

        // Clamp translational velocity if the magnitude is above maxVelocity:
        float norm = sqrt(v_forward * v_forward + v_strafe * v_strafe);
        if (norm > maxVelocity) {
            output.velocity_x = (v_forward / norm) * maxVelocity;
            output.velocity_y = -(v_strafe / norm) * maxVelocity;
        }

        // Compute rotational (angular) velocity command:
        output.omega = -kP_rotation * deltaTheta;
        if (output.omega > maxOmega) output.omega = maxOmega;
        if (output.omega < -maxOmega) output.omega = -maxOmega;
        
        return output;
    }

    bool robotIsReadyToMoveOnToNextState(state currentState, state targetState) {
        // Function that checks if the robot is ready to move on to the next
        //state
        float pos_threshold = 45; 
        float ang_threshold = 5; 

        float ErrorX = targetState.pos_x - currentState.pos_x;
        float ErrorY = targetState.pos_y - currentState.pos_y;
        float distanceToTarget = sqrt((ErrorX * ErrorX) + (ErrorY * ErrorY));

         //angle stuff Normalize the angle difference 

        float angleDifference = targetState.theta - currentState.theta;

        normalize_angle_deg(angleDifference);

        if (distanceToTarget < pos_threshold && abs(angleDifference) < ang_threshold) {  
          return true;
        }
        else {
          return false;
        }
    }

    void follow_tragectory (std::list<state> trajectory) {

        for (state targetState : trajectory) {

            while (!robotIsReadyToMoveOnToNextState(Robot_state,targetState)) {

                control movement = inverseKinematics(Robot_state, targetState);

                robot.drive_with_velocity(movement);

                wait(.01, seconds);
            }

            // Update additional actuators (intake, piston) if needed:

            robot.set_piston_to(targetState.piston_clamped);
            robot.run_intake(targetState.intake_moving); 

            Brain.Screen.print("nextstate");
            Brain.Screen.newLine();

        }
        
        TR_motor.SetSetpoint(0);
        TL_motor.SetSetpoint(0);
        BR_motor.SetSetpoint(0);
        BL_motor.SetSetpoint(0);

    }
};

auto_control_funcs auto_func;

void drive_robot() {

    while(true) {

        control driver_direction;
        driver_direction.velocity_x = (Controller1.Axis4.position(percent)*20);
        driver_direction.velocity_y = (Controller1.Axis3.position(percent)*20);
        driver_direction.omega = (Controller1.Axis1.position(percent)*20);

        robot.drive_with_velocity(driver_direction);

        if (Controller1.ButtonR1.pressing() == true) {
            robot.close_piston();
        }else if(Controller1.ButtonL1.pressing()) {
            robot.open_piston();
        }

        if(Controller1.ButtonL2.pressing() == true) {
            robot.run_intake(-1);
        } else if(Controller1.ButtonR2.pressing() == true) {
            robot.run_intake(1);
        } else {
            robot.run_intake(0);
        }

        if (debug_mode == true) {

            if (Controller1.ButtonX.pressing() == true) {

                Brain.Screen.print("X%.1f Y%.1f T%.1f",Robot_state.pos_x, Robot_state.pos_y, Robot_state.theta);
                Brain.Screen.newLine();
                wait(1, seconds); 

            }
        }
    }
}

void auto_loop() {

   while(GPS.isCalibrating() == true){wait(0.1, seconds);}

   auto_func.follow_tragectory(Automonus_tragectory);

}

int main()
{
   
    // these two thread are for programs that run cuntiniously

    vex::thread spinnythread = vex::thread(Robot::spin);

    vex::thread statethready = vex::thread(Robot::state_updater);

    vex::thread screen_ui_updater = vex::thread(robot.robot_heat);
   
    //set the gains of the PID

    TR_motor.SetGains(0.07,0.0000000001,0.2);
    TL_motor.SetGains(0.07,0.0000000001,0.2);
    BR_motor.SetGains(0.07,0.0000000001,0.2);
    BL_motor.SetGains(0.07,0.0000000001,0.2);


    TR_motor.SetOutputLimits(-12, 12); // Voltage limits (-12V to 12V)
    TL_motor.SetOutputLimits(-12, 12);
    BR_motor.SetOutputLimits(-12, 12);
    BL_motor.SetOutputLimits(-12, 12);

    // this is where the robot starts anything that should be initized in main should happen above this comment 

    competition Competition;

    Competition.drivercontrol(drive_robot);

    Competition.autonomous(auto_loop);

}

