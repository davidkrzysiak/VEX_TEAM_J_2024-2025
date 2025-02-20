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

// this is the number that will be updated constantly

static state Robot_state;

//this below is the list that controls auto its a sequence that tells the code where the robot should be 
//currently its based on position so it will stop at every point(kinda theres nuiance)
std::list<state> Automonus_tragectory = {
    state{1200,1200,0,false,1},
    state{1300,1100,10,true,0},
    state{1400,1000,20,false,-1}
};

//robot class!!!! yay this define the things the robot can do and its parameters

class Robot
{
private:
   
    float diameter_of_wheels = 2.75;

    int angle_of_wheels = 45;

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

    void polarToCartesian(double r, double theta, double &x, double &y) {
        double rad = (theta + 45.0) * M_PI / 180.0;  // Convert to radians and apply offset
        x = r * cos(rad);
        y = r * sin(rad);
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

    static float sigmoid(float x) {
    return 1.0f / (1.0f + expf(-x));
    }   

    static float expf_approx(float x) {
    // Fast exp approximation for embedded systems
    x = 1.0f + x / 1024.0f;
    x *= x; x *= x; x *= x; x *= x;
    x *= x; x *= x; x *= x; x *= x;
    return x;
    }
public:

    static void state_updater() {

    Inertial5.calibrate();
    GPS.calibrate();
    while(GPS.isCalibrating()) { wait(0.1, seconds); }

    // Configuration constants
    const float WHEEL_DIAMETER = 2.75; // inches
    const float ENCODER_TPI = (360.0 / (WHEEL_DIAMETER * M_PI)); // Ticks per inch
    const float GPS_TRUST_THRESHOLD = 0.3f; // Minimum GPS quality to trust
    const float ODOMETRY_DECAY = 0.95f; // Odometry error decay factor

    // State variables
    float previous_degree_x = X_encoder.position(degrees);
    float previous_degree_y = Y_encoder.position(degrees);
    float odometry_x = GPS.xPosition();
    float odometry_y = GPS.yPosition();
    float heading_offset = GPS.heading() - Inertial5.heading();

    while(true) {
        // Calculate time delta for integration
        static uint32_t last_time = vex::timer::system();
        uint32_t current_time = vex::timer::system();
        float dt = (current_time - last_time) / 1000.0f;
        last_time = current_time;

        // Get sensor data with error checking
        float gps_x = GPS.xPosition();
        float gps_y = GPS.yPosition();
        float gps_heading = GPS.heading();
        float gps_quality = GPS.quality() / 100.0f;
        float inertial_heading = Inertial5.heading();

        // Calculate encoder deltas in inches
        float dx_enc = (X_encoder.position(degrees) - previous_degree_x) / ENCODER_TPI;
        float dy_enc = (Y_encoder.position(degrees) - previous_degree_y) / ENCODER_TPI;
        previous_degree_x = X_encoder.position(degrees);
        previous_degree_y = Y_encoder.position(degrees);

        // Fuse heading sources
        float fused_heading = inertial_heading + heading_offset;
        if(gps_quality > GPS_TRUST_THRESHOLD) {
            fused_heading = gps_heading;
            heading_offset = fused_heading - inertial_heading;
        }

        // Convert encoder deltas to global coordinates
        float theta_rad = deg_to_rad(fused_heading);
        float cos_theta = cosf(theta_rad);
        float sin_theta = sinf(theta_rad);
        
        odometry_x += dx_enc * cos_theta - dy_enc * sin_theta;
        odometry_y += dx_enc * sin_theta + dy_enc * cos_theta;

        // Fuse GPS and odometry using adaptive weighting
        if(gps_quality > GPS_TRUST_THRESHOLD) {
            // Sigmoid weighting for smooth transition
            float gps_weight = 1.0f / (1.0f + expf(-10.0f * (gps_quality - 0.5f)));
            
            Robot_state.pos_x = gps_weight * gps_x + (1 - gps_weight) * odometry_x;
            Robot_state.pos_y = gps_weight * gps_y + (1 - gps_weight) * odometry_y;
            
            // Reset odometry accumulation error when GPS is good
            odometry_x = Robot_state.pos_x;
            odometry_y = Robot_state.pos_y;
        } else {
            // Use pure odometry when GPS is unreliable
            Robot_state.pos_x = odometry_x;
            Robot_state.pos_y = odometry_y;
        }

        // Update heading with complementary filter
        Robot_state.theta = normalize_angle_deg(
            0.98f * fused_heading + 
            0.02f * heading_to_angle(Inertial5.heading())
        );

        // Apply odometry error decay
        odometry_x += (Robot_state.pos_x - odometry_x) * (1 - ODOMETRY_DECAY);
        odometry_y += (Robot_state.pos_y - odometry_y) * (1 - ODOMETRY_DECAY);

        wait(10, msec); // Consistent update interval
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
        intake_motor.spin(forward, direction*10, volt);
        intake_arm_half_motor.spin(forward,direction*10, volt);

    }

    void toggle_piston(){

        clamp_piston1.set(!piston_pos);

        piston_pos = !piston_pos;

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
     
            Controller1.Screen.print(Robot_state.pos_x);
            Controller1.Screen.newLine();
            Controller1.Screen.print(Robot_state.pos_y);
            Controller1.Screen.newLine();
            Controller1.Screen.print(Robot_state.theta);

        wait(0.1, seconds);
      }
    }
};

//created robot object 

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
        control output{0, 0, 0};
        constexpr float MAX_LINEAR = 200.0f;   // mm/s
        constexpr float MAX_ANGULAR = 90.0f;    // deg/s
        constexpr float POSITION_TOL = 10.0f;   // mm
        constexpr float ANGLE_TOL = 2.0f;       // degrees
        constexpr float KP_LINEAR = 0.8f;
        constexpr float KP_ANGULAR = 2.5f;
        constexpr float LOOKAHEAD_DIST = 150.0f; // mm

        // Calculate position error in world coordinates
        float errorX = endState.pos_x - startState.pos_x;
        float errorY = endState.pos_y - startState.pos_y;
        float targetDistance = hypotf(errorX, errorY);
        
        // Calculate angle errors
        float targetHeading = atan2f(errorY, errorX) * (180/M_PI);
        float headingError = normalize_angle_deg(targetHeading - startState.theta);
        float finalHeadingError = normalize_angle_deg(endState.theta - startState.theta);

        // Calculate adaptive velocity scaling
        float linearSpeed = KP_LINEAR * targetDistance;
        linearSpeed = fminf(fmaxf(linearSpeed, 50.0f), MAX_LINEAR);
        
        // Calculate lookahead point for smoother motion
        float lookaheadX = endState.pos_x + LOOKAHEAD_DIST * cosf(deg_to_rad(endState.theta));
        float lookaheadY = endState.pos_y + LOOKAHEAD_DIST * sinf(deg_to_rad(endState.theta));
        
        // Switch to final orientation control when close
        if(targetDistance < POSITION_TOL * 3) {
            headingError = finalHeadingError;
            linearSpeed *= 0.3f; // Slow down for final approach
        }

        // Convert world-frame vectors to robot-frame using rotation matrix
        float cosTheta = cosf(deg_to_rad(startState.theta));
        float sinTheta = sinf(deg_to_rad(startState.theta));
        
        // Transform lookahead point to robot coordinates
        float robotX = (lookaheadX - startState.pos_x) * cosTheta + 
                    (lookaheadY - startState.pos_y) * sinTheta;
        float robotY = -(lookaheadX - startState.pos_x) * sinTheta + 
                        (lookaheadY - startState.pos_y) * cosTheta;

        // Calculate curvature for smooth path following
        float curvature = 2.0f * robotY / (robotX * robotX + robotY * robotY);
        float angularSpeed = KP_ANGULAR * headingError + linearSpeed * curvature;

        // Velocity limiting
        float speedRatio = fminf(1.0f, MAX_LINEAR / linearSpeed);
        linearSpeed *= speedRatio;
        angularSpeed = fminf(fmaxf(angularSpeed, -MAX_ANGULAR), MAX_ANGULAR);

        // Convert to robot-centric velocity commands
        output.velocity_x = linearSpeed * cosf(deg_to_rad(headingError));
        output.velocity_y = linearSpeed * sinf(deg_to_rad(headingError));
        output.omega = angularSpeed;

        // Final position lock conditions
        if(fabs(finalHeadingError) < ANGLE_TOL && targetDistance < POSITION_TOL) {
            output.velocity_x = 0;
            output.velocity_y = 0;
            output.omega = 0;
        }

        return output;
    }

    bool robotIsReadyToMoveOnToNextState(state currentState, state targetState) {
        // Function that checks if the robot is ready to move on to the next
        //state
        float pos_threshold = 10; 
        float ang_threshold = 10; 

        float ErrorX = targetState.pos_x - currentState.pos_x;
        float ErrorY = targetState.pos_y - currentState.pos_y;
        float distanceToTarget = sqrt((ErrorX * ErrorX) + (ErrorY * ErrorY));

         //angle stuff Normalize the angle difference 

        float angleDifference = targetState.theta - currentState.theta;
        while (angleDifference > 180) angleDifference -= 360;
        while (angleDifference < -180) angleDifference += 360;

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

        if(Controller1.ButtonX.pressing() == true) {
            lady_brown.spin(forward, 3, volt);
        } else if(Controller1.ButtonB.pressing() == true) {
            lady_brown.spin(reverse, 3, volt);
        } else {
            lady_brown.stop(hold); 
        }

        // x and b for lady brown 
    }
}

void auto_loop() {

   auto_func.follow_tragectory(Automonus_tragectory);

   control forward;
   forward.velocity_y = -100;
   forward.velocity_x = 0;
   forward.omega = 0;

   robot.drive_with_velocity(forward);

   forward.velocity_y = -2000;

   wait(0.5,seconds);

   robot.drive_with_velocity(forward);

   wait(1, seconds);

   forward.velocity_y= 0 ; 

   robot.drive_with_velocity(forward);

   robot.open_piston();

   wait(0.1, seconds);

   robot.run_intake(-1);

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

