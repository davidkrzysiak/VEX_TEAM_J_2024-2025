#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"


#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)

#define PIDController_h
/**
  * @brief Custom made PID controller class, instead of using the
library we will be using this because this will save us some precious
flash storage (we getting close to limit)
  *
  */
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
         currentComputeTime = micros();
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
         m_previousComputeTime = micros();
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



