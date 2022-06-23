/**
 * @file PIDControl.h
 * @author MikeP (mpopelov@gmail.com)
 * @brief A simple implementation of PID control class
 * @details PID control class to be used for calculating controlling signal value
 *          to be applied to oven's heating element relay: either discrete (On/Off) or PWM
 * @version 0.1
 * @date 2022-04-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef PIDControl_h
#define PIDControl_h


class PIDControl
{
    public:
     /**
      * @brief Construct a new PIDControl object
      * 
      * @param Kp Proportional coeffitient
      * @param Ki Integral coeffitient
      * @param Kd Derivative coeffitient
      * @param dt Expected discrete time step (in ms)
      */
     PIDControl(double Kp, double Ki, double Kd, unsigned long dt)
     : poll(dt), _d0(0.0), _d1(0.0), _fd0(0.0), _fd1(0.0)
     {
         // seed values for PID controller
         _error[0] = _error[1] = _error[2] = 0.0;
         // seed PID coeffitients
         _A0 = Kp + Ki / dt;
         _A1 = -Kp;
         // IIR filter related values
         _A0d = Kd / dt;
         _A1d = -2.0 * Kd / dt;
         _A2d = Kd / dt;

         double alpha = dt/(2 * Kd / (Kp * 5.0));
         _alpha1 = alpha / (alpha + 1);
         _alpha2 = (alpha - 1) / (alpha + 1);
     }

     /**
      * @brief Evaluate controlling signal value over subsequent step
      * 
      * @param SP setpoint value (target temperature at this step)
      * @param PV process value (measured temperature in the oven)
      * @param U current actuator value
      * @return calculated controlling signal value to be applied
      */
     double Evaluate(double SP, double PV, double U);

     // polling interval that this instance of PID control was initialized with
     unsigned long poll;

    private:
     // PID coeffitients
     double _A0;
     double _A1;
     // array for accumulating error
     double _error[3];
     // IIR filter related values
     double _A0d;
     double _A1d;
     double _A2d;
     double _alpha1;
     double _alpha2;
     double _d0;
     double _d1;
     double _fd0;
     double _fd1;
};


#endif