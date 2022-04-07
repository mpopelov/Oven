/**
 * @file PIDControl.cpp
 * @author MikeP (mpopelov@gmail.com)
 * @brief PIDControl implementation
 * @version 0.1
 * @date 2022-04-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "PIDControl.h"



/**
 * @brief Evaluate controlling signal value over subsequent step
 * 
 * @param SP setpoint value (target temperature at this step)
 * @param PV process value (measured temperature in the oven)
 * @param U current actuator value
 * @return calculated controlling signal value to be applied 
 */
double PIDControl::Evaluate(double SP, double PV, double U)
{
    _error[2] = _error[1];
    _error[1] = _error[0];
    _error[0] = SP - PV;

    // PID: calculate filtered D portion
    _d1 = _d0;
    _d0 = _A0d*_error[0] + _A1d*_error[1] + _A2d*_error[2];
    _fd1 = _fd0;
    _fd0 = _alpha1 * (_d0+_d1) - _alpha2 * _fd1;

    // PID: sum up P, I and filtered D portions
    return U + _A0*_error[0] + _A1*_error[1] + _fd0;
}