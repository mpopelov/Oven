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



void PIDControlBasic::Reset(double Kp, double Ki, double Kd, unsigned long dt)
{
    _i = _e = 0.0;
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
    poll = dt;
}

double PIDControlBasic::Evaluate(double SP, double PV, double U)
{
    // proportional part
    double e = SP - PV;
    // derivative part
    double d = (e - _e) / poll;
    // integral part
    _i += (e * poll);
    // save current error for next step
    _e = e;
    // return calculated value
    return _Kp*e + _Ki*_i + _Kd*d;
}

void PIDControlSimple::Reset(double Kp, double Ki, double Kd, unsigned long dt)
{
    poll = dt;
    _e0 = _e1 = _e2 = 0.0;
    
    _A0 = Kp + Ki*dt + Kd/dt;
    _A1 = -Kp - 2*Kd/dt;
    _A2 = Kd/dt;
}

double PIDControlSimple::Evaluate(double SP, double PV, double U)
{
    _e2 = _e1;
    _e1 = _e0;
    _e0 = SP - PV;
    return U + _A0*_e0 + _A1*_e1 + _A2*_e2;
}

void PIDControlIIR::Reset(double Kp, double Ki, double Kd, unsigned long dt)
{
    // reset coeffitients to initial values
    _d0 = _d1 = _fd0 = _fd1 = 0.0;

    // reset accumulated error to 0.0
    _e0 = _e1 = _e2 = 0.0;

    // save polliing interval
    poll = dt;

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

double PIDControlIIR::Evaluate(double SP, double PV, double U)
{
    _e2 = _e1;
    _e1 = _e0;
    _e0 = SP - PV;

    // PID: calculate filtered D portion
    _d1 = _d0;
    _d0 = _A0d*_e0 + _A1d*_e1 + _A2d*_e2;
    _fd1 = _fd0;
    _fd0 = _alpha1 * (_d0+_d1) - _alpha2 * _fd1;

    // PID: sum up P, I and filtered D portions
    return U + _A0*_e0 + _A1*_e1 + _fd0;
}