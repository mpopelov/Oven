/**
 * @file TProgram.cpp
 * @author MikeP (mpopelov@gmail.com)
 * @brief TProgram class implementation
 * @version 0.1
 * @date 2022-04-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "TProgram.h"





/**
 * @brief Initializes program step parameters
 * 
 * @param T_s step start temperature, C
 * @param T_e step end temperature, C
 * @param d step duration in microseconds
 */
void TProgramStep::Init(double T_s, double T_e, unsigned long d)
{
    T_start = T_s;
    T_end = T_e;
    duration = d;
    slope = (T_e - T_s)/d;
}

/**
 * @brief Calculates new SetPoint (desired temperature for current step) at a given time from step start
 * 
 * @param t time from step start to calculate SetPoint for, expressed in microseconds
 * @return calculated temperature
 */
double TProgramStep::SetPoint(unsigned long t)
{
    // calculate set point for provided step based on linear function defined by this step
    // SetPoint(t) = A*t + B
    // slope A = (T_end - T_start)/duration
    // constant B = T_start
    // So the final result would be SetPoint(t) = (T_end - T_start)/duration * t + T_start
    return slope * t + T_start;
};