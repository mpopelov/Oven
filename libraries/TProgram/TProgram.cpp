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
 * @param d step duration in milliseconds
 * @param dt dueTime relative to program start
 */
void TProgramStep::Init(double T_s, double T_e, unsigned long d, unsigned long dt)
{
    T_start = T_s;
    T_end = T_e;
    duration = d;
    dueTime = dt;
    slope = (T_e - T_s)/d;
}

/**
 * @brief Calculates new SetPoint (desired temperature for current step) at a given time from step start
 * 
 * @param t time from step start to calculate SetPoint for, expressed in milliseconds
 * @return calculated temperature
 */
double TProgramStep::CalculateSetPoint(unsigned long t)
{
    // calculate set point for provided step based on linear function defined by this step
    // SetPoint(t) = A*t + B
    // slope A = (T_end - T_start)/duration
    // constant B = T_start
    // So the final result would be SetPoint(t) = (T_end - T_start)/duration * t + T_start
    return slope * t + T_start;
}




/**
 * @brief Initialize next program step with relevant values (updates the last step available)
 * 
 * @param T_s desired temperature at the start of the step
 * @param T_e desired temperature at the end of the step
 * @param d duration of the step expressed in milliseconds
 * @return true on success or false on input out of bounds
 */
bool TProgram::AddStep(double T_s, double T_e, unsigned long d)
{
    if(_nSteps < TPGM_STEPS_MAX){
        // still have room for another step
        _totalDuration += d;                                // adjust total program duration by added step duration
        _steps[_nSteps].Init(T_s, T_e, d, _totalDuration);  // init current step and set it's dueTime
        _nSteps++;                                          // move over to next step
    }else{
        return false;
    }
}

/**
 * @brief Advance step index to start program from
 * 
 */
void TProgram::StepForward(){
    if(_idx >= _nSteps - 1) return; // prevent moving index beyond total steps
    _idx++; // advance index
    _timeElapsed = _steps[_idx].GetDueTime() - _steps[_idx].GetDuration(); // adjust program elapsed time to calculate setpoints correctly
}

/**
 * @brief Decrease index of the step to start program from
 * 
 */
void TProgram::StepBack(){
    if(_idx == 0) return; // prevent moving step index below 0
    _idx--; // reduce index
    _timeElapsed = _steps[_idx].GetDueTime() - _steps[_idx].GetDuration(); // adjust program elapsed time to calculate setpoints correctly
}

/**
 * @brief Initializes program to start running and return the initial SetPoint value
 * 
 * @return initial SetPoint / NAN in case of error
 */
double TProgram::Begin()
{
    // return NAN if there are no program steps defined
    if(_nSteps == 0) return NAN;

    // do not touch _idx and _timeElapsed values as they might have been adjusted
    // by user requesting different step to start program from
    _timeLast = millis();   // save timer value at start of the program
    _timeElapsedStep = 0;   // reset step elapsed time to the beginning of the step
    return _steps[_idx].CalculateSetPoint( _timeElapsedStep );
}

/**
 * @brief Returns current SetPoint value of the program
 * 
 * @return SetPoint value or NAN in case program is over or there was an error
 */
double TProgram::CalculateSetPoint()
{
    unsigned long timeNow = millis();               // get current millis()

    _timeElapsed += (timeNow - _timeLast);          // adjust elapsed time to correctly identify the step to execute
    if(_timeElapsed > _totalDuration) return NAN;   // total program duration exceeded - just return NAN
    _timeLast = timeNow;                            // save timestamp for calculating delta next time
    
    // figure out which step is to be used for calculating SetPoint this time
    while(_timeElapsed > _steps[_idx].GetDueTime()){
        // elapsed time is beyond current step due time - advance step index
        _idx++;
        // just another safety check - make sure _idx is within boundaries. should not happen but still
        if(_idx >= _nSteps) return NAN;
    }

    // _idx should now point to the correct step
    // calculate time delta to get correct SetPoint and return it to the caller
    // step SetPoint function expects time delta from start of the step
    // which is equal: step duration - (dueTime - timeElapsed)
    _timeElapsedStep = _steps[_idx].GetDuration() - _steps[_idx].GetDueTime() + _timeElapsed;
    return _steps[_idx].CalculateSetPoint( _timeElapsedStep );
}

/**
 * @brief Reset program to its initial state as before running
 * 
 */
void TProgram::Reset()
{
    //reset program to its initial state
    _timeElapsed = 0;
    _timeElapsedStep = 0;
    _timeLast = 0;
    _idx = 0;
}

/**
 * @brief Clear the program making it invalid
 * 
 */
void TProgram::Clear(){
    _name[0] = 0;
    _timeElapsed = 0;
    _timeElapsedStep = 0;
    _timeLast = 0;
    _totalDuration = 0;
    _idx = 0;
    _nSteps = 0;
}