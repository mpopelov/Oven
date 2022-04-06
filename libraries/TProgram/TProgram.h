/**
 * @file TProgram.h
 * @author MikeP (mpopelov@gmail.com)
 * @brief A class to define oven controller program to execute
 * @version 0.1
 * @date 2022-04-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef TProgram_h
#define TProgram_h


/**
 * @brief Class describing one program step.
 * @details Each controller program step is described by:
 *          - desired temperature at start of program step
 *          - desired temperature to be reached by end of program step
 *          - desired step duration.
 *          The two values of the temperature at start and end of program step
 *          along with the desired step duration define the linear function
 *          that will be used to calculate current desired temperature (SetPoint)
 *          at each tick of the controller algorithm.
 *          Controller will then decide whether to turn heater on or off.
 * 
 */
class TProgramStep
{
    double          T_start;    // desired temperature at step start, C
    double          T_end;      // desired temperature at step end, C
    double          slope;      // precalculated slope value
    unsigned long   duration;   // desired duration of step - in microseconds

    void Init(double T_s, double T_e, unsigned long d);
    double SetPoint(unsigned long t);
};



/**
 * @brief Class describing controller program to execute
 * @details This class represents a program to be executed by oven controller.
 *          Each program consists of several steps that describe duration and desired temperatures at start and end of that step.
 * 
 */
class TProgram
{
    public:
     TProgram(int numSteps) : _nSteps(numSteps) {}

    private:
     int            _nSteps;    // number of steps
     TProgramStep   _steps[];   // arrays of steps
};


#endif
