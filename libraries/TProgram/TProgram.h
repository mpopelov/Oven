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

#include <Arduino.h>


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
    public:

     // default constructor
     TProgramStep() : T_start(NAN), T_end(NAN), slope(NAN), duration(0), dueTime(0) {}
     // initialization of the step
     void Init(double T_s, double T_e, unsigned long d, unsigned long o);
     // SetPoint calculation
     double CalculateSetPoint(unsigned long t);

     // helper functions to access private data members
     unsigned long GetDueTime() { return dueTime; } // get due time for program step to be over relative to program start

    private:

     double         T_start;    // desired temperature at step start, C
     double         T_end;      // desired temperature at step end, C
     double         slope;      // precalculated slope value
     unsigned long  duration;   // desired duration of step - in milliseconds
     unsigned long  dueTime;    // an offset from program start when this step is to be over
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
     TProgram(int numSteps, String name)
     : _name(name), _nSteps(numSteps), _timeElapsed(0), _timeLast(0), _totalDuration(0), _idx(0)
     {
         if(_nSteps > 0) _steps = new TProgramStep[_nSteps];
     }

     ~TProgram()
     {
         if(_nSteps > 0) delete []_steps;
     }

     // initialization routine for steps in the program
     bool AddStep(double T_s, double T_e, unsigned long d);

     // runtime routines
     double Begin();                // start executing the program
     double CalculateSetPoint();    // calculate current program SetPoint
     void   Reset();                // reset program

    private:
     String         _name;          // program human readable name
     unsigned long  _timeElapsed;   // elapsed time from program start in milliseconds
     unsigned long  _timeLast;      // timestamp of last SetPoint calculation to measure time offsets
     unsigned long  _totalDuration; // precalculated value of total program duration

     int            _idx;           // current step index

     const int      _nSteps;        // total number of steps
     TProgramStep*  _steps;         // arrays of steps

     // prevent copying the class
     TProgram(const TProgram& p) = delete;
     TProgram& operator=(const TProgram& p) = delete;
};


#endif
