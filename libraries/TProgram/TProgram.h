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
     void Init(double T_s, double T_e, unsigned long d, unsigned long dt);
     // SetPoint calculation
     double CalculateSetPoint(unsigned long t);

     // helper functions to access private data members
     unsigned long GetDueTime() { return dueTime; }     // get due time for program step to be over relative to program start
     unsigned long GetDuration() { return duration; }   // return duration
     double GetTStart() { return T_start; }
     double GetTEnd() { return T_end; }

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
     : _name(name), _nSteps(numSteps), _timeElapsed(0), _timeElapsedStep(0), _timeLast(0), _totalDuration(0), _idx(0)
     {
         if(_nSteps > 0)
            _steps = new TProgramStep[_nSteps];
        else
            _steps = nullptr;
     }

     // copy constructor to create disconnected copy for active program to run
     TProgram(const TProgram& p)
     : _name(p._name), _nSteps(p._nSteps), _timeElapsed(p._timeElapsed), _timeElapsedStep(p._timeElapsedStep), _timeLast(p._timeLast), _totalDuration(p._totalDuration), _idx(p._idx)
     {
        // copy steps
        if(_nSteps > 0){
            _steps = new TProgramStep[_nSteps];
            for(int i = 0; i < _nSteps; i++) _steps[i] = p._steps[i];
        } else _steps = nullptr;
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

     // property getters
     const String&       GetName() { return _name; }; // return program name
     int                 GetStepsTotal() { return _nSteps; }    // return total number of steps in control program
     int                 GetStepsCurrent() { return _idx + 1; } // return current step of the program in human acceptable form (i.e. starting with 1)
     unsigned long       GetDurationTotal() { return _totalDuration; }  // return precalculated total duration of the program
     unsigned long       GetDurationElapsed() { return _timeElapsed; }  // return time elapsed since program start
     unsigned long       GetDurationElapsedStep() { return _timeElapsedStep; }  // return time elapsed within current step
     TProgramStep* GetStep(int i) { if(i>=0 && i<_nSteps) return &(_steps[i]); else return nullptr; } //return step #i

    private:
     String         _name;          // program human readable name
     unsigned long  _timeElapsed;   // elapsed time from program start in milliseconds
     unsigned long  _timeElapsedStep; // time elapsed within the step
     unsigned long  _timeLast;      // timestamp of last SetPoint calculation to measure time offsets
     unsigned long  _totalDuration; // precalculated value of total program duration

     int            _idx;           // current step index

     const int      _nSteps;        // total number of steps
     TProgramStep*  _steps;         // arrays of steps

     // prevent copying the class
     TProgram& operator=(const TProgram& p) = delete;
};

/**
 * @brief Some useful macros for extracting hours, minutes and seconds part from milliseconds value
 * @details detailed formulas:
 *          uint32_t sec = millis() / 1000ul;       // millis into seconds
 *          int timeHours = (sec / 3600ul);         // hours
 *          int timeMins = (sec % 3600ul) / 60ul;   // minutes
 *          int timeSecs = (sec % 3600ul) % 60ul;   // seconds
 */
#define TPGM_MS_HOURS(_val_)     ((_val_ / 1000ul) / 3600ul)
#define TPGM_MS_MINUTES(_val_)  (((_val_ / 1000ul) % 3600ul) / 60ul)
#define TPGM_MS_SECONDS(_val_)  (((_val_ / 1000ul) % 3600ul) % 60ul)

#define TPGM_HOURS_MS(_val_)    (_val_ * 3600000ul)
#define TPGM_MINUTES_MS(_val_)  (_val_ * 60000ul)
#define TPGM_SECONDS_MS(_val_)  (_val_ * 1000ul)

#endif
