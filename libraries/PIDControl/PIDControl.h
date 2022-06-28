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

/**
 * @brief Abstract base class for different types of PID control implementations
 * @details Basic usage assumes exact implementation should be instantiated with no parameters.
 *          An instance should be Reset before first use (given PID coeffitients and expected time interval).
 *          Instance is then Evaluated repeatedly every given interval (given set point value, measured process value and latest controlling signal value)
 * 
 */
class PIDControl {
    public:
        // default ctor
        PIDControl() : poll{0} {}
        
        /**
         * @brief Reset specific PID control implementation
         * 
         * @param Kp Proportional coeffitient
         * @param Ki Integral coeffitient
         * @param Kd Derivative coeffitient
         * @param dt Expected discrete time step (in ms)
         */
        virtual void Reset(double Kp, double Ki, double Kd, unsigned long dt) = 0;

        /**
         * @brief Evaluate controlling signal value over subsequent step
         * 
         * @param SP setpoint value (target temperature at this step)
         * @param PV process value (measured temperature in the oven)
         * @param U current actuator value
         * @return calculated controlling signal value to be applied 
         */
        virtual double Evaluate(double SP, double PV, double U) = 0;

    public:
        unsigned long poll;    // polling interval - must be exposed by all implementations
};


/**
 * @brief A very basic PID control implementation
 * 
 */
class PIDControlBasic : public PIDControl
{
    public:
        PIDControlBasic() : PIDControl{},
        _Kp{0.0}, _Ki{0.0}, _Kd{0.0},
        _e{0.0}, _i{0.0} {}

        void Reset(double Kp, double Ki, double Kd, unsigned long dt);
        double Evaluate(double SP, double PV, double U);
    
    private:
        double _Kp;
        double _Ki;
        double _Kd;
        double _e;
        double _i;
}

/**
 * @brief A simple PID control implementation with integral loop
 * 
 */
class PIDControlSimple : public PIDControl
{
    public:
        PIDControlSimple() : PIDControl{},
        _A0{0.0}, _A1{0.0}, _A2{0.0},
        _e0{0.0}, _e1{0.0}, _e2{0.0} {}

        void Reset(double Kp, double Ki, double Kd, unsigned long dt);
        double Evaluate(double SP, double PV, double U);
    
    private:
        double _A0;
        double _A1;
        double _A2;
        double _e0;
        double _e1;
        double _e2;
}

/**
 * @brief PID control implementation with infinite impulse response
 * 
 */
class PIDControlIIR : public PIDControl
{
    public:
        PIDControlIIR() : PIDControl{},
        _A0{0.0}, _A1{0.0},
        _e2{0.0}, _e1{0.0}, _e0{0.0},
        _A0d{0.0}, _A1d{0.0}, _A2d{0.0},
        _alpha1{0.0}, _alpha2{0.0},
        _d0{0.0}, _d1{0.0}, _fd0{0.0}, _fd1{0.0} { }

        void Reset(double Kp, double Ki, double Kd, unsigned long dt);
        double Evaluate(double SP, double PV, double U);

    private:
        // PID coeffitients
        double _A0;
        double _A1;
        // array for accumulating error
        double _e2;
        double _e1;
        double _e0;
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