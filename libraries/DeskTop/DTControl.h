/**
 * @file DTControl.h
 * @author MikeP (mpopelov@gmail.com)
 * @brief Base control class all other controls inherit from
 * @details Contains description of the basic GUI control class capable of positioning itself
 *          on the screen. It also contains the template class for defining callbacks that can
 *          be used in derived control classes to change their state in responce to child control
 *          actions.
 * @version 0.1
 * @date 2022-03-24
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef DTControl_h
#define DTControl_h

#include "TFT_eSPI.h"

/**
 * @brief Flag definitions
 * @details Definitions for different flags that can be used to modify the behavior of a control.
 *          32 bits _flags member holds basic flags (that can be masked out) and allows derived classes
 *          to use available bits for their own purposes
 */
#define DTCONTROL_FLAGS_MASK               0x000000FF // lower 16 bits are reserved for use by DTControl class related flags
#define DTCONTROL_FLAGS_VISIBLE            0x00000001 // visibility flag defines whether the control is visible and should draw itself using context
#define DTCONTROL_FLAGS_INVALIDATED        0x00000002 // invalidation flag to be raised whenever the control has its state changed and needs to be redrawn
#define DTCONTROL_FLAGS_NBITS       16

/**
 * @brief Base class for all different types of controls (windows, labels, buttons, etc.)
 * 
 */
class DTControl
{
    public:

     /**
      * @brief Construct a new DTControl object
      * 
      * @param gfx pointer to TFT_eSPI class instance that is to be used for rendering
      * @param x control starting point X coordinate
      * @param y control starting point Y coordinate
      * @param w control width
      * @param h control height
      * @param flags flags that impact control behavior
      */
     DTControl(TFT_eSPI* gfx, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint32_t flags) :
     _gfx(gfx), _x(x), _y(y), _w(w), _h(h), _flags(flags) {}

     /**
      * @brief Destroy the DTControl object - virtual to be overriden in derived classes
      * 
      */
     virtual ~DTControl(){}
     
     /**
      * @brief Trigger invalidation so that control is redrawn next time Render is called
      * 
      */
     virtual void Invalidate();

     /**
      * @brief Change visibility of control and all it's children
      * 
      * @param v true if control is visible, false otherwise
      */
     virtual void Visible(bool v);
    
     /**
      * @brief Render control to the display
      * 
      * @param parentCleared true if parent has cleared the area, false otherwise
      */
     virtual void Render(bool parentCleared);

     /**
      * @brief Handle touchscreen event
      * 
      * @param x X coordinate of touch point
      * @param y Y coordinate of touch point
      * @param pressed true if pressed, false if released event occured
      * @return true if event is handled and should not be passed further
      * @return false if event was not completely handled by this control and should be passed to other controls for handling
      */
     virtual bool HandleEvent(uint16_t x, uint16_t y, bool pressed);

    protected:
     TFT_eSPI* _gfx;  // graphical context to be used to redraw element
     uint16_t  _x;     // top lef corner x
     uint16_t  _y;     // top left corner y
     uint16_t  _w;     // control width
     uint16_t  _h;     // control height
     uint32_t  _flags; // Different flags to handle state and behavior of the control
};


/**
 * @brief A delegate class to pass call-back member functions to chid controls
 * @details At the moment implemented for functions of type: "void MemberFunc()"
 *          meaning call-back functions do not accept arguments and do not return any values.
 * 
 */
class DTDelegate
{
    public:
    DTDelegate() : _obj_ptr_(0), _stub_ptr_(0) {} // default CTor
    
    template <class T, void(T::*TMethod)()>
    static DTDelegate create(T* object_ptr)
    {
        DTDelegate d;
        d._obj_ptr_ = object_ptr;
        d._stub_ptr_ = &method_stub<T, TMethod>;
        return d;
    }

    void operator()() const
    {
        return (*_stub_ptr_)(_obj_ptr_);
    }

    private:

    typedef void (*STUBTYPE)(void* _obj_ptr_);

    void* _obj_ptr_;
    STUBTYPE _stub_ptr_;

    template <class T, void (T::*TMethod)()>
    static void method_stub(void* object_ptr)
    {
        T* p = static_cast<T*>(object_ptr);
        return (p->*TMethod)();
    }
};

#endif