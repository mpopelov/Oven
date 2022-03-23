/*
Base control class to inherit from
*/

#ifndef DTControl_h
#define DTControl_h

#include "TFT_eSPI.h"


//
// Definitions for different flags that can be used to modify the behavior of a control
// 32 bit _flags member holds basic flags (that can be masked out) and allows inheriting classes to use available bits on their own
//
#define DTCONTROL_FLAGS_MASK        0x000000FF // lower 4 bits are for controlling purposes
#define DTCONTROL_FLAGS_VISIBLE     0x00000001 // visibility flag defines whether the control is visible and can draw itself using context
#define DTCONTROL_FLAGS_INVALIDATED 0x00000002 // invalidation flag to be raised whenever the control has its state changed and needs to be redrawn
#define DTCONTROL_FLAGS_NBITS       8



//
// base class for all different types of controls (windows, labels, buttons, etc.)
//
class DTControl
{
    public:
     // CTor
     DTControl(TFT_eSPI* gfx, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint32_t flags)
     {
         _gfx = gfx;
         _x = x;
         _y = y;
         _w = w;
         _h = h;
         _flags = flags;
     }

     // DTor - virtual for correct deallocation - to be implemented in inheriting classes
     virtual ~DTControl(){}
     
     // Trigger invalidation so that control is redrawn next time Render is called
     virtual void Invalidate();

     // Change visibility
     virtual void Visible(bool v);
    
     // Render - take necessary actions to draw the control on screen
     virtual void Render();

     // Handle touch screen event
     // Return true if event is handled and should not be passed further.
     // Return false if event was not completely handled by this control and should be passed to other controls for handling.
     virtual bool HandleEvent(uint16_t x, uint16_t y, bool pressed);
    
    protected:
     TFT_eSPI* _gfx; // graphical context to be used to redraw element
     uint16_t _x; // top lef corner x
     uint16_t _y; // top left corner y
     uint16_t _w; // control width
     uint16_t _h; // control height
     uint32_t _flags; // Different flags to handle state and behavior of the control
};


#endif