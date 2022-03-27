/*
* A class representing a simple window that can hold and update/redraw child components as necessary.
* a class can hold and track DTButton, DTLabel, etc.
*/

#ifndef DTWindow_h
#define DTWindow_h

#include "DTControl.h"

class DTCList
{
    public:
     DTCList *next, *prev;
     DTControl* control;

     DTCList(DTControl* c, DTCList* n, DTCList* p)
     {
         control = c;
         next = n;
         if( n!= NULL ) n->prev = this;
         prev = p;
         if( p!= NULL ) p->next = this;
     }
};


class DTWindow : public DTControl
{
    public:
     // CTor
     DTWindow(TFT_eSPI* gfx,
               uint16_t x,
               uint16_t y,
               uint16_t w,
               uint16_t h,
               uint32_t flags,
               uint16_t bkgc) :
     DTControl(gfx, x, y, w, h, flags), _bkg_color(bkgc), _controls(NULL) {}

     // add controls to window plane
     virtual void AddControl(DTControl* c);
     // handle events
     virtual bool HandleEvent(uint16_t x, uint16_t y, bool pressed);
     // handle rendering
     virtual void Render();
     // handle invalidate
     virtual void Invalidate();

     // DTor
     virtual ~DTWindow()
     {
        while (_controls != NULL)
        {
            DTCList* l = _controls->next;
            delete _controls;
            _controls = l;
        }
     }

    protected:
     uint16_t _bkg_color;
     DTCList* _controls;
};


#endif