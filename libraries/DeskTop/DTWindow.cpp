/*
* Simple window implementation
*/

#include "DTWindow.h"

// add control to a linked list of controls
void DTWindow::AddControl(DTControl* c)
{
    // only add non-null controls
    if( c != NULL ){
        // add control to the start of linked list
        DTCList* l = new DTCList(c, _controls, NULL);
        _controls = l;
    }
}

// handle events
bool DTWindow::HandleEvent(uint16_t x, uint16_t y, bool pressed)
{
    DTCList* l = _controls;
    bool result = false;

    while (l != NULL)
    {
        if(result = l->control->HandleEvent(x,y,pressed)){
            // in case any of our controls handled the event - invalidate window and break;
            break;
        }
        l = l->next;
    }
    
    return result;
}

// handle rendering
void DTWindow::Render()
{
    // skip control rendering if it's hidden but do not reset invalidation flag as we might expect updates
    // to be redrawn once we are visible again
    if( !(_flags & DTCONTROL_FLAGS_VISIBLE) ) return;

    // go hit child controls to render themselves
    DTCList* l = _controls;
    while (l != NULL)
    {
        l->control->Render();
        l = l->next;
    }
    
}