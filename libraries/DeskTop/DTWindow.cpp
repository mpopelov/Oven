/*
* Simple window implementation
*/

#include "DTWindow.h"

// add control to a linked list of controls
void DTWindow::AddControl(DTControl* c)
{
    // only add non-null controls
    if( c != nullptr ){
        // add control to the start of linked list
        DTCList* l = new DTCList(c, _controls);
        _controls = l;
    }
}

// handle events
bool DTWindow::HandleEvent(uint16_t x, uint16_t y, bool pressed)
{
    DTCList* l = _controls;
    bool result = false;

    while (l != nullptr)
    {
        if(result = l->control->HandleEvent(x,y,pressed)){
            // in case any of our controls handled the event
            break;
        }
        l = l->next;
    }
    
    return result;
}

// handle rendering
void DTWindow::Render(bool parentCleared)
{
    // skip control rendering if it's hidden but do not reset invalidation flag as we might expect updates
    // to be redrawn once we are visible again
    if( !(_flags & DTCONTROL_FLAGS_VISIBLE) ) return;

    // redraw ourselves in case we are invalidated
    if(_flags & DTCONTROL_FLAGS_INVALIDATED)
    {
        // if parent has not invalidated - clear window contents with background color
        if(!parentCleared){
            _gfx.fillRect(_x, _y, _w, _h, _bkg_color);
            parentCleared = true;
        }

        // remember to reset invalidation flags
        _flags &= ~DTCONTROL_FLAGS_INVALIDATED;
    }

    // trigger rendering of child components - they might need to be redrawn even if this one is not in invalidated state
    DTCList* l = _controls;
    while (l != nullptr)
    {
        l->control->Render(parentCleared); // parentCleared is adjusted in the code above to reflect clearing of the area either by parent or this component
        l = l->next;
    }
}

void DTWindow::Invalidate()
{
    _flags |= DTCONTROL_FLAGS_INVALIDATED;

    // go hit child controls to render themselves as we are about to redraw the entire window
    DTCList* l = _controls;
    while (l != nullptr)
    {
        l->control->Invalidate(); 
        l = l->next;
    }
}