#include "DTControl.h"

//
// Base class implementation
//

// Trigger invalidation flag to render control updates next time Render is called
void DTControl::Invalidate()
{
    _flags |= DTCONTROL_FLAGS_INVALIDATED;
}

// Change visibility
void DTControl::Visible(bool v)
{
    if(v){ _flags |= DTCONTROL_FLAGS_VISIBLE; } else {_flags &= ~DTCONTROL_FLAGS_VISIBLE; }
}

// Render control in case state has changed
void DTControl::Render()
{
    // skip control rendering if it's hidden but do not reset invalidation flag as we might expect updates
    // to be redrawn once we are visible again
    if( !(_flags & DTCONTROL_FLAGS_VISIBLE) ) return;

    // only render if invalidated flag is set
    if(_flags & DTCONTROL_FLAGS_INVALIDATED){
        
        // some basic drawing goes here: just clean the surface with black color
        _gfx->fillRect(_x, _y, _w, _h, TFT_BLACK);

        // remember to reset invalidation flag
        _flags &= ~DTCONTROL_FLAGS_INVALIDATED;
    }
    // nothing to do at this point
}

bool DTControl::HandleEvent(uint16_t x, uint16_t y, bool pressed)
{
    return false;
}
