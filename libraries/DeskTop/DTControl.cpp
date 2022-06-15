#include "DTControl.h"

/**
 * @brief Trigger invalidation flag to render control updates next time Render is called
 * 
 */
void DTControl::Invalidate()
{
    _flags |= DTCONTROL_FLAGS_INVALIDATED;
}

/**
 * @brief Change visibility
 * 
 * @param v Set to true if control is visible, false otherwise.
 */
void DTControl::Visible(bool v)
{
    if(v){ _flags |= DTCONTROL_FLAGS_VISIBLE; } else {_flags &= ~DTCONTROL_FLAGS_VISIBLE; }
}

/**
 * @brief Render control on the screen using supplied graphical context
 * 
 */
void DTControl::Render(bool parentCleared)
{
    // skip control rendering if it's hidden but do not reset invalidation flag as the control
    // might need to be rendered once visible again
    if( !(_flags & DTCONTROL_FLAGS_VISIBLE) ) return;

    // only render if invalidated flag is set
    if(_flags & DTCONTROL_FLAGS_INVALIDATED){
        
        // first step is to clear the surface.
        // in case parent control has also been invalidated we may skip this step to avoid costly drawing on TFT via SPI bus and avoid screen flickering.
        if(!parentCleared) _gfx.fillRect(_x, _y, _w, _h, TFT_BLACK);

        // remember to reset invalidation flags
        _flags &= ~DTCONTROL_FLAGS_INVALIDATED;
    }
    // nothing to do at this point
}

/**
 * @brief basic stub for event handler: DTControl::HandleEvent simply does nothing and indicates it did not process the event
 * 
 * @param x event X coordinate
 * @param y event Y coordinate
 * @param pressed touchscreen event type: true = pressed, false = released 
 * @return true - event was handled, false - not handled
 */
bool DTControl::HandleEvent(uint16_t x, uint16_t y, bool pressed)
{
    return false;
}
