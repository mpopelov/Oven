#include "DTWeird.h"

void DTWeird::Render(bool parentCleared)
{
    // skip control rendering if it's hidden but do not reset invalidation flag as we might expect updates
    // to be redrawn once we are visible again
    if( !(_flags & DTCONTROL_FLAGS_VISIBLE) ) return;

    // only render if invalidated flag is set
    if(_flags & DTCONTROL_FLAGS_INVALIDATED){
        
        // some basic drawing goes here: just clean the surface with back color if parent control was not invalidated
        if(!parentCleared) _gfx.fillRect(_x, _y, _w, _h, TFT_BLACK);

        // draw the text
        _gfx.setTextSize(1);
        _gfx.setTextColor(TFT_CYAN, TFT_DARKGREY);
        _gfx.setTextDatum(ML_DATUM);
        _gfx.setTextPadding(0);

        // draw string - note _x padding with 2 pixels to compensate for borders
        // !!
        // !! NB! make sure the call to drawString(const char*, ...) is made
        // !! for some reason overloaded version drawString(const String&, ...) does a local copy on the stack
        // !! which is a waste of resources! String class constructor in case of PROGMEM strings already stores a copy in RAM
        // !!
        _gfx.drawString(c_str(), _x+2, _y+(_h/2)-1);
        
        // remember to reset invalidation flags
        _flags &= ~DTCONTROL_FLAGS_INVALIDATED;
    }
    // nothing to do at this point
}