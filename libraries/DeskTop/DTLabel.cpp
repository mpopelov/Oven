#include "DTLabel.h"

/*void DTLabel::SetText(const String& t)
{
    _lbl = t;
    Invalidate(); // parent control is not invalidating - it's just us setting text
}*/

void DTLabel::SetTextColor(uint16_t c)
{
    _lbl_color = c;
    Invalidate();
}

void DTLabel::SetBackColor(uint16_t c){
    _bkg_color = c;
    Invalidate();
}

void DTLabel::Render(bool parentCleared)
{
    // skip control rendering if it's hidden but do not reset invalidation flag as we might expect updates
    // to be redrawn once we are visible again
    if( !(_flags & DTCONTROL_FLAGS_VISIBLE) ) return;

    // only render if invalidated flag is set
    if(_flags & DTCONTROL_FLAGS_INVALIDATED){
        
        // some basic drawing goes here: just clean the surface with back color if parent control was not invalidated
        if(!parentCleared) _gfx.fillRect(_x, _y, _w, _h, _bkg_color);

        // redraw borders if necessary
        if(_flags & DTLABEL_BRDR_TOP) _gfx.drawFastHLine(_x, _y, _w, _brd_color);
        if(_flags & DTLABEL_BRDR_LEFT) _gfx.drawFastVLine(_x, _y, _h, _brd_color);
        if(_flags & DTLABEL_BRDR_RIGHT) _gfx.drawFastVLine(_x+_w-1, _y, _h, _brd_color);
        if(_flags & DTLABEL_BRDR_BOTTOM) _gfx.drawFastHLine(_x, _y+_h-1, _w, _brd_color);

        // draw the text
        _gfx.setFreeFont(_font);
        _gfx.setTextSize(1);
        _gfx.setTextColor(_lbl_color, _bkg_color);
        _gfx.setTextDatum(ML_DATUM);
        _gfx.setTextPadding(0);

        // draw string - note _x padding with 2 pixels to compensate for borders
        _gfx.drawString(c_str(), _x+2, _y+(_h/2)-1);
        
        // remember to reset invalidation flags
        _flags &= ~DTCONTROL_FLAGS_INVALIDATED;
    }
    // nothing to do at this point
}
