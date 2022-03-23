#include "DTLabel.h"

void DTLabel::SetText(const char* t)
{
    _lbl_text = t;
    Invalidate();
}

void DTLabel::Render()
{
    // skip control rendering if it's hidden but do not reset invalidation flag as we might expect updates
    // to be redrawn once we are visible again
    if( !(_flags & DTCONTROL_FLAGS_VISIBLE) ) return;

    // only render if invalidated flag is set
    if(_flags & DTCONTROL_FLAGS_INVALIDATED){
        
        // some basic drawing goes here: just clean the surface with black color
        _gfx->fillRect(_x, _y, _w, _h, _bkg_color);

        // redraw borders if necessary
        if(_flags & DTLABEL_BRDR_TOP) _gfx->drawFastHLine(_x, _y, _w, _brd_color);
        if(_flags & DTLABEL_BRDR_LEFT) _gfx->drawFastVLine(_x, _y, _h, _brd_color);
        if(_flags & DTLABEL_BRDR_RIGHT) _gfx->drawFastVLine(_x+_w-1, _y, _h, _brd_color);
        if(_flags & DTLABEL_BRDR_BOTTOM) _gfx->drawFastHLine(_x, _y+_h-1, _w, _brd_color);

        // draw the text
        _gfx->setFreeFont(_font);
        _gfx->setTextSize(1);
        _gfx->setTextColor(_lbl_color, _bkg_color);
        _gfx->setTextDatum(ML_DATUM);
        _gfx->setTextPadding(0);

        // draw string - note _x padding with 2 pixels
        if(_flags & DTLABEL_FPSTR){
            _gfx->drawString(_lbl_rom, _x+2, _y+(_h/2)-1);
        }else{
            _gfx->drawString(_lbl_text, _x + 2, _y + (_h/2) -1);
        }
        
        
        // remember to reset invalidation flag
        _flags &= ~DTCONTROL_FLAGS_INVALIDATED;
    }
    // nothing to do at this point
}
