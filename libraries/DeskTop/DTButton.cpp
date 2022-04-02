#include "DTButton.h"

void DTButton::Render()
{
    // skip control rendering if it's hidden but do not reset invalidation flag as we might expect updates
    // to be redrawn once we are visible again
    if( !(_flags & DTCONTROL_FLAGS_VISIBLE) ) return;

    // only render if invalidated flag is set
    if(_flags & DTCONTROL_FLAGS_INVALIDATED){
        
        // in case parent has not cleared the drawing plane: draw rounded rectangle as button; r = 4
        if(!(_flags & DTCONTROL_FLAGS_PARENT_INVALIDATED)) _gfx->fillRoundRect(_x, _y, _w, _h, 4, _btn_color);

        // draw the text
        _gfx->setFreeFont(_font);
        _gfx->setTextSize(1);
        _gfx->setTextColor(_txt_color, _btn_color);
        _gfx->setTextDatum(MC_DATUM);
        _gfx->setTextPadding(0);


        // draw string - note _x padding with 2 pixels
        _gfx->drawString(_lbl, _x + (_w/2) -1, _y + (_h/2) -1);
        // remember to reset invalidation flags
        _flags &= ~DTCONTROL_FLAGS_INVALIDATIONRST;
    }
    // nothing to do at this point
}

bool DTButton::HandleEvent(uint16_t x, uint16_t y, bool pressed)
{
    if( (x >= _x) && (y >= _y) && (x <= _x + _w) && (y <= _y + _h) )
    {
        // invert the color regardless of pressed/released event
        _btn_color = ~_btn_color;
        _txt_color = ~_txt_color;

        // trigger call-back function
        _callback();

        // invalidate the control so it is redrawn correctly
        Invalidate(false);
        return true;
    }
    return false;
}